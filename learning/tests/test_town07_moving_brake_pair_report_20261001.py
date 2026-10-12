"""HH_261001 - CPU-only checks for truthful retained moving-brake pair reporting."""

from copy import deepcopy

import pytest

from scripts.e2e import report_town07_moving_brake_pair_20261001 as report


ARMS = ("A_original", "B_suppress_normal_moving_brake")


def decision(arm=ARMS[1], desired=1.4, original_brake=.03):
    original = dict(throttle=0., brake=original_brake, steer=.04, reverse=False,
                    hand_brake=False, desired_speed_mps=desired,
                    pursuit_xy_base_m=[1.8, -.04],
                    steering_convention="CARLA_right_positive_from_base_link_y_left")
    returned = deepcopy(original)
    if arm == ARMS[1] and desired > .05:
        returned["brake"] = 0.
    changed = returned["brake"] != original_brake
    return dict(control_source="learned_trajectory_tracker", learned_control_proposed=True,
                emergency_reason=None, control=returned, command_frame=101, inference_frame=100,
                application=dict(learned_control_applied=True),
                moving_brake_ab=dict(policy_id="town07_w1_normal_moving_brake_ab_v1", arm=arm,
                    desired_speed_mps=desired, stop_speed_threshold_mps=.05,
                    moving_target=desired > .05, original_normal_tracker_control=original,
                    original_normal_brake=original_brake, proposed_normal_brake=returned["brake"],
                    normal_brake_suppression_proposed=changed,
                    normal_tracker_called=True, normal_control_returned=True,
                    normal_brake_suppression_returned=changed,
                    feedforward_added=False, integral_added=False, control_cadence_changed=False,
                    stop_branch_changed=False, emergency_authority_changed=False,
                    actuator_acknowledged=False,
                    actual_control_source_requires_original_collector_receipt=True))


def control():
    return dict(throttle=.1, brake=0., steer=.04, reverse=False, hand_brake=False,
                manual_gear_shift=False, gear=1)


def state(frame, timestamp, vx, progress=0., phase="driving", sequence=1):
    return dict(frame=frame, timestamp=timestamp, vx=vx, route_progress_m=progress,
                route_cte_m=.1, capture_phase=phase, current_control=control(),
                control_transport=dict(status="PASS", expected_receipt_sequence=sequence,
                    expected_command_before_frame=frame - 1, observed_frame=frame,
                    control_read_frame_before=frame, control_read_frame_after=frame,
                    frame_binding_pass=True, mismatched_fields=[], absolute_tolerance=1e-6,
                    automatic_gear_equality_required=False, lookback_relabeling_allowed=False,
                    physical_actuation_proven=False))


def receipt(frame=101, sequence=1, actor_id=114):
    return dict(sequence=sequence, mode="acknowledged_batch", status="ACKNOWLEDGED",
                actor_id=actor_id, response_actor_id=actor_id, server_accepted=True,
                before_frame=frame - 1, after_ack_frame=frame - 1,
                expected_before_frame=frame - 1, do_tick=False,
                reason="next_driving_control", requested_control=control(),
                physical_actuation_proven=False)


@pytest.mark.parametrize("arm", ARMS)
def test_normal_arm_audit_preserves_all_fields_except_declared_b_brake(arm):
    row = decision(arm)
    before = deepcopy(row)
    assert report.audit_control_decision(row, arm) == []
    assert row == before


@pytest.mark.parametrize("desired", [0., .01, .05, .0500000001])
def test_normal_brake_suppression_has_strict_moving_threshold(desired):
    row = decision(desired=desired)
    assert report.audit_control_decision(row, ARMS[1]) == []
    assert row["control"]["brake"] == (0. if desired > .05 else .03)
    assert row["moving_brake_ab"]["normal_brake_suppression_returned"] is (desired > .05)


def test_zero_original_brake_does_not_claim_a_suppression():
    row = decision(original_brake=0.)
    assert report.audit_control_decision(row, ARMS[1]) == []
    assert row["moving_brake_ab"]["normal_brake_suppression_proposed"] is False


@pytest.mark.parametrize("field,value", [
    ("throttle", .01), ("brake", .03), ("steer", .1), ("reverse", True),
    ("hand_brake", True), ("desired_speed_mps", 1.3),
    ("pursuit_xy_base_m", [1.8, .04]),
    ("normal_brake_suppression_returned", True),
])
def test_returned_normal_control_cannot_hide_another_change_or_actuator_audit(field, value):
    row = decision()
    row["control"][field] = value
    assert report.audit_control_decision(row, ARMS[1])


@pytest.mark.parametrize("field,value", [
    ("arm", ARMS[0]), ("moving_target", False), ("normal_tracker_called", False),
    ("normal_control_returned", False), ("normal_brake_suppression_proposed", False),
    ("normal_brake_suppression_returned", False),
    ("stop_speed_threshold_mps", .06), ("original_normal_brake", .04),
])
def test_stored_intervention_flags_cannot_replace_recomputed_control_audit(field, value):
    row = decision()
    row["moving_brake_ab"][field] = value
    assert report.audit_control_decision(row, ARMS[1])


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True])
def test_nonfinite_or_boolean_original_brake_is_not_numeric_evidence(bad):
    row = decision()
    row["moving_brake_ab"]["original_normal_tracker_control"]["brake"] = bad
    assert report.audit_control_decision(row, ARMS[1])


def test_late_deadline_emergency_retains_proposal_without_claiming_returned_suppression():
    row = decision()
    row.update(control_source="emergency_brake", learned_control_proposed=False,
               emergency_reason="inference/control deadline exceeded",
               control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False),
               application=dict(learned_control_applied=False))
    row["moving_brake_ab"].update(normal_control_returned=False,
                                   normal_brake_suppression_returned=False)
    assert report.audit_control_decision(row, ARMS[1]) == []
    row["moving_brake_ab"]["normal_brake_suppression_returned"] = True
    assert report.audit_control_decision(row, ARMS[1])


def test_expert_startup_without_policy_audit_is_accepted_but_learned_control_requires_it():
    startup = dict(control_source="expert_startup", learned_control_proposed=False)
    assert report.audit_control_decision(startup, ARMS[0]) == []
    row = decision()
    del row["moving_brake_ab"]
    assert report.audit_control_decision(row, ARMS[1])


def test_partial_native_summary_keeps_observed_span_and_nominal_coverage_separate():
    states = [state(100, 10., 0., phase="stationary_warmup"),
              state(101, 10.05, 0., phase="stationary_warmup"),
              state(102, 10.1, 0., phase="driving"),
              state(103, 10.15, .51, progress=.1),
              state(104, 10.2, -.3, progress=.3),
              state(105, 10.25, .04, progress=.6)]
    states[4]["route_cte_m"] = -.4
    rows = [decision(), decision()]
    rows[1]["moving_brake_ab"]["normal_brake_suppression_returned"] = False
    rows[1]["application"]["learned_control_applied"] = False
    result = report.summarize_states(states, rows)
    assert result["state_count"] == 6
    assert result["observed_native_span_seconds"] == pytest.approx(.25)
    assert result["nominal_native_coverage_seconds"] == pytest.approx(.3)
    assert result["full_twenty_second_native_count"] is False
    assert result["net_route_progress_m"] == pytest.approx(.6)
    assert result["maximum_abs_route_cte_m"] == pytest.approx(.4)
    assert result["first_speed_at_or_below_0p05_after_motion_s"] == pytest.approx(.25)
    assert result["unique_inference_frames_count"] == 1
    assert result["normal_suppression_returned_command_count"] == 1
    assert result["acknowledged_learned_command_count"] == 1


@pytest.mark.parametrize("end_speed,expected", [(.1, 0), (.099, 1)])
def test_native_signed_shock_threshold_is_strict(end_speed, expected):
    states = [state(100, 0., .5), state(101, .05, end_speed)]
    result = report.summarize_states(states, [])
    assert result["signed_rate_below_minus_8_mps2_count"] == expected
    assert result["minimum_signed_rate_mps2"] == pytest.approx((end_speed - .5) / .05)


def test_initial_stationary_or_reverse_speed_does_not_invent_a_near_stop():
    states = [state(100, 0., 0., phase="stationary_warmup"),
              state(101, .05, .51), state(102, .1, -.3), state(103, .15, -.06)]
    assert report.summarize_states(states, [])["first_speed_at_or_below_0p05_after_motion_s"] is None
    no_motion = [state(100, 0., 0., phase="stationary_warmup"), state(101, .05, .04)]
    assert report.summarize_states(no_motion, [])["first_speed_at_or_below_0p05_after_motion_s"] is None


def test_full_native_count_does_not_relabel_first_to_last_span_as_twenty_seconds():
    states = [state(100 + index, index * .05, 1., progress=index * .05)
              for index in range(400)]
    result = report.summarize_states(states, [])
    assert result["full_twenty_second_native_count"] is True
    assert result["observed_native_span_seconds"] == pytest.approx(19.95)
    assert result["nominal_native_coverage_seconds"] == pytest.approx(20.)


def test_peak_speeds_and_latency_p95_have_explicit_noninterpolated_convention():
    states = [state(100, 0., 1.), state(101, .05, 2.)]
    states[0]["world_velocity_carla"] = [3., 4., 0.]
    states[1]["world_velocity_carla"] = [0., 2., 0.]
    decisions = [dict(inference_frame=frame, total_seconds=frame/1000.) for frame in range(1, 21)]
    # HH_261001 - A repeated command is not another latency observation.
    decisions.append(dict(inference_frame=1, total_seconds=999.))
    result = report.summarize_states(states, decisions)
    assert result["maximum_forward_speed_mps"] == 2.
    assert result["maximum_world_planar_speed_mps"] == 5.
    assert result["inference_seconds"]["p95"] == .019
    assert result["inference_seconds"]["sample_count"] == 20
    assert result["inference_seconds"]["p95_convention"] == "nearest rank: sorted[ceil(0.95*n)-1], no interpolation"


def test_peak_world_speed_is_not_invented_from_forward_speed_or_partial_raw_velocity():
    states = [state(100, 0., 1.), state(101, .05, 2.)]
    states[0]["world_velocity_carla"] = [3., 4., 0.]
    result = report.summarize_states(states, [])
    assert result["maximum_world_planar_speed_mps"] is None
    assert result["inference_seconds"]["p95"] is None
    assert report.STATIC_MONTAGE_SECONDS == 5.0


def test_camera_montage_uses_five_seconds_and_gif_retains_full_two_hz_samples(tmp_path, monkeypatch):
    Image = pytest.importorskip("PIL.Image")
    ImageDraw = pytest.importorskip("PIL.ImageDraw")
    states = [dict(frame=index, timestamp=index*.05, x=index*.05, y=0., yaw=0.) for index in range(400)]
    cameras = [dict(frame=index, timestamp=index*.05,
                    images={name: name for name in report.CAMERAS}) for index in range(0, 400, 2)]
    texts, saved = [], []
    original_text = ImageDraw.ImageDraw.text
    original_open = Image.open
    def record_text(self, xy, text, *args, **kwargs):
        texts.append(text)
        return original_text(self, xy, text, *args, **kwargs)
    monkeypatch.setattr(ImageDraw.ImageDraw, "text", record_text)
    monkeypatch.setattr(Image, "open", lambda path: Image.new("RGB", (640, 360))
                        if isinstance(path, report.Path) else original_open(path))
    monkeypatch.setattr(Image.Image, "save", lambda self, path, **kwargs: saved.append((path.name, kwargs)))
    trial = dict(trial_id="fixture", arm=ARMS[0])
    bundle = dict(states=states, cameras=cameras, episode=tmp_path,
                  route=dict(route=[]), decisions=[])
    report.camera_playback(dict(trials=[trial]), dict(fixture=bundle), tmp_path, True)
    assert any("post-handoff snapshot | t=5.0 s" in value for value in texts)
    assert any("sampled at 2 Hz | t=0.0 s" in value for value in texts)
    assert any("sampled at 2 Hz | t=19.5 s" in value for value in texts)
    assert saved[0][0] == "actual_six_camera_montage.png"
    assert saved[1][0] == "actual_cameras_sampled_2hz.gif"
    assert saved[1][1]["duration"] == 500
    assert len(saved[1][1]["append_images"]) == 39


def test_state_receipt_binding_accepts_actual_native_alignment_and_auto_gear_change():
    row = state(101, .05, 1.)
    row["current_control"]["gear"] = 2
    assert report.check_state_receipt_binding([row], [receipt()], 114)["issue_counts"] == {}


@pytest.mark.parametrize("field,value", [
    ("status", "FAILED"), ("server_accepted", False), ("server_accepted", 1),
    ("actor_id", 115), ("response_actor_id", 115), ("do_tick", True),
    ("before_frame", 99), ("after_ack_frame", 101), ("expected_before_frame", 99),
])
def test_bad_ack_or_command_frame_cannot_pass_stored_alignment(field, value):
    ack = receipt()
    ack[field] = value
    assert report.check_state_receipt_binding([state(101, .05, 1.)], [ack], 114)["issue_counts"]


@pytest.mark.parametrize("field,value", [
    ("throttle", float("nan")), ("brake", float("inf")), ("steer", True),
    ("reverse", 0), ("hand_brake", 0), ("manual_gear_shift", 0), ("gear", True),
])
@pytest.mark.parametrize("side", ["observed", "requested"])
def test_control_binding_requires_finite_numeric_pedals_and_strict_flags(field, value, side):
    row, ack = state(101, .05, 1.), receipt()
    target = row["current_control"] if side == "observed" else ack["requested_control"]
    target[field] = value
    assert report.check_state_receipt_binding([row], [ack], 114)["issue_counts"]


def test_missing_receipt_and_control_read_frame_drift_are_audit_failures():
    row = state(101, .05, 1.)
    assert report.check_state_receipt_binding([row], [], 114)["issue_counts"]
    row["control_transport"]["control_read_frame_after"] += 1
    assert report.check_state_receipt_binding([row], [receipt()], 114)["issue_counts"]


def test_fresh_report_output_is_created_outside_retained_campaign(tmp_path):
    campaign = tmp_path / "retained"
    campaign.mkdir()
    output = tmp_path / "new_report"
    assert report.fresh_output(output, campaign) == output
    assert output.is_dir() and not list(output.iterdir())


def test_existing_report_evidence_is_preserved(tmp_path):
    campaign, output = tmp_path / "retained", tmp_path / "existing_report"
    campaign.mkdir()
    output.mkdir()
    evidence = output / "report.json"
    evidence.write_text("retained evidence")
    with pytest.raises(ValueError):
        report.fresh_output(output, campaign)
    assert evidence.read_text() == "retained evidence"


def test_report_output_cannot_be_inside_campaign_or_a_symlink(tmp_path):
    campaign = tmp_path / "retained"
    campaign.mkdir()
    inside = campaign / "A1_original" / "report"
    with pytest.raises(ValueError):
        report.fresh_output(inside, campaign)
    assert not inside.exists()
    target = tmp_path / "existing"
    target.mkdir()
    link = tmp_path / "linked_report"
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError):
        report.fresh_output(link, campaign)
    assert link.is_symlink() and not list(target.iterdir())


def test_sparse_sample_arrays_hash_and_actual_decision_link(tmp_path):
    """HH_261001 - Data-only input evidence is joined to its real recorded decision."""
    import hashlib
    import json
    import numpy as np

    directory = tmp_path / "sampled_exact_runtime_inputs"
    directory.mkdir()
    value = np.arange(6, dtype=np.float32).reshape(2, 3)
    path = directory / "000000_50000000.npz"
    np.savez(path, images=value)
    header = json.dumps(dict(shape=list(value.shape), dtype=str(value.dtype)), sort_keys=True).encode()
    raw = dict(candidate_logits=[1.0])
    journal = dict(sequence=0, tensor_file=path.name, tensor_file_sha256=report.digest(path),
                   tensor_sha256=dict(images=hashlib.sha256(header+b"\0"+value.tobytes()).hexdigest()),
                   timing=dict(now_ns=50_000_000), source_checkpoint_sha256=report.CHECKPOINT,
                   raw_output_sha256=hashlib.sha256(json.dumps(raw, sort_keys=True, separators=(",", ":")).encode()).hexdigest())
    (directory / "index.jsonl").write_text(json.dumps(journal)+"\n")
    recorded = [dict(command_frame=101, raw_output=raw)]
    result = report.sample_archive_checks(tmp_path, recorded, [dict(frame=101, timestamp=.05)], 128*1024**2)
    assert result["issue_counts"] == {} and result["sample_count"] == 1
    changed = [dict(command_frame=101, raw_output=dict(candidate_logits=[2.0]))]
    result = report.sample_archive_checks(tmp_path, changed, [dict(frame=101, timestamp=.05)], 128*1024**2)
    assert result["issue_counts"]["sample_decision_raw_output_link"] == 1


def test_sparse_sample_budget_counts_all_retained_directory_bytes(tmp_path):
    directory = tmp_path / "sampled_exact_runtime_inputs"
    directory.mkdir()
    (directory / "index.jsonl").write_text("")
    (directory / "retained_sidecar.json").write_text("{}")
    result = report.sample_archive_checks(tmp_path, [], [], 1)
    assert result["directory_bytes"] == 2
    assert result["issue_counts"]["sample_directory_allowance"] == 1
