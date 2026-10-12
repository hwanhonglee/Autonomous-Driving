"""HH_261001 - Full fixed45s media, prior-any-source ACK semantics, and no invented terminal effect."""

import pytest

from scripts.e2e import publish_temporal_route_recovery_fast45_batch_20261001 as publisher


def bundle(failure=None):
    return dict(cameras=[dict(frame=2*i) for i in range(450)],trial=dict(first_latched_failure=failure))


def test_full45s_fixed_bins_include_all_failure_tail():
    assert publisher.indices()==list(range(0,450,5)) and publisher.indices()[-1]==445
    for count in (449,451,True):
        with pytest.raises(RuntimeError):publisher.indices(count)


def test_success_quartiles_and_actual_last_camera_are_fixed_not_selected():
    assert list(publisher.png_indices(bundle()).values())==[0,112,224,337,449]


def test_failure_rejection_and_first_actual_emergency_camera():
    result=publisher.png_indices(bundle(dict(outgoing_command_frame=100,first_observed_emergency_frame=101)))
    assert result["02_last_before_rejection.png"]==49 and result["03_rejection_observation.png"]==50
    assert result["04_observed_emergency.png"]==51 and result["05_last.png"]==449


def test_terminal_ACK_without_later_observation_is_never_invented():
    result=publisher.png_indices(bundle(dict(outgoing_command_frame=898,first_observed_emergency_frame=None)))
    assert result["04_terminal_ACK_no_later_camera.png"]==449 and "04_observed_emergency.png" not in result


def test_last_odd_native_rejection_has_no_invented_camera_exposure():
    result=publisher.png_indices(bundle(dict(outgoing_command_frame=899,first_observed_emergency_frame=None)))
    assert result["03_no_camera_at_terminal_rejection.png"]==449
    assert result["04_terminal_ACK_no_later_camera.png"]==449 and result["05_last.png"]==449
    assert "03_rejection_observation.png" not in result


def test_most_recent_prior_emergency_clears_any_older_learned_plan():
    state=dict(frame=13,current_control={})
    receipts=[dict(sequence=1,before_frame=10,status="ACKNOWLEDGED",server_accepted=True),
              dict(sequence=2,before_frame=12,status="ACKNOWLEDGED",server_accepted=True)]
    commands=[dict(ack_receipt_sequence=1,control_source="learned_trajectory_tracker"),
              dict(ack_receipt_sequence=2,control_source="emergency_brake",
                   application=dict(acknowledged=True,applied_source="emergency_brake",learned_control_applied=False))]
    receipt,command,plan,label=publisher.prior.observed_prior_ack(state,receipts,commands,{})
    assert receipt["sequence"]==2 and command["control_source"]=="emergency_brake" and plan==[] and "cleared" in label


def test_same_frame_outgoing_emergency_is_not_already_observed():
    state=dict(frame=12)
    receipts=[dict(sequence=1,before_frame=10,status="ACKNOWLEDGED",server_accepted=True,reason="stationary_warmup_start"),
              dict(sequence=2,before_frame=12,status="ACKNOWLEDGED",server_accepted=True)]
    receipt,command,plan,label=publisher.prior.observed_prior_ack(state,receipts,[],{})
    assert receipt["sequence"]==1 and command is None and plan==[] and "warmup" in label


def test_existing_output_rejected_before_read_or_render(tmp_path,monkeypatch):
    monkeypatch.setattr(publisher,"load_archive",lambda *a,**kw:pytest.fail("must not read"))
    with pytest.raises(RuntimeError,match="create-only"):publisher.publish(tmp_path,"unused","a"*64)


def test_cli_requires_explicit_render_or_validate(monkeypatch):
    monkeypatch.setattr(publisher,"load_archive",lambda *a,**kw:pytest.fail("must not read"))
    with pytest.raises(RuntimeError,match="choose"):
        publisher.main(["--report","unused","--report-sha256","a"*64,"--output","unused"])
