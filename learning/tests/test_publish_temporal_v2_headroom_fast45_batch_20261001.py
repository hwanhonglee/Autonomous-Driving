"""HH_261001 - Bound full45s rendering and preserve the actual latest prior-ACK emergency interpretation."""

import pytest

from scripts.e2e import publish_temporal_v2_headroom_fast45_batch_20261001 as publisher
from scripts.e2e.publish_temporal_v2_actual_failure_20261001 import observed_prior_ack


def test_whole45s_fixed_ninety_bins_not_success_selected():
    assert publisher.indices()==list(range(0,450,5))
    assert publisher.indices()[-1]==445
    for count in (200,449,451,True):
        with pytest.raises(RuntimeError):publisher.indices(count)


def test_after_emergency_clears_purple_even_with_earlier_learned_ACK():
    state=dict(frame=13)
    receipts=[dict(sequence=1,before_frame=10,status="ACKNOWLEDGED",server_accepted=True),
              dict(sequence=2,before_frame=12,status="ACKNOWLEDGED",server_accepted=True)]
    commands=[dict(ack_receipt_sequence=1,control_source="learned_trajectory_tracker"),
              dict(ack_receipt_sequence=2,control_source="emergency_brake",
                   application=dict(acknowledged=True,applied_source="emergency_brake",learned_control_applied=False))]
    receipt,command,plan,label=observed_prior_ack(state,receipts,commands,{})
    assert receipt["sequence"]==2 and command["control_source"]=="emergency_brake" and plan==[]
    assert "EMERGENCY" in label and "cleared" in label


def test_last_png_includes_actual_last_camera_not_last_twoHz_bin():
    bundle=dict(cameras=[dict(frame=i*2) for i in range(450)],trial=dict(first_latched_failure=
        dict(outgoing_command_frame=100,first_observed_emergency_frame=101)))
    result=publisher.png_indices(bundle)
    assert result["03_rejection_observation.png"]==50
    assert result["04_observed_emergency.png"]==51
    assert result["05_full_capture_last_camera.png"]==449


def test_existing_output_rejected_before_camera_read(tmp_path,monkeypatch):
    monkeypatch.setattr(publisher,"load_archive",lambda **kw:pytest.fail("existing output must fail before any camera read"))
    with pytest.raises(RuntimeError,match="create-only"):publisher.publish(tmp_path)
