"""HH_260906 - Keep the preregistered Town05 sign DEV gate conservative and reproducible."""

import pytest

from scripts.e2e import audit_map_town05_sign_dev_scout as audit


@pytest.mark.parametrize("pixels,expected", [
    (0, "0"), (1, "1-4"), (4, "1-4"), (5, "5-64"),
    (64, "5-64"), (65, ">=65"),
])
def test_whole_frame_sign_bins(pixels, expected):
    assert audit.sign_bin(pixels) == expected


def test_sign_gate_does_not_promote_an_unreviewed_upper_bound():
    """HH_260906 - Even 24 potentially qualifying frames need independent RGB object review."""
    assert audit.preliminary_status(119, 24) == "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"
    assert audit.preliminary_status(120, 23) == "REJECTED_SIGN_CAPABLE_DEV"
    assert audit.preliminary_status(120, 24) == "PENDING_RGB_OBJECT_REVIEW_NOT_ADMITTED"


def test_exact_epic_owner_and_source_archive():
    proof = audit.audit_owner()
    assert proof["route_sha256"] == audit.ROUTE_SHA256
    assert proof["quality"] == "Epic"
    assert proof["exit_code"] == 0
    assert proof["archived_source_count"] == 13
    assert proof["port_released_after_stop"] is True
    with pytest.raises(ValueError, match="route bytes differ"):
        audit.audit_owner(expected_sha="0" * 64)


def test_published_audit_fails_sign_dev_without_training():
    summary = audit.build_summary()
    assert summary["raw_frames"] == 240
    assert summary["retained_frames_0p25m"] == 195
    assert summary["retained_gate_passed"] is True
    assert summary["retained_qualifying_frame_upper_bound"] == 0
    assert summary["raw_max_whole_frame_tag8_pixels"] == 4
    assert summary["retained_max_whole_frame_tag8_pixels"] == 3
    assert summary["qualifying_rgb_confirmed_distinct_object_count"] == 0
    assert summary["all_rgb_sign_objects_exhaustively_counted"] is False
    assert summary["status"] == "REJECTED_SIGN_CAPABLE_DEV"
    assert summary["sign_dev_gate_passed"] is False
    assert summary["dataset_admitted"] is False
    assert summary["model_predictions_made"] is False
    audit.write_or_check(summary, audit.preview(summary))
