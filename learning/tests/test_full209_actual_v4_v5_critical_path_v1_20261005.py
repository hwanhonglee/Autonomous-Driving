"""HH_261005 - Synthetic scalar-only V4/V5 reader tests never load actual sensor pixels, models or TEST data."""

import copy
import math

import pytest

from scripts.e2e import analyze_full209_actual_v4_v5_critical_path_v1_20261005 as reader


def test_mean_and_linear_p95_retain_cold_failed_tail():
    values = [200., 1., 2., 101.]
    result = reader.stats(values)
    assert result["count"] == 4 and result["mean"] == 76. and result["maximum"] == 200.
    assert result["p95_linear"] == pytest.approx(185.15)


@pytest.mark.parametrize("values", ([], [True], [math.nan], [math.inf]))
def test_nonfinite_empty_or_boolean_timing_denies(values):
    with pytest.raises(RuntimeError):
        reader.stats(values)


def test_exact_endpoint_denominator_keeps_slow_last_interval():
    result = reader.cycles([{"ns": v} for v in (1000000, 2000000, 202000000)], "ns")
    assert result["endpoint_count"] == 3 and result["interval_count"] == 2
    assert result["interval_ms"]["mean"] == 100.5
    assert result["rate_hz"] == pytest.approx(2 / .201)
    assert result["cold_or_failed_endpoint_removed"] is False


@pytest.mark.parametrize("values", ((1,), (1, 1), (2, 1), (1., 2), (True, 2)))
def test_invalid_endpoint_clock_denies(values):
    with pytest.raises(RuntimeError):
        reader.cycles([{"ns": v} for v in values], "ns")


def test_metric_missing_failed_row_field_denies():
    with pytest.raises(RuntimeError):
        reader.metrics([dict(NN_ms=1.), dict(failed=True)])


def recording_fixture():
    receipt = dict(frame=100, ACK_receipt_sequence=4, recording_started_monotonic_ns=2000000,
        recording_finished_monotonic_ns=3000000, recording_seconds=.001, failure=None,
        enqueued_camera_count=6, written_camera_count=6, acknowledged_before_recording=True,
        all_queued_images_written=True, no_source_image_dropped=True, source_timestamp_changed=False,
        source_pixels_changed=False, synchronous_flush_before_next_tick=True,
        parallel_encoding_research=dict(all_submitted_encodes_joined=True, submitted_encode_count=6,
            original_role_ordered_writes=True, background_work_after_flush=False, all_encodes_joined_monotonic_ns=2500000))
    walls = [dict(ACK_sequence=4, command_frame=100, ACK_server_accepted=True, ACK_finished_monotonic_ns=1000000)]
    groups = {100: {}, 101: {"world_tick_started": dict(stage_finished_monotonic_ns=4000000)}}
    return [receipt], [dict(frame=100)], walls, groups


def test_real_clock_scalar_after_ACK_before_next_native_join():
    rows = reader.recording_join(*recording_fixture())
    assert rows == [dict(frame=100, ACK_sequence=4, recording_started_monotonic_ns=2000000,
        recording_finished_monotonic_ns=3000000, synchronous_after_ACK_JPEG_flush_ms=1.)]


@pytest.mark.parametrize("fault", ("missing", "camera", "ACK", "ACK_fail", "early", "next_tick", "duration",
    "failure", "dropped", "changed", "background", "unjoined", "count", "join_clock"))
def test_every_failed_recording_or_chronology_denies_not_dropped(fault):
    receipts, cameras, walls, groups = copy.deepcopy(recording_fixture())
    row = receipts[0]
    if fault == "missing": receipts.clear()
    elif fault == "camera": cameras[0]["frame"] += 1
    elif fault == "ACK": walls[0]["command_frame"] += 1
    elif fault == "ACK_fail": walls[0]["ACK_server_accepted"] = False
    elif fault == "early": row["recording_started_monotonic_ns"] = 0
    elif fault == "next_tick": groups[101]["world_tick_started"]["stage_finished_monotonic_ns"] = 2500000
    elif fault == "duration": row["recording_seconds"] = 0.
    elif fault == "failure": row["failure"] = "synthetic_failed_tail"
    elif fault == "dropped": row["no_source_image_dropped"] = False
    elif fault == "changed": row["source_pixels_changed"] = True
    elif fault == "background": row["parallel_encoding_research"]["background_work_after_flush"] = True
    elif fault == "unjoined": row["parallel_encoding_research"]["all_submitted_encodes_joined"] = False
    elif fault == "count": row["written_camera_count"] = 5
    elif fault == "join_clock": row["parallel_encoding_research"]["all_encodes_joined_monotonic_ns"] = 0
    with pytest.raises(RuntimeError):
        reader.recording_join(receipts, cameras, walls, groups)
