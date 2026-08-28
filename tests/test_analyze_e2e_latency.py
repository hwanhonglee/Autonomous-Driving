from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


pytest.importorskip("rosbag2_py")

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "e2e" / "analyze_e2e_latency.py"
SPEC = importlib.util.spec_from_file_location("analyze_e2e_latency", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
latency = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(latency)


def _records(receipts: list[float], stamps: list[float] | None = None):
    stamps = receipts if stamps is None else stamps
    return [
        {
            "bag_ns": int(round(receipt * 1.0e9)),
            "stamp_ns": int(round(stamp * 1.0e9)),
            "index": index,
        }
        for index, (receipt, stamp) in enumerate(zip(receipts, stamps))
    ]


def test_latest_prior_pairing_is_causal_and_respects_gap() -> None:
    upstream = _records([1.0, 2.0, 4.0])
    downstream = _records([0.5, 1.5, 3.0, 7.0])

    assert latency.latest_prior_pairs(upstream, downstream) == [(0, 1), (1, 2), (2, 3)]
    assert latency.latest_prior_pairs(upstream, downstream, 2.0) == [(0, 1), (1, 2)]


def test_first_after_pairing_never_uses_an_earlier_downstream() -> None:
    upstream = _records([1.1, 2.1, 5.0])
    downstream = _records([1.0, 1.2, 2.2, 4.9])

    assert latency.first_after_pairs(upstream, downstream) == [(0, 1), (1, 2)]


def test_samples_keep_wall_receipt_and_sim_stamp_domains_separate() -> None:
    upstream = _records([100.0], [5.0])
    downstream = _records([103.0], [5.2])

    samples = latency.samples_from_pairs(
        "stage", "causal", "/up", "/down", upstream, downstream, [(0, 0)], 100_000_000_000
    )

    assert samples[0]["receipt_latency_sec"] == pytest.approx(3.0)
    assert samples[0]["stamp_delta_sec"] == pytest.approx(0.2)
    assert samples[0]["downstream_relative_wall_sec"] == pytest.approx(3.0)


def test_summary_uses_expected_percentiles() -> None:
    result = latency.summary([0.0, 1.0, 2.0, 3.0, 4.0])

    assert result["available"] is True
    assert result["median"] == pytest.approx(2.0)
    assert result["p95"] == pytest.approx(3.8)
    assert latency.summary([]) == {"available": False, "count": 0}


def test_event_rate_preserves_median_rate_and_adds_effective_rate() -> None:
    records = _records([0.0, 0.2, 0.4, 1.4])

    metrics = latency.event_rate(records)

    assert metrics["receipt_rate_hz"] == pytest.approx(5.0)
    assert metrics["effective_receipt_rate_hz"] == pytest.approx(
        1.0 / ((0.2 + 0.2 + 1.0) / 3.0)
    )


def test_candidate_front_acceptance_reports_output_to_input_ratio() -> None:
    metrics = latency.candidate_front_acceptance_metrics(
        latency.VAD_OUTPUT_TOPICS[0],
        latency.CAMERA_INFO_TOPICS[0],
        _records([1.0, 2.0]),
        _records([0.5, 1.0, 1.5, 2.0, 2.5]),
    )

    assert metrics["available"] is True
    assert metrics["candidate_count"] == 2
    assert metrics["front_count"] == 5
    assert metrics["acceptance_percent"] == pytest.approx(40.0)


def test_trajectory_freshness_reports_stale_fractions() -> None:
    samples = [
        {
            "upstream_index": index,
            "downstream_index": index,
            "receipt_latency_sec": value,
            "stamp_delta_sec": value,
        }
        for index, value in enumerate((0.1, 0.6, 1.2, 2.5))
    ]

    metrics = latency.freshness_metrics(samples)

    assert metrics["stamp_older_than_0.5_sec_percent"] == pytest.approx(75.0)
    assert metrics["stamp_older_than_1.0_sec_percent"] == pytest.approx(50.0)
    assert metrics["stamp_older_than_2.0_sec_percent"] == pytest.approx(25.0)


def test_stage_coverage_distinguishes_update_and_sample_sides() -> None:
    samples = [
        {
            "upstream_index": 0,
            "downstream_index": 2,
            "receipt_latency_sec": 0.01,
            "stamp_delta_sec": 0.02,
        },
        {
            "upstream_index": 1,
            "downstream_index": 4,
            "receipt_latency_sec": 0.01,
            "stamp_delta_sec": 0.02,
        },
    ]

    metrics = latency.stage_metrics(samples, upstream_count=2, downstream_count=10)

    assert metrics["upstream_coverage_percent"] == pytest.approx(100.0)
    assert metrics["downstream_coverage_percent"] == pytest.approx(20.0)


def test_missing_sensor_is_explicit_but_trajectory_age_still_works() -> None:
    final = _records([1.0, 2.0], [10.0, 11.0])
    control = _records([1.1, 1.5, 2.1, 2.5], [10.1, 10.5, 11.1, 11.5])
    records = {latency.FINAL_TOPIC: final, latency.CONTROL_TOPICS[0]: control}
    selected = {
        "sensor": None,
        "vad_output": None,
        "final_trajectory": latency.FINAL_TOPIC,
        "control": latency.CONTROL_TOPICS[0],
        "actuation": None,
        "processing_time": None,
    }

    report, samples = latency.analyze_records(records, selected)

    assert report["stages"]["sensor_to_vad_output"]["available"] is False
    age = report["stages"]["trajectory_age_at_control"]
    assert age["available"] is True
    assert age["pair_count"] == 4
    assert any(sample["stage"] == "trajectory_age_at_control" for sample in samples)


def test_topic_selection_prefers_lightweight_camera_info_and_raw_control() -> None:
    available = {
        latency.SENSOR_TOPIC_CANDIDATES[0],
        latency.SENSOR_TOPIC_CANDIDATES[2],
        latency.CONTROL_TOPICS[0],
        latency.CONTROL_TOPICS[2],
    }

    assert (
        latency.select_first_available(available, latency.SENSOR_TOPIC_CANDIDATES)
        == latency.SENSOR_TOPIC_CANDIDATES[0]
    )
    assert (
        latency.select_first_available(available, latency.CONTROL_TOPICS)
        == latency.CONTROL_TOPICS[0]
    )


def test_camera_bundle_allows_small_stamp_skew() -> None:
    records = {}
    for index, topic in enumerate(latency.CAMERA_INFO_TOPICS):
        records[topic] = _records([10.0 + index * 0.001], [5.0 + index * 0.005])

    metrics = latency.camera_bundle_metrics(records)

    assert metrics["available"] is True
    assert metrics["matched_bundle_count"] == 1
    assert metrics["stamp_span_sec"]["max"] == pytest.approx(0.025)
    assert metrics["receipt_span_sec"]["max"] == pytest.approx(0.005)


def test_vad_input_sync_uses_nearest_physics_stamp() -> None:
    sensor = _records([1.0, 2.0], [10.0, 11.0])
    odometry = _records([1.0, 2.0], [9.98, 11.03])
    acceleration = _records([1.0, 2.0], [10.01, 10.96])

    metrics = latency.vad_input_sync_metrics(sensor, odometry, acceleration)

    assert metrics["camera_to_odometry_abs_stamp_delta_sec"]["max"] == pytest.approx(0.03)
    assert metrics["camera_to_acceleration_abs_stamp_delta_sec"]["max"] == pytest.approx(0.04)
