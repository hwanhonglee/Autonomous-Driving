"""HH_261001 - Offline qualification retains stops, failed windows, and missing evidence."""

import math

import pytest

from scripts.e2e import audit_town12_j15824_capture_20261001 as audit


def states(count, speed=1):
    return [{"frame": i, "timestamp": i * .05, "capture_phase": "driving", "vx": speed, "vy": 0}
            for i in range(count)]


def test_thirty_seconds_requires_elapsed_duration_not_sample_count():
    assert audit.continuous_motion(states(600))["status"] == "FAIL"
    assert audit.continuous_motion(states(601))["status"] == "PASS"


def test_stops_and_frame_gaps_are_not_stitched():
    rows = states(1000)
    rows[500]["vx"] = 0
    result = audit.continuous_motion(rows)
    assert result["status"] == "FAIL" and len(result["all_runs"]) == 2
    rows = states(1000)
    for row in rows[500:]:
        row["frame"] += 1
    result = audit.continuous_motion(rows)
    assert result["status"] == "FAIL" and len(result["all_runs"]) == 2


def test_threshold_is_strict_and_stationary_tail_not_moving_qualification():
    assert audit.continuous_motion(states(1000, .5))["status"] == "FAIL"
    rows = states(1000)
    for row in rows:
        row["capture_phase"] = "stationary_tail"
    assert audit.continuous_motion(rows)["longest_continuous_seconds"] == 0


def test_runtime_audit_retains_incomplete_windows():
    rows = [{"frame": 1, "capture_phase": "driving"}, {"frame": 2, "capture_phase": "stationary_tail"}]
    diagnostics = [{**row, "valid_points": 12, "disposition": "incomplete_future_retained"} for row in rows]
    summary, records = audit.runtime_windows([], rows, diagnostics)
    assert len(records) == 2 and summary["runtime_status_counts"] == {"INCOMPLETE_OR_CONTEXT_ONLY": 2}
    assert summary["strong_right_nine_metric_runtime64_candidate_count"] == 0


def test_all_six_exact_runtime_slots_and_failures_are_retained(monkeypatch):
    ego = {"frame": 1, "vx": 1., "vy": 0.}
    timeline = [(0, ego)]
    cameras = [{"frame": 1, "capture_phase": "driving"}]
    target = [{"frame": 1, "valid_points": 64, "diagnostic": {"steps": [
        {"metrics": {"speed_limit": [1., False], "xy_curvature": [1., True]}}]}}]
    monkeypatch.setattr(audit.raw, "future_state", lambda *a: ({"vx": 1., "vy": 0.}, "exact"))
    monkeypatch.setattr(audit.raw.adapter, "_relative_xy_yaw", lambda *a: ([1., 0.], -math.pi / 2))
    def runtime(xy, speeds, logits, *, current_speed_mps):
        assert len(xy) == len(speeds) == len(logits) == 6
        assert all(len(row) == 64 for row in xy) and current_speed_mps == 1
        raise audit.runtime_contract.ContractError("measured geometric failure")
    monkeypatch.setattr(audit.runtime_contract, "validate_and_select_trajectory", runtime)
    summary, rows = audit.runtime_windows(timeline, cameras, target)
    assert rows[0]["status"] == "FAIL" and rows[0]["strong_right_45deg"]
    assert rows[0]["runtime_failure"] == "measured geometric failure"
    assert rows[0]["nine_metric_violations"] == ["xy_curvature"]
    assert summary["strong_right_nine_metric_runtime64_candidate_count"] == 0


def test_unsupported_evidence_is_not_a_pass():
    def broken():
        raise ValueError("missing evidence")
    result = audit.checked_section(broken)
    assert result["status"] == "UNSUPPORTED_OR_INVALID"
    assert "missing evidence" in result["error"]


def test_existing_raw_output_cannot_be_overwritten(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    with pytest.raises(ValueError, match="fresh audit output"):
        audit.run(tmp_path, output, "0" * 64)
