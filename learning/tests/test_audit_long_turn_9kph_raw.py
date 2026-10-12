"""Pure CPU checks for the fixed Town01 turn selection and fail-closed gates."""

import copy
import hashlib
import json
import math

import pytest

from scripts.e2e import audit_long_turn_9kph_raw as qa
from scripts.e2e import audit_portable_target_feasibility as targets


def route():
    return {"route": [
        {"x": 0., "y": 0., "distance_m": 0., "road_option": "LANEFOLLOW"},
        {"x": 40., "y": 0., "distance_m": 40., "road_option": "RIGHT"},
        {"x": 56., "y": 0., "distance_m": 56., "road_option": "RIGHT"},
        {"x": 200., "y": 0., "distance_m": 200., "road_option": "LANEFOLLOW"}],
        "physical_turn_preflight": {"selected_block": {"start_index": 1, "end_index": 2}}}


def series(duration_s=55., speed=2.5):
    states, cameras = [], []
    for i in range(round(duration_s / .05) + 1):
        t = i * .05
        x = speed * t
        yaw = -math.pi / 2 * min(1., max(0., (x - 40) / 16))
        state = {"frame": 1000 + i, "timestamp": t, "capture_phase": "driving", "x": x, "y": 0.,
                 "vx": speed, "vy": 0., "yaw": yaw, "route_progress_m": x, "command": 1 if 40 <= x <= 56 else 3}
        states.append(state)
        if i % 2 == 0:
            cameras.append({"frame": state["frame"], "timestamp": t})
    return states, cameras


def diagnostic(frame, *, violation=None):
    steps = []
    for index in range(64):
        entries = {name: [0., False] for name in targets.METRICS}
        if index == 0 and violation:
            entries[violation] = [99., True]
        steps.append({"index": index, "target_timestamp_ns": 100_000_000 * (index + 1),
                      "episode_relative_100ms_tick": index + 1, "phase": "pre_tail",
                      "future_region": "interior", "metrics": entries,
                      "speed_xy_absolute_error_mps": 0., "absolute_yaw_rate_radps": 0.})
    return {"frame": frame, "anchor_timestamp_ns": 0, "valid_points": 64,
            "diagnostic": {"sample_id": f"raw-frame:{frame}", "episode_id": "single_raw_trial",
                           "valid_points": 64, "steps": steps}}


def turn_centered_fixture():
    """HH_260906 - Pin a synthetic multi-maneuver route and its static window report."""
    points = [
        {"x": arc, "y": 0., "distance_m": arc, "road_option": option}
        for arc, option in ((0., "LANEFOLLOW"), (60., "LANEFOLLOW"), (70., "RIGHT"),
                            (80., "RIGHT"), (90., "LANEFOLLOW"), (220., "STRAIGHT"),
                            (230., "LANEFOLLOW"), (270., "LANEFOLLOW"))]
    chosen_route = {"scenario": "right", "coordinate_reference": "base_link",
                    "route": points, "route_length_m": 270.}
    route_bytes = json.dumps(chosen_route, separators=(",", ":")).encode()
    route_pin = hashlib.sha256(route_bytes).hexdigest()
    report = {"schema": "portable_e2e.turn_centered_subwindow.v1",
              "status": "STATIC_WINDOW_CANDIDATE", "candidate_count": 1,
              "route_sha256": route_pin,
              "window_policy": {"approach_before_right_command_m": 30.,
                                "moving_anchor_window_m": 90.,
                                "post_right_command_inside_window_m": 30.,
                                "route_lead_before_window_m": 30.,
                                "9kph_6p4s_future_distance_m": 16.,
                                "future_clearance_guard_m": 10.,
                                "previous_maneuver_guard_m": 15.},
              "selected": {"right_block_start_index": 2, "right_block_end_index": 3,
                           "right_block_start_m": 65., "right_block_end_m": 85.,
                           "window_start_m": 35., "window_end_m": 125.,
                           "route_length_m": 270.}}
    report_bytes = json.dumps(report, separators=(",", ":")).encode()
    pins = {"route_source_bytes": route_bytes, "expected_route_sha256": route_pin,
            "window_report_bytes": report_bytes,
            "expected_window_report_sha256": hashlib.sha256(report_bytes).hexdigest()}
    return chosen_route, report, pins


def test_projection_uses_serialized_arc_and_raw_xy_not_saved_progress():
    points = [{"x": 0., "y": 0., "distance_m": 0.},
              {"x": 10., "y": 0., "distance_m": 12.}]
    progress, error, segment = qa.project(points, {"x": 5., "y": 1., "route_progress_m": 99.})
    assert progress == 6. and error == 1. and segment == 0


def test_every_30_to_120_camera_anchor_is_selected_without_input_mutation():
    states, cameras = series()
    original = copy.deepcopy((states, cameras))
    window, selected, projections = qa.select_window(route(), states, cameras)
    assert len(projections) == len(states)
    assert selected[0]["frame"] == 1240 and selected[-1]["frame"] == 1960
    assert window["selected_anchor_count"] == 361 and window["selected_span_ns"] == 36_000_000_000
    assert window["first_progress_over_120_m"]["frame"] == 1961
    assert window["turn_command_1_state_count"] > 0
    assert all(window["flags"].values())
    assert (states, cameras) == original


def test_first_native_over_120_latches_selection_closed_even_after_backtrack():
    states, cameras = series()
    for row in states:
        if row["frame"] > 1961:
            row["x"] = 100.
    window, selected, _ = qa.select_window(route(), states, cameras)
    assert selected[-1]["frame"] == 1960
    assert window["first_progress_over_120_m"]["frame"] == 1961
    assert window["full_driving_progress_drop_count"] > 0


def test_one_slow_native_tick_and_progress_drop_fail_without_removing_anchor():
    states, cameras = series()
    states[500]["vx"] = .5
    states[600]["x"] -= .2
    window, selected, _ = qa.select_window(route(), states, cameras)
    assert len(selected) == 361
    assert not window["flags"]["selected_native_speed_all_over_0p5_mps"]
    assert not window["flags"]["selected_native_progress_monotone"]
    assert window["selected_native_progress_drops"]


def test_one_camera_gap_fails_continuity_without_shorter_slice():
    states, cameras = series()
    cameras = [row for row in cameras if row["frame"] != 1500]
    window, selected, _ = qa.select_window(route(), states, cameras)
    assert len(selected) == 360
    assert not window["flags"]["selected_camera_10hz_continuous"]


def test_pinned_turn_centered_window_selects_all_anchors_and_latches_closure():
    chosen_route, _, pins = turn_centered_fixture()
    states, cameras = series(duration_s=55.)
    for row in states:
        row["yaw"] = -math.pi / 2 * min(1., max(0., (row["x"] - 70.) / 10.))
        row["command"] = 1 if 70. <= row["x"] <= 80. else 3
    window, selected, projections = qa.select_window(chosen_route, states, cameras, **pins)
    assert len(projections) == len(states)
    assert selected[0]["frame"] == 1280 and selected[-1]["frame"] == 2000
    assert window["selected_anchor_count"] == 361
    assert window["first_progress_over_window_end_m"]["frame"] == 2001
    assert window["pinned_window"]["route_sha256"] == pins["expected_route_sha256"]
    assert all(window["flags"].values())


def test_turn_centered_window_rejects_unpinned_tampered_and_mismatched_inputs():
    chosen_route, _, pins = turn_centered_fixture()
    states, cameras = series()
    for key in pins:
        partial = {**pins, key: None}
        with pytest.raises(qa.AuditError):
            qa.select_window(chosen_route, states, cameras, **partial)
    with pytest.raises(qa.AuditError, match="SHA256 pin"):
        qa.select_window(chosen_route, states, cameras,
                         **{**pins, "route_source_bytes": pins["route_source_bytes"] + b" "})
    with pytest.raises(qa.AuditError, match="SHA256 pin"):
        qa.select_window(chosen_route, states, cameras,
                         **{**pins, "window_report_bytes": pins["window_report_bytes"] + b" "})
    other_route = copy.deepcopy(chosen_route)
    other_route["route"][2]["x"] += .1
    with pytest.raises(qa.AuditError, match="in-memory route"):
        qa.select_window(other_route, states, cameras, **pins)


@pytest.mark.parametrize("change", [
    lambda report: report["selected"].update(window_end_m=124.),
    lambda report: report["selected"].update(right_block_start_index=3),
    lambda report: report["selected"].update(window_start_m=20., window_end_m=110.),
    lambda report: report["selected"].update(route_length_m=140.),
    lambda report: report.update(status="NO_ELIGIBLE_WINDOW"),
])
def test_turn_centered_window_rejects_invalid_pinned_geometry(change):
    chosen_route, report, pins = turn_centered_fixture()
    change(report)
    report_bytes = json.dumps(report, separators=(",", ":")).encode()
    pins.update(window_report_bytes=report_bytes,
                expected_window_report_sha256=hashlib.sha256(report_bytes).hexdigest())
    states, cameras = series()
    with pytest.raises(qa.AuditError):
        qa.select_window(chosen_route, states, cameras, **pins)


def test_turn_centered_window_independently_rejects_other_maneuver_in_protected_future():
    chosen_route, report, pins = turn_centered_fixture()
    chosen_route["route"][5]["distance_m"] = 200.
    chosen_route["route"][5]["x"] = 200.
    route_bytes = json.dumps(chosen_route, separators=(",", ":")).encode()
    report["route_sha256"] = hashlib.sha256(route_bytes).hexdigest()
    report_bytes = json.dumps(report, separators=(",", ":")).encode()
    pins.update(route_source_bytes=route_bytes, expected_route_sha256=report["route_sha256"],
                window_report_bytes=report_bytes,
                expected_window_report_sha256=hashlib.sha256(report_bytes).hexdigest())
    states, cameras = series()
    with pytest.raises(qa.AuditError, match="another maneuver"):
        qa.select_window(chosen_route, states, cameras, **pins)


def test_all_nine_future_bounds_and_pre_tail_horizon_are_required():
    selected = [{"frame": 1000}]
    clean = diagnostic(1000)
    result = qa.summarize_selected_futures([clean], selected, 7_000_000_000)
    assert all(result["flags"].values())
    bad = diagnostic(1000, violation="xy_curvature")
    result = qa.summarize_selected_futures([bad], selected, 7_000_000_000)
    assert result["original_nine_violating_anchor_counts"]["xy_curvature"] == 1
    assert not result["flags"]["all_original_nine_motion_bounds_clear"]
    result = qa.summarize_selected_futures([clean], selected, 6_400_000_000)
    assert not result["flags"]["all_selected_64_points_before_brake_tail"]
    incomplete = {"frame": 1000, "anchor_timestamp_ns": 0, "valid_points": 63}
    result = qa.summarize_selected_futures([incomplete], selected, 7_000_000_000)
    assert not result["flags"]["all_selected_have_64_actual_future_points"]
