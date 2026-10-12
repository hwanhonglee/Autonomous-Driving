"""HH_261001 - Pure fail-closed tests for the prospective native low-speed feasibility gate."""

from copy import deepcopy

from scripts.e2e import town07_low_speed_10_30_pilot_contract_20261001 as contract


def rows_for_feasible_pilot():
    rows = []
    for tick in range(contract.WARMUP_TICKS):
        rows.append(dict(phase="stationary_warmup", frame=tick + 1,
            timestamp=round((tick + 1) * .05, 8),
            route_progress_m=contract.WARMUP_REFERENCE_PROGRESS_M,
            speed_mps=0., route_cte_m=0., x=0., y=0.,
            control_alignment={"status": "PASS"}, collision_count=0,
            lane_invasion_count=0, traffic_light_hazard={"hazard": False},
            observed_free_disk_bytes=contract.FLOOR_BYTES + contract.CLOSING_RESERVE_BYTES))
    speed, progress = 0., contract.WARMUP_REFERENCE_PROGRESS_M
    for tick in range(1, 900):
        speed = min(1.1, speed + .1)
        progress += speed * .05
        rows.append(dict(phase="driving", frame=len(rows) + 1,
            timestamp=round((len(rows) + 1) * .05, 8),
            route_progress_m=progress, speed_mps=speed, route_cte_m=0.,
            x=progress, y=0., control_alignment={"status": "PASS"},
            collision_count=0, lane_invasion_count=0,
            traffic_light_hazard={"hazard": False},
            observed_free_disk_bytes=contract.FLOOR_BYTES + contract.CLOSING_RESERVE_BYTES))
        if progress > contract.WINDOW_HI_M:
            return rows
    raise AssertionError("synthetic feasible episode did not close")


def test_full_fixed_native_window_is_feasible_only():
    result = contract.assess_native_window(rows_for_feasible_pilot())
    assert result["status"] == "PASS_FEASIBILITY_ONLY"
    assert result["selected_span_seconds"] >= 12.
    assert result["observed_warmup_state_count"] == 70
    assert not result["training_data_approved"]
    assert not result["whole_body_lane_clearance_proven"]


def test_stationary_or_overspeed_selected_tick_blocks_whole_pilot():
    original = rows_for_feasible_pilot()
    selected = next(i for i, row in enumerate(original) if 16. <= row["route_progress_m"] <= 25.)
    for speed in (0., 1.6):
        rows = deepcopy(original)
        rows[selected]["speed_mps"] = speed
        result = contract.assess_native_window(rows)
        assert result["status"] == "FAIL"
        assert any(item.startswith("speed_window_") for item in result["failures"])


def test_no_posthoc_window_trimming_or_reentry():
    original = rows_for_feasible_pilot()
    rows = deepcopy(original)
    rows.append(dict(rows[-1], frame=rows[-1]["frame"] + 1,
        timestamp=round(rows[-1]["timestamp"] + .05, 8), route_progress_m=29.99))
    result = contract.assess_native_window(rows)
    assert result["status"] == "FAIL"
    assert any(item.startswith("postclosure_reentry_") for item in result["failures"])


def test_missing_warmup_or_ack_or_native_frame_blocks():
    original = rows_for_feasible_pilot()
    for mutation in ("warmup", "ack", "frame"):
        rows = deepcopy(original)
        if mutation == "warmup":
            rows.pop(10)
            for i, row in enumerate(rows):
                row["frame"] = i + 1
                row["timestamp"] = round((i + 1) * .05, 8)
        elif mutation == "ack":
            rows[200]["control_alignment"] = {"status": "FAIL"}
        else:
            rows[200]["frame"] += 1
        result = contract.assess_native_window(rows)
        assert result["status"] == "FAIL"


def test_prewindow_moderate_rate_is_diagnostic_not_hidden_and_emergency_rate_blocks():
    original = rows_for_feasible_pilot()
    rows = deepcopy(original)
    rows[76]["speed_mps"] += .16  # HH_261001 - A 3.2 m/s² launch excursion is outside the selected window.
    result = contract.assess_native_window(rows)
    assert result["status"] == "PASS_FEASIBILITY_ONLY"
    assert 76 in result["prewindow_or_postclosure_2p9_exceedance_indices"]
    rows[76]["speed_mps"] += .5
    result = contract.assess_native_window(rows)
    assert result["status"] == "FAIL"
    assert any(item.startswith("global_emergency_scalar_rate_") for item in result["failures"])


def test_source_contract_remains_native_only_and_prospective():
    fixed = contract.fixed_contract()
    assert fixed["camera_count"] == 0
    assert fixed["physics_hz"] == 20
    assert fixed["stationary_warmup_ticks"] == 70
    assert fixed["selected_route_progress_m"] == [10., 30.]
    assert fixed["full_six_camera_capture_authorized_by_this_plan"] is False
    assert fixed["training_data_approved"] is False


def test_warmup_midpoint_drift_cannot_be_hidden_by_returning_to_start():
    rows = rows_for_feasible_pilot()
    rows[35]["x"] = .06
    result = contract.assess_native_window(rows)
    assert result["status"] == "FAIL"
    assert "warmup_vehicle_drift_35" in result["failures"]


def test_first_closing_state_must_be_the_last_native_state():
    rows = rows_for_feasible_pilot()
    rows.append(dict(rows[-1], frame=rows[-1]["frame"] + 1,
        timestamp=round(rows[-1]["timestamp"] + .05, 8),
        route_progress_m=rows[-1]["route_progress_m"] + .05))
    result = contract.assess_native_window(rows)
    assert result["status"] == "FAIL"
    assert any(item.startswith("postclosure_extra_state_") for item in result["failures"])


def test_malformed_native_evidence_returns_fail_instead_of_raising():
    original = rows_for_feasible_pilot()
    mutations = (("timestamp", "not-a-number"),
                 ("observed_free_disk_bytes", "unknown"),
                 ("x", None), ("y", float("nan")))
    for key, value in mutations:
        rows = deepcopy(original)
        rows[35][key] = value
        result = contract.assess_native_window(rows)
        assert result["status"] == "FAIL"
        assert any(item.startswith("malformed_native_state_35:") for item in result["failures"])
