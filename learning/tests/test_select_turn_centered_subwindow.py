"""HH_260906 - Keep multi-maneuver windows fixed before nominal or driven outcomes."""

from __future__ import annotations

import copy
import sys

import pytest

from scripts.e2e import select_turn_centered_subwindow as window


def _real_p06() -> dict:
    # HH_260906 - Reuse a real serialized Town01 turn geometry, not a drawn toy corner.
    path = window.source.ROOT / (
        "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/"
        "routes/town01/right/town01_right_s0002_p06.json"
    )
    route, _ = window.source.read_json(
        path, "49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142")
    return route


def _multi_maneuver_fixture() -> dict:
    # HH_260906 - Add one upstream command in memory; the selected p06 corner remains byte-derived.
    route = copy.deepcopy(_real_p06())
    first, second = route["route"][:2]
    delta = [second[key] - first[key] for key in ("x", "y", "z")]
    length = sum(value * value for value in delta) ** 0.5
    direction = [value / length for value in delta]
    added = []
    for index in range(60):
        point = copy.deepcopy(first)
        for key, unit in zip(("x", "y", "z"), direction):
            point[key] = first[key] - (60 - index) * unit
        point["distance_m"] = float(index)
        point["road_option"] = "STRAIGHT" if 10 <= index <= 12 else "LANEFOLLOW"
        point["index"] = index
        added.append(point)
    for point in route["route"]:
        point["distance_m"] += 60.0
        point["index"] += 60
    route["route"] = added + route["route"]
    route["route_length_m"] += 60.0
    route["option_counts"] = {"STRAIGHT": 3, "RIGHT": route["option_counts"]["RIGHT"],
                              "LANEFOLLOW": route["option_counts"]["LANEFOLLOW"] + 57}
    return route


def test_real_catalog_route_is_not_multi_maneuver() -> None:
    report = window.select_window(_real_p06())
    assert report["status"] == "NO_ELIGIBLE_WINDOW"
    assert report["candidate_count"] == 0
    assert report["capture_authorized"] is False


def test_fixed_window_uses_real_turn_geometry_and_preserves_future() -> None:
    route = _multi_maneuver_fixture()
    report = window.select_window(route)
    assert report["status"] == "STATIC_WINDOW_CANDIDATE", report["rejections"]
    chosen = report["selected"]
    assert chosen["window_start_m"] == pytest.approx(chosen["right_block_start_m"] - 30.0)
    assert chosen["window_end_m"] - chosen["window_start_m"] == pytest.approx(90.0)
    assert chosen["window_end_m"] - chosen["right_block_end_m"] >= 30.0
    assert chosen["future_end_m_at_9kph"] == pytest.approx(chosen["window_end_m"] + 16.0)
    assert chosen["protected_maneuver_free_end_m"] <= route["route_length_m"]
    assert chosen["physical_turn_preflight"]["status"] == "PASS"
    assert report["capture_authorized"] is False
    assert report["dataset_admitted"] is False


def test_second_maneuver_inside_any_selected_future_rejects_route() -> None:
    route = _multi_maneuver_fixture()
    baseline = window.select_window(route)["selected"]
    target = baseline["window_end_m"] + 8.0
    point = min(route["route"], key=lambda row: abs(row["distance_m"] - target))
    point["road_option"] = "STRAIGHT"
    report = window.select_window(route)
    assert report["candidate_count"] == 0
    assert "another directional maneuver" in str(report["rejections"])


def test_short_full_future_fails_without_shifting_window() -> None:
    route = _multi_maneuver_fixture()
    baseline = window.select_window(route)["selected"]
    route["route"] = [point for point in route["route"]
                      if point["distance_m"] <= baseline["window_end_m"] + 10.0]
    route["route_length_m"] = route["route"][-1]["distance_m"]
    report = window.select_window(route)
    assert report["candidate_count"] == 0
    assert "full 6.4 s static future" in str(report["rejections"])


def test_zero_arc_lane_transition_is_not_silently_collapsed() -> None:
    route = _multi_maneuver_fixture()
    repeated = copy.deepcopy(route["route"][70])
    repeated["road_id"] += 1
    route["route"].insert(71, repeated)
    with pytest.raises(window.WindowError, match="zero-arc lane-ID"):
        window.select_window(route)


def test_cli_no_candidate_returns_nonzero(monkeypatch, capsys) -> None:
    path = window.source.ROOT / (
        "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/"
        "routes/town01/right/town01_right_s0002_p06.json"
    )
    monkeypatch.setattr(sys, "argv", ["selector", "--route", str(path), "--expected-sha256",
                                    "49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142"])
    assert window.main() == 2
    assert '"candidate_count": 0' in capsys.readouterr().out
