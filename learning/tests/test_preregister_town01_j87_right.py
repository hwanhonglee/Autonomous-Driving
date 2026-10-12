"""HH_260928 - Test a deterministic, non-owned Town01 turn before any capture."""

import hashlib
import json

import carla
import pytest
from agents.navigation.global_route_planner import GlobalRoutePlanner

from scripts.e2e import preregister_town01_j87_right as screen


def test_exact_offline_route_preserves_all_gates():
    route, report = screen.reconstruct(carla, GlobalRoutePlanner)
    assert report["status"] == "STATIC_FEASIBLE_OWNER_REQUIRED"
    assert report["split"] == "TRAIN_CANDIDATE_TOWN01_ONLY"
    assert route["town"] == "Town01" and route["scenario"] == "right"
    assert route["physical_turn_preflight"]["status"] == "PASS"
    assert report["physical_turn_transitions"] == [[103, 0, -1], [5, 0, 1]]
    assert 30 <= report["turn_start_m"] <= report["turn_end_m"] <= 120
    assert report["route_length_m"] >= 120 + 16 + 10
    assert report["protected_future_has_extra_junction"] is False
    assert report["known_site_count_compared"] >= 18
    assert report["prior_search_route_sites_compared"] == 18
    assert report["nearest_previous_screened_turn_center_m"] < 10
    assert report["independent_junction_certified"] is False
    nominal = report["nominal"]
    assert nominal["status"] == "PASS"
    assert nominal["production_lane_gate_supported"] is True
    assert nominal["body_count"] == 901 and nominal["sweep_count"] == 900
    assert all(nominal[f"{area}_{shape}_outside_count"] == 0
               for area in ("raw", "inset") for shape in ("body", "sweep"))
    assert nominal["minimum_inset_sweep_clearance_m"] > 0.09
    assert route["offline_preregistration"]["capture_authorized"] is False
    assert report["capture_authorized"] is False
    assert report["dataset_admitted"] is False
    assert report["training_executed"] is False


def test_changed_xodr_is_rejected_before_planner(monkeypatch):
    monkeypatch.setattr(screen, "XODR_SHA256", "0" * 64)
    with pytest.raises(screen.PreflightError, match="XODR changed"):
        screen.reconstruct(carla, GlobalRoutePlanner)


def test_reject_duplicate_output_without_reconstruction(monkeypatch, tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    monkeypatch.setattr(screen, "reconstruct", lambda *_: pytest.fail("reconstructed an existing output"))
    with pytest.raises(screen.PreflightError, match="already exists"):
        screen.publish(output)


def test_published_route_is_explicitly_provisional():
    route_file = screen.OUTPUT / "provisional_route.json"
    report_file = screen.OUTPUT / "feasibility.json"
    route = json.loads(route_file.read_text())
    report = json.loads(report_file.read_text())
    assert hashlib.sha256(route_file.read_bytes()).hexdigest() == report["provisional_route_sha256"]
    assert route["offline_preregistration"]["status"] == "OFFLINE_PROVISIONAL_ONLY"
    assert report["status"] == "STATIC_FEASIBLE_OWNER_REQUIRED"
    assert not any(report[key] for key in ("capture_authorized", "dataset_admitted",
                                       "training_executed", "learned_closed_loop_tested"))


def test_fingerprint_rejects_non_right_turn():
    route = {"route": [{"road_option": "LEFT", "road_id": 1, "section_id": 0,
                         "lane_id": -1, "x": 0.0, "y": 0.0}]}
    preflight = {"selected_block": {"start_index": 0, "end_index": 0}}
    with pytest.raises(screen.PreflightError, match="not a right turn"):
        screen.turn_fingerprint(route, preflight)
