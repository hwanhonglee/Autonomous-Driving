"""HH_260928 - Keep the independent-site scout bounded, pinned, and non-authorizing."""

import json

import carla
import pytest
from agents.navigation.global_route_planner import GlobalRoutePlanner

from scripts.e2e import scout_independent_c_track_right as scout


def test_frozen_offline_search_matches_published_evidence():
    report = scout.scan(carla, GlobalRoutePlanner)
    assert report == json.loads(scout.OUTPUT.read_text())
    assert report["status"] == "NO_PHYSICAL_TURN_PASS"
    assert report["split"] == "TRAIN_CANDIDATE_C_TRACK_1_0_7"
    assert report["split_source_sha256"] == scout.SPLIT_SOURCE_SHA256
    assert report["split_source_ct_slot_count"] == 6
    assert report["connector_count"] == 65
    assert report["signal_controlled_connectors_excluded"] == 0
    assert [(row["upstream_m"], row["downstream_m"]) for row in report["scopes"]] == [
        (50, 90), (60, 80), (60, 100), (70, 80),
    ]
    assert [row["stages"]["endpoint_pairs"] for row in report["scopes"]] == [
        650, 692, 896, 777,
    ]
    assert report["total_endpoint_pairs"] == 3015
    assert report["total_physical_turn_pass"] == 0
    assert report["fixed_window_m"] == [30.0, 120.0]
    assert report["minimum_route_length_m"] == 146.0
    assert report["moving_contract"] == {
        "minimum_continuous_sec": 30.0, "speed_strictly_above_mps": 0.5,
    }
    assert report["goal_tolerance_m"] == 2.5
    assert report["best_rejected_witness"]["maximum_grade_ratio"] > 0.09983341664682815
    assert not any(report[key] for key in (
        "live_map_sha_verified", "nominal_body_corridor_verified",
        "continuous_moving_verified", "actual_goal_verified",
        "capture_authorized", "dataset_admitted", "training_executed",
    ))


def test_changed_source_is_rejected_before_map_construction(monkeypatch):
    monkeypatch.setattr(scout, "XODR_SHA256", "0" * 64)

    class NoMap:
        def Map(self, *_):
            pytest.fail("constructed an offline map from changed XODR")

    with pytest.raises(scout.ScoutError, match="XODR changed"):
        scout.scan(NoMap(), GlobalRoutePlanner)


def test_existing_output_is_rejected_before_search(monkeypatch, tmp_path):
    output = tmp_path / "existing.json"
    output.write_text("preserve me")
    monkeypatch.setattr(scout, "scan", lambda *_: pytest.fail("searched before overwrite guard"))
    with pytest.raises(scout.ScoutError, match="already exists"):
        scout.publish(output)
    assert output.read_text() == "preserve me"


def test_right_blocks_preserve_separate_maneuvers():
    points = [{"road_option": option} for option in
              ("LANEFOLLOW", "RIGHT", "RIGHT", "LANEFOLLOW", "RIGHT")]
    assert scout.right_blocks(points) == [(1, 2), (4, 4)]
