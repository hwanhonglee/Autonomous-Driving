"""HH_260906 - Preserve V5 source ownership, site deduplication and no-capture ranking."""

from __future__ import annotations

import json

import pytest

from scripts.e2e import rank_town01_v5_long_turns as ranker
from scripts.e2e import audit_future_actor_lane_containment as lane


def test_completed_v5_owner_is_bound_to_sealed_catalog():
    plan, result, catalog = ranker.completed_v5_owner(ranker.OWNER)
    assert plan["split_intent"] == "TRAIN_CANDIDATE"
    assert result["exit_code"] == 0
    assert catalog["status"] == "complete"
    assert result["right_candidate_count"] == 16


def test_missing_owner_result_fails_before_catalog_read(tmp_path):
    plan = json.loads((ranker.OWNER / "owner_plan.json").read_text())
    (tmp_path / "owner_plan.json").write_text(json.dumps(plan))
    with pytest.raises(ranker.InvalidEvidence, match="owner_result"):
        ranker.completed_v5_owner(tmp_path)


def test_known_site_inventory_includes_v4_train_and_c_capture():
    sites, manifest, static = ranker.known_sites()
    assert len(sites) == 18
    assert sites[0]["source"] == "historical_TRAIN"
    assert sites[-1]["source"] == "C_p06_expert_capture"
    assert static["status"] == "STATIC_REVIEW_ONLY"
    assert manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] == "vehicle.toyota.prius"


def test_grouped_zero_arc_seam_keeps_both_lane_identities():
    def point(arc, x, road):
        return {"distance_m": arc, "x": x, "y": 0.0, "z": 0.0, "yaw": 0.0,
                "road_id": road, "section_id": 0, "lane_id": -1}

    points = [point(0.0, 0.0, 150), point(5.0, 5.0, 150),
              point(5.0, 5.0, 169), point(10.0, 10.0, 169)]
    geometry, seams = ranker.grouped_seam_geometry(points)
    assert len(geometry) == 3
    assert seams == [{"arc_m": 5.0, "from_lane": [150, 0, -1],
                      "to_lane": [169, 0, -1], "pose_identical": True,
                      "production_lane_gate_supported": False}]
    assert lane._route_lanes_for_arc_interval(points, 4.9, 5.1) == {
        (150, 0, -1), (169, 0, -1)}
    points[2]["x"] = 5.01
    with pytest.raises(ranker.InvalidEvidence, match="zero-arc seam changes pose"):
        ranker.grouped_seam_geometry(points)


def test_v5_rank_fails_closed_on_inset_sweep_and_downstream_seam(tmp_path):
    result = ranker.audit(ranker.OWNER, tmp_path / "rank.json")
    assert result["right_candidate_count"] == 16
    assert result["physical_site_count"] == 7
    assert result["capture_authorized"] is False
    assert result["dataset_admitted"] is False
    assert result["training_executed"] is False
    assert result["status"] == "NO_NOMINAL_SAFE_NOVEL_CANDIDATE"
    rows = {row["route_id"]: row for row in result["routes"]}
    for route_id in ("town01_right_s0004_p05", "town01_right_s0005_p02"):
        row = rows[route_id]
        assert row["known_site_matches"] == []
        assert row["nominal"]["status"] == "FAIL"
        assert row["nominal"]["production_lane_gate_supported"] is False
        assert row["nominal"]["raw_body_outside_count"] == 0
        assert row["nominal"]["inset_body_outside_count"] == 0
        assert row["nominal"]["raw_sweep_outside_count"] == 0
        assert row["nominal"]["inset_sweep_outside_count"] == 38
        assert "nominal_body_or_sweep_outside_10cm_inner_lane" in row["exclusion_reasons"]
        assert "production_lane_gate_rejects_zero_arc_lane_transition" in row["exclusion_reasons"]
        assert row["capture_authorized"] is False
    assert json.loads((tmp_path / "rank.json").read_text()) == result
