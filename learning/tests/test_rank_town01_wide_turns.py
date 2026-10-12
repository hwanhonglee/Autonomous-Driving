"""HH_260906 - Keep the wide-turn screen conservative and non-admitting."""

from __future__ import annotations

import json
import math

import pytest

from scripts.e2e import rank_town01_wide_turns as ranker


def point(arc: float, x: float, y: float, yaw: float = 0.0) -> dict:
    return {"distance_m": arc, "x": x, "y": y, "z": 0.0, "yaw": yaw,
            "road_id": 12, "section_id": 0, "lane_id": -1}


def test_actor_center_is_wheelbase_half_ahead_of_ros_rear_axle():
    pose, index = ranker.pose_at_arc([point(0.0, 0.0, 0.0), point(10.0, 10.0, 0.0)], 5.0, 0)
    assert index == 0
    assert pose["x"] == pytest.approx(6.425)
    assert pose["y"] == pytest.approx(0.0)
    assert pose["yaw"] == pytest.approx(0.0)


def test_yaw_interpolation_uses_short_wrapped_turn():
    points = [point(0.0, 0.0, 0.0, math.radians(179)),
              point(1.0, -1.0, 0.0, math.radians(-179))]
    pose, _ = ranker.pose_at_arc(points, 0.5, 0)
    assert abs(abs(pose["yaw"]) - 180.0) < 1e-6


def test_duplicate_command_node_cannot_change_lane_or_geometry():
    same = [point(0.0, 0.0, 0.0), point(0.0, 0.0, 0.0), point(1.0, 1.0, 0.0)]
    assert len(ranker.distinct_points(same)) == 2
    changed = [point(0.0, 0.0, 0.0), point(0.0, 0.0, 0.0), point(1.0, 1.0, 0.0)]
    changed[1]["lane_id"] = 1
    with pytest.raises(ValueError, match="zero-length node"):
        ranker.distinct_points(changed)


def test_owned_archive_has_no_offline_approved_novel_capture(tmp_path):
    result = ranker.rank(tmp_path / "rank.json")
    assert result["status"] == "NO_NOMINAL_SAFE_NOVEL_CANDIDATE"
    assert result["candidate_count"] == 16
    assert result["distinct_offline_ranked_site_count"] == 0
    assert result["capture_executed"] is False
    assert result["dataset_admitted"] is False
    assert result["training_executed"] is False
    rows = {row["route_id"]: row for row in result["candidates"]}
    assert rows["town01_right_s0003_p00"]["nominal"]["status"] == "FAIL"
    assert rows["town01_right_s0003_p07"]["nominal"]["status"] == "FAIL"
    assert rows["town01_right_s0003_p00"]["nominal"]["raw_body_outside_count"] == 0
    assert rows["town01_right_s0003_p00"]["nominal"]["inset_sweep_outside_count"] > 0
    assert json.loads((tmp_path / "rank.json").read_text()) == result
