from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
ROUTE_DIR = ROOT / "autoware_e2e_vad_launch/config/routes"
COMMAND_BY_OPTION = {
    "LEFT": 0,
    "RIGHT": 1,
    "STRAIGHT": 2,
    "LANEFOLLOW": 3,
    "CHANGELANELEFT": 4,
    "CHANGELANERIGHT": 5,
}


def _load(name: str) -> dict:
    return json.loads((ROUTE_DIR / name).read_text(encoding="utf-8"))


def _assert_common_contract(route: dict) -> Counter[str]:
    assert route["town"] == "C_track_1_0_7"
    assert route["coordinate_reference"] == "base_link"
    assert route["spawn_point_reference"] == "base_link"
    points = route["route"]
    assert len(points) >= 2
    assert [point["distance_m"] for point in points] == sorted(
        point["distance_m"] for point in points
    )
    assert points[-1]["distance_m"] == pytest.approx(route["route_length_m"])
    assert points[-1]["remaining_m"] == pytest.approx(0.0)
    for point in points:
        assert point["vad_command"] == COMMAND_BY_OPTION[point["road_option"]]

    spawn = [float(value) for value in route["spawn_point"].split(",")]
    transform = route["start_carla_transform"]
    expected_spawn = [
        transform[key] for key in ("x", "y", "z", "roll", "pitch", "yaw")
    ]
    assert spawn == pytest.approx(expected_spawn, abs=1.0e-6)
    assert math.hypot(
        points[0]["x"] - route["start_ros_pose"]["x"],
        points[0]["y"] - route["start_ros_pose"]["y"],
    ) < 1.0e-5
    assert math.hypot(
        points[-1]["x"] - route["goal_ros_pose"]["x"],
        points[-1]["y"] - route["goal_ros_pose"]["y"],
    ) < 1.0e-5
    counts = Counter(point["road_option"] for point in points)
    assert dict(counts) == route["option_counts"]
    return counts


def test_c_track_straight_lane_follow_route_is_separate_and_unambiguous() -> None:
    route = _load("c_track_straight_394_290.json")
    counts = _assert_common_contract(route)

    assert route["scenario"] == "lane_follow"
    assert (route["start_spawn_index"], route["goal_spawn_index"]) == (394, 290)
    assert route["route_length_m"] == pytest.approx(59.7267587184906)
    assert counts == {"LANEFOLLOW": 63}


def test_c_track_left_route_contains_left_and_no_right_command() -> None:
    route = _load("c_track_left_369_425.json")
    counts = _assert_common_contract(route)

    assert route["scenario"] == "left"
    assert (route["start_spawn_index"], route["goal_spawn_index"]) == (369, 425)
    assert route["route_length_m"] == pytest.approx(70.87968999147415)
    assert counts == {"LANEFOLLOW": 25, "STRAIGHT": 36, "LEFT": 16}
    assert "RIGHT" not in counts
