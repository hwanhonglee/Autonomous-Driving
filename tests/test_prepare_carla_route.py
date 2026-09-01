from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts/e2e/prepare_carla_route.py"
SPEC = importlib.util.spec_from_file_location("prepare_carla_route", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
prepare_route = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare_route)


class Location:
    def __init__(self, x: float, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other) -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


class Transform:
    def __init__(
        self,
        x: float,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0,
    ):
        self.location = Location(x, y, z)
        self.rotation = SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw)


def route_point(
    x: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
    yaw: float = 0.0,
    option=None,
):
    waypoint = SimpleNamespace(
        transform=Transform(x, y, z, yaw),
        road_id=1,
        section_id=0,
        lane_id=-1,
        is_junction=False,
    )
    return waypoint, option or prepare_route.RoadOption.LANEFOLLOW


def test_town02_flat_route_ignores_spawn_clearance_z_at_exact_goal() -> None:
    route = [route_point(0.0), route_point(9.0)]
    goal = Transform(10.0, z=0.5, yaw=12.0)

    points = prepare_route.serialize_route(route, goal)

    assert points[-1]["x"] == pytest.approx(10.0)
    assert points[-1]["y"] == pytest.approx(0.0)
    assert points[-1]["z"] == pytest.approx(0.0)
    assert points[-1]["yaw"] == pytest.approx(-math.radians(12.0))
    assert points[-1]["distance_m"] == pytest.approx(10.0)
    assert points[-1]["distance_m"] - points[-2]["distance_m"] == pytest.approx(
        math.sqrt(
            (points[-1]["x"] - points[-2]["x"]) ** 2
            + (points[-1]["y"] - points[-2]["y"]) ** 2
            + (points[-1]["z"] - points[-2]["z"]) ** 2
        )
    )

    goal_carla, goal_ros, provenance = prepare_route.normalized_goal_metadata(
        goal,
        route,
        points,
        endpoint_source="spawn_points",
        endpoint_index=17,
    )
    assert goal_carla == pytest.approx(
        {"x": 10.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 12.0}
    )
    assert goal_ros == pytest.approx(
        {"x": 10.0, "y": 0.0, "z": 0.0, "yaw": -math.radians(12.0)}
    )
    assert provenance["endpoint_index"] == 17
    assert provenance["original_goal_carla_transform"]["z"] == pytest.approx(0.5)
    assert provenance["original_goal_ros_pose"]["z"] == pytest.approx(0.5)


def test_town04_same_xy_different_z_keeps_exact_xy_yaw_without_z_jump() -> None:
    road_z = 0.0043487548828125
    route = [route_point(0.0, z=road_z), route_point(20.0, z=road_z)]
    goal = Transform(20.0, z=0.2819424271583557, yaw=-91.25)

    points = prepare_route.serialize_route(route, goal)

    assert len(points) == 2
    assert points[-1]["x"] == pytest.approx(20.0)
    assert points[-1]["y"] == pytest.approx(0.0)
    assert points[-1]["z"] == pytest.approx(road_z)
    assert points[-1]["yaw"] == pytest.approx(math.radians(91.25))
    assert points[-1]["distance_m"] == pytest.approx(20.0)
    assert points[-1]["remaining_m"] == pytest.approx(0.0)


def test_route_distance_and_terminal_gap_share_one_3d_definition() -> None:
    route = [route_point(0.0), route_point(3.0, z=4.0)]
    goal = Transform(6.0, z=4.5)

    points = prepare_route.serialize_route(route, goal)

    assert points[1]["distance_m"] == pytest.approx(5.0)
    assert points[-1]["z"] == pytest.approx(4.0)
    assert points[-1]["distance_m"] == pytest.approx(8.0)
    assert points[-1]["remaining_m"] == pytest.approx(0.0)
