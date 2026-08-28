import math
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "e2e"))

from agents.navigation.local_planner import RoadOption
from prepare_carla_route import serialize_route


class Location:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other):
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


class Transform:
    def __init__(self, x, y, z=0.0, yaw=0.0):
        self.location = Location(x, y, z)
        self.rotation = type(
            "Rotation", (), {"roll": 0.0, "pitch": 0.0, "yaw": yaw}
        )()


class Waypoint:
    def __init__(self, transform):
        self.transform = transform
        self.road_id = 1
        self.section_id = 0
        self.lane_id = -1
        self.is_junction = False


def test_serialize_route_appends_exact_goal_and_recalculates_distances():
    route = [
        (Waypoint(Transform(0.0, 0.0)), RoadOption.LANEFOLLOW),
        (Waypoint(Transform(10.0, 0.0)), RoadOption.LANEFOLLOW),
    ]

    points = serialize_route(route, Transform(11.0, -2.0, z=0.3, yaw=90.0))

    assert len(points) == 3
    assert points[-1]["x"] == 11.0
    assert points[-1]["y"] == 2.0
    assert points[-1]["z"] == 0.3
    assert points[-1]["yaw"] == -math.pi / 2.0
    assert points[-1]["distance_m"] == 10.0 + math.sqrt(5.0)
    assert points[-1]["remaining_m"] == 0.0
    assert points[-2]["remaining_m"] == math.sqrt(5.0)
    assert points[-1]["vad_command"] == points[-2]["vad_command"]


def test_serialize_route_normalizes_matching_terminal_without_duplicate():
    route = [
        (Waypoint(Transform(0.0, 0.0)), RoadOption.LANEFOLLOW),
        (Waypoint(Transform(10.0, 0.0)), RoadOption.LANEFOLLOW),
    ]

    points = serialize_route(route, Transform(10.0, 0.0, z=0.3, yaw=45.0))

    assert len(points) == 2
    assert points[-1]["z"] == 0.3
    assert points[-1]["yaw"] == -math.pi / 4.0
    assert points[-1]["distance_m"] == 10.0
    assert points[-1]["remaining_m"] == 0.0
