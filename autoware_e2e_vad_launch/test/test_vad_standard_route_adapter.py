import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from autoware_planning_msgs.msg import RouteState

from vad_standard_route_adapter import (
    load_route_pose,
    route_alignment_metrics,
    route_state_supports_alignment,
)


def pose(x, y, yaw):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw * 0.5),
            w=math.cos(yaw * 0.5),
        ),
    )


def test_load_route_pose_uses_named_pose(tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {"x": 1, "y": 2, "z": 3, "yaw": 0.5},
                "route": [],
            }
        ),
        encoding="utf-8",
    )
    assert load_route_pose(route_file, "goal_ros_pose", -1) == (1.0, 2.0, 3.0, 0.5)


def test_load_route_pose_falls_back_to_route_point(tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps({"route": [{"x": 4, "y": 5, "z": 0, "yaw": -1.0}]}),
        encoding="utf-8",
    )
    assert load_route_pose(route_file, "goal_ros_pose", -1) == (4.0, 5.0, 0.0, -1.0)


def test_load_route_pose_rejects_non_finite_values(tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {"x": math.inf, "y": 2, "z": 0, "yaw": 0},
                "route": [],
            }
        ),
        encoding="utf-8",
    )
    try:
        load_route_pose(route_file, "goal_ros_pose", -1)
    except ValueError as error:
        assert "non-finite" in str(error)
    else:
        raise AssertionError("expected a non-finite goal to be rejected")


def test_route_alignment_metrics_cover_start_goal_yaw_and_segments():
    message = SimpleNamespace(
        start_pose=pose(1.0, 2.0, 0.0),
        goal_pose=pose(9.5, 2.0, -math.pi),
        segments=[object(), object()],
    )
    metrics = route_alignment_metrics(
        message,
        start=(1.0, 2.0, 0.0, 0.0),
        goal=(10.0, 2.0, 0.0, math.pi),
    )

    assert metrics["start_distance_m"] == 0.0
    assert metrics["goal_distance_m"] == 0.5
    assert metrics["goal_yaw_error_rad"] < 1.0e-9
    assert metrics["segment_count"] == 2


def test_route_alignment_only_accepts_active_or_arrived_route():
    assert route_state_supports_alignment(RouteState.SET)
    assert route_state_supports_alignment(RouteState.ARRIVED)
    for state in (
        RouteState.UNKNOWN,
        RouteState.INITIALIZING,
        RouteState.UNSET,
        RouteState.ROUTING,
        RouteState.REROUTING,
        RouteState.ABORTED,
        RouteState.INTERRUPTED,
    ):
        assert not route_state_supports_alignment(state)
