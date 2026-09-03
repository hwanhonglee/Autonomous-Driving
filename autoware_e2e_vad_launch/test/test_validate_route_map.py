import json
import math
from pathlib import Path
import shutil
import sys

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "e2e"))

import validate_route_map as route_map_validator  # noqa: E402
from validate_route_map import ValidationError, validate_route_map  # noqa: E402


FIXTURE_ROOT = Path(__file__).parent / "fixtures" / "route_map"
ROUTE = FIXTURE_ROOT / "town99_route.json"
MAP = FIXTURE_ROOT / "Town99_full"


def copy_fixture(tmp_path):
    route = tmp_path / ROUTE.name
    map_path = tmp_path / MAP.name
    shutil.copy2(ROUTE, route)
    shutil.copytree(MAP, map_path)
    return route, map_path


def append_connected_road_lanelet(map_path: Path) -> None:
    osm_path = map_path / "lanelet2_map.osm"
    osm = osm_path.read_text(encoding="utf-8")
    addition = """
  <node id="5" lat="0.0" lon="0.0">
    <tag k="local_x" v="20.0" />
    <tag k="local_y" v="2.0" />
    <tag k="ele" v="0.0" />
  </node>
  <node id="6" lat="0.0" lon="0.0">
    <tag k="local_x" v="20.0" />
    <tag k="local_y" v="-2.0" />
    <tag k="ele" v="0.0" />
  </node>
  <way id="12">
    <nd ref="2" />
    <nd ref="5" />
  </way>
  <way id="13">
    <nd ref="4" />
    <nd ref="6" />
  </way>
  <relation id="101">
    <member type="way" ref="12" role="left" />
    <member type="way" ref="13" role="right" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />
  </relation>
"""
    osm_path.write_text(osm.replace("</osm>", addition + "</osm>"), encoding="utf-8")


def write_route_endpoints(route: Path, start_x: float, goal_x: float, yaw: float = 0.0) -> None:
    payload = json.loads(route.read_text(encoding="utf-8"))
    start = {"x": start_x, "y": 0.0, "z": 0.0, "yaw": yaw}
    goal = {"x": goal_x, "y": 0.0, "z": 0.0, "yaw": yaw}
    payload["start_ros_pose"] = start
    payload["goal_ros_pose"] = goal
    payload["route"] = [start, {"x": 10.0, "y": 0.0, "z": 0.0, "yaw": yaw}, goal]
    payload["route_length_m"] = abs(goal_x - start_x)
    route.write_text(json.dumps(payload), encoding="utf-8")


def test_valid_route_and_local_map_pass():
    result = validate_route_map(ROUTE, MAP)

    assert result["status"] == "PASS"
    assert result["town"] == "Town99"
    assert result["route_pose_count"] == 5
    assert result["lanelet_count"] == 1
    assert result["pcd"]["points"] == 2
    assert result["pcd"]["encoding"] == "ascii"
    assert result["maximum_lanelet_vertical_distance_m"] == pytest.approx(0.0)
    assert result["directed_connectivity"]["start_lanelet_id"] == 100
    assert result["directed_connectivity"]["goal_lanelet_id"] == 100
    assert result["directed_connectivity"]["path_lanelet_ids"] == [100]
    assert result["directed_connectivity"]["routing_policy"]["lane_changes_allowed"] is True
    assert result["warnings"] == []


def test_directed_routing_accepts_connected_forward_lanelets(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    append_connected_road_lanelet(map_path)
    write_route_endpoints(route, 1.0, 19.0)

    result = validate_route_map(route, map_path)

    connectivity = result["directed_connectivity"]
    assert connectivity["status"] == "PASS"
    assert connectivity["start_lanelet_id"] == 100
    assert connectivity["goal_lanelet_id"] == 101
    assert connectivity["path_lanelet_ids"] == [100, 101]
    assert connectivity["no_lane_change_route_available"] is True
    assert connectivity["requires_lane_change"] is False


def test_directed_routing_rejects_reverse_topology_despite_polygon_membership(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    append_connected_road_lanelet(map_path)
    write_route_endpoints(route, 19.0, 1.0)

    with pytest.raises(ValidationError, match="no directed Lanelet2 route exists") as caught:
        validate_route_map(route, map_path)

    assert "[101]" in str(caught.value)
    assert "goal road lanelet 100" in str(caught.value)


def test_goal_yaw_must_match_autoware_goal_threshold(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload["goal_ros_pose"]["yaw"] = 2.0
    payload["route"][-1]["yaw"] = 2.0
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match="Autoware goal threshold=45.000 deg"):
        validate_route_map(route, map_path)


def test_start_yaw_must_match_autoware_start_threshold(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload["start_ros_pose"]["yaw"] = math.pi
    payload["route"][0]["yaw"] = math.pi
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match="Autoware start threshold=90.000 deg"):
        validate_route_map(route, map_path)


def test_missing_lanelet2_python_dependency_fails_closed(monkeypatch):
    def unavailable():
        raise ValidationError(
            ["lanelet2 Python routing dependency is unavailable; fail-closed fixture"]
        )

    monkeypatch.setattr(route_map_validator, "_lanelet2_routing_modules", unavailable)

    with pytest.raises(ValidationError, match="dependency is unavailable"):
        validate_route_map(ROUTE, MAP)


def test_custom_map_bundle_supplies_exact_carla_identity(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    renamed_map_path = map_path.with_name("custom_full")
    map_path.rename(renamed_map_path)
    (renamed_map_path / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "canonical_carla_map": "/Game/maps/Town99/Town99",
            }
        ),
        encoding="utf-8",
    )

    result = validate_route_map(route, renamed_map_path)

    assert result["town"] == "Town99"
    assert result["warnings"] == []


def test_custom_map_bundle_rejects_different_route_identity(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    (map_path / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "canonical_carla_map": "/Game/maps/AnotherMap/AnotherMap",
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValidationError, match="does not match map bundle identity"):
        validate_route_map(route, map_path)


def test_route_point_outside_lanelet_reports_point_and_distance(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload["route"][1]["y"] = 9.0
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError) as caught:
        validate_route_map(route, map_path, tolerance_m=0.5)

    message = str(caught.value)
    assert "route[1]" in message
    assert "outside every lanelet" in message
    assert "nearest lanelet 100" in message


def test_town_metadata_and_map_path_mismatch_is_rejected(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload["town"] = "Town98"
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match="does not match map bundle identity"):
        validate_route_map(route, map_path)


def test_non_local_projector_is_rejected(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    (map_path / "map_projector_info.yaml").write_text(
        "projector_type: MGRS\n", encoding="utf-8"
    )

    with pytest.raises(ValidationError, match="must be 'Local'"):
        validate_route_map(route, map_path)


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ({"route": []}, "non-empty array"),
        ({"goal_ros_pose": {"x": float("inf"), "y": 1, "z": 0, "yaw": 0}}, "finite"),
    ],
)
def test_invalid_route_structure_is_rejected(tmp_path, mutation, expected):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload.update(mutation)
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match=expected):
        validate_route_map(route, map_path)


def test_route_terminal_must_match_goal_pose(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    payload["route"][-1]["x"] -= 1.0
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match="route terminal must match goal_ros_pose"):
        validate_route_map(route, map_path)


def test_route_elevation_must_match_lanelet(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    payload = json.loads(route.read_text(encoding="utf-8"))
    for key in ("start_ros_pose", "goal_ros_pose"):
        payload[key]["z"] = 12.0
    for point in payload["route"]:
        point["z"] = 12.0
    route.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValidationError, match="differs from lanelet .* elevation"):
        validate_route_map(route, map_path, vertical_tolerance_m=5.0)


def test_invalid_pcd_header_is_rejected(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    (map_path / "pointcloud_map.pcd").write_text(
        "VERSION 0.7\nFIELDS x y z\nDATA ascii\n0 0 0\n", encoding="ascii"
    )

    with pytest.raises(ValidationError, match="PCD header is missing"):
        validate_route_map(route, map_path)


def test_missing_osm_local_coordinate_is_rejected(tmp_path):
    route, map_path = copy_fixture(tmp_path)
    osm_path = map_path / "lanelet2_map.osm"
    osm = osm_path.read_text(encoding="utf-8")
    osm_path.write_text(
        osm.replace('    <tag k="local_x" v="0.0" />\n', "", 1), encoding="utf-8"
    )

    with pytest.raises(ValidationError, match="finite local_x/local_y"):
        validate_route_map(route, map_path)
