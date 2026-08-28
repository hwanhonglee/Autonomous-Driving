from __future__ import annotations

import importlib.util
import hashlib
import json
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/trim_route_goal.py"


def load_module():
    spec = importlib.util.spec_from_file_location("trim_route_goal", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def point(index, x, y=0.0, z=0.0, yaw=0.0, option="LANEFOLLOW"):
    return {
        "index": index,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "distance_m": float(index * 10),
        "remaining_m": float((3 - index) * 10),
        "road_option": option,
        "road_option_value": 4,
        "vad_command": 3,
        "road_id": 1,
        "section_id": 0,
        "lane_id": -1,
        "is_junction": False,
    }


def route_payload():
    return {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "town": "TestTown",
        "route_length_m": 30.0,
        "goal_spawn_index": 7,
        "goal_ros_pose": {"x": 30.0, "y": 0.0, "z": 4.0, "yaw": 0.0},
        "goal_carla_transform": {
            "x": 30.0,
            "y": 0.0,
            "z": 4.0,
            "roll": 1.0,
            "pitch": 2.0,
            "yaw": 0.0,
        },
        "option_counts": {"LANEFOLLOW": 3, "RIGHT": 1},
        "route": [
            point(0, 0.0),
            point(1, 10.0),
            point(2, 20.0, option="RIGHT"),
            point(3, 30.0, option="RIGHT"),
        ],
    }


def trim(module, route, setback):
    return module.trim_route_payload(
        route,
        setback,
        source_route="/source/route.json",
        source_sha256="a" * 64,
    )


def test_setback_interpolates_goal_and_recomputes_all_route_metadata():
    module = load_module()
    source = route_payload()

    output = trim(module, source, 5.0)

    assert len(output["route"]) == 4
    assert [item["index"] for item in output["route"]] == [0, 1, 2, 3]
    assert [item["distance_m"] for item in output["route"]] == pytest.approx(
        [0.0, 10.0, 20.0, 25.0]
    )
    assert [item["remaining_m"] for item in output["route"]] == pytest.approx(
        [25.0, 15.0, 5.0, 0.0]
    )
    assert output["route_length_m"] == pytest.approx(25.0)
    assert output["option_counts"] == {"LANEFOLLOW": 2, "RIGHT": 2}
    assert output["goal_ros_pose"] == pytest.approx(
        {"x": 25.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    )
    assert output["goal_carla_transform"]["x"] == pytest.approx(25.0)
    assert output["goal_carla_transform"]["y"] == pytest.approx(0.0)
    assert output["goal_carla_transform"]["z"] == pytest.approx(0.0)
    assert output["goal_carla_transform"]["roll"] == 1.0
    assert output["goal_carla_transform"]["pitch"] == 2.0
    assert output["goal_spawn_index"] is None
    assert source["route"][-1]["x"] == 30.0
    assert source["goal_spawn_index"] == 7


def test_provenance_preserves_original_goal_and_exact_terminal_source():
    module = load_module()
    source = route_payload()

    output = trim(module, source, 10.0)
    provenance = output["goal_adjustment"]

    assert len(output["route"]) == 3
    assert output["route"][-1]["x"] == pytest.approx(20.0)
    assert provenance["method"] == "route_arc_length_setback"
    assert provenance["source_route"] == "/source/route.json"
    assert provenance["source_route_sha256"] == "a" * 64
    assert provenance["original_goal_spawn_index"] == 7
    assert provenance["original_goal_ros_pose"] == source["goal_ros_pose"]
    assert provenance["original_goal_carla_transform"] == source["goal_carla_transform"]
    assert provenance["terminal_source_segment"] == [1, 2]
    assert provenance["terminal_source_index"] == 2
    assert provenance["terminal_interpolation_ratio"] == pytest.approx(1.0)
    assert provenance["actual_setback_m"] == pytest.approx(10.0)


def test_same_xy_elevated_spawn_pose_is_not_counted_as_route_length():
    module = load_module()
    source = route_payload()
    source["route"] = [point(0, 0.0), point(1, 10.0), point(2, 10.0, z=4.0)]
    source["route_length_m"] = 10.0
    source["goal_ros_pose"] = {"x": 10.0, "y": 0.0, "z": 4.0, "yaw": 0.0}

    output = trim(module, source, 2.0)

    assert output["route_length_m"] == pytest.approx(8.0)
    assert len(output["route"]) == 2
    assert output["goal_ros_pose"]["x"] == pytest.approx(8.0)
    assert output["goal_ros_pose"]["z"] == pytest.approx(0.0)
    assert output["goal_adjustment"]["original_route_length_m"] == pytest.approx(10.0)


def test_yaw_uses_shortest_interpolation_and_ros_to_carla_conversion():
    module = load_module()
    source = route_payload()
    source["route"] = [
        point(0, 0.0, y=2.0, z=1.0, yaw=math.radians(170.0)),
        point(1, 10.0, y=4.0, z=3.0, yaw=math.radians(-170.0)),
    ]

    output = trim(module, source, math.sqrt(108.0) / 2.0)

    goal = output["goal_ros_pose"]
    assert goal["x"] == pytest.approx(5.0)
    assert goal["y"] == pytest.approx(3.0)
    assert goal["z"] == pytest.approx(2.0)
    assert abs(abs(math.degrees(goal["yaw"])) - 180.0) < 1.0e-9
    carla_goal = output["goal_carla_transform"]
    assert carla_goal["x"] == pytest.approx(5.0)
    assert carla_goal["y"] == pytest.approx(-3.0)
    assert carla_goal["z"] == pytest.approx(2.0)
    assert abs(abs(carla_goal["yaw"]) - 180.0) < 1.0e-9


def test_aligned_route_goal_is_inverse_transformed_back_to_carla():
    module = load_module()
    source = route_payload()
    source["route"] = [
        point(0, 100.0, y=200.0, z=11.0, yaw=math.pi / 2.0),
        point(1, 100.0, y=210.0, z=12.0, yaw=math.pi / 2.0),
        point(2, 100.0, y=220.0, z=13.0, yaw=math.pi / 2.0),
    ]
    source["coordinate_alignment"] = {
        "schema_version": 1,
        "source_frame": "carla_map",
        "target_frame": "map",
        "carla_to_map_transform": {
            "x_m": 100.0,
            "y_m": 200.0,
            "z_m": 10.0,
            "yaw_rad": math.pi / 2.0,
        },
    }
    segment = math.sqrt(101.0)

    output = trim(module, source, segment / 2.0)

    goal = output["goal_ros_pose"]
    assert goal["x"] == pytest.approx(100.0)
    assert goal["y"] == pytest.approx(215.0)
    assert goal["z"] == pytest.approx(12.5)
    carla_goal = output["goal_carla_transform"]
    assert carla_goal["x"] == pytest.approx(15.0)
    assert carla_goal["y"] == pytest.approx(0.0, abs=1.0e-12)
    assert carla_goal["z"] == pytest.approx(2.5)
    assert carla_goal["yaw"] == pytest.approx(0.0)
    assert (
        output["goal_adjustment"]["goal_carla_transform_conversion"]
        == "inverse_map_alignment_then_ros_to_carla"
    )


@pytest.mark.parametrize("setback", [0.0, -1.0, 30.0, 40.0, math.inf])
def test_invalid_setback_is_rejected(setback):
    module = load_module()

    with pytest.raises(module.GoalSetbackError):
        trim(module, route_payload(), setback)


def test_reapplying_setback_is_rejected():
    module = load_module()
    adjusted = trim(module, route_payload(), 5.0)

    with pytest.raises(module.GoalSetbackError, match="recorded source"):
        trim(module, adjusted, 1.0)


def test_file_writer_is_idempotent_but_refuses_different_output(tmp_path):
    module = load_module()
    source_path = tmp_path / "source.json"
    output_path = tmp_path / "shortened.json"
    source_path.write_text(json.dumps(route_payload()), encoding="utf-8")

    first = module.trim_route_file(source_path, output_path, 5.0)
    repeated = module.trim_route_file(source_path, output_path, 5.0)

    assert repeated == first
    assert json.loads(output_path.read_text(encoding="utf-8")) == first
    with pytest.raises(module.GoalSetbackError, match="refusing to overwrite"):
        module.trim_route_file(source_path, output_path, 6.0)
    with pytest.raises(module.GoalSetbackError, match="must be different"):
        module.trim_route_file(source_path, source_path, 5.0)


def write_map_aware_fixture(tmp_path, *, reversed_centerline=False):
    transform = {"x_m": 100.0, "y_m": 200.0, "z_m": 10.0, "yaw_rad": math.pi / 2.0}

    def aligned(raw_x, raw_y, raw_z=0.0):
        cosine = math.cos(transform["yaw_rad"])
        sine = math.sin(transform["yaw_rad"])
        return (
            transform["x_m"] + cosine * raw_x - sine * raw_y,
            transform["y_m"] + sine * raw_x + cosine * raw_y,
            transform["z_m"] + raw_z,
        )

    center_raw = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)]
    if reversed_centerline:
        center_raw.reverse()
    point_groups = {
        "center": [aligned(x, y) for x, y in center_raw],
        # Boundary order is deliberately opposite to route travel direction.
        "left": [aligned(30.0, 2.0), aligned(0.0, 2.0)],
        "right": [aligned(30.0, -2.0), aligned(0.0, -2.0)],
    }
    node_lines = []
    way_lines = []
    next_id = 1
    way_ids = {"left": "101", "right": "102", "center": "103"}
    for role, points in point_groups.items():
        refs = []
        for x, y, z in points:
            node_id = str(next_id)
            next_id += 1
            refs.append(node_id)
            node_lines.append(
                f'<node id="{node_id}"><tag k="local_x" v="{x}"/>'
                f'<tag k="local_y" v="{y}"/><tag k="ele" v="{z}"/></node>'
            )
        nds = "".join(f'<nd ref="{node_id}"/>' for node_id in refs)
        way_lines.append(f'<way id="{way_ids[role]}">{nds}</way>')
    osm_text = (
        "<osm>"
        + "".join(node_lines)
        + "".join(way_lines)
        + '<relation id="500"><member type="way" role="left" ref="101"/>'
        '<member type="way" role="right" ref="102"/>'
        '<member type="way" role="centerline" ref="103"/>'
        '<tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation></osm>'
    )
    map_directory = tmp_path / "map"
    map_directory.mkdir()
    osm_path = map_directory / "lanelet2_map.osm"
    osm_path.write_text(osm_text, encoding="utf-8")
    osm_sha256 = hashlib.sha256(osm_path.read_bytes()).hexdigest()
    bundle = {
        "schema_version": 1,
        "profile": "test_map",
        "carla_to_map_transform": transform,
        "bundle_sources": {
            "lanelet2_map": {
                "resolved_path": str(osm_path),
                "sha256": osm_sha256,
            }
        },
    }
    (map_directory / "map_bundle.json").write_text(json.dumps(bundle), encoding="utf-8")
    vehicle_path = tmp_path / "vehicle_info.param.yaml"
    vehicle_path.write_text(
        """/**:
  ros__parameters:
    wheel_base: 2.0
    wheel_tread: 2.0
    front_overhang: 0.5
    rear_overhang: 0.5
    left_overhang: 0.2
    right_overhang: 0.2
""",
        encoding="utf-8",
    )
    route = route_payload()
    route["route"] = [point(0, 0.0), point(1, 10.0), point(2, 20.0), point(3, 29.0)]
    route["route_length_m"] = 29.0
    route["goal_ros_pose"] = {"x": 29.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    route["goal_carla_transform"].update({"x": 29.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
    route_path = tmp_path / "route.json"
    route_path.write_text(json.dumps(route), encoding="utf-8")
    return route_path, map_directory, vehicle_path


def test_map_aware_mode_uses_explicit_forward_centerline_and_safe_footprint(tmp_path):
    module = load_module()
    route_path, map_directory, vehicle_path = write_map_aware_fixture(tmp_path)

    output = module.trim_route_file_map_aware(
        route_path,
        tmp_path / "safe_route.json",
        map_directory,
        vehicle_path,
        minimum_footprint_clearance_m=0.5,
    )

    adjustment = output["goal_adjustment"]
    assert adjustment["method"] == "lanelet_explicit_centerline_safe_setback"
    assert adjustment["terminal_source_index"] == 2
    assert adjustment["centerline"]["lanelet_id"] == "500"
    assert adjustment["centerline"]["way_id"] == "103"
    assert adjustment["centerline"]["direction_cosine"] == pytest.approx(1.0)
    assert adjustment["centerline"]["direction_was_reversed"] is False
    assert adjustment["aligned_snapped_goal_pose"] == pytest.approx(
        {"x": 100.0, "y": 220.0, "z": 10.0, "yaw": math.pi / 2.0}
    )
    assert adjustment["footprint_validation"]["status"] == "PASS"
    assert adjustment["footprint_validation"]["boundary_clearance_m"] == pytest.approx(0.8)
    assert output["goal_ros_pose"] == pytest.approx(
        {"x": 20.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    )
    assert output["goal_carla_transform"]["x"] == pytest.approx(20.0)
    assert output["goal_carla_transform"]["yaw"] == pytest.approx(0.0)
    assert output["route"][-1]["x"] == pytest.approx(20.0)
    assert output["route_length_m"] == pytest.approx(20.0)


def test_map_aware_mode_does_not_reverse_an_explicit_centerline(tmp_path):
    module = load_module()
    route_path, map_directory, vehicle_path = write_map_aware_fixture(
        tmp_path, reversed_centerline=True
    )

    with pytest.raises(module.GoalSetbackError, match="forward explicit centerline"):
        module.trim_route_file_map_aware(
            route_path,
            tmp_path / "must_not_exist.json",
            map_directory,
            vehicle_path,
        )


def test_current_woraksan_safe_terminal_regression(tmp_path):
    module = load_module()
    source = (
        ROOT
        / "data/training/suites/woraksan_packaged_straight_smoke/woraksan_1_0_3/"
        "woraksan_1_0_3_straight_s0000_p00/ClearNoon/seed_0103/episode/route.json"
    )
    bundle = ROOT / "data/maps/Woraksan_v1_0_3_parking_lot_hegiht_fit_full"
    if not source.is_file() or not (bundle / "map_bundle.json").is_file():
        pytest.skip("pinned Woraksan route/map bundle is not available")

    output = module.trim_route_file_map_aware(
        source,
        tmp_path / "woraksan_safe_route.json",
        bundle,
        module.DEFAULT_VEHICLE_INFO,
    )

    adjustment = output["goal_adjustment"]
    aligned_goal = adjustment["aligned_snapped_goal_pose"]
    footprint = adjustment["footprint_validation"]
    assert adjustment["terminal_source_index"] == 38
    assert adjustment["centerline"]["lanelet_id"] == "4584"
    assert adjustment["centerline"]["way_id"] == "6568"
    assert aligned_goal["x"] == pytest.approx(63.0400519746, abs=1.0e-9)
    assert aligned_goal["y"] == pytest.approx(-14.2397553431, abs=1.0e-9)
    assert aligned_goal["z"] == pytest.approx(-30.1596282270, abs=1.0e-9)
    assert aligned_goal["yaw"] == pytest.approx(0.4314170300, abs=1.0e-9)
    assert footprint["status"] == "PASS"
    assert footprint["covering_lanelet_id"] == "4584"
    assert footprint["boundary_clearance_m"] == pytest.approx(0.3756455243, abs=1.0e-9)
