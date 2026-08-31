from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/prepare_carla_expert_route_catalog.py"
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("prepare_carla_expert_route_catalog", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Location:
    def __init__(self, x, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return (
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        ) ** 0.5


class Transform:
    def __init__(self, x, y=0.0, z=0.0, yaw=0.0):
        self.location = Location(x, y, z)
        self.rotation = SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw)


def route_point(x, option="LANEFOLLOW", y=0.0, z=0.0, yaw=0.0):
    return SimpleNamespace(transform=Transform(x, y, z, yaw)), option


def test_initial_approach_preflight_rejects_an_uncommanded_sharp_bend():
    module = load_module()
    args = SimpleNamespace(
        initial_approach_distance_m=15.0,
        maximum_initial_lateral_deviation_m=1.5,
        maximum_initial_heading_change_deg=30.0,
    )
    contract = module.initial_approach_contract(args)
    stable = [
        route_point(0.0),
        route_point(5.0),
        route_point(10.0),
        route_point(15.0, "STRAIGHT"),
        route_point(20.0, "STRAIGHT"),
    ]
    sharp = [
        route_point(0.0),
        route_point(5.0),
        route_point(5.0, y=5.0, yaw=90.0),
        route_point(5.0, "STRAIGHT", y=10.0, yaw=90.0),
        route_point(5.0, "STRAIGHT", y=15.0, yaw=90.0),
    ]

    stable_result = module.analyze_initial_approach(stable, contract)
    sharp_result = module.analyze_initial_approach(sharp, contract)

    assert stable_result["status"] == "PASS"
    assert stable_result["covered_distance_m"] == pytest.approx(15.0)
    assert stable_result["maximum_lateral_deviation_m"] == pytest.approx(0.0)
    assert sharp_result["status"] == "FAIL"
    assert sharp_result["maximum_lateral_deviation_m"] > 1.5
    assert sharp_result["maximum_heading_change_deg"] == pytest.approx(90.0)
    assert len(sharp_result["failure_reasons"]) == 2


def test_initial_approach_contract_is_explicit_and_complete():
    module = load_module()

    assert module.initial_approach_contract(SimpleNamespace())["enabled"] is False
    with pytest.raises(module.CatalogError, match="requires both"):
        module.initial_approach_contract(
            SimpleNamespace(initial_approach_distance_m=15.0)
        )
    with pytest.raises(module.CatalogError, match="require a positive"):
        module.initial_approach_contract(
            SimpleNamespace(
                initial_approach_distance_m=0.0,
                maximum_initial_lateral_deviation_m=1.5,
            )
        )


def turn_geometry_args(**overrides):
    values = {
        "minimum_turn_arc_length_m": 10.0,
        "maximum_turn_arc_length_m": 30.0,
        "minimum_turn_heading_change_deg": 60.0,
        "maximum_turn_heading_change_deg": 120.0,
        "maximum_turn_heading_excess_deg": 20.0,
        "turn_alignment_heading_margin_deg": 10.0,
        "maximum_turn_command_lead_m": 8.0,
        "maximum_turn_command_tail_m": 8.0,
        "maximum_turn_p95_curvature_per_m": 0.20,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def compact_left_route():
    route = [route_point(float(index)) for index in range(16)]
    for offset in range(13):
        route.append(
            route_point(
                float(16 + offset),
                "LEFT",
                yaw=90.0 * offset / 12.0,
            )
        )
    route.extend(
        route_point(float(index), yaw=90.0) for index in range(29, 33)
    )
    return route


def test_turn_geometry_preflight_requires_one_compact_aligned_turn():
    module = load_module()
    contract = module.turn_geometry_contract(turn_geometry_args())
    compact = compact_left_route()
    compound = list(compact)
    compound.extend(
        [
            route_point(33.0, "LEFT", yaw=90.0),
            route_point(34.0, "LEFT", yaw=135.0),
            route_point(35.0, "LEFT", yaw=180.0),
            route_point(36.0, yaw=180.0),
        ]
    )
    additional = list(compact)
    additional[-1] = route_point(32.0, "STRAIGHT", yaw=90.0)

    accepted = module.analyze_turn_geometry(compact, "left", contract)
    rejected_compound = module.analyze_turn_geometry(compound, "left", contract)
    rejected_additional = module.analyze_turn_geometry(
        additional, "left", contract
    )

    assert accepted["status"] == "PASS"
    assert accepted["directional_block_count"] == 1
    assert accepted["selected_block"]["command_arc_length_m"] == pytest.approx(13.0)
    assert accepted["selected_block"]["absolute_net_heading_change_deg"] == (
        pytest.approx(90.0)
    )
    assert accepted["selected_block"]["command_lead_distance_m"] < 8.0
    assert accepted["selected_block"]["command_tail_distance_m"] < 8.0
    assert rejected_compound["status"] == "FAIL"
    assert rejected_compound["directional_block_count"] == 2
    assert rejected_additional["status"] == "FAIL"
    assert rejected_additional["additional_maneuver_commands"] == ["STRAIGHT"]


def test_turn_geometry_contract_is_all_or_nothing():
    module = load_module()

    assert module.turn_geometry_contract(SimpleNamespace()) == {"enabled": False}
    with pytest.raises(module.CatalogError, match="requires every limit"):
        module.turn_geometry_contract(
            SimpleNamespace(minimum_turn_arc_length_m=10.0)
        )
    with pytest.raises(module.CatalogError, match="below its maximum"):
        module.turn_geometry_contract(
            turn_geometry_args(maximum_turn_arc_length_m=5.0)
        )


def test_catalog_deterministically_skips_a_failed_initial_approach(
    tmp_path, monkeypatch
):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    spawns = [Transform(0.0), Transform(20.0), Transform(40.0)]
    monkeypatch.setattr(module, "SCENARIOS", ("straight",))
    monkeypatch.setattr(
        module,
        "deterministic_pairs",
        lambda *_args, **_kwargs: [(0, 1), (0, 2)],
    )

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return spawns

    class World:
        @staticmethod
        def get_map():
            return Map()

    class Client:
        def __init__(self, _host, _port):
            pass

        def set_timeout(self, _timeout):
            pass

        @staticmethod
        def get_world():
            return World()

        @staticmethod
        def get_client_version():
            return "0.9.15"

        @staticmethod
        def get_server_version():
            return "0.9.15"

    class Planner:
        def __init__(self, _map, _resolution):
            pass

        @staticmethod
        def trace_route(start, goal):
            if goal.x == 20.0:
                return [
                    route_point(start.x),
                    route_point(5.0),
                    route_point(5.0, "STRAIGHT", y=5.0, yaw=90.0),
                    route_point(5.0, "STRAIGHT", y=10.0, yaw=90.0),
                    route_point(goal.x),
                ]
            return [
                route_point(start.x),
                route_point(10.0),
                route_point(20.0, "STRAIGHT"),
                route_point(30.0, "STRAIGHT"),
                route_point(goal.x),
            ]

    def length(route):
        return sum(
            route[index - 1][0].transform.location.distance(
                route[index][0].transform.location
            )
            for index in range(1, len(route))
        )

    def serialize(route, _goal):
        command = {"LANEFOLLOW": 3, "STRAIGHT": 2}
        cumulative = 0.0
        output = []
        for index, (waypoint, option) in enumerate(route):
            if index:
                cumulative += route[index - 1][0].transform.location.distance(
                    waypoint.transform.location
                )
            output.append(
                {
                    "index": index,
                    "x": waypoint.transform.location.x,
                    "y": -waypoint.transform.location.y,
                    "z": waypoint.transform.location.z,
                    "yaw": -waypoint.transform.rotation.yaw,
                    "distance_m": cumulative,
                    "road_option": option,
                    "vad_command": command[option],
                }
            )
        return output

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: scenario == "straight"
        and any(option == "STRAIGHT" for _, option in route),
        route_length=length,
        serialize_route=serialize,
        transform_dict=lambda transform: {"x": transform.location.x},
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )
    base_args = {
        "map_id": "town01",
        "host": "127.0.0.1",
        "port": 2100,
        "timeout": 3.0,
        "weather": "ClearNoon",
        "seeds": (0,),
        "pairs_per_seed": 1,
        "min_distance": 20.0,
        "max_distance": 80.0,
        "preferred_distance": 40.0,
        "sampling_resolution": 1.0,
        "max_endpoint_offset": 2.0,
        "max_traces": 20,
        "exclude_spawn_indices": (),
        "map_load_settle_sec": 0.0,
        "allow_map_load": False,
        "active_server_profile": "packaged_0915",
        "initial_approach_distance_m": 15.0,
        "maximum_initial_lateral_deviation_m": 1.5,
        "maximum_initial_heading_change_deg": 30.0,
    }

    catalogs = []
    for name in ("first", "repeated"):
        args = SimpleNamespace(**base_args, output_root=tmp_path / name)
        catalogs.append(
            module.build_catalog(args, manifest, SimpleNamespace(Client=Client), helper)
        )

    for catalog in catalogs:
        assert catalog["routes"][0]["goal_spawn_index"] == 2
        assert catalog["routes"][0]["initial_approach_preflight"]["status"] == "PASS"
        coverage = catalog["scenario_results"][0]["trace_coverage"]
        assert coverage["attempted"] == 2
        assert coverage["initial_approach_rejected"] == 1
    assert catalogs[0]["routes"][0]["id"] == catalogs[1]["routes"][0]["id"]
    assert catalogs[0]["routes"][0]["sha256"] == catalogs[1]["routes"][0]["sha256"]


def test_catalog_deterministically_skips_compound_turn_geometry(
    tmp_path, monkeypatch
):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    spawns = [Transform(0.0), Transform(40.0), Transform(80.0)]
    monkeypatch.setattr(module, "SCENARIOS", ("left",))
    monkeypatch.setattr(
        module,
        "deterministic_pairs",
        lambda *_args, **_kwargs: [(0, 1), (0, 2)],
    )

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return spawns

    class World:
        @staticmethod
        def get_map():
            return Map()

    class Client:
        def __init__(self, _host, _port):
            pass

        def set_timeout(self, _timeout):
            pass

        @staticmethod
        def get_world():
            return World()

        @staticmethod
        def get_client_version():
            return "0.9.15"

        @staticmethod
        def get_server_version():
            return "0.9.15"

    class Planner:
        def __init__(self, _map, _resolution):
            pass

        @staticmethod
        def trace_route(_start, goal):
            route = compact_left_route()
            if goal.x == 40.0:
                route.extend(
                    [
                        route_point(33.0, "LEFT", yaw=90.0),
                        route_point(34.0, "LEFT", yaw=135.0),
                        route_point(35.0, "LEFT", yaw=180.0),
                        route_point(40.0, yaw=180.0),
                    ]
                )
            else:
                route.append(route_point(80.0, yaw=90.0))
            return route

    def length(route):
        return sum(
            route[index - 1][0].transform.location.distance(
                route[index][0].transform.location
            )
            for index in range(1, len(route))
        )

    def serialize(route, _goal):
        cumulative = 0.0
        output = []
        for index, (waypoint, option) in enumerate(route):
            if index:
                cumulative += route[index - 1][0].transform.location.distance(
                    waypoint.transform.location
                )
            output.append(
                {
                    "index": index,
                    "x": waypoint.transform.location.x,
                    "y": -waypoint.transform.location.y,
                    "z": waypoint.transform.location.z,
                    "yaw": -waypoint.transform.rotation.yaw,
                    "distance_m": cumulative,
                    "road_option": option,
                    "vad_command": 0 if option == "LEFT" else 3,
                }
            )
        return output

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: scenario == "left"
        and any(option == "LEFT" for _, option in route),
        route_length=length,
        serialize_route=serialize,
        transform_dict=lambda transform: {"x": transform.location.x},
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )
    base_args = {
        "map_id": "town01",
        "host": "127.0.0.1",
        "port": 2100,
        "timeout": 3.0,
        "weather": "ClearNoon",
        "seeds": (0,),
        "pairs_per_seed": 1,
        "min_distance": 20.0,
        "max_distance": 100.0,
        "preferred_distance": 60.0,
        "sampling_resolution": 1.0,
        "max_endpoint_offset": 2.0,
        "max_traces": 20,
        "exclude_spawn_indices": (),
        "map_load_settle_sec": 0.0,
        "allow_map_load": False,
        "active_server_profile": "packaged_0915",
        "initial_approach_distance_m": 15.0,
        "maximum_initial_lateral_deviation_m": 1.5,
        "maximum_initial_heading_change_deg": 30.0,
        **vars(turn_geometry_args()),
    }

    catalogs = []
    for name in ("first", "repeated"):
        args = SimpleNamespace(**base_args, output_root=tmp_path / name)
        catalogs.append(
            module.build_catalog(args, manifest, SimpleNamespace(Client=Client), helper)
        )

    for catalog in catalogs:
        assert catalog["routes"][0]["goal_spawn_index"] == 2
        assert catalog["routes"][0]["turn_geometry_preflight"]["status"] == "PASS"
        coverage = catalog["scenario_results"][0]["trace_coverage"]
        assert coverage["attempted"] == 2
        assert coverage["turn_geometry_rejected"] == 1
    assert catalogs[0]["routes"][0]["id"] == catalogs[1]["routes"][0]["id"]
    assert catalogs[0]["routes"][0]["sha256"] == catalogs[1]["routes"][0]["sha256"]


def test_deterministic_pairs_are_repeatable_and_seeded():
    module = load_module()
    spawns = [Transform(float(index * 20)) for index in range(6)]

    first = module.deterministic_pairs(spawns, 7, 10.0, 100.0, 50.0, 1.0)
    repeated = module.deterministic_pairs(spawns, 7, 10.0, 100.0, 50.0, 1.0)
    another_seed = module.deterministic_pairs(spawns, 8, 10.0, 100.0, 50.0, 1.0)

    assert first == repeated
    assert first != another_seed
    assert all(start != goal for start, goal in first)

    excluding = module.deterministic_pairs(
        spawns, 7, 10.0, 100.0, 50.0, 1.0, excluded_indices=(1, 4)
    )
    assert excluding
    assert all(start not in (1, 4) and goal not in (1, 4) for start, goal in excluding)


def test_excluded_spawn_indices_parse_and_validate():
    module = load_module()

    assert module.parse_excluded_spawn_indices("") == ()
    assert module.parse_excluded_spawn_indices("4, 1") == (4, 1)
    for invalid in ("-1", "1,1", "1,,2", "one"):
        with pytest.raises(module.CatalogError):
            module.parse_excluded_spawn_indices(invalid)

    assert module.validate_excluded_spawn_indices((4,), 5) == (4,)
    with pytest.raises(module.CatalogError, match="out of range"):
        module.validate_excluded_spawn_indices((5,), 5)
    with pytest.raises(module.CatalogError, match="unique"):
        module.validate_excluded_spawn_indices((1, 1), 5)
    with pytest.raises(module.CatalogError, match="non-negative"):
        module.validate_excluded_spawn_indices((-1,), 5)
    with pytest.raises(module.CatalogError, match="fewer than two"):
        module.validate_excluded_spawn_indices((0, 1, 2, 3), 5)


def test_route_payload_preserves_base_link_schema():
    module = load_module()
    spawns = [Transform(0.0), Transform(30.0)]
    points = [
        {"index": 0, "x": 0.0, "y": 0.0, "distance_m": 0.0, "road_option": "LANEFOLLOW"},
        {"index": 1, "x": 30.0, "y": 0.0, "distance_m": 30.0, "road_option": "LANEFOLLOW"},
    ]
    helper = SimpleNamespace(
        serialize_route=lambda route, goal: points,
        transform_dict=lambda transform: {"x": transform.location.x},
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )
    payload = module._route_payload(
        helper,
        {"canonical_name": "Town01"},
        "ClearNoon",
        "lane_follow",
        1.0,
        0,
        1,
        spawns,
        [1, 2],
    )

    assert payload["coordinate_reference"] == "base_link"
    assert payload["spawn_point_reference"] == "base_link"
    assert payload["town"] == "Town01"
    assert payload["route_length_m"] == 30.0
    assert payload["option_counts"] == {"LANEFOLLOW": 2}


def test_route_endpoint_offsets_reject_snap_to_another_lane() -> None:
    module = load_module()
    route = [
        (SimpleNamespace(transform=Transform(6.58)), "LANEFOLLOW"),
        (SimpleNamespace(transform=Transform(30.0)), "LANEFOLLOW"),
    ]

    start_offset, goal_offset = module.route_endpoint_offsets(
        route, Location(0.0), Location(30.0)
    )

    assert start_offset == pytest.approx(6.58)
    assert goal_offset == pytest.approx(0.0)


def test_route_endpoint_offsets_ignore_elevation_only_mismatch() -> None:
    module = load_module()
    route = [
        (SimpleNamespace(transform=Transform(0.0, 0.0, -4.0)), "LANEFOLLOW"),
        (SimpleNamespace(transform=Transform(30.0, 0.0, 4.0)), "LANEFOLLOW"),
    ]

    start_offset, goal_offset = module.route_endpoint_offsets(
        route, Location(0.0, 0.0, 0.0), Location(30.0, 0.0, 0.0)
    )

    assert start_offset == pytest.approx(0.0)
    assert goal_offset == pytest.approx(0.0)


def test_normalize_route_endpoints_trims_planner_goal_overshoot() -> None:
    module = load_module()
    route = [
        route_point(-8.0),
        route_point(0.0),
        route_point(10.0),
        route_point(20.0),
        route_point(34.4),
    ]

    normalized = module.normalize_route_endpoints(
        route, Location(0.0, z=4.0), Location(20.0, z=4.0)
    )

    assert normalized == route[1:4]


def test_normalize_route_endpoints_keeps_last_exact_goal_duplicate() -> None:
    module = load_module()
    route = [
        route_point(0.0),
        route_point(20.0, "RIGHT"),
        route_point(20.0, "LANEFOLLOW"),
        route_point(34.4),
    ]

    normalized = module.normalize_route_endpoints(
        route, Location(0.0), Location(20.0)
    )

    assert normalized == route[:3]
    assert normalized[-1][1] == "LANEFOLLOW"


def test_catalog_rejects_route_with_planar_endpoint_snap(tmp_path) -> None:
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    spawns = [Transform(0.0), Transform(30.0)]

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return spawns

    class World:
        @staticmethod
        def get_map():
            return Map()

    class Client:
        def __init__(self, _host, _port):
            pass

        def set_timeout(self, _timeout):
            pass

        @staticmethod
        def get_world():
            return World()

        @staticmethod
        def get_client_version():
            return "0.9.15"

        @staticmethod
        def get_server_version():
            return "0.9.15"

    class Planner:
        def __init__(self, _map, _resolution):
            pass

        @staticmethod
        def trace_route(start, goal):
            return [
                route_point(start.x + 6.58),
                route_point(goal.x),
            ]

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda *_args: pytest.fail("endpoint gate must run first"),
        route_length=lambda *_args: pytest.fail("endpoint gate must run first"),
        serialize_route=lambda *_args: pytest.fail("endpoint gate must run first"),
    )
    args = SimpleNamespace(
        map_id="town01",
        host="127.0.0.1",
        port=2100,
        timeout=3.0,
        output_root=tmp_path,
        weather="ClearNoon",
        seeds=(0,),
        pairs_per_seed=1,
        min_distance=20.0,
        max_distance=80.0,
        preferred_distance=30.0,
        sampling_resolution=1.0,
        max_endpoint_offset=2.0,
        max_traces=1,
        exclude_spawn_indices=(),
        map_load_settle_sec=0.0,
        allow_map_load=False,
        active_server_profile="packaged_0915",
    )

    catalog = module.build_catalog(args, manifest, SimpleNamespace(Client=Client), helper)

    assert catalog["status"] == "blocked"
    assert catalog["routes"] == []
    assert all(
        result["trace_coverage"]["endpoint_rejected"] == 1
        for result in catalog["scenario_results"]
    )


def test_unavailable_map_is_rejected_without_constructing_client(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    args = SimpleNamespace(map_id="town08")
    carla = SimpleNamespace(Client=lambda *_args: pytest.fail("must not connect"))

    with pytest.raises(module.CatalogError, match="unavailable"):
        module.build_catalog(args, manifest, carla, SimpleNamespace())


def test_catalog_generation_records_missing_maneuvers_as_skip(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    spawns = [Transform(0.0), Transform(30.0), Transform(60.0), Transform(90.0)]

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return spawns

    class World:
        @staticmethod
        def get_map():
            return Map()

    class Client:
        def __init__(self, host, port):
            assert (host, port) == ("127.0.0.1", 2100)

        def set_timeout(self, timeout):
            assert timeout == 3.0

        @staticmethod
        def get_world():
            return World()

        def load_world(self, *_args):
            pytest.fail("current requested map must not be reloaded")

        @staticmethod
        def get_client_version():
            return "0.9.15"

        @staticmethod
        def get_server_version():
            return "0.9.15"

    class Planner:
        calls = 0

        def __init__(self, _map, _resolution):
            pass

        @classmethod
        def trace_route(cls, start, goal):
            cls.calls += 1
            if cls.calls == 1:
                raise module.nx.NetworkXNoPath(
                    "disconnected spawn pair: source=1 target=18"
                )
            if cls.calls == 2:
                return []
            return [(SimpleNamespace(transform=Transform(start.x)), "LANEFOLLOW"),
                    (SimpleNamespace(transform=Transform(goal.x)), "LANEFOLLOW")]

    def serialize(route, goal):
        length = route[0][0].transform.location.distance(route[-1][0].transform.location)
        return [
            {"index": 0, "x": route[0][0].transform.location.x, "y": 0.0,
             "distance_m": 0.0, "road_option": "LANEFOLLOW"},
            {"index": 1, "x": goal.location.x, "y": 0.0,
             "distance_m": length, "road_option": "LANEFOLLOW"},
        ]

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: scenario == "lane_follow",
        route_length=lambda route: route[0][0].transform.location.distance(
            route[-1][0].transform.location
        ),
        serialize_route=serialize,
        transform_dict=lambda transform: {"x": transform.location.x},
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )
    args = SimpleNamespace(
        map_id="town01", host="127.0.0.1", port=2100, timeout=3.0,
        output_root=tmp_path, weather="ClearNoon", seeds=(1,), pairs_per_seed=1,
        min_distance=20.0, max_distance=80.0, preferred_distance=40.0,
        sampling_resolution=1.0, max_traces=20,
        exclude_spawn_indices=(3,),
        map_load_settle_sec=0.0, allow_map_load=False,
    )

    catalog = module.build_catalog(args, manifest, SimpleNamespace(Client=Client), helper)

    assert catalog["status"] == "complete"
    assert len(catalog["routes"]) == 1
    assert catalog["server"]["map_load_performed"] is False
    assert catalog["server"]["map_load_allowed"] is False
    assert catalog["server"]["map_lifecycle_managed"] is False
    assert catalog["server"]["initial_map_name"].endswith("Town01")
    assert catalog["server"]["active_map_name"].endswith("Town01")
    assert catalog["server"]["server_lifecycle_managed"] is False
    assert [item["status"] for item in catalog["scenario_results"]] == [
        "READY", "SKIP", "SKIP", "SKIP"
    ]
    lane_coverage = catalog["scenario_results"][0]["trace_coverage"]
    coverage_without_samples = {
        key: value
        for key, value in lane_coverage.items()
        if key != "planner_error_samples"
    }
    assert coverage_without_samples == {
        "attempted": 3,
        "accepted": 1,
        "skipped": 2,
        "planner_errors": 1,
        "no_route": 1,
        "endpoint_rejected": 0,
        "scenario_mismatch": 0,
        "distance_rejected": 0,
        "initial_approach_rejected": 0,
        "planner_error_types": {"NetworkXNoPath": 1},
    }
    error_sample = lane_coverage["planner_error_samples"][0]
    assert error_sample["start_spawn_index"] != error_sample["goal_spawn_index"]
    assert error_sample["error_type"] == "NetworkXNoPath"
    assert error_sample["message"] == "disconnected spawn pair: source=1 target=18"
    total_coverage = catalog["generation"]["trace_coverage"]
    assert total_coverage["accepted"] == 1
    assert total_coverage["planner_errors"] == 1
    assert total_coverage["no_route"] == 1
    assert total_coverage["endpoint_rejected"] == 0
    assert total_coverage["planner_error_types"] == {"NetworkXNoPath": 1}
    assert total_coverage["skipped"] == total_coverage["attempted"] - 1
    assert catalog["generation"]["excluded_spawn_indices"] == [3]
    assert catalog["generation"]["excluded_spawn_point_count"] == 1
    assert catalog["generation"]["eligible_spawn_point_count"] == 3
    assert total_coverage["excluded_spawn_point_count"] == 1
    assert total_coverage["eligible_spawn_point_count"] == 3
    assert all(
        route["start_spawn_index"] != 3 and route["goal_spawn_index"] != 3
        for route in catalog["routes"]
    )
    route_path = tmp_path / catalog["routes"][0]["path"]
    assert json.loads(route_path.read_text())["coordinate_reference"] == "base_link"
    written_catalog = json.loads((tmp_path / "route_catalog.json").read_text())
    assert written_catalog["status"] == "complete"
    assert written_catalog["generation"]["trace_coverage"] == total_coverage


def test_trace_route_candidate_does_not_hide_unexpected_planner_errors():
    module = load_module()

    class BrokenPlanner:
        @staticmethod
        def trace_route(_start, _goal):
            raise ValueError("invalid planner topology")

    with pytest.raises(ValueError, match="invalid planner topology"):
        module.trace_route_candidate(BrokenPlanner(), Location(0.0), Location(1.0))


def test_cli_requires_explicit_output_root():
    module = load_module()
    with pytest.raises(SystemExit):
        module.parse_args(["--map-id", "town01"])


def test_cli_defaults_to_safe_map_load_settle():
    module = load_module()
    args = module.parse_args(
        ["--map-id", "town01", "--output-root", "/tmp/expert-routes"]
    )

    assert args.map_load_settle_sec == pytest.approx(10.0)
    assert args.allow_map_load is False
    assert args.active_server_profile is None
    assert args.exclude_spawn_indices == ()
    assert args.max_endpoint_offset == pytest.approx(2.0)

    explicit = module.parse_args(
        [
            "--map-id",
            "town01",
            "--output-root",
            "/tmp/expert-routes",
            "--exclude-spawn-indices",
            "4,2",
        ]
    )
    assert explicit.exclude_spawn_indices == (4, 2)


def test_activate_map_never_loads_by_default_and_requires_current_map_match():
    module = load_module()
    town01 = SimpleNamespace(get_map=lambda: SimpleNamespace(name="Town01"))
    town12 = SimpleNamespace(
        get_map=lambda: SimpleNamespace(name="/Game/Carla/Maps/Town12/Town12")
    )
    loads = []
    sleeps = []
    client = SimpleNamespace(load_world=lambda name: loads.append(name) or town12)
    entry = {
        "load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_name": "Town12",
    }

    with pytest.raises(module.CatalogError, match="automatic map loading is disabled"):
        module.activate_map(client, town01, entry, 10.0, sleep=sleeps.append)
    same_world, loaded = module.activate_map(
        client, town12, entry, 10.0, sleep=sleeps.append
    )

    assert same_world is town12
    assert loaded is False
    assert loads == []
    assert sleeps == [10.0, 10.0]


def test_activate_map_load_requires_explicit_unsafe_opt_in():
    module = load_module()
    town01 = SimpleNamespace(get_map=lambda: SimpleNamespace(name="Town01"))
    town12 = SimpleNamespace(
        get_map=lambda: SimpleNamespace(name="/Game/Carla/Maps/Town12/Town12")
    )
    loads = []
    sleeps = []
    client = SimpleNamespace(
        load_world=lambda name: loads.append(name) or town12,
    )
    entry = {
        "load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_name": "Town12",
    }

    world, loaded = module.activate_map(
        client, town01, entry, 10.0, allow_map_load=True, sleep=sleeps.append
    )

    assert world is town12
    assert loaded is True
    assert loads == [entry["load_name"]]
    assert sleeps == [10.0, 10.0]


def test_source_editor_map_requires_explicit_matching_profile():
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    town12 = next(entry for entry in manifest["maps"] if entry["id"] == "town12")

    with pytest.raises(module.CatalogError, match="explicit --active-server-profile"):
        module.validate_active_server_profile(manifest, town12, None)
    with pytest.raises(module.CatalogError, match="not declared active profile"):
        module.validate_active_server_profile(manifest, town12, "packaged_0915")
    module.validate_active_server_profile(manifest, town12, "source_editor_0915_4ws")


def test_main_reports_invalid_manifest_without_loading_carla(tmp_path, monkeypatch, capsys):
    module = load_module()
    manifest = tmp_path / "invalid.yaml"
    manifest.write_text("schema_version: 1\n", encoding="utf-8")
    monkeypatch.setattr(
        module,
        "_load_runtime_modules",
        lambda: pytest.fail("CARLA must not be imported for an invalid manifest"),
    )

    result = module.main(
        [
            "--manifest",
            str(manifest),
            "--map-id",
            "town01",
            "--output-root",
            str(tmp_path / "output"),
        ]
    )

    assert result == 1
    assert "ERROR:" in capsys.readouterr().err
