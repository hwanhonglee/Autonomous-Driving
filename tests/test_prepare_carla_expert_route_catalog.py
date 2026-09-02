from __future__ import annotations

import importlib.util
import json
import math
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


def serialized_turn_points(
    scenario: str, *, lead_m: int = 30, tail_m: int = 15
) -> list[dict]:
    sign = 1.0 if scenario == "left" else -1.0
    radius_m = 10.0
    points = []
    distance_m = 0.0

    def append(
        x: float,
        y: float,
        yaw: float,
        option: str,
        z: float = 0.0,
    ) -> None:
        nonlocal distance_m
        if points:
            distance_m += math.hypot(
                x - points[-1]["x"], y - points[-1]["y"]
            )
        points.append(
            {
                "index": len(points),
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "distance_m": distance_m,
                "road_option": option,
            }
        )

    for x in range(lead_m):
        append(float(x), 0.0, 0.0, "LANEFOLLOW")
    center_x = float(lead_m - 1)
    center_y = sign * radius_m
    for sample in range(1, 18):
        fraction = sample / 17.0
        angle = -sign * math.pi / 2.0 + sign * fraction * math.pi / 2.0
        append(
            center_x + radius_m * math.cos(angle),
            center_y + radius_m * math.sin(angle),
            sign * fraction * math.pi / 2.0,
            scenario.upper(),
        )
    for offset in range(1, tail_m + 1):
        append(
            center_x + radius_m,
            center_y + sign * offset,
            sign * math.pi / 2.0,
            "LANEFOLLOW",
        )
    return points


def apply_constant_3d_grade(points: list[dict], grade_ratio: float) -> None:
    planar_progress = 0.0
    spatial_progress = 0.0
    planar_slope = grade_ratio / math.sqrt(1.0 - grade_ratio**2)
    previous = None
    for point in points:
        if previous is not None:
            planar_progress += math.hypot(
                point["x"] - previous["x"], point["y"] - previous["y"]
            )
        point["z"] = planar_progress * planar_slope
        if previous is not None:
            spatial_progress += math.sqrt(
                (point["x"] - previous["x"]) ** 2
                + (point["y"] - previous["y"]) ** 2
                + (point["z"] - previous["z"]) ** 2
            )
        point["distance_m"] = spatial_progress
        previous = point


def build_physical_straight_fixture(
    module, tmp_path, monkeypatch, endpoints, trace_route
):
    """Build one opt-in straight catalog without a running CARLA server."""
    manifest, _ = module.load_manifest(MANIFEST)
    monkeypatch.setattr(
        module,
        "deterministic_pairs",
        lambda *_args, **_kwargs: [(0, 1), (0, 2)],
    )

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return endpoints

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
            return trace_route(start, goal)

    serialize_calls = []

    def serialize(route, goal):
        goal_location = goal.location
        serialize_calls.append((goal_location.x, len(route)))
        cumulative = 0.0
        points = []
        previous = None
        for index, (waypoint, option) in enumerate(route):
            transform = waypoint.transform
            if previous is not None:
                cumulative += previous.location.distance(transform.location)
            points.append(
                {
                    "index": index,
                    "x": transform.location.x,
                    "y": -transform.location.y,
                    "yaw": -math.radians(transform.rotation.yaw),
                    "distance_m": cumulative,
                    "road_option": option,
                }
            )
            previous = transform
        terminal = points[-1]
        terminal_gap = math.hypot(
            goal_location.x - terminal["x"],
            -goal_location.y - terminal["y"],
        )
        if terminal_gap > 1.0e-6:
            cumulative += terminal_gap
            points.append(
                {
                    **terminal,
                    "index": len(points),
                    "x": goal_location.x,
                    "y": -goal_location.y,
                    "yaw": -math.radians(goal.rotation.yaw),
                    "distance_m": cumulative,
                }
            )
        return points

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: scenario == "straight"
        and any(option == "STRAIGHT" for _, option in route),
        route_length=lambda *_args: pytest.fail(
            "admission must use the exact serialized length"
        ),
        serialize_route=serialize,
        transform_dict=lambda transform: {
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z,
            "yaw": transform.rotation.yaw,
        },
        ros_pose_dict=lambda transform: {
            "x": transform.location.x,
            "y": -transform.location.y,
            "z": transform.location.z,
            "yaw": -math.radians(transform.rotation.yaw),
        },
    )
    args = SimpleNamespace(
        map_id="town01",
        host="127.0.0.1",
        port=2100,
        timeout=3.0,
        output_root=tmp_path,
        weather="ClearNoon",
        seeds=(0,),
        scenarios=("straight",),
        pairs_per_seed=1,
        min_distance=170.0,
        max_distance=260.0,
        preferred_distance=210.0,
        sampling_resolution=1.0,
        endpoint_waypoint_spacing_m=None,
        physical_straight_profile="speed_30kph",
        max_endpoint_offset=2.0,
        max_traces=20,
        exclude_spawn_indices=(),
        map_load_settle_sec=0.0,
        allow_map_load=False,
        active_server_profile="packaged_0915",
    )
    catalog = module.build_catalog(
        args, manifest, SimpleNamespace(Client=Client), helper
    )
    return catalog, serialize_calls


def build_physical_turn_fixture(
    module,
    tmp_path,
    monkeypatch,
    *,
    first_lead_m: int = 15,
    first_grade_ratio: float = 0.0,
):
    """Build one opt-in turn catalog without a running CARLA server."""
    manifest, _ = module.load_manifest(MANIFEST)
    endpoints = [Transform(0.0), Transform(1.0), Transform(2.0)]
    monkeypatch.setattr(
        module,
        "deterministic_pairs",
        lambda *_args, **_kwargs: [(0, 1), (0, 2)],
    )

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return endpoints

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
            scenario = "left" if goal.x == 1.0 else "left"
            return [
                route_point(start.x),
                route_point(goal.x, scenario.upper()),
            ]

    serialize_calls = []

    def serialize(route, goal):
        scenario = str(route[-1][1]).lower()
        lead_m = first_lead_m if goal.location.x == 1.0 else 30
        serialize_calls.append((scenario, lead_m))
        points = serialized_turn_points(scenario, lead_m=lead_m)
        if goal.location.x == 1.0 and first_grade_ratio:
            apply_constant_3d_grade(points, first_grade_ratio)
        return points

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: any(
            option == scenario.upper() for _, option in route
        ),
        route_length=lambda *_args: pytest.fail(
            "admission must use the exact serialized length"
        ),
        serialize_route=serialize,
        transform_dict=lambda transform: {
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z,
            "yaw": transform.rotation.yaw,
        },
        ros_pose_dict=lambda transform: {
            "x": transform.location.x,
            "y": -transform.location.y,
            "z": transform.location.z,
            "yaw": -math.radians(transform.rotation.yaw),
        },
    )
    args = SimpleNamespace(
        map_id="town01",
        host="127.0.0.1",
        port=2100,
        timeout=3.0,
        output_root=tmp_path,
        weather="ClearNoon",
        seeds=(0,),
        scenarios=("left", "right"),
        pairs_per_seed=1,
        min_distance=20.0,
        max_distance=120.0,
        preferred_distance=60.0,
        sampling_resolution=1.0,
        endpoint_waypoint_spacing_m=None,
        physical_straight_profile=None,
        physical_turn_profile="speed_30kph",
        initial_approach_distance_m=0.0,
        max_endpoint_offset=2.0,
        max_traces=20,
        exclude_spawn_indices=(),
        map_load_settle_sec=0.0,
        allow_map_load=False,
        active_server_profile="packaged_0915",
    )
    catalog = module.build_catalog(
        args, manifest, SimpleNamespace(Client=Client), helper
    )
    return catalog, serialize_calls


def test_physical_straight_admission_uses_exact_terminal_goal_length(
    tmp_path, monkeypatch
):
    module = load_module()
    endpoints = [Transform(0.0), Transform(261.0), Transform(210.0)]

    def trace_route(start, goal):
        return [
            route_point(start.x),
            route_point(goal.x - 1.5, "STRAIGHT"),
        ]

    catalog, serialize_calls = build_physical_straight_fixture(
        module, tmp_path, monkeypatch, endpoints, trace_route
    )

    assert catalog["status"] == "complete"
    assert len(catalog["routes"]) == 1
    accepted = catalog["routes"][0]
    assert accepted["goal_spawn_index"] == 2
    assert accepted["route_length_m"] == pytest.approx(210.0)
    assert accepted["physical_straight_preflight"]["status"] == "PASS"
    coverage = catalog["scenario_results"][0]["trace_coverage"]
    assert coverage["attempted"] == 2
    assert coverage["distance_rejected"] == 1
    assert coverage["physical_straight_rejected"] == 0
    assert len(serialize_calls) == 2
    assert catalog["generation"]["physical_straight_contract"] == {
        "enabled": True,
        "profile_id": "speed_30kph",
        "measurement_source": "exact_serialized_route_with_terminal_goal",
        "admission_policy": "reject_before_accepted_pair_quota",
        "limits": dict(module.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT),
    }


def test_physical_straight_admission_skips_curved_straight_label_before_quota(
    tmp_path, monkeypatch
):
    module = load_module()
    endpoints = [
        Transform(0.0),
        Transform(70.0, y=140.0, yaw=90.0),
        Transform(210.0),
    ]

    def trace_route(start, goal):
        if goal.y:
            return [
                route_point(start.x),
                route_point(70.0, "STRAIGHT"),
                route_point(70.0, y=70.0, yaw=90.0),
                route_point(70.0, y=140.0, yaw=90.0),
            ]
        return [route_point(start.x), route_point(goal.x, "STRAIGHT")]

    catalog, serialize_calls = build_physical_straight_fixture(
        module, tmp_path, monkeypatch, endpoints, trace_route
    )

    assert catalog["status"] == "complete"
    assert len(catalog["routes"]) == 1
    assert catalog["routes"][0]["goal_spawn_index"] == 2
    coverage = catalog["scenario_results"][0]["trace_coverage"]
    assert coverage["attempted"] == 2
    assert coverage["accepted"] == 1
    assert coverage["physical_straight_rejected"] == 1
    assert len(coverage["physical_straight_rejection_samples"]) == 1
    rejected = coverage["physical_straight_rejection_samples"][0]
    assert rejected["goal_spawn_index"] == 1
    assert "arc/direct ratio exceeds the maximum" in rejected["failure_reasons"]
    assert len(serialize_calls) == 2


def test_physical_turn_admission_skips_short_lead_before_quota(
    tmp_path, monkeypatch
):
    module = load_module()
    catalog, serialize_calls = build_physical_turn_fixture(
        module, tmp_path, monkeypatch
    )

    assert catalog["status"] == "complete"
    assert len(catalog["routes"]) == 1
    accepted = catalog["routes"][0]
    assert accepted["scenario"] == "left"
    assert accepted["goal_spawn_index"] == 2
    assert accepted["physical_turn_preflight"]["status"] == "PASS"
    coverage = catalog["scenario_results"][0]["trace_coverage"]
    assert coverage["attempted"] == 2
    assert coverage["accepted"] == 1
    assert coverage["physical_turn_rejected"] == 1
    assert len(coverage["physical_turn_rejection_samples"]) == 1
    rejected = coverage["physical_turn_rejection_samples"][0]
    assert rejected["goal_spawn_index"] == 1
    assert (
        "route lead distance is below the maneuver-lookahead buffer"
        in rejected["failure_reasons"]
    )
    assert serialize_calls[:2] == [("left", 15), ("left", 30)]
    assert catalog["generation"]["physical_turn_contract"] == {
        "enabled": True,
        "profile_id": "speed_30kph",
        "applicability": "packaged_town_only",
        "measurement_source": "exact_serialized_3d_route_with_terminal_goal",
        "admission_policy": "reject_before_accepted_pair_quota",
        "limits": dict(module.SPEED_30KPH_TURN_GEOMETRY_CONTRACT),
        "provenance": dict(module.SPEED_30KPH_TURN_CONTRACT_PROVENANCE),
    }


def test_physical_turn_admission_skips_unsupported_grade_before_quota(
    tmp_path, monkeypatch
):
    module = load_module()
    catalog, serialize_calls = build_physical_turn_fixture(
        module,
        tmp_path,
        monkeypatch,
        first_lead_m=30,
        first_grade_ratio=0.36,
    )

    assert catalog["status"] == "complete"
    assert len(catalog["routes"]) == 1
    assert catalog["routes"][0]["goal_spawn_index"] == 2
    coverage = catalog["scenario_results"][0]["trace_coverage"]
    assert coverage["attempted"] == 2
    assert coverage["accepted"] == 1
    assert coverage["physical_turn_rejected"] == 1
    rejected = coverage["physical_turn_rejection_samples"][0]
    assert rejected["goal_spawn_index"] == 1
    assert rejected["longitudinal_grade"][
        "maximum_absolute_grade_ratio"
    ] == pytest.approx(0.36)
    assert (
        "longitudinal grade exceeds the controller capability"
        in rejected["failure_reasons"]
    )
    assert serialize_calls[:2] == [("left", 30), ("left", 30)]


def test_physical_turn_candidate_measurement_error_is_evidenced_and_skipped(
    tmp_path, monkeypatch
):
    module = load_module()
    analyze = module.analyze_serialized_physical_turn
    calls = 0

    def analyze_with_one_bad_candidate(payload, contract):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise module.PhysicalTurnGeometryError(
                "terminal elevation changes without 3D progress",
                error_code="inconsistent_3d_route_progress",
            )
        return analyze(payload, contract)

    monkeypatch.setattr(
        module,
        "analyze_serialized_physical_turn",
        analyze_with_one_bad_candidate,
    )

    catalog, _serialize_calls = build_physical_turn_fixture(
        module, tmp_path, monkeypatch
    )

    assert catalog["status"] == "complete"
    assert catalog["routes"][0]["goal_spawn_index"] == 2
    coverage = catalog["scenario_results"][0]["trace_coverage"]
    assert coverage["physical_turn_rejected"] == 1
    sample = coverage["physical_turn_rejection_samples"][0]
    assert sample["goal_spawn_index"] == 1
    assert sample["failure_reasons"] == [
        "terminal elevation changes without 3D progress"
    ]
    assert sample["analysis_error"] == {
        "error_type": "PhysicalTurnGeometryError",
        "error_scope": "candidate",
        "error_code": "inconsistent_3d_route_progress",
        "fatal": False,
        "message": "terminal elevation changes without 3D progress",
    }


@pytest.mark.parametrize("error_scope", ("contract", "schema", "nonfinite"))
def test_physical_turn_contract_schema_and_nonfinite_errors_remain_fatal(
    tmp_path, monkeypatch, error_scope
):
    module = load_module()

    def fatal_analysis(_payload, _contract):
        raise module.PhysicalTurnGeometryError(
            f"{error_scope} physical-turn defect",
            error_scope=error_scope,
            error_code=f"{error_scope}_defect",
        )

    monkeypatch.setattr(module, "analyze_serialized_physical_turn", fatal_analysis)

    with pytest.raises(
        module.CatalogError,
        match=rf"failed fatally \({error_scope}\)",
    ):
        build_physical_turn_fixture(module, tmp_path, monkeypatch)


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
        "scenarios": ("straight",),
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
        assert catalog["generation"]["scenarios"] == ["straight"]
        assert [item["scenario"] for item in catalog["scenario_results"]] == [
            "straight"
        ]
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
                    # Model the small waypoint/exact-serialized yaw difference
                    # that exposed strict provenance drift on C_track_1_0_7.
                    "yaw": -math.radians(
                        waypoint.transform.rotation.yaw
                        + (2.0e-5 if index == 29 else 0.0)
                    ),
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
        "scenarios": ("left",),
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

    geometry_contract = module.turn_geometry_contract(turn_geometry_args())
    for name, catalog in zip(("first", "repeated"), catalogs):
        assert catalog["generation"]["scenarios"] == ["left"]
        assert [item["scenario"] for item in catalog["scenario_results"]] == [
            "left"
        ]
        assert catalog["routes"][0]["goal_spawn_index"] == 2
        assert catalog["routes"][0]["turn_geometry_preflight"]["status"] == "PASS"
        route_entry = catalog["routes"][0]
        route_payload = json.loads(
            ((tmp_path / name) / route_entry["path"]).read_text(encoding="utf-8")
        )
        serialized_geometry = module.analyze_serialized_custom_turn(
            route_payload["route"], "left", geometry_contract
        )
        raw_geometry = module.analyze_turn_geometry(
            compact_left_route() + [route_point(80.0, yaw=90.0)],
            "left",
            geometry_contract,
        )
        assert route_entry["turn_geometry_preflight"] == serialized_geometry
        assert route_payload["turn_geometry_preflight"] == serialized_geometry
        assert raw_geometry != serialized_geometry
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
        [route_point(0.0), route_point(30.0)],
    )

    assert payload["coordinate_reference"] == "base_link"
    assert payload["spawn_point_reference"] == "base_link"
    assert payload["town"] == "Town01"
    assert payload["route_length_m"] == 30.0
    assert payload["option_counts"] == {"LANEFOLLOW": 2}
    assert "endpoint_source" not in payload
    assert (payload["start_spawn_index"], payload["goal_spawn_index"]) == (0, 1)
    assert payload["goal_ros_pose"]["z"] == pytest.approx(0.0)
    assert payload["goal_carla_transform"]["z"] == pytest.approx(0.0)
    assert payload["route"][-1]["z"] == pytest.approx(0.0)
    goal_provenance = payload["goal_endpoint_provenance"]
    assert goal_provenance["endpoint_source"] == "spawn_points"
    assert goal_provenance["original_goal_carla_transform"]["z"] == pytest.approx(
        0.0
    )
    assert goal_provenance["terminal_z_normalization"]["policy"] == (
        "last_road_waypoint_z"
    )


def test_route_payload_normalizes_spawn_goal_z_and_preserves_raw_provenance():
    module = load_module()
    spawns = [Transform(0.0, z=0.5), Transform(30.0, z=0.5, yaw=37.0)]
    road_route = [
        route_point(0.0, z=0.0),
        route_point(30.0, z=0.0, yaw=37.0),
    ]
    serialized = [
        {
            "index": 0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "distance_m": 0.0,
            "road_option": "LANEFOLLOW",
        },
        {
            "index": 1,
            "x": 30.0,
            "y": 0.0,
            "z": 0.5,
            "yaw": -math.radians(37.0),
            "distance_m": 30.0,
            "road_option": "LANEFOLLOW",
        },
    ]
    helper = SimpleNamespace(
        serialize_route=lambda _route, _goal: serialized,
        transform_dict=lambda transform: {
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z,
            "roll": transform.rotation.roll,
            "pitch": transform.rotation.pitch,
            "yaw": transform.rotation.yaw,
        },
        ros_pose_dict=lambda transform: {
            "x": transform.location.x,
            "y": -transform.location.y,
            "z": transform.location.z,
            "yaw": -math.radians(transform.rotation.yaw),
        },
    )

    payload = module._route_payload(
        helper,
        {"canonical_name": "Town02_Opt"},
        "ClearNoon",
        "lane_follow",
        1.0,
        0,
        1,
        spawns,
        road_route,
    )

    assert payload["goal_carla_transform"] == pytest.approx(
        {"x": 30.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 37.0}
    )
    assert payload["goal_ros_pose"] == pytest.approx(
        {"x": 30.0, "y": 0.0, "z": 0.0, "yaw": -math.radians(37.0)}
    )
    assert payload["route"][-1]["z"] == pytest.approx(0.0)
    provenance = payload["goal_endpoint_provenance"]
    assert provenance["original_goal_carla_transform"]["z"] == pytest.approx(0.5)
    assert provenance["original_goal_ros_pose"]["z"] == pytest.approx(0.5)
    normalization = provenance["terminal_z_normalization"]
    assert normalization["policy"] == "last_road_waypoint_z"
    assert {
        key: value for key, value in normalization.items() if key != "policy"
    } == pytest.approx(
        {
            "original_endpoint_z_m": 0.5,
            "last_road_waypoint_z_m": 0.0,
            "runtime_goal_z_m": 0.0,
            "serialized_terminal_z_m": 0.0,
            "applied_offset_m": -0.5,
        }
    )


def test_generated_waypoint_endpoints_are_deduplicated_finite_and_stably_ordered():
    module = load_module()

    class Map:
        def __init__(self, waypoints):
            self.waypoints = waypoints
            self.spacings = []

        def generate_waypoints(self, spacing):
            self.spacings.append(spacing)
            return list(self.waypoints)

    ordered = module.generated_waypoint_endpoint_transforms(
        Map(
            [
                SimpleNamespace(transform=Transform(30.0, yaw=90.0)),
                SimpleNamespace(transform=Transform(-10.0, yaw=-90.0)),
                SimpleNamespace(transform=Transform(5.0, yaw=0.0)),
            ]
        ),
        10.0,
    )
    assert [transform.location.x for transform in ordered] == [-10.0, 5.0, 30.0]

    duplicate = SimpleNamespace(transform=Transform(5.0, yaw=0.0))
    deduplicated, provenance = module._generated_waypoint_endpoint_pool(
        Map(
            [
                duplicate,
                SimpleNamespace(transform=Transform(5.0, yaw=0.0)),
                SimpleNamespace(transform=Transform(15.0, yaw=0.0)),
            ]
        ),
        10.0,
    )
    assert [transform.location.x for transform in deduplicated] == [5.0, 15.0]
    assert provenance == {
        "api_count": 3,
        "junction_policy": "include",
        "junction_waypoint_count": 0,
        "junction_excluded_count": 0,
        "eligible_api_count": 3,
        "duplicate_transform_count": 1,
    }
    with pytest.raises(module.CatalogError, match="must be finite"):
        module.generated_waypoint_endpoint_transforms(
            Map(
                [
                    SimpleNamespace(transform=Transform(float("nan"))),
                    SimpleNamespace(transform=Transform(5.0)),
                ]
            ),
            10.0,
        )


def test_waypoint_route_payload_uses_generic_indices_and_bridge_owned_z_offset():
    module = load_module()
    endpoints = [Transform(0.0, z=0.25), Transform(30.0, z=0.25)]
    points = [
        {
            "index": 0,
            "x": 0.0,
            "y": 0.0,
            "distance_m": 0.0,
            "road_option": "LANEFOLLOW",
        },
        {
            "index": 1,
            "x": 30.0,
            "y": 0.0,
            "distance_m": 30.0,
            "road_option": "STRAIGHT",
        },
    ]
    helper = SimpleNamespace(
        serialize_route=lambda route, goal: points,
        transform_dict=lambda transform: {
            "x": transform.location.x,
            "z": transform.location.z,
        },
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )

    payload = module._route_payload(
        helper,
        {"canonical_name": "Town01"},
        "ClearNoon",
        "straight",
        1.0,
        0,
        1,
        endpoints,
        [route_point(0.0, z=0.25), route_point(30.0, z=0.25)],
        endpoint_source=module.GENERATED_WAYPOINT_ENDPOINT_SOURCE,
        endpoint_waypoint_spacing_m=10.0,
    )

    assert payload["endpoint_source"] == "generated_waypoints"
    assert (payload["start_endpoint_index"], payload["goal_endpoint_index"]) == (
        0,
        1,
    )
    assert "start_spawn_index" not in payload
    assert "goal_spawn_index" not in payload
    assert payload["spawn_point"].split(",")[2] == "0.250000"
    assert payload["goal_ros_pose"]["z"] == pytest.approx(0.25)
    assert payload["goal_carla_transform"]["z"] == pytest.approx(0.25)
    assert payload["route"][-1]["z"] == pytest.approx(0.25)
    goal_provenance = payload["goal_endpoint_provenance"]
    assert goal_provenance["endpoint_source"] == "generated_waypoints"
    assert goal_provenance["endpoint_index"] == 1
    assert goal_provenance["original_goal_ros_pose"]["z"] == pytest.approx(0.25)
    height = payload["spawn_height_contract"]
    assert height["catalog_z_offset_m"] == pytest.approx(0.0)
    assert height["bridge_z_offset_m"] == pytest.approx(2.0)
    assert height["actor_spawn_z_before_base_link_to_center_shift_m"] == (
        pytest.approx(2.25)
    )
    assert "pitched transforms" in height["base_link_to_center_shift"]
    assert height["bridge_source"].endswith(
        "InitializeInterface._parse_spawn_point"
        )


def test_generated_waypoint_endpoint_junction_exclusion_is_fail_closed():
    module = load_module()

    class Map:
        @staticmethod
        def generate_waypoints(spacing):
            assert spacing == pytest.approx(0.5)
            return [
                SimpleNamespace(transform=Transform(0.0), is_junction=False),
                SimpleNamespace(transform=Transform(10.0), is_junction=True),
                SimpleNamespace(transform=Transform(20.0), is_junction=False),
            ]

    endpoints, waypoints, provenance = module._generated_waypoint_endpoint_records(
        Map(), 0.5, "exclude"
    )

    assert [item.location.x for item in endpoints] == [0.0, 20.0]
    assert [item.transform.location.x for item in waypoints] == [0.0, 20.0]
    assert provenance == {
        "api_count": 3,
        "junction_policy": "exclude",
        "junction_waypoint_count": 1,
        "junction_excluded_count": 1,
        "eligible_api_count": 2,
        "duplicate_transform_count": 0,
    }

    class MissingJunctionFlagMap:
        @staticmethod
        def generate_waypoints(_spacing):
            return [
                SimpleNamespace(transform=Transform(0.0)),
                SimpleNamespace(transform=Transform(20.0), is_junction=False),
            ]

    with pytest.raises(module.CatalogError, match="lacks boolean is_junction"):
        module._generated_waypoint_endpoint_records(
            MissingJunctionFlagMap(), 0.5, "exclude"
        )


def test_directed_topology_straight_prefilter_is_deterministic_and_reachable():
    module = load_module()
    endpoints = [
        Transform(0.0, yaw=0.0),
        Transform(170.0, yaw=0.0),
        Transform(171.0, yaw=0.0),
        Transform(-170.0, yaw=180.0),
    ]
    graph = module.nx.DiGraph()
    graph.add_edges_from(((0, 1), (1, 2), (9, 10), (11, 12)))
    localized = {
        0.0: (0, 1),
        170.0: (1, 2),
        171.0: (9, 10),
        -170.0: (11, 12),
    }
    planner = SimpleNamespace(
        _graph=graph,
        _localize=lambda location: localized[location.x],
    )

    first, report = module.deterministic_directed_topology_straight_pairs(
        endpoints,
        planner,
        0,
        170.0,
        182.0,
        172.0,
        1.0,
        module.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
    )
    second, second_report = module.deterministic_directed_topology_straight_pairs(
        endpoints,
        planner,
        0,
        170.0,
        182.0,
        172.0,
        1.0,
        module.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
    )

    assert first == second == [(0, 1)]
    assert report == second_report
    assert report["planar_chord_heading_candidate_count"] == 2
    assert report["directed_reachable_candidate_count"] == 1
    assert report["postfilter_authority"] == (
        "exact serialized route distance and physical-straight analysis"
    )


def test_catalog_records_generated_waypoint_endpoint_provenance(
    tmp_path: Path, monkeypatch
) -> None:
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    generated = [
        SimpleNamespace(transform=Transform(220.0, z=0.25)),
        SimpleNamespace(transform=Transform(0.0, z=0.25)),
    ]
    monkeypatch.setattr(
        module,
        "deterministic_pairs",
        lambda *_args, **_kwargs: [(0, 1)],
    )

    class Map:
        name = "/Game/Carla/Maps/Town01"

        @staticmethod
        def get_spawn_points():
            return [Transform(1000.0, z=0.75), Transform(2000.0, z=0.75)]

        @staticmethod
        def generate_waypoints(spacing):
            assert spacing == pytest.approx(10.0)
            return generated

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
                route_point(start.x),
                route_point(goal.x, "STRAIGHT"),
            ]

    helper = SimpleNamespace(
        GlobalRoutePlanner=Planner,
        route_matches=lambda route, scenario: scenario == "straight"
        and any(option == "STRAIGHT" for _, option in route),
        route_length=lambda route: route[0][0].transform.location.distance(
            route[-1][0].transform.location
        ),
        serialize_route=lambda route, goal: [
            {
                "index": 0,
                "x": route[0][0].transform.location.x,
                "y": 0.0,
                "distance_m": 0.0,
                "road_option": "LANEFOLLOW",
            },
            {
                "index": 1,
                "x": route[-1][0].transform.location.x,
                "y": 0.0,
                "distance_m": route[0][0].transform.location.distance(
                    route[-1][0].transform.location
                ),
                "road_option": "STRAIGHT",
            },
        ],
        transform_dict=lambda transform: {
            "x": transform.location.x,
            "z": transform.location.z,
        },
        ros_pose_dict=lambda transform: {"x": transform.location.x, "y": 0.0},
    )
    args = SimpleNamespace(
        map_id="town01",
        host="127.0.0.1",
        port=2100,
        timeout=3.0,
        output_root=tmp_path,
        weather="ClearNoon",
        seeds=(0,),
        scenarios=("straight",),
        pairs_per_seed=1,
        min_distance=200.0,
        max_distance=260.0,
        preferred_distance=210.0,
        sampling_resolution=1.0,
        endpoint_waypoint_spacing_m=10.0,
        max_endpoint_offset=2.0,
        max_traces=20,
        exclude_spawn_indices=(),
        map_load_settle_sec=0.0,
        allow_map_load=False,
        active_server_profile="packaged_0915",
    )

    catalog = module.build_catalog(
        args, manifest, SimpleNamespace(Client=Client), helper
    )

    generation = catalog["generation"]
    assert generation["endpoint_source"] == "generated_waypoints"
    assert generation["endpoint_waypoint_spacing_m"] == pytest.approx(10.0)
    assert generation["endpoint_count"] == 2
    assert generation["spawn_point_count"] == 2
    assert "eligible_spawn_point_count" not in generation
    assert generation["spawn_height_contract"]["bridge_z_offset_m"] == (
        pytest.approx(2.0)
    )
    route_entry = catalog["routes"][0]
    assert (route_entry["start_endpoint_index"], route_entry["goal_endpoint_index"]) == (
        0,
        1,
    )
    assert "start_spawn_index" not in route_entry
    payload = json.loads((tmp_path / route_entry["path"]).read_text())
    assert payload["start_carla_transform"]["x"] == pytest.approx(0.0)
    assert payload["spawn_point"].startswith("0.000000,")

    incompatible = SimpleNamespace(**vars(args))
    incompatible.exclude_spawn_indices = (0,)
    with pytest.raises(module.CatalogError, match="cannot be used with generated"):
        module.build_catalog(
            incompatible, manifest, SimpleNamespace(Client=Client), helper
        )


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
    assert catalog["generation"]["scenarios"] == list(module.SCENARIOS)
    assert catalog["generation"]["endpoint_source"] == "spawn_points"
    assert catalog["generation"]["endpoint_waypoint_spacing_m"] is None
    assert catalog["generation"]["endpoint_count"] == 4
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
    assert args.scenarios == module.SCENARIOS
    assert args.endpoint_waypoint_spacing_m is None
    assert args.physical_straight_profile is None

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


def test_cli_waypoint_endpoint_spacing_is_opt_in_positive_and_finite():
    module = load_module()
    args = module.parse_args(
        [
            "--map-id",
            "town01",
            "--output-root",
            "/tmp/expert-routes",
            "--endpoint-waypoint-spacing-m",
            "10.0",
        ]
    )

    assert args.endpoint_waypoint_spacing_m == pytest.approx(10.0)
    for invalid in ("0", "-1", "nan", "inf"):
        with pytest.raises(SystemExit):
            module.parse_args(
                [
                    "--map-id",
                    "town01",
                    "--output-root",
                    "/tmp/expert-routes",
                    "--endpoint-waypoint-spacing-m",
                    invalid,
                ]
            )


def test_cli_physical_straight_profile_is_explicit_and_straight_only():
    module = load_module()
    args = module.parse_args(
        [
            "--map-id",
            "town01",
            "--output-root",
            "/tmp/expert-routes",
            "--scenarios",
            "straight",
            "--physical-straight-profile",
            "speed_30kph",
        ]
    )

    assert args.physical_straight_profile == "speed_30kph"
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "--map-id",
                "town01",
                "--output-root",
                "/tmp/expert-routes",
                "--physical-straight-profile",
                "speed_30kph",
            ]
        )


def test_cli_60kph_physical_straight_profile_reuses_geometry_with_distinct_id():
    module = load_module()
    args = module.parse_args(
        [
            "--map-id",
            "town06",
            "--output-root",
            "/tmp/expert-routes",
            "--scenarios",
            "straight",
            "--physical-straight-profile",
            "speed_60kph_straight_pilot",
        ]
    )

    contract = module.physical_straight_contract(args)
    assert contract["profile_id"] == "speed_60kph_straight_pilot"
    assert contract["limits"] == module.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT


def test_cli_town10_compact_capacity_profile_is_exact_and_fail_closed():
    module = load_module()
    exact = [
        "--map-id",
        "town10hd_opt",
        "--output-root",
        "/tmp/expert-routes",
        "--weather",
        "ClearNoon",
        "--scenarios",
        "straight",
        "--seeds",
        "0",
        "--pairs-per-seed",
        "8",
        "--min-distance",
        "170",
        "--max-distance",
        "182",
        "--preferred-distance",
        "172",
        "--sampling-resolution",
        "1",
        "--endpoint-waypoint-spacing-m",
        "0.5",
        "--endpoint-junction-policy",
        "exclude",
        "--candidate-enumeration-policy",
        "directed_topology_straight_v1",
        "--straight-capacity-profile",
        "town10hd_opt_30kph_compact_v1",
        "--physical-straight-profile",
        "speed_30kph",
        "--max-endpoint-offset",
        "2",
        "--max-traces",
        "20000",
    ]
    args = module.parse_args(exact)
    capacity = module.straight_capacity_contract(args)

    assert capacity["enabled"] is True
    assert capacity["provenance"]["derived_required_distance_m"] == pytest.approx(
        167.534
    )
    assert capacity["provenance"]["minimum_route_margin_m"] == pytest.approx(
        2.466
    )
    assert capacity["provenance"]["validation_threshold_reuse"] is False

    for old, replacement in (
        ("town10hd_opt", "town01"),
        ("182", "183"),
        ("0.5", "10"),
        ("exclude", "include"),
    ):
        drifted = list(exact)
        drifted[drifted.index(old)] = replacement
        with pytest.raises(SystemExit):
            module.parse_args(drifted)


def test_cli_town06_60kph_capacity_profile_is_exact_and_fail_closed():
    module = load_module()
    exact = [
        "--map-id",
        "town06",
        "--output-root",
        "/tmp/expert-routes",
        "--weather",
        "ClearNoon",
        "--scenarios",
        "straight",
        "--seeds",
        "0",
        "--pairs-per-seed",
        "1",
        "--min-distance",
        "430",
        "--max-distance",
        "460",
        "--preferred-distance",
        "445",
        "--sampling-resolution",
        "1",
        "--endpoint-waypoint-spacing-m",
        "0.5",
        "--endpoint-junction-policy",
        "exclude",
        "--candidate-enumeration-policy",
        "directed_topology_straight_v1",
        "--straight-capacity-profile",
        "town06_60kph_straight_pilot_v1",
        "--physical-straight-profile",
        "speed_60kph_straight_pilot",
        "--max-endpoint-offset",
        "2",
        "--max-traces",
        "20000",
    ]
    args = module.parse_args(exact)
    capacity = module.straight_capacity_contract(args)

    assert capacity == {
        "enabled": True,
        "profile_id": "town06_60kph_straight_pilot_v1",
        "map_id": "town06",
        "scenario": "straight",
        "endpoint_waypoint_spacing_m": 0.5,
        "endpoint_junction_policy": "exclude",
        "candidate_enumeration_policy": "directed_topology_straight_v1",
        "admission_policy": "prefilter_then_exact_serialized_physical_postfilter",
        "provenance": module.TOWN06_60KPH_STRAIGHT_CAPACITY_PROVENANCE,
    }
    provenance = capacity["provenance"]
    assert provenance["simulation_only"] is True
    assert provenance["real_vehicle_ready"] is False
    assert provenance["calibration_claim"] is False
    assert provenance["target_speed_mps"] == 16.666666666666668
    assert provenance["minimum_sustained_speed_mps"] == 15.0
    assert provenance["minimum_sustained_speed_duration_sec"] == 1.0
    assert provenance["observed_30kph_threshold_entry_distance_m"] == 73.58
    assert provenance["threshold_entry_distance_m"] == 294.32
    assert provenance["minimum_exposure_distance_m"] == 15.0
    assert provenance["assumed_deceleration_mps2"] == 2.0
    assert provenance["minimum_stop_distance_m"] == 69.444
    assert provenance["derived_required_distance_m"] == 378.764
    assert provenance["minimum_route_margin_m"] == 51.236
    assert "not a vehicle or control calibration claim" in provenance["claim_limit"]

    def replace_option(arguments, option, replacement):
        drifted = list(arguments)
        drifted[drifted.index(option) + 1] = replacement
        return drifted

    for option, replacement in (
        ("--map-id", "town01"),
        ("--weather", "CloudyNoon"),
        ("--scenarios", "lane_follow"),
        ("--seeds", "1"),
        ("--pairs-per-seed", "2"),
        ("--min-distance", "431"),
        ("--max-distance", "461"),
        ("--preferred-distance", "446"),
        ("--sampling-resolution", "2"),
        ("--endpoint-waypoint-spacing-m", "1.0"),
        ("--endpoint-junction-policy", "include"),
        ("--candidate-enumeration-policy", "all_pairs"),
        ("--physical-straight-profile", "speed_30kph"),
        ("--max-endpoint-offset", "3"),
        ("--max-traces", "19999"),
    ):
        with pytest.raises(SystemExit):
            module.parse_args(replace_option(exact, option, replacement))


def test_cli_physical_turn_profile_is_explicit_and_packaged_turn_only():
    module = load_module()
    args = module.parse_args(
        [
            "--map-id",
            "town01",
            "--output-root",
            "/tmp/expert-routes",
            "--scenarios",
            "left,right",
            "--physical-turn-profile",
            "speed_30kph",
        ]
    )

    assert args.physical_turn_profile == "speed_30kph"
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "--map-id",
                "town01",
                "--output-root",
                "/tmp/expert-routes",
                "--physical-turn-profile",
                "speed_30kph",
            ]
        )
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "--map-id",
                "town01",
                "--output-root",
                "/tmp/expert-routes",
                "--scenarios",
                "left,right",
                "--physical-turn-profile",
                "speed_30kph",
                "--initial-approach-distance-m",
                "15",
                "--maximum-initial-lateral-deviation-m",
                "1.5",
                "--maximum-initial-heading-change-deg",
                "30",
            ]
        )


def test_cli_scenario_filter_is_explicit_unique_and_ordered():
    module = load_module()

    args = module.parse_args(
        [
            "--map-id",
            "town01",
            "--output-root",
            "/tmp/expert-routes",
            "--scenarios",
            "left,right",
        ]
    )

    assert args.scenarios == ("left", "right")
    for invalid in ("", "straight,straight", "straight,uturn"):
        with pytest.raises(SystemExit):
            module.parse_args(
                [
                    "--map-id",
                    "town01",
                    "--output-root",
                    "/tmp/expert-routes",
                    "--scenarios",
                    invalid,
                ]
            )


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
