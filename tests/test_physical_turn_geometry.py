from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts/e2e/physical_turn_geometry.py"
TOWN03_GRADE_FIXTURE = (
    Path(__file__).parent / "fixtures/town03_v3_left_turn_grade_routes.json"
)
SPEC = importlib.util.spec_from_file_location("physical_turn_geometry", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
turn_geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(turn_geometry)


def make_turn_payload(
    scenario: str = "left",
    *,
    lead_m: int = 30,
    tail_m: int = 15,
    radius_m: float = 10.0,
    turn_samples: int = 17,
) -> dict:
    sign = 1.0 if scenario == "left" else -1.0
    points = []
    distance_m = 0.0

    def append(
        x: float,
        y: float,
        yaw: float,
        road_option: str,
        z: float = 0.0,
    ) -> None:
        nonlocal distance_m
        if points:
            distance_m += math.hypot(x - points[-1]["x"], y - points[-1]["y"])
        points.append(
            {
                "index": len(points),
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "distance_m": distance_m,
                "road_option": road_option,
            }
        )

    for x in range(lead_m):
        append(float(x), 0.0, 0.0, "LANEFOLLOW")
    center_x = float(lead_m - 1)
    center_y = sign * radius_m
    for sample in range(1, turn_samples + 1):
        fraction = sample / turn_samples
        angle = -sign * math.pi / 2.0 + sign * fraction * math.pi / 2.0
        x = center_x + radius_m * math.cos(angle)
        y = center_y + radius_m * math.sin(angle)
        yaw = sign * fraction * math.pi / 2.0
        append(x, y, yaw, scenario.upper())
    for offset in range(1, tail_m + 1):
        append(
            center_x + radius_m,
            center_y + sign * offset,
            sign * math.pi / 2.0,
            "LANEFOLLOW",
        )
    return {
        "scenario": scenario,
        "route_length_m": points[-1]["distance_m"],
        "route": points,
    }


def apply_constant_3d_grade(payload: dict, grade_ratio: float) -> None:
    """Set dz/ds while keeping distance_m on the exact 3D progress axis."""
    planar_progress = 0.0
    spatial_progress = 0.0
    planar_slope = grade_ratio / math.sqrt(1.0 - grade_ratio**2)
    previous = None
    for point in payload["route"]:
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
    payload["route_length_m"] = spatial_progress


@pytest.mark.parametrize("scenario", ("left", "right"))
def test_speed_30kph_turn_contract_accepts_isolated_turn_with_buffered_ends(
    scenario: str,
) -> None:
    result = turn_geometry.analyze_serialized_physical_turn(
        make_turn_payload(scenario),
        turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
    )

    assert result["status"] == "PASS"
    assert result["directional_command"] == scenario.upper()
    assert result["directional_block_count"] == 1
    assert result["additional_maneuver_commands"] == []
    assert result["initial_command_contract"] == {
        "required_command": "LANEFOLLOW",
        "maneuver_lookahead_m": 4.0,
        "lead_buffer_m": 5.0,
        "minimum_route_lead_m": 25.0,
    }
    assert result["selected_block"]["route_lead_distance_m"] >= 25.0
    assert result["selected_block"]["route_tail_distance_m"] >= 10.0
    assert result["longitudinal_grade"]["status"] == "PASS"
    assert result["longitudinal_grade"][
        "maximum_absolute_grade_ratio"
    ] == pytest.approx(0.0)
    assert result["longitudinal_grade"]["real_vehicle_ready"] is False
    assert result["contract_provenance"]["real_vehicle_ready"] is False


def test_turn_contract_rejects_command_inside_initial_maneuver_lookahead() -> None:
    result = turn_geometry.analyze_serialized_physical_turn(
        make_turn_payload(lead_m=19),
        turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
    )

    assert result["status"] == "FAIL"
    assert result["selected_block"]["route_lead_distance_m"] < 20.0
    assert (
        "route lead distance is below the maneuver-lookahead buffer"
        in result["failure_reasons"]
    )


def test_turn_contract_rejects_missing_recovery_tail() -> None:
    result = turn_geometry.analyze_serialized_physical_turn(
        make_turn_payload(tail_m=0),
        turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
    )

    assert result["status"] == "FAIL"
    assert result["selected_block"]["route_tail_distance_m"] == pytest.approx(0.0)
    assert (
        "route tail distance is below the post-turn recovery minimum"
        in result["failure_reasons"]
    )


def test_exact_serialized_terminal_yaw_glue_is_measured() -> None:
    payload = make_turn_payload(tail_m=0)
    last = payload["route"][-1]
    payload["route"].append(
        {
            "index": len(payload["route"]),
            "x": last["x"],
            "y": last["y"] + 0.2,
            "z": last["z"],
            "yaw": last["yaw"] + 1.0,
            "distance_m": last["distance_m"] + 0.2,
            "road_option": "LEFT",
        }
    )
    payload["route_length_m"] = payload["route"][-1]["distance_m"]

    result = turn_geometry.analyze_serialized_physical_turn(
        payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    assert result["status"] == "FAIL"
    assert result["selected_block"]["route_tail_distance_m"] == pytest.approx(0.0)
    assert result["selected_block"]["maximum_absolute_curvature_per_m"] > 4.9
    assert "turn peak absolute curvature exceeds the maximum" in result[
        "failure_reasons"
    ]


def test_turn_contract_rejects_additional_or_disjoint_maneuvers() -> None:
    payload = make_turn_payload()
    payload["route"][5]["road_option"] = "CHANGELANELEFT"
    payload["route"][10]["road_option"] = "LEFT"

    result = turn_geometry.analyze_serialized_physical_turn(
        payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    assert result["status"] == "FAIL"
    assert result["directional_block_count"] == 2
    assert result["additional_maneuver_commands"] == ["CHANGELANELEFT"]


def test_turn_contract_rejects_heading_opposite_to_command() -> None:
    payload = make_turn_payload("right")
    for point in payload["route"]:
        point["yaw"] *= -1.0

    result = turn_geometry.analyze_serialized_physical_turn(
        payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    assert result["status"] == "FAIL"
    assert (
        "turn heading direction disagrees with the directional command"
        in result["failure_reasons"]
    )


def test_turn_contract_fails_closed_on_bad_payload_or_contract() -> None:
    payload = make_turn_payload()
    payload["route"][3]["distance_m"] = -1.0
    with pytest.raises(
        turn_geometry.PhysicalTurnGeometryError, match="distance_m is not monotonic"
    ) as candidate_error:
        turn_geometry.analyze_serialized_physical_turn(
            payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
        )
    assert candidate_error.value.fatal is False
    assert candidate_error.value.error_code == "non_monotonic_route_progress"

    bad_contract = dict(turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
    bad_contract["minimum_route_lead_m"] = 8.0
    with pytest.raises(
        turn_geometry.PhysicalTurnGeometryError,
        match="cover maneuver lookahead plus lead buffer",
    ) as contract_error:
        turn_geometry.analyze_serialized_physical_turn(
            make_turn_payload(), bad_contract
        )
    assert contract_error.value.fatal is True
    assert contract_error.value.error_scope == "contract"


def test_turn_contract_rejects_grade_outside_compensation_envelope() -> None:
    payload = make_turn_payload()
    apply_constant_3d_grade(payload, 0.36)

    result = turn_geometry.analyze_serialized_physical_turn(
        payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    grade = result["longitudinal_grade"]
    assert result["status"] == "FAIL"
    assert grade["status"] == "FAIL"
    assert grade["window_length_m"] == pytest.approx(5.0)
    assert grade["maximum_absolute_grade_ratio"] == pytest.approx(0.36)
    assert grade["maximum_allowed_absolute_grade_ratio"] == pytest.approx(
        math.sin(0.1)
    )
    assert grade["capability"][
        "maximum_pre_gate_total_acceleration_mps2"
    ] == pytest.approx(1.5 + 9.81 * math.sin(0.1))
    assert grade["capability"][
        "downstream_vehicle_cmd_gate_acceleration_cap_mps2"
    ] == pytest.approx(1.5)
    assert grade["capability"][
        "ideal_net_uphill_acceleration_reserve_mps2"
    ] == pytest.approx(1.5 - 9.81 * math.sin(0.1))
    assert grade["capability"]["real_vehicle_ready"] is False
    assert (
        "longitudinal grade exceeds the controller capability"
        in result["failure_reasons"]
    )


def test_turn_contract_fails_closed_when_serialized_z_is_missing() -> None:
    payload = make_turn_payload()
    del payload["route"][4]["z"]

    with pytest.raises(
        turn_geometry.PhysicalTurnGeometryError,
        match="lacks finite x/y/z/yaw/distance_m",
    ) as error:
        turn_geometry.analyze_serialized_physical_turn(
            payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
        )
    assert error.value.fatal is True
    assert error.value.error_scope == "nonfinite"


def test_turn_contract_fails_closed_on_serialized_schema_error() -> None:
    payload = make_turn_payload()
    del payload["route"][4]["road_option"]

    with pytest.raises(
        turn_geometry.PhysicalTurnGeometryError,
        match="lacks road_option",
    ) as error:
        turn_geometry.analyze_serialized_physical_turn(
            payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
        )

    assert error.value.fatal is True
    assert error.value.error_scope == "schema"
    assert error.value.error_code == "invalid_serialized_route_schema"


def test_exact_pitch_boundary_uses_3d_sine_grade_without_relaxation() -> None:
    payload = make_turn_payload()
    boundary = math.sin(0.1)
    apply_constant_3d_grade(payload, boundary)

    accepted = turn_geometry.analyze_serialized_physical_turn(
        payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    grade = accepted["longitudinal_grade"]
    assert accepted["status"] == "PASS"
    assert grade["status"] == "PASS"
    assert grade["maximum_absolute_grade_ratio"] == pytest.approx(
        boundary, abs=1.0e-12
    )
    assert grade["maximum_allowed_absolute_grade_ratio"] == pytest.approx(
        boundary, abs=1.0e-12
    )

    above = make_turn_payload()
    apply_constant_3d_grade(above, boundary + 1.0e-6)
    rejected = turn_geometry.analyze_serialized_physical_turn(
        above, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )
    assert rejected["longitudinal_grade"]["status"] == "FAIL"


def test_serialized_distance_m_must_equal_3d_point_distance() -> None:
    payload = make_turn_payload()
    payload["route"][-1]["z"] = 0.5

    with pytest.raises(
        turn_geometry.PhysicalTurnGeometryError,
        match="distance_m is not the 3D point distance",
    ) as error:
        turn_geometry.analyze_serialized_physical_turn(
            payload, turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
        )

    assert error.value.fatal is False
    assert error.value.evidence() == {
        "error_type": "PhysicalTurnGeometryError",
        "error_scope": "candidate",
        "error_code": "inconsistent_3d_route_progress",
        "fatal": False,
        "message": str(error.value),
    }


@pytest.mark.parametrize(
    ("route_key", "expected_grade", "expected_status"),
    (
        ("p00", 0.3546248501732128, "FAIL"),
        ("p01", 0.06111927032470703, "PASS"),
        ("p02", 0.05506142973899841, "PASS"),
        ("p04", 0.06111927032470703, "PASS"),
        ("p05", 0.06111927032470703, "PASS"),
    ),
)
def test_town03_v3_exact_route_grade_regression(
    route_key: str, expected_grade: float, expected_status: str
) -> None:
    fixture = json.loads(TOWN03_GRADE_FIXTURE.read_text(encoding="utf-8"))
    source = fixture["routes"][route_key]
    points = [
        {"distance_m": distance, "z": z}
        for distance, z in source["distance_z"]
    ]
    limits = turn_geometry._validated_contract(
        turn_geometry.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    )

    result = turn_geometry._longitudinal_grade_geometry(points, limits)

    assert source["source_route_id"] == f"town03_left_s0000_{route_key}"
    assert result["status"] == expected_status
    assert result["maximum_absolute_grade_ratio"] == pytest.approx(
        expected_grade, abs=1.0e-12
    )
    assert result["maximum_allowed_absolute_grade_ratio"] == pytest.approx(
        math.sin(0.1), abs=1.0e-12
    )
