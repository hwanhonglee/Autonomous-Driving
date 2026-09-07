"""HH_260906 - Keep wheel geometry diagnostics explicit about units and the declared spawn reference."""

from copy import deepcopy
import math

import pytest

from scripts.e2e.audit_carla_wheel_reference import analyze_case, inverse_rotate


def fixture():
    return {"case": {"case_id": "fixture"}, "actor_id": 1,
        "actor_center_spawn_carla": dict(x=100., y=-50., z=2., pitch=0., yaw=0., roll=0.),
        "vehicle_physics": {"values": {"center_of_mass": dict(x=.5, y=0., z=-.3), "wheels": [
            {"max_steer_angle": steer, "position": dict(x=(100.+x)*100, y=(-50.+y)*100, z=230.)}
            for x, y, steer in ((1.42, -.8, 70), (1.42, .8, 70), (-1.4, -.75, 0), (-1.4, .75, 0))]}}}


def test_world_centimetres_are_translated_and_rotated_without_mutation():
    report = fixture()
    original = deepcopy(report)
    result = analyze_case(report)
    assert report == original
    assert result["longitudinal_front_to_rear_separation_m"] == pytest.approx(2.82)
    assert result["declared_base_minus_rear_midpoint_x_m"] == pytest.approx(-.025)
    assert result["declared_wheelbase_minus_observed_separation_m"] == pytest.approx(.03)
    assert result["front_track_distance_m"] == pytest.approx(1.6)
    assert result["rear_track_distance_m"] == pytest.approx(1.5)
    assert result["reported_center_of_mass_parameter"] == report["vehicle_physics"]["values"]["center_of_mass"]


@pytest.mark.parametrize("vector,pose,expected", [
    ([0, 1, 0], dict(pitch=0, yaw=90, roll=0), [1, 0, 0]),
    ([0, 0, 1], dict(pitch=90, yaw=0, roll=0), [1, 0, 0]),
    ([0, -1, 0], dict(pitch=0, yaw=0, roll=90), [0, 0, -1]),
    ([1, 2, 3], dict(pitch=360, yaw=360, roll=360), [1, 2, 3]),
])
def test_full_carla_inverse_rotation(vector, pose, expected):
    assert inverse_rotate(vector, pose) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("change", ["metres", "wrong_pair", "missing", "nan", "wrong_origin"])
def test_inconsistent_units_or_reference_are_not_silently_accepted(change):
    report = fixture()
    wheels = report["vehicle_physics"]["values"]["wheels"]
    if change == "metres":
        for wheel in wheels:
            wheel["position"] = {key: value / 100 for key, value in wheel["position"].items()}
    elif change == "wrong_pair":
        wheels.reverse()
    elif change == "missing":
        wheels.pop()
    elif change == "nan":
        wheels[0]["position"]["z"] = math.nan
    else:
        report["actor_center_spawn_carla"]["x"] += 10
    with pytest.raises(ValueError):
        analyze_case(report)
