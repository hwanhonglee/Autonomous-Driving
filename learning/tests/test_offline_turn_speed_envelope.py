"""HH_260906 - Verify offline reference bounds without importing CARLA or using a GPU."""
import math
import pytest
from scripts.e2e.offline_turn_speed_envelope import EnvelopeConfig,plan


def points(yaws):
    """HH_260906 - Build deterministic unit-distance route fixtures."""
    return [dict(distance_m=float(i),yaw=yaw) for i,yaw in enumerate(yaws)]


def test_straight_boundaries_and_acceleration():
    """HH_260906 - Preserve stop endpoints and both acceleration directions."""
    p=plan(points([0.0]*101))
    assert p['speed_mps'][0] == p['speed_mps'][-1] == 0
    assert max(p['speed_mps']) == 8
    assert all(p['checks'].values())


def test_turn_has_anticipatory_braking():
    """HH_260906 - Brake before the bend rather than applying only a local curvature clamp."""
    yaw=[0.0]*51+[0.1*i for i in range(1,17)]+[1.6]*40
    p=plan(points(yaw))
    assert p['speed_mps'][45] < 8
    assert p['speed_mps'][55] <= math.sqrt(20)+1e-9
    assert all(p['checks'].values())


def test_yaw_wrap_not_false_hairpin():
    """HH_260906 - Normalize heading transitions across the pi discontinuity."""
    p=plan(points([3.13,-3.13,-3.12]))
    assert max(p['curvature_abs_inv_m']) < .03


def test_exact_command_boundary_duplicate_retains_source_mapping():
    """HH_260906 - Deduplicate geometry locally without modifying source route or command labels."""
    route=points([0.0]*5)
    route.insert(2,dict(route[1],road_option='RIGHT'))
    p=plan(route)
    assert p['source_point_indices'] == [0,1,3,4,5]
    assert len(route) == 6


def test_conflicting_duplicate_rejected():
    """HH_260906 - Never hide a heading jump with zero arc distance."""
    route=points([0.0]*5);route.insert(2,dict(route[1],yaw=1.0))
    with pytest.raises(ValueError):plan(route)


def test_duplicate_sub_microradian_heading_noise():
    """HH_260906 - Tolerate only bounded serialization noise at an identical-position command boundary."""
    route=points([0.0]*5);route.insert(2,dict(route[1],yaw=4e-7))
    assert len(plan(route)['distance_m']) == 5


@pytest.mark.parametrize('config',[EnvelopeConfig(acceleration_mps2=0),EnvelopeConfig(lateral_mps2=-1),
    EnvelopeConfig(cruise_mps=float('nan')),EnvelopeConfig(start_mps=9),EnvelopeConfig(end_mps=-1)])
def test_bad_config_rejected(config):
    """HH_260906 - Do not silently repair impossible control limits."""
    with pytest.raises(ValueError):plan(points([0.0]*5),config)


@pytest.mark.parametrize('route', [[],points([0.0,0.0]),
    [dict(distance_m=0,yaw=0)]*3,points([0.0,float('inf'),0.0])])
def test_bad_route_rejected(route):
    """HH_260906 - Reject short, duplicate-distance and nonfinite route geometry."""
    with pytest.raises(ValueError):plan(route)
