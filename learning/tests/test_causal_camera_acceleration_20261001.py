"""HH_261001 - Exact planar pose correction, reference-point recovery and causal first-frame behavior."""

import copy
import math

import pytest

from scripts.e2e.causal_camera_acceleration_research_20261001 import camera_acceleration, rotate


def state(t,yaw,world,omega=0.,ax=7.,ay=8.):
    x,y=rotate(world,-yaw)
    return dict(timestamp=t,yaw=yaw,vx=x,vy=y-1.425*omega,yaw_rate=omega,ax=ax,ay=ay)


def test_pose_rotation_and_rear_axle_offset_do_not_invent_acceleration():
    before=state(0.,0.,(3.,1.),omega=.3)
    now=state(.1,.03,(3.,1.),omega=.7)
    snapshot=copy.deepcopy((before,now))
    result=camera_acceleration(now,before,2.85)
    assert result["ax"] == pytest.approx(0.,abs=1e-12)
    assert result["ay"] == pytest.approx(0.,abs=1e-12)
    assert (before,now) == snapshot


def test_world_acceleration_is_expressed_in_current_body_without_yaw_rate_approximation():
    before=state(0.,0.,(3.,1.))
    now=state(.1,.2,(3.2,1.1))
    expected=rotate((2.,1.),-.2)
    got=camera_acceleration(now,before,2.85)
    assert (got["ax"],got["ay"]) == pytest.approx(expected)


def test_first_observation_preserves_original_and_future_or_gap_fails():
    now=state(0.,0.,(1.,0.))
    result=camera_acceleration(now,None,2.85)
    assert result["ax"] == 7. and result["ay"] == 8. and not result["transformed"]
    with pytest.raises(ValueError):
        camera_acceleration(now,state(.1,0.,(1.,0.)),2.85)
    with pytest.raises(ValueError):
        camera_acceleration(state(.3,0.,(1.,0.)),now,2.85)
