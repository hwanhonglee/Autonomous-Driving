"""HH_260906 - Test emergency priority and honest feasibility without any simulator connection."""
import copy
import pytest
from scripts.e2e.offline_turn_speed_envelope import plan
from scripts.e2e.research_speed_reference import ReferenceRequest,SpeedReference


@pytest.fixture
def reference():
    """HH_260906 - Use a deterministic long straight route with zero-speed endpoints."""
    return SpeedReference(plan([dict(distance_m=i,yaw=0.0) for i in range(101)]))


def test_launch_not_deadlocked(reference):
    """HH_260906 - Permit bounded launch at a zero-speed starting knot."""
    r=reference.update(ReferenceRequest(0,0,.1))
    assert 0<r['target_speed_mps']<=.1+1e-9
    assert r['normal_reference_allowed']


def test_emergency_not_smoothed(reference):
    """HH_260906 - Never weaken an emergency merely to satisfy comfort metrics."""
    r=reference.update(ReferenceRequest(40,8,.1,emergency=True))
    assert r['status']=='EMERGENCY_PASSTHROUGH' and r['target_speed_mps']==0
    assert not r['normal_reference_allowed']
    assert r['brake'] is None


def test_unknown_stop_is_not_clear(reference):
    """HH_260906 - Missing stop distance must not mean permission to proceed."""
    r=reference.update(ReferenceRequest(40,5,.1,stop_required=True))
    assert r['status']=='STOP_DISTANCE_UNKNOWN'
    assert not r['normal_reference_allowed']


def test_late_stop_infeasibility_is_visible(reference):
    """HH_260906 - Expose insufficient braking distance instead of claiming a comfortable stop."""
    r=reference.update(ReferenceRequest(40,8,.1,stop_distance_m=2,stop_required=True))
    assert r['status']=='REFERENCE_INFEASIBLE'
    assert r['comfort_stopping_distance_m']==32


def test_sufficient_stop_distance(reference):
    """HH_260906 - Accept a feasible early stop reference without commanding actuation."""
    r=reference.update(ReferenceRequest(40,4,.1,stop_distance_m=20,stop_required=True))
    assert r['normal_reference_allowed'] and r['throttle'] is None


def test_terminal_hold(reference):
    """HH_260906 - A stopped vehicle at the terminal knot receives a zero reference."""
    assert reference.update(ReferenceRequest(100,0,.1))['target_speed_mps']==0


@pytest.mark.parametrize('observation',[ReferenceRequest(-1,0,.1),ReferenceRequest(101,0,.1),
    ReferenceRequest(0,-1,.1),ReferenceRequest(0,0,0),ReferenceRequest(0,0,.3),
    ReferenceRequest(0,float('nan'),.1),ReferenceRequest(0,0,.1,stop_distance_m=-1),
    ReferenceRequest(0,0,.1,emergency=1)])
def test_invalid_observation_rejected(reference,observation):
    """HH_260906 - Reject unsupported observation states rather than guessing defaults."""
    with pytest.raises(ValueError):reference.update(observation)


def test_corrupt_envelope_rejected():
    """HH_260906 - Recompute spatial limits rather than trusting report pass flags."""
    p=plan([dict(distance_m=i,yaw=0.0) for i in range(101)])
    p=copy.deepcopy(p);p['speed_mps'][1]=8
    with pytest.raises(ValueError):SpeedReference(p)


@pytest.mark.parametrize('dt',[.025,.05,.1,.2])
def test_ideal_straight_traversal(reference,dt):
    """HH_260906 - Catch self-inconsistent braking preview; ideal integration is not CARLA evidence."""
    s=v=0.
    for _ in range(10000):
        r=reference.update(ReferenceRequest(s,v,dt))
        assert r['normal_reference_allowed']
        nv=r['target_speed_mps'];s+=r['ideal_step_distance_m'];v=nv
        if 100-s<.01 and v<.15:break
    assert 100-s<.01 and v<.15


def test_stop_inside_one_interval(reference):
    """HH_260906 - Do not integrate a full timestep of deceleration after speed has reached zero."""
    r=reference.update(ReferenceRequest(99.995,.1,.2))
    assert r['normal_reference_allowed']
    assert r['target_speed_mps']==0
    assert r['ideal_step_distance_m']==pytest.approx(.005)
    assert r['ideal_stop_time_s']==pytest.approx(.1)
