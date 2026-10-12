"""HH_260906 - Check preview filtering and normal braking separately from emergency overrides."""
from types import SimpleNamespace as NS
import math
import pytest
from scripts.e2e.research_stop_preview import stop_speed_cap,signal_preview,bounded_normal_step


def test_cap_stops_before_buffer_and_rejects_invalid():
    assert stop_speed_cap(12.)==0.
    assert stop_speed_cap(42.)==pytest.approx(6.)
    with pytest.raises(ValueError):stop_speed_cap(math.nan)


@pytest.mark.parametrize('state,lane,direction,cte,expected',[
    ('Red',1,1,0.,True),('Yellow',1,1,0.,True),('Green',1,1,0.,False),
    ('Red',2,1,0.,False),('Red',1,-1,0.,False),('Red',1,1,3.,False)])
def test_native_stop_waypoint_selection(state,lane,direction,cte,expected):
    wp=NS(road_id=7,lane_id=lane,transform=NS(location=NS(x=20.,y=0.),
        get_forward_vector=lambda:NS(x=direction,y=0.)))
    ego=NS(road_id=7,lane_id=1,transform=NS(get_forward_vector=lambda:NS(x=1.,y=0.)))
    light=NS(id=4,state=state,get_stop_waypoints=lambda:[wp])
    sentinel=object()
    agent=NS(_map=NS(get_waypoint=lambda _:ego),research_vehicle=NS(get_location=lambda:None),
        _lights_list=[light],_last_traffic_light=sentinel)
    projector=NS(length_m=100.,project=lambda *a,**kw:NS(progress_m=20.,cross_track_error_m=cte))
    result=signal_preview(agent,projector,0.)
    assert (result is not None)==expected
    assert agent._last_traffic_light is sentinel


def test_normal_ramp_leaves_emergency_override_possible():
    c=NS(brake=.15,throttle=.5)
    result=bounded_normal_step(lambda:c,0.,.05)
    assert result is c
    assert c.brake==pytest.approx(.001)
    assert c.throttle==.2
    # HH_260906 - The upstream agent may subsequently override this with its original emergency strength.
    c.brake=.5
    assert c.brake==.5


def test_low_speed_coast_precedes_unchanged_emergency():
    c=NS(brake=.15,throttle=0.)
    assert bounded_normal_step(lambda:c,.04,.05,1.5).brake==0.
