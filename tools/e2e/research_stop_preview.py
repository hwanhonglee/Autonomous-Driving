"""HH_260906 - Conservative simulator stop preview; never replace upstream emergency checks."""
import math


def stop_speed_cap(distance_m, *, buffer_m=12., deceleration_mps2=.6):
    """HH_260906 - Return a spatial speed cap with explicit distance and conservative stop buffer."""
    if not all(math.isfinite(x) for x in (distance_m,buffer_m,deceleration_mps2)):
        raise ValueError('finite stop inputs required')
    if buffer_m<0 or deceleration_mps2<=0:raise ValueError('invalid stop limits')
    return math.sqrt(2*deceleration_mps2*max(0.,distance_m-buffer_m))


def signal_preview(agent, projector, progress):
    """HH_260906 - Inspect native stop waypoints without querying or mutating the agent hazard cache."""
    ego=agent._map.get_waypoint(agent.research_vehicle.get_location())
    records=[]
    for light in agent._lights_list:
        if str(light.state).split('.')[-1] not in ('Red','Yellow'):continue
        for waypoint in light.get_stop_waypoints():
            if (waypoint.road_id,waypoint.lane_id)!=(ego.road_id,ego.lane_id):continue
            a=waypoint.transform.get_forward_vector();b=ego.transform.get_forward_vector()
            if a.x*b.x+a.y*b.y<=0:continue
            loc=waypoint.transform.location
            projection=projector.project(loc.x,-loc.y,0.,forward_m=projector.length_m)
            distance=projection.progress_m-progress
            if projection.cross_track_error_m>2. or not 0.<=distance<=60.:continue
            records.append(dict(actor_id=light.id,state=str(light.state),distance_m=distance,
                speed_cap_mps=stop_speed_cap(distance),stop_progress_m=projection.progress_m,
                road_id=waypoint.road_id,lane_id=waypoint.lane_id))
    return min(records,key=lambda r:r['distance_m']) if records else None


def bounded_normal_step(original, previous_brake, dt, speed_mps=math.inf):
    """HH_260906 - Limit normal PID braking before the agent's unchanged emergency override."""
    control=original()
    # HH_260906 - R01 measured a discontinuous low-speed brake response; coast below 2 m/s in this pilot only.
    control.brake=min(float(control.brake),.04 if speed_mps>2. else 0.,previous_brake+.02*dt)
    control.throttle=min(float(control.throttle),.20)
    return control
