"""HH_260906 - Pure offline speed-reference policy; not a vehicle actuator or approved controller."""
from bisect import bisect_right
from dataclasses import dataclass
import math
from scripts.e2e.offline_turn_speed_envelope import number


@dataclass(frozen=True)
class ReferenceRequest:
    """HH_260906 - Require explicit measured state and externally known stop constraints."""
    progress_m: float
    speed_mps: float
    dt_s: float
    stop_distance_m: float | None = None
    emergency: bool = False
    stop_required: bool = False


def validate_envelope(envelope):
    """HH_260906 - Fail closed on malformed or modified reference proposals."""
    x=envelope['distance_m'];v=envelope['speed_mps'];config=envelope['config']
    if len(x)<3 or len(x)!=len(v):raise ValueError('invalid envelope lengths')
    x=[number(t) for t in x];v=[number(t) for t in v]
    if any(b<=a for a,b in zip(x,x[1:])):raise ValueError('nonincreasing distance')
    for key in ('cruise_mps','acceleration_mps2','deceleration_mps2','lateral_mps2'):
        if number(config[key])<=0:raise ValueError('invalid reference limit')
    if any(not 0<=speed<=config['cruise_mps'] for speed in v):raise ValueError('invalid reference speed')
    for a,b,va,vb in zip(x,x[1:],v,v[1:]):
        acceleration=(vb*vb-va*va)/(2*(b-a))
        if not -config['deceleration_mps2']-1e-9<=acceleration<=config['acceleration_mps2']+1e-9:
            raise ValueError('inconsistent spatial acceleration')
    return x,v,config


class SpeedReference:
    """HH_260906 - Produce causal targets and explicit infeasibility, never brake or throttle commands."""
    def __init__(self,envelope):
        self.distance,self.speed,self.config=validate_envelope(envelope)

    def local_speed(self,progress):
        """HH_260906 - Interpolate squared speed consistently with spatial acceleration bounds."""
        progress=number(progress)
        if not self.distance[0]<=progress<=self.distance[-1]:raise ValueError('progress outside route')
        i=min(bisect_right(self.distance,progress)-1,len(self.distance)-2)
        ratio=(progress-self.distance[i])/(self.distance[i+1]-self.distance[i])
        return math.sqrt(max(0,(1-ratio)*self.speed[i]**2+ratio*self.speed[i+1]**2))

    def update(self,request):
        """HH_260906 - Preserve emergency priority and distinguish requested comfort from feasibility."""
        progress=number(request.progress_m);speed=number(request.speed_mps);dt=number(request.dt_s)
        if speed<0 or not 0<dt<=.2:raise ValueError('invalid speed or timestep')
        if type(request.emergency) is not bool or type(request.stop_required) is not bool:
            raise ValueError('explicit Boolean stop flags required')
        if not self.distance[0]<=progress<=self.distance[-1]:raise ValueError('progress outside route')
        distance=request.stop_distance_m
        if distance is not None:
            distance=number(distance)
            if distance<0:raise ValueError('negative stop distance')
        if request.emergency:
            return dict(status='EMERGENCY_PASSTHROUGH',target_speed_mps=0.0,
                        emergency_must_be_preserved=True,normal_reference_allowed=False,
                        throttle=None,brake=None,vehicle_response_proven=False)
        if request.stop_required and distance is None:
            return dict(status='STOP_DISTANCE_UNKNOWN',target_speed_mps=0.0,
                        emergency_must_be_preserved=True,normal_reference_allowed=False,
                        throttle=None,brake=None,vehicle_response_proven=False)
        deceleration=self.config['deceleration_mps2'];acceleration=self.config['acceleration_mps2']
        # HH_260906 - Solve a trapezoidal next-step position rather than assuming acceleration while braking.
        def caps(candidate):
            travel=.5*(speed+candidate)*dt
            # HH_260906 - A full stop reached inside the interval travels only the braking distance, then holds.
            if candidate==0.0 and speed/deceleration<dt:
                travel=speed*speed/(2*deceleration)
            preview=min(self.distance[-1],progress+travel)
            route_cap=self.local_speed(preview)
            stop_cap=(math.sqrt(2*deceleration*max(0,distance-travel)) if distance is not None else None)
            return min(route_cap,stop_cap) if stop_cap is not None else route_cap,route_cap,stop_cap,travel
        stopping_distance=speed*speed/(2*deceleration)
        lower=max(0,speed-deceleration*dt)
        upper=speed+acceleration*dt
        if upper<=caps(upper)[0]:target=upper
        else:
            lo,hi=0.,upper
            for _ in range(48):
                mid=(lo+hi)/2
                if mid<=caps(mid)[0]:lo=mid
                else:hi=mid
            target=lo
        _,route_cap,stop_cap,travel=caps(target)
        feasible=(target+1e-8>=lower and travel<=self.distance[-1]-progress+1e-8
                  and (distance is None or stopping_distance<=distance+1e-8))
        return dict(status='REFERENCE_FEASIBLE' if feasible else 'REFERENCE_INFEASIBLE',
                    target_speed_mps=target,route_preview_speed_mps=route_cap,
                    stop_cap_mps=stop_cap,comfort_stopping_distance_m=stopping_distance,
                    ideal_step_distance_m=travel,
                    ideal_stop_time_s=(min(dt,speed/deceleration) if target==0 else None),
                    speed_rate_request_mps2=(target-speed)/dt,
                    emergency_must_be_preserved=True,normal_reference_allowed=feasible,
                    throttle=None,brake=None,vehicle_response_proven=False)
