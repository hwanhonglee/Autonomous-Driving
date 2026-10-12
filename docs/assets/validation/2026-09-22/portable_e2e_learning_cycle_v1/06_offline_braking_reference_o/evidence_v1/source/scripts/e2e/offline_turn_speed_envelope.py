#!/usr/bin/env python3
"""HH_260906 - Design an offline spatial speed envelope without changing live control or labels."""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class EnvelopeConfig:
    """HH_260906 - Use conservative research targets below current runtime acceleration limits."""
    cruise_mps: float = 8.0
    lateral_mps2: float = 2.0
    acceleration_mps2: float = 1.0
    deceleration_mps2: float = 1.0
    start_mps: float = 0.0
    end_mps: float = 0.0


def number(value):
    """HH_260906 - Reject malformed numeric inputs before producing a speed proposal."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('finite real number required')
    return float(value)


def plan(points, config=EnvelopeConfig()):
    """HH_260906 - Propagate curvature caps backwards for braking and forwards for acceleration."""
    # HH_260906 - Collapse exact geometry duplicates at command transitions only in the offline view.
    unique=[];source_indices=[]
    for index, point in enumerate(points):
        if unique and number(point['distance_m']) == number(unique[-1]['distance_m']):
            # HH_260906 - CARLA command boundaries can differ by sub-microradian yaw serialization noise.
            heading_delta=number(point['yaw'])-number(unique[-1]['yaw'])
            if (any(point.get(key) != unique[-1].get(key) for key in ('x','y','z'))
                    or abs(math.atan2(math.sin(heading_delta),math.cos(heading_delta))) > 1e-6):
                raise ValueError('conflicting geometry at duplicate route distance')
            continue
        unique.append(point);source_indices.append(index)
    points=unique
    if len(points) < 3:
        raise ValueError('at least three route points required')
    values={key:number(value) for key,value in asdict(config).items()}
    if any(values[key] <= 0 for key in ('cruise_mps','lateral_mps2','acceleration_mps2','deceleration_mps2')):
        raise ValueError('positive speed and acceleration limits required')
    if any(not 0 <= values[key] <= config.cruise_mps for key in ('start_mps','end_mps')):
        raise ValueError('boundary speeds must lie inside cruise limit')
    distance=[number(p['distance_m']) for p in points]
    yaw=[number(p['yaw']) for p in points]
    ds=[b-a for a,b in zip(distance,distance[1:])]
    if any(d <= 1e-6 for d in ds):
        raise ValueError('strictly increasing nondegenerate route distance required')
    segment_curvature=[abs(math.atan2(math.sin(b-a), math.cos(b-a)))/d
                       for a,b,d in zip(yaw,yaw[1:],ds)]
    curvature=[max(segment_curvature[max(0,i-1):min(len(ds),i+1)]) for i in range(len(points))]
    caps=[min(config.cruise_mps,math.sqrt(config.lateral_mps2/k)) if k > 1e-9 else config.cruise_mps
          for k in curvature]
    speeds=caps.copy();speeds[-1]=min(speeds[-1],config.end_mps)
    for i in range(len(speeds)-2,-1,-1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*config.deceleration_mps2*ds[i]))
    speeds[0]=min(speeds[0],config.start_mps)
    for i in range(1,len(speeds)):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i-1]**2+2*config.acceleration_mps2*ds[i-1]))
    acceleration=[(b*b-a*a)/(2*d) for a,b,d in zip(speeds,speeds[1:],ds)]
    lateral=[v*v*k for v,k in zip(speeds,curvature)]
    return dict(config=values,source_point_indices=source_indices,distance_m=distance,curvature_abs_inv_m=curvature,
                local_cap_mps=caps,speed_mps=speeds,spatial_acceleration_mps2=acceleration,
                lateral_acceleration_mps2=lateral,
                checks=dict(acceleration=max(acceleration)<=config.acceleration_mps2+1e-9,
                            deceleration=min(acceleration)>=-config.deceleration_mps2-1e-9,
                            lateral=max(lateral)<=config.lateral_mps2+1e-9),
                live_control_modified=False,labels_modified=False,training_admitted=False,
                limitations=['Spatial reference only; not a closed-loop controller or measured vehicle result.',
                             'No traffic-light, obstacle, jerk, latency or steering tracking model.',
                             'Boundary speeds are caps; feasibility from a measured initial state is not proven.',
                             'Catalog heading curvature may differ from driven trajectory curvature.'])


def main():
    """HH_260906 - Save one isolated research proposal with exact source fingerprints."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=args.route.read_bytes();route=json.loads(source)
    if route.get('town') not in ('Town01','Town07','C_track_1_0_7') or route.get('scenario') != 'right':
        raise ValueError('active TRAIN right-turn routes only')
    result=plan(route['route'])
    result.update(route_sha256=hashlib.sha256(source).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  route=str(args.route),schema='portable_e2e.offline_turn_envelope.v1')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(checks=result['checks'], minimum_curve_cap_kph=min(result['local_cap_mps'])*3.6,
                         maximum_reference_kph=max(result['speed_mps'])*3.6,
                         output=str(args.output))))


if __name__ == '__main__':
    main()
