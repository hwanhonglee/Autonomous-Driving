"""HH_260906 - Screen measured body perimeter against drivable lanes; retain sampling limitations."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scripts.e2e.audit_offline_map_x import map_points


def perimeter(measurement, spacing=.2):
    """HH_260906 - Convert actor-relative bbox to the historical rear-axle ROS base frame."""
    box=measurement['bounding_box']
    if any(abs(box['rotation'][key])>1e-8 for key in ('roll','pitch','yaw')):
        raise ValueError('rotated boxes require explicit support')
    ex,ey=box['extent']['x'],box['extent']['y']
    xs=np.linspace(-ex,ex,int(np.ceil(2*ex/spacing))+1)
    ys=np.linspace(-ey,ey,int(np.ceil(2*ey/spacing))+1)
    points=np.array([(x,y) for x in xs for y in (-ey,ey)]+[(x,y) for y in ys for x in (-ex,ex)])
    points[:,0]+=box['location']['x']+measurement['historical_collector_wheelbase_m']/2
    points[:,1]+=box['location']['y'];points[:,1]*=-1
    return points


def main():
    """HH_260906 - Test every dense reference pose, not just the four final corners."""
    import carla
    p=argparse.ArgumentParser()
    p.add_argument('--measurement',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    measurement=json.loads(a.measurement.read_text());body=perimeter(measurement)
    root=Path('artifacts/training/2026-09-22')
    traces=json.loads((root/'curvature_preview_v_v2/traces.json').read_text())
    samples=json.loads((root/'offline_map_x_v3/samples.json').read_text())
    maps=json.loads((root/'offline_map_x_v3/results.json').read_text())['rows']
    reports=[]
    for row in maps:
        trace=traces[row['token']]
        if trace is None:continue
        payload=Path(row['map_file']).read_bytes()
        if hashlib.sha256(payload).hexdigest()!=row['map_sha256']:raise ValueError('map changed')
        world=carla.Map('offline',payload.decode())
        xy=np.asarray(trace['route_xy']);derivative=np.gradient(xy,axis=0)
        yaw=np.arctan2(derivative[:,1],derivative[:,0])
        bad=[];bad_points=0
        for i,(position,angle) in enumerate(zip(xy,yaw)):
            c,s=np.cos(angle),np.sin(angle)
            local=body@np.array([[c,s],[-s,c]])+position
            corners=map_points(local,samples[row['token']]['ego'])
            count=0
            for x,y in corners:
                wp=world.get_waypoint(carla.Location(x=float(x),y=float(y),z=float(samples[row['token']]['ego']['position_m'][2])),
                    project_to_road=False,lane_type=carla.LaneType.Driving)
                if wp is None:count+=1
            if count:bad.append(dict(pose_index=i,progress_m=trace['arc_m'][i],outside_points=count))
            bad_points+=count
        report=dict(token=row['token'],poses_checked=len(xy),perimeter_points_per_pose=len(body),
            outside_drivable_poses=len(bad),outside_drivable_points=bad_points,first_failures=bad[:12],
            last_failure=bad[-1] if bad else None,
            full_containment_proven=False,training_admitted=False)
        reports.append(report);print(json.dumps(report),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:json.dump(dict(rows=reports,
        measurement_sha256=hashlib.sha256(a.measurement.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Sampled perimeter, not continuous swept polygon.',
        'Any driving lane membership, not intended lane polygon containment.',
        'Planar yaw only and fixed initial height.',
        'Path tangent is an ideal heading, not measured vehicle yaw.',
        'Current asset measured; historical asset identity unproven.']),stream,indent=2)


if __name__=='__main__':main()
