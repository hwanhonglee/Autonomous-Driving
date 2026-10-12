"""HH_260906 - Link frozen samples to local OpenDRIVE without contacting or mutating a simulator."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess

import numpy as np


def export_samples(tokens):
    """HH_260906 - Read only the selected TRAIN episodes and verify manifest file digests remotely."""
    program = '''import json,hashlib,pathlib
root=pathlib.Path('/home/pro6000/personal/hwanhong/dataset/prepared/moving-recovery-k1-20260922-v1/source_0/episodes')
tokens=TOKENS
out={}
for episode in sorted({t.rsplit(':',1)[0] for t in tokens}):
 p=root/episode
 manifest=json.loads((p/'episode.json').read_text())
 for filename,key in [('samples.jsonl','sample_jsonl_sha256'),('route_geometry.json','route_geometry_sha256')]:
  assert hashlib.sha256((p/filename).read_bytes()).hexdigest()==manifest[key]
 for filename,key in [('source_manifest.json','source_manifest_sha256'),('collection_config.json','collection_config_sha256')]:
  assert hashlib.sha256((p/filename).read_bytes()).hexdigest()==manifest['source_provenance'][key]
 provenance=json.loads((p/'source_manifest.json').read_text())
 config=json.loads((p/'collection_config.json').read_text())
 for line in (p/'samples.jsonl').open():
  sample=json.loads(line)
  if sample['sample_id'] in tokens:
   out[sample['sample_id']]={'ego':sample['ego'],'map_id':manifest['map_id'],
    'route_source':provenance['route_source'],'runtime_town':config['native_runtime']['town'],
    'sample_file_sha256':manifest['sample_jsonl_sha256'],
    'vehicle_id':manifest['vehicle_id']}
assert set(out)==set(tokens)
print(json.dumps(out))
'''.replace('TOKENS', repr(tokens))
    remote = '/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -c '+shlex.quote(program)
    completed = subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',
                                'pro6000-training',remote], capture_output=True, text=True, check=True, timeout=60)
    return json.loads(completed.stdout)


def map_points(points, ego):
    """HH_260906 - Transform ROS base-link positions into CARLA map coordinates using stored yaw."""
    x, y, z, w = ego['orientation_xyzw']
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    c, s = math.cos(yaw), math.sin(yaw)
    world = np.asarray(points) @ np.array([[c,s],[-s,c]]) + np.asarray(ego['position_m'][:2])
    world[:, 1] *= -1
    return world


def main():
    """HH_260906 - Report point membership only; no footprint or historical-map identity approval."""
    import carla
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    traces=json.loads((args.run/'traces.json').read_text())
    samples=export_samples(list(traces))
    # HH_260906 - Locate collection-time route payloads by content digest, never by map name alone.
    native_routes={}
    for path in Path('artifacts').rglob('route.json'):
        native_routes.setdefault(hashlib.sha256(path.read_bytes()).hexdigest(),path)
    maps=Path('/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive')
    rows=[]
    for token,trace in traces.items():
        sample=samples[token]
        file=maps/(sample['map_id']+'.xodr')
        payload=file.read_bytes()
        offline=carla.Map(sample['map_id'],payload.decode())
        row=dict(token=token,map_file=str(file),map_sha256=hashlib.sha256(payload).hexdigest(),
                 historical_map_identity_verified=False,vehicle_footprint_verified=False,training_admitted=False)
        native=native_routes.get(sample['route_source']['source_artifact_sha256'])
        row['native_route_path']=str(native) if native else None
        row['native_route_hash_matched']=native is not None
        if native:
            original=json.loads(native.read_text())
            errors=[];mismatches=0; mismatch_details=[]
            for index,point in enumerate(original['route']):
                location=carla.Location(x=point['x'],y=-point['y'],z=point.get('z',0.))
                wp=offline.get_waypoint(location,project_to_road=True,lane_type=carla.LaneType.Driving)
                if wp is None:
                    mismatches+=1
                    continue
                errors.append(math.hypot(location.x-wp.transform.location.x,location.y-wp.transform.location.y))
                if (wp.road_id,wp.lane_id)!=(point.get('road_id'),point.get('lane_id')):
                    mismatches+=1
                    mismatch_details.append(dict(index=index,expected=[point.get('road_id'),point.get('lane_id')],
                        actual=[wp.road_id,wp.lane_id],expected_junction=point.get('is_junction'),
                        actual_junction=wp.is_junction,nearest_distance_m=errors[-1]))
            row['native_route_comparison']=dict(points=len(original['route']),
                nearest_road_lane_mismatches=mismatches,maximum_nearest_distance_m=max(errors) if errors else None,
                mismatch_details=mismatch_details)
        if trace is not None:
            world=map_points(trace['route_xy'],sample['ego'])
            outside=[];clearances=[];identities=set()
            for index,(x,y) in enumerate(world):
                location=carla.Location(x=float(x),y=float(y),z=float(sample['ego']['position_m'][2]))
                inside=offline.get_waypoint(location,project_to_road=False,lane_type=carla.LaneType.Driving)
                nearest=offline.get_waypoint(location,project_to_road=True,lane_type=carla.LaneType.Driving)
                if inside is None:outside.append(index)
                if nearest is not None:
                    transform=nearest.transform
                    yaw=math.radians(transform.rotation.yaw)
                    lateral=abs(-(x-transform.location.x)*math.sin(yaw)+(y-transform.location.y)*math.cos(yaw))
                    clearances.append(nearest.lane_width/2-lateral)
                    identities.add((nearest.road_id,nearest.section_id,nearest.lane_id))
            row.update(points_checked=len(world),outside_driving_lane_points=len(outside),
                       first_outside_indices=outside[:10],minimum_point_lateral_clearance_m=min(clearances) if clearances else None,
                       nearest_lane_ids=sorted(identities))
        else:row['status']='initial_state_rejected_by_V'
        rows.append(row)
        print(json.dumps(row))
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'samples.json').write_text(json.dumps(samples,indent=2))
    (args.output/'results.json').write_text(json.dumps(dict(rows=rows,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256((args.run/'traces.json').read_bytes()).hexdigest(),
        limitations=['Point membership is not swept vehicle containment.',
                     'Nearest lane may be a different lane or junction branch.',
                     'Current local map is not proven identical to collection-time map.',
                     'Stored initial height is used throughout; grade-separated roads require a height profile.']),indent=2))


if __name__=='__main__':main()
