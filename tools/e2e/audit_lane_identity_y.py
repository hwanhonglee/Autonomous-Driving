"""HH_260906 - Resolve ambiguous nearest-lane queries against the recorded road and lane explicitly."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def main():
    """HH_260906 - Preserve residuals instead of treating map-name equality as historical proof."""
    import carla
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=json.loads(args.input.read_text())
    reports=[];seen=set()
    for row in source['rows']:
        if row['native_route_path'] in seen:continue
        seen.add(row['native_route_path'])
        payload=Path(row['map_file']).read_bytes()
        if hashlib.sha256(payload).hexdigest()!=row['map_sha256']:raise ValueError('map changed')
        road_lengths={int(r.attrib['id']):float(r.attrib['length']) for r in ET.fromstring(payload).findall('road')}
        world=carla.Map(Path(row['map_file']).stem,payload.decode())
        route=json.loads(Path(row['native_route_path']).read_text())['route']
        details=[]
        for mismatch in row['native_route_comparison']['mismatch_details']:
            point=route[mismatch['index']]
            road,lane=mismatch['expected']
            length=road_lengths[road]
            best=None
            def search(values):
                # HH_260906 - Query the intended lane, never accept a closer unrelated junction branch.
                nonlocal best
                for s in values:
                    wp=world.get_waypoint_xodr(road,lane,float(s))
                    if wp is None:continue
                    loc=wp.transform.location
                    error=math.hypot(loc.x-point['x'],loc.y+point['y'])
                    if best is None or error<best[0]:best=(error,float(s),wp)
            search(np.linspace(.00001,length-.00001,max(2,int(length/.1)+1)))
            if best:
                search(np.linspace(max(.00001,best[1]-.15),min(length-.00001,best[1]+.15),601))
            details.append(dict(**mismatch,intended_lane_exists=best is not None,
                intended_lane_distance_m=best[0] if best else None,
                intended_lane_s=best[1] if best else None,
                intended_section_id=best[2].section_id if best else None,
                intended_heading_carla_deg=best[2].transform.rotation.yaw if best else None))
        reports.append(dict(map_file=row['map_file'],map_sha256=row['map_sha256'],
            native_route_path=row['native_route_path'],details=details,
            maximum_intended_distance_m=max(d['intended_lane_distance_m'] for d in details),
            historical_map_identity_verified=False))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(dict(rows=reports,
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),stream,indent=2)
    for report in reports:
        print(json.dumps(report))


if __name__=='__main__':main()
