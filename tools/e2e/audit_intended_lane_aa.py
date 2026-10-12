"""HH_260906 - Compare measured body polygons against only recorded lane IDs, without a live server."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from scripts.e2e.audit_offline_map_x import map_points
from scripts.e2e.audit_prius_boundary_z import perimeter


def lane_corridor(world, payload, route, spacing):
    """HH_260906 - Tessellate only recorded road/section/lane triples; never add adjacent lanes."""
    selected={(p['road_id'],p.get('section_id',0),p['lane_id']) for p in route}
    quads=[];found=set()
    for road in ET.fromstring(payload).findall('road'):
        rid=int(road.attrib['id']);length=float(road.attrib['length'])
        sections=road.findall('./lanes/laneSection')
        for sec,section in enumerate(sections):
            lo=float(section.attrib['s']);hi=float(sections[sec+1].attrib['s']) if sec+1<len(sections) else length
            for triple in sorted(t for t in selected if t[:2]==(rid,sec)):
                boundaries=[]
                bounds=next((p for p in route if (p['road_id'],p.get('section_id',0),p['lane_id'])==triple and 'audit_s_min' in p),{})
                start=max(lo,float(bounds.get('audit_s_min',lo)));end=min(hi,float(bounds.get('audit_s_max',hi)))
                for s in np.linspace(start+1e-6,end-1e-6,max(2,int(np.ceil((end-start)/spacing))+1)):
                    wp=world.get_waypoint_xodr(rid,triple[2],float(s))
                    if wp is None:
                        # HH_260906 - CARLA float endpoints may be outside; extrapolate only the submillimetre remainder.
                        inset=min(max(float(s),lo+.001),hi-.001)
                        wp=world.get_waypoint_xodr(rid,triple[2],inset)
                    else:inset=float(s)
                    if wp is None or wp.section_id!=sec:raise ValueError(f'missing lane sample {triple}')
                    yaw=np.deg2rad(wp.transform.rotation.yaw)
                    center=np.array([wp.transform.location.x,wp.transform.location.y])
                    # HH_260906 - Positive lanes face decreasing OpenDRIVE s.
                    center+=(float(s)-inset)*(-1 if triple[2]>0 else 1)*np.array([np.cos(yaw),np.sin(yaw)])
                    normal=np.array([-np.sin(yaw),np.cos(yaw)])*wp.lane_width/2
                    boundaries.append((center+normal,center-normal))
                for a,b in zip(boundaries,boundaries[1:]):
                    poly=Polygon([a[0],a[1],b[1],b[0]])
                    if not poly.is_valid:
                        # HH_260906 - Preserve sampled boundary vertices through explicit triangle tessellation.
                        # This also handles float-quantized duplicate centers without buffering the corridor.
                        quads.extend([Polygon([a[0],a[1],b[1]]),Polygon([a[0],b[1],b[0]])])
                        continue
                    quads.append(poly)
                found.add(triple)
    if found!=selected:raise ValueError(f'unresolved lanes {selected-found}')
    return unary_union(quads),sorted(selected)


def main():
    """HH_260906 - Retain raw outside areas and adjacent-pose hull screening separately."""
    import carla
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--spacing',type=float,default=.05)
    p.add_argument('--start-predecessor',action='store_true')
    p.add_argument('--traces',type=Path);a=p.parse_args()
    if not .01<=a.spacing<=.1:raise ValueError('invalid spacing')
    root=Path('artifacts/training/2026-09-22')
    refs=root/'curvature_preview_v_v2/traces.json';meta=root/'offline_map_x_v3/samples.json'
    if a.traces is not None:refs=a.traces
    measured=root/'prius_geometry_z_v2/geometry/measurement.json'
    traces=json.loads(refs.read_text());samples=json.loads(meta.read_text())
    body=perimeter(json.loads(measured.read_text()))
    xmin,ymin=body.min(axis=0);xmax,ymax=body.max(axis=0)
    corners=np.array([[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]])
    rows=json.loads((root/'offline_map_x_v3/results.json').read_text())['rows']
    reports=[];cache={};shapes={}
    for row in rows:
        token=row['token'];trace=traces[token]
        if trace is None:continue
        key=row['native_route_path']
        if key not in cache:
            payload=Path(row['map_file']).read_bytes()
            if hashlib.sha256(payload).hexdigest()!=row['map_sha256']:raise ValueError('map changed')
            route=json.loads(Path(key).read_text())['route']
            offline=carla.Map('offline',payload.decode())
            if a.start_predecessor and 'c_track' in token:
                # HH_260906 - Add only the last two metres of the unique mapped predecessor, not neighboring lanes.
                if (route[0]['road_id'],route[0]['lane_id'])!=(157,-1):raise ValueError('unexpected starting lane')
                choices=offline.get_waypoint_xodr(157,-1,.001).previous(2.)
                if len(choices)!=1:raise ValueError('ambiguous predecessor')
                q=choices[0]
                if q.lane_id>=0:raise ValueError('unsupported predecessor direction')
                route=route+[dict(road_id=q.road_id,section_id=q.section_id,lane_id=q.lane_id,audit_s_min=q.s,audit_s_max=q.s+2.)]
            cache[key]=lane_corridor(offline,payload,route,a.spacing)
        corridor,ids=cache[key]
        xy=np.asarray(trace['route_xy']);delta=np.gradient(xy,axis=0)
        headings=np.arctan2(delta[:,1],delta[:,0]);areas=[];hullareas=[];previous=None;worst=None
        for i,(position,heading) in enumerate(zip(xy,headings)):
            c,s=np.cos(heading),np.sin(heading)
            footprint=Polygon(map_points(corners@np.array([[c,s],[-s,c]])+position,samples[token]['ego']))
            outside=footprint.difference(corridor).area
            areas.append(outside)
            if worst is None or outside>worst[0]:worst=(outside,i,footprint)
            if previous is not None:hullareas.append(previous.union(footprint).convex_hull.difference(corridor).area)
            previous=footprint
        report=dict(token=token,poses_checked=len(areas),selected_lane_ids=ids,
            maximum_outside_area_m2=max(areas),poses_outside_over_1e6_m2=sum(v>1e-6 for v in areas),
            maximum_adjacent_hull_outside_area_m2=max(hullareas),
            worst_progress_m=trace['arc_m'][worst[1]],strict_sampled_covers_all=all(v==0 for v in areas),
            training_admitted=False)
        # HH_260906 - Separate terminal support issues from the interior; retain full-route failures above.
        interior=[(i,v) for i,v in enumerate(areas) if 5.<trace['arc_m'][i]<trace['arc_m'][-1]-5.]
        report['interior_maximum_outside_area_m2']=max((v for i,v in interior),default=None)
        report['interior_poses_outside_over_1e4_m2']=sum(v>1e-4 for i,v in interior)
        report['outside_over_1e4_progress_range_m']=[trace['arc_m'][i] for i in (next((i for i,v in enumerate(areas) if v>1e-4),0),next((i for i in range(len(areas)-1,-1,-1) if areas[i]>1e-4),0))]
        reports.append(report);shapes[token]=dict(corridor_wkt=corridor.wkt,worst_body_wkt=worst[2].wkt)
        print(json.dumps(report),flush=True)
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'results.json').write_text(json.dumps(dict(rows=reports,lane_mesh_spacing_m=a.spacing,start_predecessor=a.start_predecessor,
        source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [refs,meta,measured,Path(__file__)]},
        limitations=['Recorded-lane union across complete route, not time-indexed maneuver authorization.',
        'Tessellated boundaries, ideal tangent heading, planar geometry.',
        'Adjacent hull screening is not an exact continuous rotational sweep.']),indent=2))
    (a.output/'shapes.json').write_text(json.dumps(shapes))


if __name__=='__main__':main()
