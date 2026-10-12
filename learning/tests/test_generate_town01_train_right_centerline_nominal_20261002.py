"""HH_261002 - Pure XML/analytic mocks prove a new route law and unchanged whole-body gates, never run a real Map/server or modify captured data."""
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from scripts.e2e import generate_town01_train_right_centerline_nominal_20261002 as audit


def synthetic_xml():
    def road(rid,lane,length,x,y,h,kind="line",curvature=None,junction=-1):
        shape = '<line/>' if kind == "line" else f'<arc curvature="{curvature}"/>'
        link = '<link><predecessor elementType="road" elementId="12" contactPoint="start"/><successor elementType="road" elementId="24" contactPoint="start"/></link>' if rid==256 else ''
        lane_link = '<link><predecessor id="-1"/><successor id="1"/></link>' if rid==256 else ''
        side = "left" if lane>0 else "right"
        return f'<road id="{rid}" length="{length}" junction="{junction}">{link}<planView><geometry s="0" x="{x}" y="{y}" hdg="{h}" length="{length}">{shape}</geometry></planView><elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lateralProfile><superelevation s="0" a="0" b="0" c="0" d="0"/></lateralProfile><lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0"><{side}><lane id="{lane}" type="driving">{lane_link}<width sOffset="0" a="4" b="0" c="0" d="0"/></lane></{side}></laneSection></lanes></road>'
    roads = road(24,1,50.,0.,0.,-math.pi/2)+road(256,1,4*math.pi,8.0003,8.,math.pi,"arc",.125,255)+road(12,-1,224.25,8.0006,8.,0.)
    return ('<OpenDRIVE>'+roads+'<junction id="255"><connection incomingRoad="24" connectingRoad="256" contactPoint="end"><laneLink from="1" to="1"/></connection></junction></OpenDRIVE>').encode()


def original():
    points = [dict(x=2.,y=-38.97,z=0.,yaw=math.pi/2,road_id=24,section_id=0,lane_id=1),dict(x=2.0003,y=0.,z=0.,yaw=math.pi/2,road_id=256,section_id=0,lane_id=1),dict(x=8.0006,y=6.,z=0.,yaw=0.,road_id=12,section_id=0,lane_id=-1),dict(x=177.522,y=6.095,z=0.,yaw=0.,road_id=12,section_id=0,lane_id=-1)]
    return dict(town="Town01",weather="ClearNoon",scenario="right",coordinate_reference="base_link",route=points,goal_endpoint_provenance=dict(original_failed_spawn_goal=True),start_carla_transform=dict(x=2.,y=38.97,z=.3),goal_carla_transform=dict(x=177.522,y=-6.095,z=.3))


def test_analytic_line_arc_and_reverse_lane_tangent():
    g=dict(s=0.,x=0.,y=0.,hdg=0.,length=4*math.pi,curvature=.125)
    x,y,h=audit.reference_pose(g,4*math.pi)
    assert (x,y,h)==pytest.approx((8.,8.,math.pi/2))
    model=audit.road_model(ET.fromstring(synthetic_xml()).find("road[@id='256']"),1)
    assert audit.centre_pose(model,0.)==pytest.approx(dict(x=8.0003,y=6.,z=0.,yaw=0.))
    assert audit.centre_pose(model,model['length'])==pytest.approx(dict(x=2.0003,y=0.,z=0.,yaw=math.pi/2))
    with pytest.raises(ValueError,match="outside"):audit.centre_pose(model,-.0001)


def test_new_centerline_whole_endpoints_no_original_change_or_spawn_goal_relabel():
    old=original();before=copy.deepcopy(old);xml=synthetic_xml()
    new=audit.generate(xml,old); points=new['route'];proof=new['generation_provenance']
    assert old==before and points[0]['x']==pytest.approx(2.) and points[0]['y']==pytest.approx(-38.97)
    assert points[-1]['x']==pytest.approx(old['route'][-1]['x']) and points[-1]['y']==pytest.approx(6.)
    assert proof['endpoint_projection']['goal']['shift_xy_norm_m']==pytest.approx(.095)
    assert proof['endpoint_projection']['goal']['source_longitudinal_projection_s_m']==pytest.approx(169.5214)
    assert proof['endpoint_projection']['goal']['original_spawn_transform_unchanged']==old['goal_carla_transform']
    assert proof['original_goal_endpoint_provenance']==old['goal_endpoint_provenance']
    assert new['route_id']==audit.CASE_ID and new['admission_status']=='PENDING_QA'
    assert 'training_data_approved' not in new and 'physical_turn_preflight' not in new
    assert new['route_length_m']==points[-1]['distance_m'] and all(p['remaining_m']==points[-1]['distance_m']-p['distance_m'] for p in points)
    assert points[0]['distance_m']==0. and len(proof['seams'])==2
    assert all(s['distance_m']==pytest.approx(.0003) for s in proof['seams'])
    assert new['spawn_point'].split(',')[1]==format(38.97,'.17g')
    assert set((p['road_id'],p['section_id'],p['lane_id']) for p in points)==set(audit.CORRIDOR)


def test_fixed_reference_lattice_full_curve_whole_nominal_rear_to_actor_and_right_command():
    new=audit.generate(synthetic_xml(),original());points,groups=audit.nominal.dedup_points(new)
    assert len(points)==len(new['route']) and all(len(g)==1 for g in groups)
    for t in audit.CORRIDOR:
        lane=[p for p in points if (p['road_id'],p['section_id'],p['lane_id'])==t]
        assert all(0<abs(b['xodr_s_m']-a['xodr_s_m'])<=1. for a,b in zip(lane,lane[1:]))
        assert all(p['road_option']==('RIGHT' if t[0]==256 else 'LANEFOLLOW') for p in lane)
    turn=[p for p in points if p['road_option']=='RIGHT']
    assert audit.wrap(turn[-1]['yaw']-turn[0]['yaw'])==pytest.approx(-math.pi/2)
    assert max(abs(audit.wrap(b['yaw']-a['yaw']))/(b['distance_m']-a['distance_m']) for a,b in zip(turn,turn[1:]))<.2
    samples=audit.nominal.nominal_samples(points,2.85)
    assert samples[0]['nominal_rear_arc_m']==0. and samples[-1]['nominal_rear_arc_m']==new['route_length_m']
    assert samples[0]['nominal_actor_pose_carla']['y']==pytest.approx(38.97-1.425)
    assert all(0<b['nominal_rear_arc_m']-a['nominal_rear_arc_m']<=.12 for a,b in zip(samples,samples[1:]))
    assert all('frame' not in p and 'timestamp' not in p for p in samples)


def test_all_geometry_breakpoints_preserved_without_optimized_spacing():
    road=dict(length=7.4,geometries=[dict(s=0.),dict(s=2.6),dict(s=5.731)])
    values=audit.lattice(road,7.4,0.)
    assert values==[7.4,7.,6.,5.731,5.,4.,3.,2.6,2.,1.,0.]
    assert audit.lattice(road,0.,7.4)==list(reversed(values))


def test_mathematically_shared_endpoint_one_pose_full_lane_memberships_and_unchanged_bridge():
    root=ET.fromstring(synthetic_xml())
    root.find("road[@id='256']/planView/geometry").set('x','8.')
    root.find("road[@id='12']/planView/geometry").set('x','8.0003')
    route=audit.generate(ET.tostring(root),original());points=route['route']
    joined=[p for p in points if 'shared_topology_endpoint_memberships' in p]
    assert len(joined)==1
    joint=joined[0];members=joint['shared_topology_endpoint_memberships'];index=joint['index']
    assert [(p['road_id'],p['lane_id']) for p in members]==[(24,1),(256,1)]
    assert members[0]['xodr_s_m']==0. and members[1]['xodr_s_m']==4*math.pi
    assert {k:joint[k] for k in ('x','y','z','yaw')}==members[0]['pose']
    assert joint['road_id']==256 and joint['road_option']=='RIGHT'
    assert points[0]['xodr_s_m']==pytest.approx(38.97) and points[-1]['xodr_s_m']==pytest.approx(169.5217)
    assert audit.nominal.geometry.body._route_lanes_for_arc_interval(points,points[index-1]['distance_m'],points[index+1]['distance_m'])=={(24,0,1),(256,0,1)}
    assert all(math.hypot(b['x']-a['x'],b['y']-a['y'])>1e-4 for a,b in zip(points,points[1:]))
    assert audit.nominal.dedup_points(route)[0]==points
    assert audit.nominal.nominal_samples(points,2.85)[-1]['nominal_rear_arc_m']==route['route_length_m']


def test_real_noncoincident_tiny_seam_not_hidden_by_roundoff_representation():
    root=ET.fromstring(synthetic_xml());root.find("road[@id='256']/planView/geometry").set('x','8.00000001')
    with pytest.raises(ValueError,match='tiny segment'):audit.generate(ET.tostring(root),original())


@pytest.mark.parametrize('fault',['width','offset','grade','spiral','section','lane','topology','junction','seam','cusp'])
def test_unreviewed_xml_changes_rejected_not_repaired(fault):
    root=ET.fromstring(synthetic_xml());middle=root.find("road[@id='256']")
    if fault=='width':middle.find('lanes/laneSection/left/lane/width').set('a','4.1')
    if fault=='offset':middle.find('lanes/laneOffset').set('a','.01')
    if fault=='grade':middle.find('elevationProfile/elevation').set('b','.01')
    if fault=='spiral':middle.find('planView/geometry')[0].tag='spiral'
    if fault=='section':middle.find('lanes/laneSection').set('s','.1')
    if fault=='lane':middle.find('lanes/laneSection/left/lane').set('type','shoulder')
    if fault=='topology':middle.find('link/predecessor').set('elementId','99')
    if fault=='junction':root.find('junction/connection').set('incomingRoad','99')
    if fault=='seam':root.find("road[@id='12']/planView/geometry").set('x','8.01')
    if fault=='cusp':middle.find('planView/geometry/arc').set('curvature','.5')
    with pytest.raises(ValueError):audit.generate(ET.tostring(root),original())


@pytest.mark.parametrize('fault',['Town03','left','actor','order','outside','nan'])
def test_no_other_site_lane_or_endpoint_clamp(fault):
    old=original()
    if fault=='Town03':old['town']='Town03'
    if fault=='left':old['scenario']='left'
    if fault=='actor':old['coordinate_reference']='actor_center'
    if fault=='order':old['route'][1]['road_id']=258
    if fault=='outside':old['route'][-1]['x']=999.
    if fault=='nan':old['route'][-1]['y']=float('nan')
    with pytest.raises((ValueError,audit.nominal.geometry.body.Unsupported)):audit.generate(synthetic_xml(),old)


def fake_nominal(monkeypatch,fail=False,missing=False):
    calls=[]
    def probe(manifest,route,xml,carla):
        calls.append(copy.deepcopy(route))
        points,groups=audit.nominal.dedup_points(route);rows=audit.nominal.nominal_samples(points,2.85)
        for row in rows:row['all_eight_box_vertices_carla_xyz']=[[0.,0.,0.]]*8
        counts={k:dict(assessed_count=len(rows)-(1 if k.startswith('previous_sweep') else 0),violation_sample_indices=[5] if fail else []) for k in ('sampled_body_raw_lane','sampled_body_inset_10cm_lane','previous_sweep_raw_lane','previous_sweep_inset_10cm_lane')}
        if missing:counts['previous_sweep_inset_10cm_lane']['assessed_count']-=1
        return rows,counts,groups
    monkeypatch.setattr(audit.nominal,'probe',probe)
    return calls


@pytest.mark.parametrize('fail',[False,True])
def test_only_frozen_whole_probe_invoked_all_outcomes_retained(monkeypatch,fail):
    calls=fake_nominal(monkeypatch,fail)
    result,route,rows=audit.run_case(original(),{},synthetic_xml(),object())
    assert result['status']==('NOMINAL_BODY_FAIL_RETAINED' if fail else 'NOMINAL_CLEAR_NOT_DRIVING_PROOF')
    assert calls==[route] and len(rows)==result['nominal_sample_count']
    assert result['nominal_sweep_count']==len(rows)-1 and rows[-1]['nominal_rear_arc_m']==route['route_length_m']


def test_missing_sweep_or_unsupported_stays_failed(monkeypatch):
    fake_nominal(monkeypatch,missing=True)
    result,route,rows=audit.run_case(original(),{},synthetic_xml(),object())
    assert result['status']=='GENERATION_OR_NOMINAL_UNSUPPORTED_NOT_PROVEN' and route is not None
    old=original();old['town']='Town03'
    result,route,rows=audit.run_case(old,{},synthetic_xml(),object())
    assert result['status']=='GENERATION_OR_NOMINAL_UNSUPPORTED_NOT_PROVEN' and route is None


def caller_fixture(monkeypatch,tmp_path):
    root=tmp_path.resolve();monkeypatch.setattr(audit,'ROOT',root)
    files={root/'old_route.json':json.dumps(original()).encode(),root/'map.xodr':synthetic_xml(),root/'reference.json':b'{}'}
    for p,b in files.items():p.write_bytes(b)
    inputs={p:hashlib.sha256(b).hexdigest() for p,b in files.items()};monkeypatch.setattr(audit,'INPUTS',inputs)
    monkeypatch.setattr(audit.nominal,'ROUTE',root/'old_route.json');monkeypatch.setattr(audit.nominal,'XODR',root/'map.xodr');monkeypatch.setattr(audit.nominal,'REFERENCE',root/'reference.json')
    monkeypatch.setattr(audit.nominal,'INPUTS',inputs)
    source=root/'source.py';source.write_bytes(b'pinned synthetic source')
    sources={'source.py':hashlib.sha256(source.read_bytes()).hexdigest()};monkeypatch.setattr(audit,'source_pins',lambda:sources)
    egg=root/'offline.egg';egg.write_bytes(b'never imported real API');monkeypatch.setattr(audit.nominal,'EGG',egg)
    output=root/'new_catalog';monkeypatch.setattr(audit,'OUTPUT',output)
    history=dict(results=[dict(case_id='old_failed',status='NOMINAL_UNSUPPORTED_NOT_PROVEN',source_route_sha256='a'*64)]*32)
    monkeypatch.setattr(audit,'validate_origin',lambda data:history)
    caller=root/'caller.json';c=dict(schema=audit.SCHEMA+'.caller_pins',nominal_only=True,generation_contract=audit.generation_contract(),input_sha256={str(p):s for p,s in inputs.items()},source_sha256=sources,carla_egg_sha256='6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a',output=str(output))
    real_payload=audit.nominal.payload
    def payload(p,expected,pins):
        if Path(p)==egg:expected=hashlib.sha256(egg.read_bytes()).hexdigest()
        return real_payload(p,expected,pins)
    monkeypatch.setattr(audit.nominal,'payload',payload)
    caller.write_text(json.dumps(c));sha=hashlib.sha256(caller.read_bytes()).hexdigest()
    return caller,sha,output,source,c


def test_validate_only_no_generation_map_output(monkeypatch,tmp_path):
    caller,sha,output,_,_=caller_fixture(monkeypatch,tmp_path)
    monkeypatch.setattr(audit,'generate',lambda *a:pytest.fail('no generation'))
    assert audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha,'--validate-only'])==0
    assert not output.exists()


@pytest.mark.parametrize('kind',['clear','body_failed','namespace_failed','postcheck_changed'])
def test_actual_mode_synthetic_all_outcomes_and_original_denominator_createonly(monkeypatch,tmp_path,kind):
    caller,sha,output,source,_=caller_fixture(monkeypatch,tmp_path)
    monkeypatch.setattr(sys,'version_info',(3,10))
    provider=SimpleNamespace(__file__=str(audit.nominal.EGG)+'/carla/__init__.py')
    monkeypatch.setitem(sys.modules,'carla',provider)
    monkeypatch.setattr(audit.shutil,'disk_usage',lambda _:SimpleNamespace(free=6*1024**3))
    if kind=='namespace_failed':
        monkeypatch.setattr(audit,'run_case',lambda *a:(_ for _ in ()).throw(RuntimeError('synthetic provider namespace failure')))
    else:fake_nominal(monkeypatch,fail=kind=='body_failed')
    before=list(sys.path)
    if kind=='postcheck_changed':
        real=audit.run_case
        def changed(*a):
            result=real(*a);source.write_bytes(b'mutated source during nominal computation');return result
        monkeypatch.setattr(audit,'run_case',changed)
        with pytest.raises(ValueError,match='before output'):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])
        assert not output.exists()
    else:
        assert audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])==(0 if kind=='clear' else 2)
        summary=json.loads((output/'summary.json').read_bytes());catalog=json.loads((output/'route_catalog.json').read_bytes())
        assert summary['prospective_new_case_count']==1 and len(summary['results'])==1 and len(summary['original_all32_case_outcomes'])==32
        assert summary['training_data_approved'] is False and catalog['training_data_approved'] is False
        assert summary['split_registry_modified'] is False and summary['source_route_coordinates_modified'] is False
        assert len(catalog['routes'])==1 and catalog['routes'][0]['status']==summary['status']
        if kind=='namespace_failed':assert catalog['routes'][0]['path'] is None and 'namespace failure' in summary['results'][0]['error']
        else:
            route=output/catalog['routes'][0]['path']
            assert hashlib.sha256(route.read_bytes()).hexdigest()==summary['generated_route_sha256']
        with pytest.raises(ValueError,match='fresh'):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])
    assert sys.path==before


@pytest.mark.parametrize('fault',['hash','source','input','output','spacing','inset','existing','symlink','unknown_case'])
def test_auth_or_identity_changed_failclosed_before_output(monkeypatch,tmp_path,fault):
    caller,sha,output,source,c=caller_fixture(monkeypatch,tmp_path)
    if fault=='hash':sha='0'*64
    if fault=='source':source.write_bytes(b'changed')
    if fault=='input':c['input_sha256'][next(iter(c['input_sha256']))]='0'*64
    if fault=='output':c['output']=str(tmp_path/'elsewhere')
    if fault=='spacing':c['generation_contract']['sampling_policy']='adapt until pass'
    if fault=='inset':c['generation_contract']['body_inset_m']=.09
    if fault=='existing':output.mkdir()
    if fault=='symlink':
        link=tmp_path/'link';link.symlink_to(tmp_path,target_is_directory=True);c['output']=str(link/'output')
    if fault=='unknown_case':c['generation_contract']['case_ids']=['different_route']
    if fault not in ('hash','source','existing'):
        caller.write_text(json.dumps(c));sha=hashlib.sha256(caller.read_bytes()).hexdigest()
    with pytest.raises(ValueError):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha,'--validate-only'])
    if fault!='existing':assert not output.exists()


def test_real_source_freeze_contract_and_no_live_provider_calls():
    pins=audit.source_pins()
    assert pins['tools/e2e/audit_town01_train_curve_nominal_20261002.py']==audit.FROZEN['tools/e2e/audit_town01_train_curve_nominal_20261002.py']
    assert audit.generation_contract()['lane_triples']==[[24,0,1],[256,0,1],[12,0,-1]]
    forbidden={'Client','get_world','load_world','tick','apply_control','try_spawn_actor','spawn_actor'}
    tree=ast.parse(Path(audit.__file__).read_bytes())
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Attribute) and n.attr in forbidden]
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Import) and any(a.name.startswith(('torch','numpy')) for a in n.names)]
