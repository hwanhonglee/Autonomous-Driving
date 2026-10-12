"""HH_261002 - Synthetic XML only: a new whole TRAIN topology, unchanged frozen gates and all failed ancestors."""
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

from scripts.e2e import generate_town01_train_right_gentle_centerline_nominal_20261002 as audit


def xml(*, heading_delta=0., seam=.0003):
    def road(rid,length,geometries,junction=-1):
        shape = ''.join(f'<geometry s="{s}" x="{x}" y="{y}" hdg="{h}" length="{span}">'+(f'<arc curvature="{k}"/>' if k else '<line/>')+'</geometry>' for s,x,y,h,span,k in geometries)
        link = '<link><predecessor elementType="road" elementId="24" contactPoint="end"/><successor elementType="road" elementId="6" contactPoint="start"/></link>' if rid==108 else ''
        lane_link = '<link><predecessor id="1"/><successor id="1"/></link>' if rid==108 else ''
        return f'<road id="{rid}" length="{length}" junction="{junction}">{link}<planView>{shape}</planView><elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lateralProfile><superelevation s="0" a="0" b="0" c="0" d="0"/></lateralProfile><lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/><laneSection s="0"><left><lane id="1" type="driving">{lane_link}<width sOffset="0" a="4" b="0" c="0" d="0"/></lane></left></laneSection></lanes></road>'
    roads = road(6,200.,[(0.,100.,100.,0.,1.,0.),(1.,101.,100.,0.,199.,0.)])+road(108,5*math.pi,[(0.,90.,110.,-math.pi/2+heading_delta,5*math.pi,.1)],87)+road(24,100.,[(0.,90.,210.+seam,-math.pi/2,100.,0.)])
    return ('<OpenDRIVE>'+roads+'<junction id="87"><connection id="6" incomingRoad="6" connectingRoad="108" contactPoint="end"><laneLink from="1" to="1"/></connection></junction></OpenDRIVE>').encode()


def original():
    points = [dict(x=220.,y=102.,z=0.,yaw=math.pi,road_id=6),dict(x=100.,y=102.,z=0.,yaw=math.pi,road_id=93),
        dict(x=100.,y=102.,z=0.,yaw=math.pi,road_id=108),dict(x=92.,y=110.0003,z=0.,yaw=math.pi/2,road_id=24),
        dict(x=92.095,y=200.,z=0.,yaw=math.pi/2,road_id=24)]
    for i,p in enumerate(points):p.update(index=i,section_id=0,lane_id=1,original_metadata="keep_exact_"+str(i))
    return dict(town="Town01",weather="ClearNoon",scenario="right",coordinate_reference="base_link",route=points,
        goal_endpoint_provenance=dict(original_unqualified_catalogue=True),start_carla_transform=dict(x=220.,y=-102.,z=.3),goal_carla_transform=dict(x=92.095,y=-200.,z=.3))


def test_all_frozen_geometry_functions_and_nominal_provider_are_identical():
    for name in ('number','road_model','reference_pose','centre_pose','lattice','wrap'):
        assert getattr(audit,name) is getattr(audit.previous,name)
    assert audit.nominal.probe is audit.previous.nominal.probe
    assert (audit.nominal.STEP_M,audit.nominal.geometry.body.MAX_ROUTE_POINT_CENTER_ERROR_M,
        audit.nominal.geometry.body.LANE_INSET_M,audit.nominal.geometry.body.SWEEP_POSITION_GUARD_M)==(.12,.08,.1,.02)


def test_new_whole_topology_preserves_old_all_nodes_and_endpoint_longitudinal_extents():
    old=original();before=copy.deepcopy(old);route=audit.generate(xml(),old);p=route['generation_provenance']
    assert old==before and p['original_encoded_route']==before['route']
    assert p['original_road93_duplicate_source_indices']==[1,2]
    assert p['original_encoded_topology_groups']==[list(t) for t in audit.ORIGINAL_GROUPS]
    assert p['original_route_sha256']==audit.SOURCE_ROUTE_SHA and p['original_candidate_not_reclassified'] is True
    assert p['reference_s_spans'][0]==[6,1,120.,0.] and p['reference_s_spans'][1]==[108,1,5*math.pi,0.]
    assert p['reference_s_spans'][2]==pytest.approx([24,1,100.,10.0003])
    assert route['route'][0]['x']==220. and route['route'][-1]['y']==pytest.approx(200.)
    assert route['route'][-1]['x']==pytest.approx(92.)
    assert p['endpoint_projection']['goal']['original_spawn_transform_unchanged']==old['goal_carla_transform']
    assert p['original_goal_endpoint_provenance']==old['goal_endpoint_provenance']
    assert set((v['road_id'],v['section_id'],v['lane_id']) for v in route['route'])==set(audit.CORRIDOR)
    assert route['admission_status']=='PENDING_QA' and 'training_data_approved' not in route
    assert route['route_length_m']==route['route'][-1]['distance_m'] and all(v['remaining_m']==route['route_length_m']-v['distance_m'] for v in route['route'])
    assert route['spawn_point'].split(',')[1]==format(-102.,'.17g')


def test_unique_multirecord_line_projection_no_clamp_ambiguity_or_arc_projection():
    roads=ET.fromstring(xml());road=audit.road_model(roads.find("road[@id='6']"),1)
    assert audit.endpoint_s(road,dict(x=220.,y=102.))==120.
    for value in (dict(x=100.,y=102.),dict(x=300.,y=102.),dict(x=301.,y=102.),dict(x=float('nan'),y=102.)):
        with pytest.raises(ValueError):audit.endpoint_s(road,value)
    ambiguous=copy.deepcopy(road);ambiguous['geometries'].append(dict(s=200.,x=101.,y=100.,hdg=0.,length=199.,curvature=0.))
    with pytest.raises(ValueError,match='unique'):audit.endpoint_s(ambiguous,dict(x=220.,y=102.))
    curve=audit.road_model(roads.find("road[@id='108']"),1)
    with pytest.raises(ValueError,match='LINE'):audit.endpoint_s(curve,dict(x=100.,y=102.))


def test_one_position_two_distinct_source_headings_not_yaw_roundoff_relabel():
    route=audit.generate(xml(heading_delta=5e-14),original());shared=[v for v in route['route'] if 'shared_topology_endpoint_memberships' in v]
    assert len(shared)==1
    point=shared[0];members=point['shared_topology_endpoint_memberships'];seam=route['generation_provenance']['seams'][0]
    assert members[0]['pose']['yaw']!=members[1]['pose']['yaw']
    old_bound=64*max(math.ulp(1.),math.ulp(members[0]['pose']['yaw']),math.ulp(members[1]['pose']['yaw']))
    assert old_bound<seam['source_endpoint_heading_gap_rad']<=.001
    assert seam['source_endpoint_gap_m']<=seam['position_roundoff_bound_m']
    assert seam['heading_roundoff_equivalence_claimed'] is False
    assert {k:point[k] for k in ('x','y','z','yaw')}==members[0]['pose']
    assert [(v['road_id'],v['lane_id']) for v in members]==[(6,1),(108,1)]
    previous_index=point['index']-1
    assert audit.nominal.geometry.body._route_lanes_for_arc_interval(route['route'],route['route'][previous_index]['distance_m'],route['route'][point['index']+1]['distance_m'])=={(6,0,1),(108,0,1)}
    assert all(math.hypot(b['x']-a['x'],b['y']-a['y'])>1e-4 for a,b in zip(route['route'],route['route'][1:]))
    samples=audit.nominal.nominal_samples(audit.nominal.dedup_points(route)[0],2.85)
    assert samples[0]['nominal_rear_arc_m']==0. and samples[-1]['nominal_rear_arc_m']==route['route_length_m']
    assert all(0<b['nominal_rear_arc_m']-a['nominal_rear_arc_m']<=.12 for a,b in zip(samples,samples[1:]))
    assert all('frame' not in v and 'timestamp' not in v for v in samples)


def test_noncoincident_tiny_and_heading_discontinuous_seams_still_reject():
    for seam in (1e-8,.006):
        with pytest.raises(ValueError):audit.generate(xml(seam=seam),original())
    with pytest.raises(ValueError):audit.generate(xml(heading_delta=.002),original())


@pytest.mark.parametrize('fault',['Town03','left','actor','groups','duplicate','missing_duplicate','nan','outside'])
def test_original_site_topology_duplicate_or_endpoint_changes_rejected(fault):
    old=original()
    if fault=='Town03':old['town']='Town03'
    if fault=='left':old['scenario']='left'
    if fault=='actor':old['coordinate_reference']='actor_center'
    if fault=='groups':old['route'][1]['road_id']=94
    if fault=='duplicate':old['route'][2]['x']+=.001
    if fault=='missing_duplicate':old['route'].pop(1)
    if fault=='nan':old['route'][-1]['x']=float('nan')
    if fault=='outside':old['route'][-1]['y']=999.
    with pytest.raises(ValueError):audit.generate(xml(),old)


@pytest.mark.parametrize('fault',['width','offset','grade','spiral','section','lane','topology','lane_link','junction','junction_extra','cusp'])
def test_unsupported_xml_is_not_repaired(fault):
    root=ET.fromstring(xml());middle=root.find("road[@id='108']")
    if fault=='width':middle.find('lanes/laneSection/left/lane/width').set('a','4.1')
    if fault=='offset':middle.find('lanes/laneOffset').set('a','.01')
    if fault=='grade':middle.find('elevationProfile/elevation').set('b','.01')
    if fault=='spiral':middle.find('planView/geometry')[0].tag='spiral'
    if fault=='section':middle.find('lanes/laneSection').set('s','.1')
    if fault=='lane':middle.find('lanes/laneSection/left/lane').set('type','shoulder')
    if fault=='topology':middle.find('link/predecessor').set('elementId','99')
    if fault=='lane_link':middle.find('lanes/laneSection/left/lane/link/predecessor').set('id','-1')
    if fault=='junction':root.find('junction/connection').set('incomingRoad','99')
    if fault=='junction_extra':ET.SubElement(root.find('junction/connection'),'laneLink',{'from':'2','to':'2'})
    if fault=='cusp':middle.find('planView/geometry/arc').set('curvature','.5')
    with pytest.raises(ValueError):audit.generate(ET.tostring(root),original())


def test_private_reused_ast_changes_only_additive_failed_ancestor_field(monkeypatch):
    saved=[];real_compile=compile
    def capture(tree,*args,**kwargs):saved.append(copy.deepcopy(tree));return real_compile(tree,*args,**kwargs)
    monkeypatch.setattr(audit,'compile',capture,raising=False)
    ns=audit.provider();original_tree=ast.parse(Path(audit.previous.__file__).read_bytes())
    expected=[n for n in original_tree.body if isinstance(n,ast.FunctionDef) and n.name in ('source_pins','run_case','main')]
    actual=saved[-1].body
    summaries=[n for n in ast.walk(actual[-1]) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='summary' for t in n.targets)]
    kw=summaries[0].value.keywords.pop()
    assert kw.arg=='previous_generated_candidates_retained'
    assert ast.dump(ast.Module(body=actual,type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=expected,type_ignores=[]),include_attributes=False)
    assert ns['nominal'].ROUTE==audit.SOURCE_ROUTE and audit.nominal.ROUTE==audit.previous.nominal.ROUTE!=audit.SOURCE_ROUTE
    assert ns['generate'] is audit.generate and ns['nominal'].probe is audit.nominal.probe


def provenance(monkeypatch):
    history=dict(results=[dict(case_id='old_'+str(i),status='NOMINAL_UNSUPPORTED_NOT_PROVEN',source_route_sha256='a'*64) for i in range(32)])
    monkeypatch.setattr(audit.previous,'validate_origin',lambda data:copy.deepcopy(history))
    catalog=dict(status='complete',canonical_name='Town01',routes=[dict(id=audit.ORIGINAL_CASE,sha256=audit.SOURCE_ROUTE_SHA,path=f'routes/town01/right/{audit.ORIGINAL_CASE}.json')])
    plan=dict(split_intent='TRAIN_CANDIDATE');owner=dict(exit_code=0,catalog_sha256=audit.INPUTS[audit.BASE/'route_catalog.json'])
    prior=dict(schema=audit.previous.SCHEMA,status='NOMINAL_BODY_FAIL_RETAINED',prospective_new_case_count=1,
        generated_route_sha256=audit.INPUTS[audit.PRIOR_ROUTE],original_all32_case_outcomes=history['results'],
        results=[dict(case_id=audit.previous.CASE_ID,status='NOMINAL_BODY_FAIL_RETAINED',geometry=dict(failed_inset_sweep_indices=list(range(376,448))))])
    prior_catalog=dict(routes=[dict(id=audit.previous.CASE_ID,path=f'routes/right/{audit.previous.CASE_ID}.json',sha256=audit.INPUTS[audit.PRIOR_ROUTE],status='NOMINAL_BODY_FAIL_RETAINED')])
    return {str(path):json.dumps(value).encode() for path,value in ((audit.BASE/'route_catalog.json',catalog),(audit.BASE/'owner_plan.json',plan),
        (audit.BASE/'owner_result.json',owner),(audit.PRIOR_SUMMARY,prior),(audit.PRIOR_CATALOG,prior_catalog))}


def test_exact_original_provenance_plus_entire_failed_p00_census_retained(monkeypatch):
    data=provenance(monkeypatch);before=copy.deepcopy(data);history=audit.validate_origin(data)
    assert data==before and len(history['results'])==32
    p00=history['previous_generated_candidates_retained'][0]
    assert p00['source_summary_sha256']==audit.INPUTS[audit.PRIOR_SUMMARY] and p00['source_generated_route_sha256']==audit.INPUTS[audit.PRIOR_ROUTE]
    assert p00['status']=='NOMINAL_BODY_FAIL_RETAINED' and p00['geometry']['failed_inset_sweep_indices']==list(range(376,448))


@pytest.mark.parametrize('fault',['Town03','plan','owner_bool','owner_SHA','route_SHA','route_path','missing_case','old32','oldp00_clear','oldp00_route','prior_catalog'])
def test_provenance_or_failed_denominator_cannot_be_reinterpreted(monkeypatch,fault):
    data=provenance(monkeypatch)
    path=audit.BASE/'route_catalog.json'
    if fault in ('plan',):path=audit.BASE/'owner_plan.json'
    if fault in ('owner_bool','owner_SHA'):path=audit.BASE/'owner_result.json'
    if fault in ('old32','oldp00_clear','oldp00_route'):path=audit.PRIOR_SUMMARY
    if fault=='prior_catalog':path=audit.PRIOR_CATALOG
    value=json.loads(data[str(path)])
    if fault=='Town03':value['canonical_name']='Town03'
    if fault=='plan':value['split_intent']='DEV'
    if fault=='owner_bool':value['exit_code']=False
    if fault=='owner_SHA':value['catalog_sha256']='0'*64
    if fault=='route_SHA':value['routes'][0]['sha256']='0'*64
    if fault=='route_path':value['routes'][0]['path']='routes/town01/right/wrong.json'
    if fault=='missing_case':value['routes']=[]
    if fault=='old32':value['original_all32_case_outcomes'].pop()
    if fault=='oldp00_clear':value['status']='NOMINAL_CLEAR_NOT_DRIVING_PROOF'
    if fault=='oldp00_route':value['generated_route_sha256']='0'*64
    if fault=='prior_catalog':value['routes'][0]['status']='NOMINAL_CLEAR_NOT_DRIVING_PROOF'
    data[str(path)]=json.dumps(value).encode()
    with pytest.raises(ValueError):audit.validate_origin(data)


def census(monkeypatch,fail=False,missing=False):
    def probe(manifest,route,payload,provider):
        points,groups=audit.nominal.dedup_points(route);rows=audit.nominal.nominal_samples(points,2.85)
        for row in rows:row['all_eight_box_vertices_carla_xyz']=[[0.,0.,0.]]*8
        values={k:dict(assessed_count=len(rows)-(1 if k.startswith('previous_sweep') else 0),violation_sample_indices=[2] if fail else []) for k in ('sampled_body_raw_lane','sampled_body_inset_10cm_lane','previous_sweep_raw_lane','previous_sweep_inset_10cm_lane')}
        if missing:values['previous_sweep_inset_10cm_lane']['assessed_count']-=1
        return rows,values,groups
    monkeypatch.setattr(audit.nominal,'probe',probe)


@pytest.mark.parametrize('kind',['clear','body_failed','incomplete'])
def test_direct_frozen_probe_whole_denominator_no_failed_subset(monkeypatch,kind):
    census(monkeypatch,fail=kind=='body_failed',missing=kind=='incomplete')
    result,route,rows=audit.run_case(original(),{},xml(),object())
    assert result['status']==dict(clear='NOMINAL_CLEAR_NOT_DRIVING_PROOF',body_failed='NOMINAL_BODY_FAIL_RETAINED',incomplete='GENERATION_OR_NOMINAL_UNSUPPORTED_NOT_PROVEN')[kind]
    if kind!='incomplete':assert result['nominal_sample_count']==len(rows) and result['nominal_sweep_count']==len(rows)-1
    assert rows[-1]['nominal_rear_arc_m']==route['route_length_m']


def fixture(monkeypatch,tmp_path):
    real=audit.provider;ns=real();root=tmp_path.resolve();source=root/'source.py';source.write_bytes(b'synthetic source')
    route=root/'original.json';route.write_text(json.dumps(original()));ref=root/'reference.json';ref.write_text('{}');xodr=root/'map.xodr';xodr.write_bytes(xml())
    files=[route,ref,xodr];inputs={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    egg=root/'offline.egg';egg.write_bytes(b'not a real Map provider');output=root/'fresh_catalog'
    ns.update(ROOT=root,OUTPUT=output,INPUTS=inputs,source_pins=lambda:{'source.py':hashlib.sha256(b'synthetic source').hexdigest()},
        validate_origin=lambda data:dict(results=[dict(case_id='old_'+str(i),status='NOMINAL_UNSUPPORTED_NOT_PROVEN',source_route_sha256='a'*64) for i in range(32)],previous_generated_candidates_retained=[dict(case_id=audit.previous.CASE_ID,status='NOMINAL_BODY_FAIL_RETAINED')]))
    ns['nominal'].ROUTE=route;ns['nominal'].REFERENCE=ref;ns['nominal'].XODR=xodr;ns['nominal'].EGG=egg
    real_payload=ns['nominal'].payload
    def payload(path,sha,pins):return real_payload(path,hashlib.sha256(egg.read_bytes()).hexdigest() if Path(path)==egg else sha,pins)
    ns['nominal'].payload=payload
    contract=dict(schema=audit.SCHEMA+'.caller_pins',nominal_only=True,generation_contract=audit.generation_contract(),input_sha256={str(p):s for p,s in inputs.items()},source_sha256=ns['source_pins'](),carla_egg_sha256='6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a',output=str(output))
    caller=root/'caller.json';caller.write_text(json.dumps(contract));sha=hashlib.sha256(caller.read_bytes()).hexdigest()
    monkeypatch.setattr(audit,'provider',lambda:ns)
    return ns,caller,sha,output,source,contract


def test_contract_and_validate_only_no_generation_map_or_output(monkeypatch,tmp_path):
    ns,caller,sha,output,_,_=fixture(monkeypatch,tmp_path)
    ns['generate']=lambda *a:pytest.fail('no real generation in validate-only')
    assert audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha,'--validate-only'])==0
    assert not output.exists()


@pytest.mark.parametrize('kind',['clear','body_failed','namespace_failed','postcheck_changed'])
def test_main_synthetic_fresh_whole_outcome_32plus_failed_p00_and_restoration(monkeypatch,tmp_path,kind):
    census(monkeypatch,fail=kind=='body_failed');ns,caller,sha,output,source,_=fixture(monkeypatch,tmp_path)
    monkeypatch.setattr(sys,'version_info',(3,10));monkeypatch.setitem(sys.modules,'carla',SimpleNamespace(__file__=str(ns['nominal'].EGG)+'/carla/__init__.py'))
    monkeypatch.setattr(ns['shutil'],'disk_usage',lambda _:SimpleNamespace(free=6*1024**3));before=list(sys.path)
    if kind=='namespace_failed':ns['run_case']=lambda *a:(_ for _ in ()).throw(RuntimeError('synthetic namespace error'))
    if kind=='postcheck_changed':
        old=ns['run_case']
        def altered(*args):result=old(*args);source.write_bytes(b'changed');return result
        ns['run_case']=altered
        with pytest.raises(ValueError,match='before output'):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])
        assert not output.exists()
    else:
        assert audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])==(0 if kind=='clear' else 2)
        summary=json.loads((output/'summary.json').read_bytes())
        assert len(summary['original_all32_case_outcomes'])==32 and summary['previous_generated_candidates_retained'][0]['status']=='NOMINAL_BODY_FAIL_RETAINED'
        assert summary['prospective_new_case_count']==1 and summary['training_data_approved'] is False
        with pytest.raises(ValueError,match='fresh'):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha])
    assert sys.path==before


@pytest.mark.parametrize('fault',['SHA','input','source','output','spacing','inset','existing','symlink','case'])
def test_external_auth_changed_rejects_before_output(monkeypatch,tmp_path,fault):
    _,caller,sha,output,source,contract=fixture(monkeypatch,tmp_path)
    if fault=='SHA':sha='0'*64
    if fault=='input':contract['input_sha256'][next(iter(contract['input_sha256']))]='0'*64
    if fault=='source':source.write_bytes(b'changed')
    if fault=='output':contract['output']=str(tmp_path/'wrong')
    if fault=='spacing':contract['generation_contract']['sampling_policy']='adaptive'
    if fault=='inset':contract['generation_contract']['body_inset_m']=.09
    if fault=='existing':output.mkdir()
    if fault=='symlink':
        link=tmp_path/'alias';link.symlink_to(tmp_path,target_is_directory=True);contract['output']=str(link/'child')
    if fault=='case':contract['generation_contract']['case_ids']=['unknown']
    if fault not in ('SHA','source','existing'):caller.write_text(json.dumps(contract));sha=hashlib.sha256(caller.read_bytes()).hexdigest()
    with pytest.raises(ValueError):audit.main(['--caller-pins',str(caller),'--caller-pins-sha256',sha,'--validate-only'])
    if fault!='existing':assert not output.exists()


def test_source_contract_no_live_or_array_provider_and_previous_bytes_unchanged():
    assert hashlib.sha256(Path(audit.previous.__file__).read_bytes()).hexdigest()==audit.PREVIOUS_SHA
    assert audit.source_pins()['tools/e2e/generate_town01_train_right_centerline_nominal_20261002.py']==audit.PREVIOUS_SHA
    contract=audit.generation_contract();assert contract['intended_teacher_seconds']==70 and contract['teacher_protocol_implemented'] is False
    assert contract['previous_all32_and_generated_p00_FAILURE_retained'] is True
    assert contract['nominal_step_max_m']==.12 and contract['body_inset_m']==.1 and contract['sweep_position_guard_m']==.02
    tree=ast.parse(Path(audit.__file__).read_bytes());forbidden={'Client','get_world','load_world','tick','spawn_actor','apply_control'}
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Attribute) and n.attr in forbidden]
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Import) and any(a.name.startswith(('torch','numpy')) for a in n.names)]
