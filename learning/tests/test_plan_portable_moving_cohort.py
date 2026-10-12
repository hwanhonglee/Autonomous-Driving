"""HH_260906 - Test fixed moving-cohort planning on synthetic files without simulator or TEST access."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import plan_portable_moving_cohort as p


def write_json(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(value,allow_nan=True).encode();path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def make_catalog(root, map_id="town07", kind="straight", scenario="straight", count=3, length=210., shared=False):
    directory=root/map_id/"catalog"/kind
    entries=[]
    for i in range(count):
        name=f"{map_id}_{scenario}_{i}"
        point=lambda x,arc:dict(x=x,y=float(i*20+(0 if scenario=='straight' else 1000)),z=0.,yaw=0.,distance_m=arc,
            road_option="LANEFOLLOW" if scenario=="straight" else scenario.upper(),
            road_id=1 if shared else i+1,section_id=0,lane_id=-1)
        route=dict(town=p.coverage.inventory.MAP_CONTRACT[map_id][0],scenario=scenario,
            coordinate_reference="base_link",route_length_m=length,weather="ClearNoon",
            route=[point(0.,0.),point(length,length)])
        relative=f"routes/{map_id}/{scenario}/{name}.json"
        digest=write_json(directory/relative,route)
        entries.append(dict(id=name,status="ready",scenario=scenario,path=relative,sha256=digest,route_length_m=length))
    catalog=directory/"route_catalog.json"
    write_json(catalog,dict(status="complete",map_id=map_id,canonical_name=p.coverage.inventory.MAP_CONTRACT[map_id][0],routes=entries))
    return catalog,entries


def notes():
    return {name:dict(blockers=[]) for name,_,_,_ in p.COHORT}


def test_fixed_twenty_four_ids_splits_behavior_weather_and_locks():
    slots=p.fixed_slots()
    assert len(slots)==24 and len({r['slot_id'] for r in slots})==24
    assert p.Counter(r['split'] for r in slots)==dict(train=18,dev_val=6)
    assert p.Counter(r['scenario'] for r in slots)==dict(straight=8,left=8,right=8)
    assert {r['map_id'] for r in slots}.isdisjoint(p.coverage.PROTECTED_MAPS)
    assert next(r for r in slots if r['slot_id']=='T07-S02')['weather']=='CloudyNoon'
    assert next(r for r in slots if r['slot_id']=='V03-L01')['weather']=='CloudyNoon'
    assert next(r for r in slots if r['slot_id']=='V02-L01')['weather']=='ClearNoon'


@pytest.mark.parametrize('map_id',['town04','town15','town06'])
def test_protected_or_out_of_cohort_rejected_before_constructing_paths(map_id,monkeypatch):
    monkeypatch.setattr(p,'safe',lambda value:pytest.fail('path access before map refusal'))
    with pytest.raises(ValueError):p.load_candidates(Path('/unused'),map_id,{})


def test_exact_route_payload_hash_and_geometry_are_bound(tmp_path):
    _,entries=make_catalog(tmp_path)
    ledger={};routes,missing=p.load_candidates(tmp_path,'town07',ledger)
    assert len(routes)==3 and len(ledger)==4
    assert missing[0]['reason']=='CATALOG_NOT_AVAILABLE'
    assert routes[0]['route_sha256']==entries[0]['sha256']
    assert routes[0]['route_length_m']==routes[0]['recomputed_planar_length_m']==210.
    assert routes[0]['horizon']['status']=='CONSERVATIVE_ARC_BUDGET_AVAILABLE'
    assert routes[0]['horizon']['reserved_future_arc_m']==p.SPEED_CEILING_MPS*6.4
    assert routes[0]['horizon']['measured_timing_available'] is False


@pytest.mark.parametrize('change',['missing_sha','wrong_sha','absolute_path','parent_path','cross_map','wrong_town',
    'wrong_length','wrong_arc','nan','bool','duplicate_id','bad_status'])
def test_invalid_catalog_or_payload_is_not_silently_selected(tmp_path,change):
    catalog,entries=make_catalog(tmp_path,count=1)
    doc=json.loads(catalog.read_text());entry=doc['routes'][0]
    payload=catalog.parent/entry['path'];route=json.loads(payload.read_text())
    if change=='missing_sha':entry['sha256']=None
    elif change=='wrong_sha':entry['sha256']='0'*64
    elif change=='absolute_path':entry['path']=str(payload)
    elif change=='parent_path':entry['path']='../route.json'
    elif change=='cross_map':entry['path']='routes/town04/straight/route.json'
    elif change=='duplicate_id':doc['routes'].append(deepcopy(entry))
    elif change=='bad_status':entry['status']='failed'
    else:
        if change=='wrong_town':route['town']='Town04'
        elif change=='wrong_length':route['route_length_m']=211.
        elif change=='wrong_arc':route['route'][-1]['distance_m']=100.
        elif change=='nan':route['route'][0]['x']=float('nan')
        elif change=='bool':route['route'][0]['road_id']=True
        entry['sha256']=write_json(payload,route)
    write_json(catalog,doc)
    with pytest.raises(ValueError):p.load_candidates(tmp_path,'town07',{})


def test_missing_payload_is_explicit_not_successful_route(tmp_path):
    catalog,entries=make_catalog(tmp_path,count=1)
    (catalog.parent/entries[0]['path']).unlink()
    routes,missing=p.load_candidates(tmp_path,'town07',{})
    assert not routes and any(r['reason']=='ROUTE_PAYLOAD_NOT_AVAILABLE' for r in missing)


def test_symlinks_rejected_even_when_content_hash_would_match(tmp_path):
    catalog,entries=make_catalog(tmp_path,count=1)
    payload=catalog.parent/entries[0]['path'];original=payload.with_suffix('.original')
    payload.rename(original);payload.symlink_to(original)
    with pytest.raises(ValueError,match='symlink'):p.load_candidates(tmp_path,'town07',{})


def test_turn_future_tail_budget_reports_short_lead_and_tail_without_claiming_physics(tmp_path):
    make_catalog(tmp_path,kind='turn',scenario='left',count=1,length=80.)
    route=p.load_candidates(tmp_path,'town07',{})[0][0]
    reasons=route['horizon']['reasons']
    assert 'SHORT_ANCHOR_SPAN_AT_DECLARED_SPEED_REFERENCE' in reasons
    assert 'TURN_BEGINS_BEFORE_PROPOSED_LEAD_IN' in reasons
    assert 'INSUFFICIENT_POST_TURN_ARC_AT_DECLARED_SPEED_REFERENCE' in reasons
    assert route['turn_yaw_sign_agrees'] is False
    slot=next(r for r in p.select_slots([route],notes()) if r['route'])
    assert 'DECLARED_TURN_YAW_DIRECTION_REVIEW_NEEDED' in slot['technical_conditions_unmet']


def test_existing_thirty_second_anchor_contract_not_five_seconds_or_raw_duration(tmp_path):
    make_catalog(tmp_path,count=1,length=210.)
    route=p.load_candidates(tmp_path,'town07',{})[0][0]
    assert route['horizon']['minimum_reference_anchor_span_seconds']==30.
    assert route['horizon']['minimum_route_length_m']==pytest.approx(181.66666666666669)
    assert route['horizon']['status']=='CONSERVATIVE_ARC_BUDGET_AVAILABLE'
    assert route['full_30kph_horizon']['minimum_route_length_m']==pytest.approx(333.33333333333337)
    assert route['full_30kph_horizon']['status']=='ROUTE_EXTENSION_OR_SPEED_PLAN_NEEDED'


def test_catalog_xyz_arc_kept_separate_from_planar_context_budget(tmp_path):
    catalog,entries=make_catalog(tmp_path,count=1,length=210.)
    doc=json.loads(catalog.read_text());entry=doc['routes'][0]
    path=catalog.parent/entry['path'];route=json.loads(path.read_text())
    route['route'][-1]['z']=5.
    entry['route_length_m']=route['route_length_m']=route['route'][-1]['distance_m']=math.hypot(210.,5.)
    entry['sha256']=write_json(path,route);write_json(catalog,doc)
    checked=p.load_candidates(tmp_path,'town07',{})[0][0]
    assert checked['route_length_m']==checked['recomputed_xyz_length_m']>210.
    assert checked['recomputed_planar_length_m']==210.
    assert checked['horizon']['proposed_last_anchor_arc_m']==pytest.approx(210.-15./3.6*6.4)


def test_distinct_selection_is_deterministic_and_overlaps_are_symmetric(tmp_path):
    make_catalog(tmp_path,shared=True)
    candidates=p.load_candidates(tmp_path,'town07',{})[0]
    first=p.select_slots(candidates,notes());second=p.select_slots(list(reversed(candidates)),notes())
    assert first==second
    bound=[r for r in first if r['route']]
    assert len(bound)==2 and len({r['route']['route_sha256'] for r in bound})==2
    assert all(r['corridor_conflicts'] and 'SHARED_ROAD_SECTION_REVIEW_NEEDED' in r['technical_conditions_unmet'] for r in bound)
    assert bound[0]['corridor_conflicts'][0]['other_slot_id']==bound[1]['slot_id']
    assert bound[1]['corridor_conflicts'][0]['other_slot_id']==bound[0]['slot_id']
    assert all(r['execution_allowed'] is r['training_data_approved'] is False for r in first)


def test_geometry_alias_cannot_fill_second_independent_slot(tmp_path):
    make_catalog(tmp_path,count=1)
    candidate=p.load_candidates(tmp_path,'town07',{})[0][0]
    alias=dict(candidate,route_id='alias',route_sha256='a'*64)
    slots=p.select_slots([candidate,alias],notes())
    assert sum(r['route'] is not None for r in slots)==1
    assert 'NO_DISTINCT_AVAILABLE_ROUTE_FOR_DIRECTION' in slots[1]['technical_conditions_unmet']


def test_selection_prefers_an_available_horizon_and_nonoverlapping_corridors(tmp_path):
    make_catalog(tmp_path,shared=False)
    candidates=p.load_candidates(tmp_path,'town07',{})[0]
    candidates[0]['horizon']['reasons']=['short']
    slots=p.select_slots(candidates,notes())
    assert {r['route']['route_id'] for r in slots if r['route']}=={candidates[1]['route_id'],candidates[2]['route_id']}
    assert not any(r['corridor_conflicts'] for r in slots)


def test_repeated_input_tamper_is_rejected(tmp_path):
    path=tmp_path/'x.json';path.write_text('{}');ledger={}
    p.read(path,ledger);path.write_text('{"x":1}')
    with pytest.raises(ValueError,match='changed'):p.read(path,ledger)


def test_output_freshness_and_overlap_rejected_before_any_sources(tmp_path):
    output=tmp_path/'output.json';output.write_text('existing')
    with pytest.raises(ValueError,match='fresh'):p.run(tmp_path/'catalogs',output)
    assert output.read_text()=='existing'
    with pytest.raises(ValueError,match='fresh'):p.run(tmp_path,tmp_path/'inside.json')


def test_main_with_real_manifest_loader_but_no_runtime_or_test_access(tmp_path,monkeypatch):
    catalogs=tmp_path/'catalogs'
    for name,_,_,_ in p.COHORT:
        make_catalog(catalogs,map_id=name)
        make_catalog(catalogs,map_id=name,kind='turn',scenario='left')
    def inventory(manifest):
        return dict(maps=[dict(id=r['id'],assets_complete=True) for r in manifest['maps']],
            profiles=[dict(id='packaged_0915',executable_exists=True,executable_is_runnable=True,
                python_api_exists=True,python_api_sha256_matches=True)])
    monkeypatch.setattr(p.coverage.inventory,'collect_static_inventory',inventory)
    original=p.read
    def guarded(path,*args,**kwargs):
        assert not any(part in ('town04','town15') for part in Path(path).parts)
        return original(path,*args,**kwargs)
    monkeypatch.setattr(p,'read',guarded)
    output=tmp_path/'result/cohort.json'
    p.main(['--catalog-maps-root',str(catalogs),'--output-json',str(output)])
    report=json.loads(output.read_text())
    assert report['slot_count']==24 and report['split_counts']==dict(train=18,dev_val=6)
    assert report['route_bound_count']==16 and report['execution_commands']==[]
    assert report['source_and_input_sha256_before']==report['source_and_input_sha256_after']
    assert report['test_catalogs_or_payloads_opened'] is report['world_rpc_performed'] is report['gpu_used'] is False
    assert next(r for r in report['slots'] if r['map_id']=='town03')['split_notes']['legacy_split_conflict'] is True
