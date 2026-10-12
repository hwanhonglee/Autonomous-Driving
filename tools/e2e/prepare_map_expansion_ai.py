"""HH_260906 - Add a separately owned physical TRAIN route; preserve fixed DEV bytes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from scripts.e2e.map_expansion_contract import AI_V2_PARENT_SHA256, validate_expansion


ROOT=Path(__file__).resolve().parents[2]
AI_V2_SPLIT_SHA256={'train':'69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace',
                    'dev':'417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3'}
TOWN03_ROUTES={
    'straight':(ROOT/'storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/straight/routes/town03/straight/town03_straight_s0000_p03.json',
                'aee6b0fd6e5a6dfa882cd172b4cb10c2e0865d2729d3233f5f121bb4be0674d4'),
    'right':(ROOT/'storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/turn/routes/town03/right/town03_right_s0000_p03.json',
             '8aa4040617f733e5df11910d3ac8113307e05a53334f29bc3387eb1370e2bf3c')}
LABEL_TABLE={'1':'road','24':'lane_marking','2':'sidewalk','7':'traffic_light_fixture','8':'traffic_sign','6':'pole'}


def digest(p):
    """HH_260906 - Stream large source and dataset hashes to bound memory use."""
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def require(condition,message):
    if not condition:raise ValueError(message)


def checked_json(path):
    if path.is_symlink() or not path.is_file():raise ValueError(f'missing or symlinked provenance: {path}')
    return json.loads(path.read_text())


def build(baseline,capture,output):
    """HH_260906 - Never convert mapping RGB/semantics into vehicle-control training labels."""
    parent=json.loads((baseline/'manifest.json').read_text())
    owner=json.loads((capture/'owner_result.json').read_text())
    plan=json.loads((capture/'owner_plan.json').read_text())
    source=capture/'mapping';summary=json.loads((source/'summary.json').read_text())
    if output.exists() or parent.get('image_size_wh')!=[640,360]:raise ValueError('fresh native-resolution output required')
    if (owner['exit_code']!=0 or owner.get('source_bytes_unchanged_and_archived') is not True
            or summary['status']!='COMPLETE_ORACLE_PLUMBING_ONLY' or not summary['physics_driving']
            or summary['collision_events'] or summary['cleanup_errors'] or not summary['live_replay_exact_match']
            or summary['town']!='Town01' or summary['travel_m']<3):raise ValueError('unqualified physical capture')
    route=Path(plan['route_path'])
    if digest(route)!=plan['route_sha256'] or summary['route_sha256']!=plan['route_sha256']:
        raise ValueError('route bytes changed')
    arrays={}
    for split in ('train','dev'):
        ref=parent['splits'][split]
        if digest(baseline/ref['file'])!=ref['sha256']:raise ValueError('parent arrays changed')
        with np.load(baseline/ref['file'],allow_pickle=False) as d:arrays[split]=(d['rgb'].copy(),d['target'].copy())
    rows=json.loads((source/'records.json').read_text());kept=[]
    for row in rows:
        if not kept or np.hypot(row['ego_x']-kept[-1]['ego_x'],row['ego_y']-kept[-1]['ego_y'])>=.25:kept.append(row)
    if len(kept)<24:raise ValueError('too few distinct capture positions')
    xy=np.array([[r['ego_x'],r['ego_y']] for r in kept])
    dev=np.array([[r['ego_x'],r['ego_y']] for r in parent['splits']['dev']['source_records']])
    train=np.array([[r['ego_x'],r['ego_y']] for r in parent['splits']['train']['source_records']])
    gap=float(np.sqrt(((xy[:,None]-dev[None])**2).sum(-1)).min())
    train_gap=float(np.sqrt(((xy[:,None]-train[None])**2).sum(-1)).min())
    if gap<80 or train_gap<20:raise ValueError('new route overlaps existing DEV or TRAIN corridor')
    images=[];targets=[];hashes={};labels=parent['labels']
    for row in kept:
        for suffix in ('.png','.npz'):
            p=source/(row['stem']+suffix)
            if not p.resolve().is_relative_to(source.resolve()) or p.is_symlink():raise ValueError('unsafe frame path')
            hashes[str(p)]=digest(p)
        rgb=np.asarray(Image.open(source/(row['stem']+'.png')).convert('RGB'))
        with np.load(source/(row['stem']+'.npz'),allow_pickle=False) as d:tags=d['semantic'][...,2].copy()
        if rgb.shape!=(360,640,3) or tags.shape!=(360,640):raise ValueError('native frame shape differs')
        y=np.zeros(tags.shape,dtype=np.uint8)
        for tag,name in summary['label_table'].items():y[tags==int(tag)]=labels.index(name)
        images.append(rgb);targets.append(y)
    x=np.concatenate([arrays['train'][0],np.stack(images)])
    y=np.concatenate([arrays['train'][1],np.stack(targets)])
    manifest=json.loads(json.dumps(parent))
    manifest['parent_manifest_sha256']=digest(baseline/'manifest.json')
    manifest['new_physical_routes']=True
    manifest['expansion']=dict(added_frames=len(kept),minimum_dev_distance_m=gap,minimum_old_train_distance_m=train_gap,
        owner_exit_code=owner['exit_code'],collision_events=summary['collision_events'],physical_capture=True,
        source_bytes_unchanged_and_archived=True,dev_used_for_training=False,
        route_sha256=plan['route_sha256'],route_id=route.stem,ignores_traffic_lights=summary['ignores_traffic_lights'],
        source_summary_sha256=digest(source/'summary.json'),owner_result_sha256=digest(capture/'owner_result.json'),
        added_class_pixels=np.bincount(np.stack(targets).ravel(),minlength=len(labels)).tolist(),
        scope='RGB semantic TRAIN expansion only; not autonomous-control training or new functionality approval')
    ref=manifest['splits']['train']
    ref.update(count=len(x),class_pixels=np.bincount(y.ravel(),minlength=len(labels)).tolist(),
        source_records=ref['source_records']+[dict(r,source_collection=str(source)) for r in kept],
        source_hashes={**ref['source_hashes'],**hashes})
    validate_expansion(manifest,parent,digest(baseline/'manifest.json'))
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'train.npz',rgb=x,target=y)
    ref.update(file='train.npz',sha256=digest(output/'train.npz'))
    shutil.copy2(baseline/'dev.npz',output/'dev.npz')
    if digest(output/'dev.npz')!=parent['splits']['dev']['sha256']:raise ValueError('DEV copy differs')
    with (output/'manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
    print(json.dumps(manifest['expansion']),flush=True)


def frozen_ai_v2(baseline):
    """HH_260906 - Pin the published Town01 TRAIN497/DEV118 manifest and array bytes."""
    path=baseline/'manifest.json';parent=checked_json(path)
    require(digest(path)==AI_V2_PARENT_SHA256,'AI v2 parent manifest SHA differs')
    require(parent.get('image_size_wh')==[640,360] and parent.get('model_inputs')==['RGB']
            and parent.get('depth_input') is False,'AI v2 input contract differs')
    for split,count in (('train',497),('dev',118)):
        ref=parent['splits'][split];data=baseline/ref['file']
        require(ref['count']==count and len(ref['source_records'])==count
                and ref['source_summary'].get('town')=='Town01',f'AI v2 {split} source split differs')
        require(not data.is_symlink() and data.is_file()
                and data.resolve().is_relative_to(baseline.resolve())
                and ref['sha256']==AI_V2_SPLIT_SHA256[split]
                and digest(data)==AI_V2_SPLIT_SHA256[split],f'AI v2 {split} bytes differ')
    return parent


def checked_frame(source,stem,suffix):
    path=source/(stem+suffix)
    require(path.is_file() and not path.is_symlink()
            and path.resolve().is_relative_to(source.resolve()),f'unsafe or missing sensor frame: {path}')
    return path


def collector_arg(argv,flag):
    require(argv.count(flag)==1 and argv.index(flag)+1<len(argv),f'collector {flag} differs')
    return argv[argv.index(flag)+1]


def owned_town03_capture(capture,kind):
    """HH_260906 - Require exact route, complete owner lifecycle and archived source hashes."""
    require(not capture.is_symlink(),'symlinked capture root')
    route,route_sha=TOWN03_ROUTES[kind]
    plan=checked_json(capture/'owner_plan.json')
    started=checked_json(capture/'owner_started.json')
    result=checked_json(capture/'owner_result.json')
    source=capture/'mapping';summary=checked_json(source/'summary.json')
    require(not source.is_symlink(),'symlinked mapping source')
    require(digest(route)==route_sha and checked_json(route).get('town')=='Town03',
            f'Town03 {kind} route bytes differ')
    require(Path(plan.get('route_path','')).resolve()==route.resolve()
            and plan.get('route_sha256')==started.get('route_sha256')
            ==summary.get('route_sha256')==route_sha,f'Town03 {kind} route provenance differs')
    require(plan.get('map')==started.get('map')==summary.get('town')=='Town03'
            and plan.get('capture_mode')==result.get('capture_mode')=='mapping-probe'
            and plan.get('quality')==started.get('quality')=='Epic'
            and plan.get('source_bytes_archived') is True
            and result.get('source_bytes_unchanged_and_archived') is True
            and result.get('exit_code')==0
            and plan.get('learned_model_control') is False
            and result.get('learned_model_control') is False
            and summary.get('learned_model_control') is False
            and summary.get('research_split_intent')=='town03_additive_train_candidate'
            and summary.get('status')=='COMPLETE_ORACLE_PLUMBING_ONLY'
            and summary.get('physics_driving') is True
            and summary.get('motion_mode')=='basic-agent'
            and summary.get('controller')=='CARLA BasicAgent'
            and summary.get('ignores_traffic_lights') is True
            and summary.get('oracle_semantics') is True
            and summary.get('collision_events')==0
            and summary.get('live_replay_exact_match') is True
            and summary.get('cleanup_errors')==[]
            and summary.get('frame_count')==240
            and summary.get('travel_m',0)>=3
            and summary.get('label_table')==LABEL_TABLE,
            f'Town03 {kind} owned physical capture failed')
    argv=plan.get('collector_argv',[])
    require(len(argv)>=2 and Path(argv[0]).resolve()==source.resolve()
            and Path(argv[1]).resolve()==route.resolve()
            and collector_arg(argv,'--motion-mode')=='basic-agent'
            and collector_arg(argv,'--frames')=='240'
            and argv.count('--town03-generalization-train')==1
            and argv.count('--ignore-traffic-lights')==1
            and '--town07-generalization-test' not in argv
            and plan.get('worker_path')=='scripts/e2e/collect_carla_mapping_probe.py',
            f'Town03 {kind} collector opt-in differs')
    for stage in ('ready','after_capture','stopped'):
        lifecycle=checked_json(capture/'lifecycle'/f'{stage}.json')
        require(lifecycle.get('status')=='PASS' and lifecycle.get('stage')==stage
                and lifecycle.get('expected_map')=='Town03'
                and lifecycle.get('owner_pid')==started.get('server_pid')
                and (stage!='stopped' or lifecycle.get('port_released') is True),
                f'Town03 {kind} lifecycle failed at {stage}')
    sources=plan.get('source_sha256',{});checks=result.get('source_checks',{})
    require(bool(sources) and checks.keys()==sources.keys() and all(checks.values())
            and all(result.get('external_source_checks',{}).values()),
            f'Town03 {kind} source checks failed')
    for relative,expected in sources.items():
        archived=capture/'provenance'/relative
        require(archived.resolve().is_relative_to((capture/'provenance').resolve())
                and archived.is_file() and not archived.is_symlink()
                and digest(archived)==expected,
                f'Town03 {kind} archived source differs: {relative}')
    proof=dict(town='Town03',route_id=route.stem,route_sha256=route_sha,
        owner_exit_code=0,owner_plan_sha256=digest(capture/'owner_plan.json'),
        owner_started_sha256=digest(capture/'owner_started.json'),
        owner_result_sha256=digest(capture/'owner_result.json'),
        source_summary_sha256=digest(source/'summary.json'),
        source_records_sha256=digest(source/'records.json'),
        source_archived_count=len(sources),raw_frames=240)
    return source,proof


def town03_records(source):
    """HH_260906 - Verify all physical RGB/semantic pairs before sampling TRAIN frames."""
    rows=checked_json(source/'records.json')
    require(isinstance(rows,list) and len(rows)==240,'Town03 needs 240 raw records')
    hashes=hashlib.sha256();previous_frame=None;previous_time=None
    for index,row in enumerate(rows):
        stem=f'frames/{index:04d}'
        require(row.get('stem')==stem and type(row.get('frame')) is int
                and (previous_frame is None or row['frame']>previous_frame),
                'Town03 frame order or stem differs')
        require(isinstance(row.get('timestamp_s'),(int,float))
                and math.isfinite(row['timestamp_s'])
                and (previous_time is None or row['timestamp_s']>previous_time)
                and all(isinstance(row.get(key),(int,float)) and math.isfinite(row[key])
                        for key in ('ego_x','ego_y')),
                'Town03 timestamp or pose is invalid')
        previous_frame,previous_time=row['frame'],row['timestamp_s']
        for suffix in ('.png','.npz'):
            path=checked_frame(source,stem,suffix)
            hashes.update(f'{stem}{suffix}:{digest(path)}\n'.encode())
        with Image.open(source/(stem+'.png')) as image:
            require(image.size==(640,360) and image.mode=='RGB','Town03 RGB format differs')
            image.verify()
        with np.load(source/(stem+'.npz'),allow_pickle=False) as archive:
            semantic=archive['semantic']
            require(semantic.shape==(360,640,4) and semantic.dtype==np.uint8,
                    'Town03 semantic format differs')
    return rows,hashes.hexdigest()


def dedup_town03(rows_per_route):
    """HH_260906 - Use only Town03 XY, checking every previously retained same-Town position."""
    require(len(rows_per_route)==2,'exactly two Town03 routes required')
    selected=[];independent_counts=[];global_xy=[]
    for rows in rows_per_route:
        own_xy=[];kept=[]
        for row in rows:
            xy=(float(row['ego_x']),float(row['ego_y']))
            if all(math.hypot(xy[0]-old[0],xy[1]-old[1])>=.25 for old in own_xy):
                own_xy.append(xy)
            if all(math.hypot(xy[0]-old[0],xy[1]-old[1])>=.25 for old in global_xy):
                kept.append(dict(row))
                global_xy.append(xy)
        independent_counts.append(len(own_xy));selected.append(kept)
    require(all(n>=120 for n in independent_counts),
            'each Town03 route needs at least 120 independently retained frames')
    require(all(len(rows)>=120 for rows in selected),
            'each Town03 route needs at least 120 globally unique TRAIN frames')
    return selected,independent_counts


def preflight_town03(baseline,straight,right):
    """HH_260906 - Qualify both Town03 owner captures read-only; Town07 is never opened."""
    parent=frozen_ai_v2(baseline)
    require(straight.resolve()!=right.resolve(),'Town03 capture roots must differ')
    sources=[];proofs=[];all_rows=[]
    for capture,kind in ((straight,'straight'),(right,'right')):
        source,proof=owned_town03_capture(capture,kind)
        rows,raw_sha=town03_records(source)
        proof['raw_sensor_manifest_sha256']=raw_sha
        sources.append(source);proofs.append(proof);all_rows.append(rows)
    selected,own_counts=dedup_town03(all_rows)
    for proof,rows,own_count,source in zip(proofs,selected,own_counts,sources):
        proof['independent_retained_frames']=own_count
        proof['added_frames']=len(rows)
        for row in rows:
            row['source_town']='Town03'
            row['route_sha256']=proof['route_sha256']
            row['source_collection']=str(source.resolve())
    return dict(parent=parent,sources=sources,selected=selected,routes=proofs,
                added_frames=sum(map(len,selected)))


def build_town03(baseline,straight,right,output):
    """HH_260906 - Append qualified Town03 RGB labels to AI v2 TRAIN; copy DEV bytes."""
    require(not output.exists(),'fresh output required')
    ready=preflight_town03(baseline,straight,right)
    parent=ready['parent'];arrays={}
    for split in ('train','dev'):
        ref=parent['splits'][split]
        with np.load(baseline/ref['file'],allow_pickle=False) as archive:
            rgb,target=archive['rgb'].copy(),archive['target'].copy()
        require(rgb.shape==(ref['count'],360,640,3)
                and target.shape==(ref['count'],360,640)
                and rgb.dtype==target.dtype==np.uint8,f'AI v2 {split} arrays differ')
        arrays[split]=(rgb,target)
    labels=parent['labels'];images=[];targets=[];hashes={}
    for source,proof,rows in zip(ready['sources'],ready['routes'],ready['selected']):
        pixels=np.zeros(len(labels),dtype=np.int64)
        for row in rows:
            rgb_path=checked_frame(source,row['stem'],'.png')
            sem_path=checked_frame(source,row['stem'],'.npz')
            hashes[str(rgb_path)]=digest(rgb_path);hashes[str(sem_path)]=digest(sem_path)
            with Image.open(rgb_path) as original:rgb=np.asarray(original.convert('RGB'))
            with np.load(sem_path,allow_pickle=False) as archive:tags=archive['semantic'][...,2].copy()
            target=np.zeros(tags.shape,dtype=np.uint8)
            for tag,name in LABEL_TABLE.items():target[tags==int(tag)]=labels.index(name)
            pixels+=np.bincount(target.ravel(),minlength=len(labels))
            images.append(rgb);targets.append(target)
        proof['added_class_pixels']=pixels.tolist()
    x=np.concatenate((arrays['train'][0],np.stack(images)))
    y=np.concatenate((arrays['train'][1],np.stack(targets)))
    require(np.array_equal(x[:497],arrays['train'][0])
            and np.array_equal(y[:497],arrays['train'][1]),'AI v2 TRAIN array prefix changed')
    manifest=json.loads(json.dumps(parent))
    manifest['parent_manifest_sha256']=AI_V2_PARENT_SHA256
    manifest['new_physical_routes']=True
    manifest['split_policy']='fixed Town01 TRAIN/DEV; owner-qualified Town03 TRAIN; 0.25m within-Town dedup; Town07 sealed'
    manifest['expansion']=dict(split_mode='town_disjoint_train_v1',
        added_frames=ready['added_frames'],parent_town='Town01',added_town='Town03',held_out_town='Town07',
        same_town_dedup_m=.25,cross_town_xy_compared=False,
        minimum_dev_distance_m=None,minimum_old_train_distance_m=None,
        owner_exit_code=0,collision_events=0,source_bytes_unchanged_and_archived=True,
        physical_capture=True,dev_used_for_training=False,routes=ready['routes'],
        scope='RGB semantic TRAIN expansion only; no learned-control or HD-map approval')
    ref=manifest['splits']['train']
    ref.update(count=len(x),class_pixels=np.bincount(y.ravel(),minlength=len(labels)).tolist(),
        source_records=ref['source_records']+[row for rows in ready['selected'] for row in rows],
        source_hashes={**ref['source_hashes'],**hashes})
    validate_expansion(manifest,parent,AI_V2_PARENT_SHA256)
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'train.npz',rgb=x,target=y)
    ref.update(file='train.npz',sha256=digest(output/'train.npz'))
    shutil.copy2(baseline/parent['splits']['dev']['file'],output/'dev.npz')
    require(digest(output/'dev.npz')==AI_V2_SPLIT_SHA256['dev'],'copied AI v2 DEV differs')
    with (output/'manifest.json').open('x') as stream:json.dump(manifest,stream,indent=2)
    return dict(parent_manifest_sha256=AI_V2_PARENT_SHA256,parent_train_frames=497,
                parent_dev_frames=118,added_frames=ready['added_frames'],routes=ready['routes'],
                town07_train_frames=0,cross_town_xy_compared=False)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--town03-generalization-train',action='store_true')
    p.add_argument('--preflight-only',action='store_true')
    p.add_argument('paths',nargs='+',type=Path)
    a=p.parse_args()
    if a.town03_generalization_train:
        if len(a.paths)!=4:p.error('Town03 mode requires BASELINE STRAIGHT_CAPTURE RIGHT_CAPTURE OUTPUT')
        baseline,straight,right,output=a.paths
        if a.preflight_only:
            ready=preflight_town03(baseline,straight,right)
            print(json.dumps(dict(parent_manifest_sha256=AI_V2_PARENT_SHA256,
                parent_train_frames=497,parent_dev_frames=118,added_frames=ready['added_frames'],
                routes=ready['routes'],town07_train_frames=0,cross_town_xy_compared=False),indent=2))
        else:print(json.dumps(build_town03(baseline,straight,right,output),indent=2))
    else:
        if a.preflight_only or len(a.paths)!=3:p.error('legacy mode requires BASELINE CAPTURE OUTPUT')
        build(*a.paths)
