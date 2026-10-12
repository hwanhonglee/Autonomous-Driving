#!/usr/bin/env python3
"""HH_260906 - Inspect six frozen TRAIN-only STOPMIX teachers without fitting, selecting or deploying a model."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import ModuleType

SCHEMA = 'portable_e2e.stopmix_training_oracle_probe.v1'
PLAN_SCHEMA = 'portable_e2e.stopmix_training_oracle_plan.v1'
WORKSPACE = Path.home()/'personal/hwanhong/portable_e2e'
HELPER_PATH = WORKSPACE/'benchmarks/training_profile_v1/profile_portable_training.py'
HELPER_SHA = 'a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
SOURCE_COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
LOSS_SHA = 'cdf37262024435c3a04888bf816c24db1b769b87034f08e1541c02d22da80973'
MODEL_PATH = 'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json'
MODEL_SHA = '29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602'
MODEL_ID = 'portable_e2e.perspective_trajectory.physical_stopmix.v1'
PARENT_ID = 'hh260909-stopmix-development-ab-3seeds-v1'
DURATION_ID = 'hh260909-stopmix-duration-10epochs-v1'
CAMPAIGNS = {'parent1540':PARENT_ID,'continued2870':DURATION_ID}
CHECKPOINTS = {
    '20260903':{'parent1540':'9b949b17b3708bed5ba25eb5721187a56f4f77c11c8461e685cbbd995259f8bf',
        'continued2870':'bb8844dd77559480bf6c2f1c897c296fb9871731e2cb089d2143fb79d02bb908'},
    '20260904':{'parent1540':'2789a33c313923ee6084ddaa6dc1a3937ebedd22d5e52db360c0e45fc841e851',
        'continued2870':'3ef0acba3ae3ad4ccf7a0c124722f68365cd4a0fbc800266b729de00708cf90b'},
    '20260905':{'parent1540':'ce6ffa0bbd80db39d0c9952d1a1e8d67543ff1c69ccc9a7e76d110ffefd5d9df',
        'continued2870':'bb75905bf083fd7d02fd2c63004dcbd1912c00909290afc1b6a4b8f039057de7'},
}
RECEIPTS = {
    f'runs/campaigns/{PARENT_ID}/plan.json':'f06c68ce47aaee6941a3ca42559460ac5837b53f669bf195fa15b5d5e80a05d9',
    f'runs/campaigns/{PARENT_ID}/status.json':'9f40946bde5395c61e68a5ac5fa476a210569a7fe8f6f6d58d4588037e4ff2bc',
    f'runs/campaigns/{DURATION_ID}/plan.json':'bcdf31f09838750cd24fcabf063a9baeac8fb317ed045e6a41d734fff2e6233b',
    f'runs/campaigns/{DURATION_ID}/status.json':'52a0452f707285acc46f29d6972049c3ef3e4cbd4d606fb038088becd6d7477f',
    f'runs/campaigns/{DURATION_ID}/WORKFLOW_SHA256SUMS':'a16779b84b44621398eb4865559d0bffcbbb4117d2a3aea550dd9556c7b9fa5d',
}
INPUT_KEYS = ('images','calibration','ego_history','ego_history_mask','route_xy','route_mask')
GROUPS = ('unavailable_masks','stationary_hold','moving_to_stop','continuing_motion','other_motion')
PASS_TIMEOUT, TOTAL_TIMEOUT, EXTERNAL_TIMEOUT, RESERVE = 120, 1020, 1080, 90
DENIALS = dict(model_training=False,optimizer_created=False,validation_neural_inference=False,test_neural_inference=False,
    test_used_for_selection=False,model_promotion=False,training_data_approved=False,vehicle_control_approved=False,
    source_or_label_changes=False,near_tie_tolerance_used=False)


def require(value,message):
    if not value:raise ValueError(message)


def encoded(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def projection_definition(raw):
    # HH_260906 - Project only the authenticated function prefix; this is a diagnostic derivative, not a new loss implementation.
    require(type(raw) is bytes and sha(raw)==LOSS_SHA,'exact frozen loss source required')
    tree=ast.parse(raw.decode());matches=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='trajectory_loss']
    require(len(matches)==1,'unique original loss function required');original=matches[0]
    assignments=[(i,n) for i,n in enumerate(original.body) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='oracle_candidate' for t in n.targets)]
    require(len(assignments)==1,'unique original oracle assignment required')
    index,node=assignments[0]
    expected=ast.parse('oracle_candidate = per_candidate.detach().argmin(dim=1)').body[0]
    require(ast.dump(node,include_attributes=False)==ast.dump(expected,include_attributes=False) and not original.decorator_list,
        'original oracle boundary structure changed')
    prior=original.body[:index]
    require(isinstance(prior[-1],ast.Assign) and ast.dump(prior[-1],include_attributes=False)==ast.dump(ast.parse(
        'per_candidate = per_candidate + float(cfg.final_displacement_weight) * final_displacement').body[0],include_attributes=False),
        'composite final-displacement boundary changed')
    projected=deepcopy(original);projected.body=deepcopy(prior)+[ast.Return(value=ast.Name(id='per_candidate',ctx=ast.Load()))]
    require(ast.dump(ast.arguments(**{field:deepcopy(getattr(projected.args,field)) for field in projected.args._fields}),include_attributes=False)
        == ast.dump(original.args,include_attributes=False),'projected signature changed')
    require([ast.dump(n,include_attributes=False) for n in projected.body[:-1]]==[ast.dump(n,include_attributes=False) for n in prior],
        'projected prefix statements changed')
    derived=ast.fix_missing_locations(ast.Module(body=[projected],type_ignores=[]))
    prefix=b''.join(raw.splitlines(keepends=True)[original.lineno-1:node.lineno-1])
    proof=dict(original_loss_sha256=sha(raw),original_function_ast_sha256=sha(ast.dump(original,include_attributes=False).encode()),
        prefix_source_sha256=sha(prefix),prefix_first_line=original.lineno,prefix_last_line=node.lineno-1,
        derived_ast_sha256=sha(ast.dump(derived,include_attributes=False).encode()),
        modification='Return per_candidate immediately before the unchanged detached argmin; all prior statements and signature unchanged.')
    return derived,proof


def projected_costs(loss_module):
    raw=Path(loss_module.__file__).read_bytes();tree,proof=projection_definition(raw)
    namespace=dict(vars(loss_module));exec(compile(tree,'<authenticated-loss-prefix>','exec'),namespace)
    require(sha(Path(loss_module.__file__).read_bytes())==LOSS_SHA,'loss source changed while projecting')
    return namespace['trajectory_loss'],proof


def motion_group(raw_vx,target_speed,valid):
    require(type(raw_vx) in (int,float) and math.isfinite(raw_vx),'finite raw current speed required')
    require(len(target_speed)==len(valid)==64 and all(type(v) is bool for v in valid)
        and all(v is None and not mask or type(v) in (int,float) and math.isfinite(v) for v,mask in zip(target_speed,valid)),
        'full64 finite-or-masked-null original target/mask required')
    if not all(valid):return 'unavailable_masks'
    if abs(raw_vx)<=.1 and all(v<=.1 for v in target_speed):return 'stationary_hold'
    if raw_vx>.1 and all(v<=.1 for v in target_speed[-10:]):return 'moving_to_stop'
    if all(v>=.5 for v in target_speed):return 'continuing_motion'
    return 'other_motion'


def candidate_rows(xy,speed,logits,target,target_speed,valid,target_yaw,cfg,loss_module,projected):
    torch=loss_module.torch
    require(xy.ndim==4 and xy.shape[1:]==(12,64,2),'complete K12 x64 shape required')
    require(target_yaw is not None,'original target yaw is required for composite teacher diagnosis')
    costs=projected(xy,speed,logits,target,target_speed,valid,cfg,target_yaw=target_yaw)
    native=loss_module.trajectory_loss(xy,speed,logits,target,target_speed,valid,cfg,target_yaw=target_yaw)
    require(costs.shape==logits.shape and bool(torch.isfinite(costs).all()),'finite complete composite costs required')
    oracle=costs.detach().argmin(dim=1)
    require(torch.equal(oracle,native['oracle_candidate_index']) and torch.equal(
        costs.gather(1,oracle[:,None]).mean().detach(),native['regression_loss']),'projected oracle/regression differs from original loss')
    mask=valid[:,None].to(xy.dtype);count=mask.sum(dim=2).clamp_min(1.)
    ade=((xy-target[:,None]).norm(dim=-1)*mask).sum(dim=2)/count
    arrays=[value.detach().cpu().tolist() for value in (costs,logits,logits.softmax(dim=1),ade,valid)]
    rows=[]
    for c,l,p,a,m in zip(*arrays):
        minimum=min(c);ordered=sorted(c);selected=max(range(12),key=lambda i:l[i]);best=c.index(minimum)
        ties=[i for i,value in enumerate(c) if value==minimum]
        rows.append(dict(candidate_composite_costs=c,candidate_logits=l,candidate_probabilities=p,candidate_ade_m=a,
            candidate_cost_dtype=str(costs.dtype),margin_arithmetic='Difference of the two smallest stored costs; exact equality determines ties.',
            composite_oracle_index=best,exact_minimum_indices=ties,exact_minimum_count=len(ties),
            minimum_composite_cost=minimum,second_minus_first_composite_cost=ordered[1]-minimum,
            selected_candidate_index=selected,selected_in_exact_minimum=selected in ties,
            selected_composite_regret=c[selected]-minimum,ade_oracle_index=a.index(min(a)),
            selected_ade_m=a[selected],composite_oracle_ade_m=a[best],oracle_ade_m=min(a),
            original_target_valid=m,native_oracle_and_regression_exact=True))
    return rows


def summarize_rows(rows):
    require(bool(rows),'nonempty rows required')
    keys=('selected_ade_m','composite_oracle_ade_m','oracle_ade_m','selected_composite_regret','second_minus_first_composite_cost')
    def aggregate(items):
        return dict(sample_count=len(items),selected_in_exact_minimum_count=sum(r['selected_in_exact_minimum'] for r in items),
            multiple_exact_minimum_count=sum(r['exact_minimum_count']>1 for r in items),
            raw_vx_nonpositive_count=sum(r['raw_current_vx_mps']<=0 for r in items),
            selected_composite_oracle_index_agreement_count=sum(r['selected_candidate_index']==r['composite_oracle_index'] for r in items),
            composite_ade_oracle_index_agreement_count=sum(r['composite_oracle_index']==r['ade_oracle_index'] for r in items),
            selected_histogram={str(i):sum(r['selected_candidate_index']==i for r in items) for i in range(12)},
            composite_oracle_histogram={str(i):sum(r['composite_oracle_index']==i for r in items) for i in range(12)},
            means={k:sum(r[k] for r in items)/len(items) if items else None for k in keys})
    result={'all_samples':aggregate(rows)}
    for key in ('episode_id','target_motion_group','capture_phase'):
        categories=GROUPS if key=='target_motion_group' else sorted({r[key] for r in rows})
        result[key]={value:aggregate([r for r in rows if r[key]==value]) for value in categories}
    result['raw_nonpositive'] = aggregate([r for r in rows if r['raw_current_vx_mps']<=0])
    return result


def compare_teachers(parent,continued):
    identity=('sample_id','episode_id','sequence_index','anchor_timestamp_ns','model_input_sha256','target_sha256',
        'raw_current_vx_mps','target_motion_group','capture_phase')
    require(len(parent)==len(continued)==1147 and all(all(a[k]==b[k] for k in identity) for a,b in zip(parent,continued)),
        'teacher comparisons require identical ordered input/target identities')
    changed=[i for i,(a,b) in enumerate(zip(parent,continued)) if a['composite_oracle_index']!=b['composite_oracle_index']]
    disjoint=[i for i,(a,b) in enumerate(zip(parent,continued)) if not set(a['exact_minimum_indices'])&set(b['exact_minimum_indices'])]
    return dict(sample_count=1147,oracle_index_changed_count=len(changed),oracle_index_changed_indices=changed,
        exact_minimum_sets_disjoint_count=len(disjoint),exact_minimum_sets_disjoint_indices=disjoint,
        by_episode={episode:dict(sample_count=sum(r['episode_id']==episode for r in parent),
            changed_count=sum(parent[i]['episode_id']==episode for i in changed)) for episode in sorted({r['episode_id'] for r in parent})},
        interpretation='Same-lineage generator and selector changed between two fixed checkpoints; index changes alone are not instability or incorrect labels.')


def load_helper():
    require(HELPER_PATH.is_file() and not any(p.is_symlink() for p in (HELPER_PATH,*HELPER_PATH.parents)),'regular frozen helper required')
    raw=HELPER_PATH.read_bytes();require(sha(raw)==HELPER_SHA,'frozen helper mismatch')
    base=ModuleType('frozen_training_oracle_owner_helper');base.__file__=str(HELPER_PATH)
    exec(compile(raw,str(HELPER_PATH),'exec'),base.__dict__)
    return base


def source_paths(base):
    return (*base.SOURCE_PATHS,MODEL_PATH,'portable_e2e/audit_runtime.py','portable_e2e/evaluate.py','portable_e2e/runtime.py')


def plan_contract(base,projection):
    return dict(schema=PLAN_SCHEMA,diagnostic_id='hh260909-stopmix-train-oracle-six-pass-v1',source_commit=SOURCE_COMMIT,
        gpu_uuid=base.GPU_UUID,dataset=base.DATASET,dataset_manifest_sha256=base.MANIFEST_SHA,
        corpus_fingerprint_sha256=base.CORPUS_SHA,train_fingerprint_sha256=base.TRAIN_SHA,
        model_config=MODEL_PATH,model_config_sha256=MODEL_SHA,model_id=MODEL_ID,model_parameter_count=1056362,
        seeds=list(CHECKPOINTS),endpoints=list(CAMPAIGNS),checkpoint_sha256=CHECKPOINTS,original_receipt_sha256=RECEIPTS,
        expected_pass_count=6,expected_samples_per_pass=1147,expected_training_episodes=3,candidate_count=12,future_points=64,
        batch_size=4,num_workers=0,evaluation_split='train',loss_config=base.LOSS_CONFIG,loss_projection=projection,
        inference_pass_wall_timeout_seconds=PASS_TIMEOUT,internal_wall_timeout_seconds=TOTAL_TIMEOUT,external_wall_timeout_seconds=EXTERNAL_TIMEOUT,
        timing_scope='Per-inference-pass timer includes ordered CPU loading, transfer, forward, loss projection and row persistence. Checkpoint loading and source/integrity checks use the separate whole-study timer.',
        safety_reserve_seconds=RESERVE,finish_before_utc='2026-09-09T01:00:00Z',frozen_helper_sha256=HELPER_SHA,
        group_definition='Full64 masks: stationary_hold iff abs(raw vx)<=0.1 and every target speed<=0.1; moving_to_stop iff raw vx>0.1 and final10 speeds<=0.1; continuing_motion iff all64 speeds>=0.5; otherwise other_motion. Incomplete masks separate.',
        raw_nonpositive_definition='raw current vx<=0.0, no clamp; independent of the future-derived group.',
        capture_phase_definition='Inclusive [round(first_timestamp*1e9),round(last_timestamp*1e9)] from hash-bound TRAIN collection_config native phases; unmatched explicit, never retimed.',
        tie_definition='Exact equality to minimum composite tensor cost, no near-tie tolerance.',
        test_integrity_scope='The unchanged full-corpus validator may read test metadata/JSON/JPEG for integrity only; no val/test model forward.',
        **DENIALS)


def source_identity(plan,base,repo):
    environment=dict(os.environ,GIT_NO_LAZY_FETCH='1',GIT_ALLOW_PROTOCOL='',GIT_TERMINAL_PROMPT='0')
    def git(*args):return subprocess.check_output(['git','-c','protocol.allow=never',*args],cwd=repo,env=environment,timeout=10)
    require(git('rev-parse','HEAD').decode().strip()==SOURCE_COMMIT and not git('status','--porcelain').strip(),'clean b478 source required')
    result={}
    for name in source_paths(base):
        raw=base.regular(repo/name).read_bytes()
        require(raw==git('show',SOURCE_COMMIT+':'+name) and sha(raw)==plan['source_sha256'][name],'training source changed: '+name)
        result[name]=sha(raw)
    require(base.digest(Path(__file__))==plan['worker_source_sha256'] and base.digest(HELPER_PATH)==HELPER_SHA,'worker/helper source changed')
    return dict(result,**{'independent_oracle_probe.py':plan['worker_source_sha256'],'frozen_helper.py':HELPER_SHA})


def validate_plan(plan,base,projection):
    expected=plan_contract(base,projection)
    require(set(plan)==set(expected)|{'declared_at_utc','source_sha256','worker_source_sha256'},'exact plan inventory required')
    require(encoded({k:plan[k] for k in expected})==encoded(expected),'unreviewed oracle probe settings')
    require(set(plan['source_sha256'])==set(source_paths(base)),'full source inventory required')
    for value in [*plan['source_sha256'].values(),plan['worker_source_sha256']]:
        require(type(value) is str and len(value)==64 and all(c in '0123456789abcdef' for c in value),'exact source SHA required')
    now=datetime.now(timezone.utc)
    require(base.timestamp(plan['declared_at_utc'])<=now and (base.timestamp(plan['finish_before_utc'])-now).total_seconds()>=EXTERNAL_TIMEOUT+RESERVE,
        'prospective plan/deadline reserve differs')


def checkpoint_path(seed,endpoint):
    return WORKSPACE/'runs/campaigns'/CAMPAIGNS[endpoint]/f'seed_{seed}/B_drive_stop_mix/training/checkpoints/latest.pt'


def input_receipts(base,plan):
    pins={name:base.digest(WORKSPACE/name) for name in RECEIPTS};require(pins==RECEIPTS,'original completion receipts changed')
    normal=base.json_read(WORKSPACE/f'runs/campaigns/{PARENT_ID}/status.json')
    continued=base.json_read(WORKSPACE/f'runs/campaigns/{DURATION_ID}/status.json')
    require(normal['status']=='TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED' and len(normal['stages'])==18
        and continued['status']=='COMPLETE_NOT_PROMOTED' and continued['completed_stage_count']==24
        and continued['parent_source_and_output_postcheck_pass'] is True,'original completed lineages required')
    require(base.timestamp(continued['finished_at_utc'])<=base.timestamp(plan['declared_at_utc']),'probe must be declared after duration completion')
    for seed in CHECKPOINTS:
        run=f'seed_{seed}/B_drive_stop_mix'
        for endpoint,state in (('parent1540',normal),('continued2870',continued)):
            checkpoint=checkpoint_path(seed,endpoint);checksum=base.digest(checkpoint)
            require(checksum==CHECKPOINTS[seed][endpoint],'exact original checkpoint bytes required')
            pins[str(checkpoint.relative_to(WORKSPACE))]=checksum
            item=checkpoint.parents[2]
            for stage,name in (('train','training/run.json'),('evaluate','evaluation/metrics.json'),('audit','gate_v8.json')):
                record=next(r for r in state['stages'] if r['run']==run and r['stage']==stage)
                require(record['status']=='COMPLETE' and type(record['returncode']) is int and record['returncode']==0
                    and record['report']['sha256']==base.digest(item/name),'completed original stage bytes differ')
                if stage!='train':require(base.json_read(item/name)['checkpoint_sha256']==checksum,'original checkpoint/evaluation association differs')
                pins[str((item/name).relative_to(WORKSPACE))]=base.digest(item/name)
    return pins


def phase_metadata(dataset_root,episode_ids,base):
    result={};pins={};dataset=base.json_read(dataset_root/'dataset.json')
    for ref in dataset['episodes']:
        if ref['episode_id'] not in episode_ids:continue
        path=dataset_root/ref['manifest'];require(base.digest(path)==ref['sha256'],'TRAIN episode manifest changed')
        episode=base.json_read(path);require(episode['split']=='train','only TRAIN phase metadata permitted')
        provenance=episode['source_provenance'];config=path.parent/provenance['collection_config_file']
        require(base.digest(config)==provenance['collection_config_sha256'],'TRAIN capture metadata changed')
        phases=base.json_read(config).get('native_result',{}).get('capture_phases',{});bounds={}
        for name in ('stationary_warmup','driving','stationary_tail'):
            row=phases.get(name,{})
            if row.get('first_timestamp') is not None and row.get('last_timestamp') is not None:
                a,b=row['first_timestamp'],row['last_timestamp']
                require(type(a) in (int,float) and type(b) in (int,float) and math.isfinite(a) and math.isfinite(b) and a<=b,'invalid native phase bounds')
                bounds[name]=[round(a*1e9),round(b*1e9)]
        result[episode['episode_id']]=bounds
        pins[str(path.relative_to(dataset_root))]=ref['sha256'];pins[str(config.relative_to(dataset_root))]=provenance['collection_config_sha256']
    require(set(result)==set(episode_ids),'all three TRAIN phase metadata required')
    return result,pins


def tensor_sha(batch,index,keys):
    value=hashlib.sha256()
    for name in keys:
        tensor=batch[name][index].detach().cpu().contiguous()
        value.update(encoded([name,str(tensor.dtype),list(tensor.shape)]));value.update(tensor.numpy().tobytes())
    return value.hexdigest()


def completed_rows_intact(output,passes,base):
    # HH_260906 - Later passes must never replace bytes behind an earlier completed-pass receipt.
    return all(base.digest(output/p['samples_file'])==p['samples_sha256']
        for p in passes if p['status']=='PASS_COMPLETE_NOT_PROMOTED')


@contextmanager
def inference_alarm():
    # HH_260906 - Bound each inference pass while restoring, never restarting, the pre-existing whole-study deadline.
    remaining,interval=signal.getitimer(signal.ITIMER_REAL);started=time.monotonic()
    require(remaining>0,'whole-study alarm must already be active')
    signal.setitimer(signal.ITIMER_REAL,min(PASS_TIMEOUT,remaining))
    try:yield
    finally:
        left=remaining-(time.monotonic()-started)
        signal.setitimer(signal.ITIMER_REAL,max(left,1e-6),interval)


def analyze_pass(dataset,model,loss_module,projected,cfg,phases,path,torch):
    from torch.utils.data import DataLoader
    rows=[];started=time.monotonic()
    with path.open('x') as output,torch.no_grad(),inference_alarm():
        for batch in DataLoader(dataset,batch_size=4,shuffle=False,num_workers=0,pin_memory=False):
            require(time.monotonic()-started<PASS_TIMEOUT,'bounded inference pass timed out')
            tensors={k:v.to('cuda:0') if isinstance(v,torch.Tensor) else v for k,v in batch.items()}
            xy,speed,logits=model(*(tensors[k] for k in INPUT_KEYS))
            numeric=candidate_rows(xy,speed,logits,tensors['target_xy'],tensors['target_speed_mps'],tensors['target_valid'],
                tensors['target_yaw_rad'],cfg,loss_module,projected)
            for position,row in enumerate(numeric):
                example=dataset.examples[len(rows)];require(batch['sample_id'][position]==example.token,'fixed TRAIN sample order changed')
                # HH_260906 - Keep original source doubles for diagnostic groups; model float32 tensors and loss inputs remain unchanged.
                raw_vx=float(example.features[1]);targets=list(example.target_speed_mps)
                group=motion_group(raw_vx,targets,row['original_target_valid'])
                matches=[n for n,(a,b) in phases[example.episode_id].items() if a<=example.anchor_timestamp_ns<=b]
                require(len(matches)<=1,'ambiguous native capture phase')
                row.update(index=len(rows),sample_id=example.token,episode_id=example.episode_id,sequence_index=example.sequence_index,
                    anchor_timestamp_ns=example.anchor_timestamp_ns,camera_sha256=list(example.camera_sha256),source_manifest_sha256=example.source_manifest_sha256,
                    raw_current_vx_mps=raw_vx,raw_current_vx_nonpositive=raw_vx<=0,target_motion_group=group,
                    model_tensor_current_vx_mps=float(batch['ego_history'][position,-1,1]),raw_target_speed_mps=targets,
                    capture_phase=matches[0] if matches else 'unmatched_metadata_interval',
                    model_input_sha256=tensor_sha(batch,position,INPUT_KEYS),
                    target_sha256=tensor_sha(batch,position,('target_xy','target_speed_mps','target_yaw_rad','target_valid')))
                output.write(encoded(row).decode()+'\n');output.flush();rows.append(row)
    require(len(rows)==1147 and len({r['sample_id'] for r in rows})==1147,'full ordered TRAIN1147 required')
    return rows


def run(args,base):
    repo=WORKSPACE/'autoware_e2e';base.check_environment(WORKSPACE,repo)
    plan_path=base.regular(args.plan);plan=base.json_read(plan_path);plan_sha=base.digest(plan_path)
    _,projection=projection_definition(base.regular(repo/'portable_e2e/losses.py').read_bytes());validate_plan(plan,base,projection)
    dataset_root=(WORKSPACE/base.DATASET).resolve(strict=True)
    require(dataset_root.is_relative_to((WORKSPACE.parent/'dataset').resolve()),'dataset escaped personal storage')
    output=base.checked_output(args.output_dir,dataset_root,WORKSPACE);before=source_identity(plan,base,repo)
    report=dict(schema=SCHEMA,status='STARTED',started_at_utc=base.utc(),plan_sha256=plan_sha,source_sha256=before,
        loss_projection=projection,passes=[],teacher_changes=[],**DENIALS)
    with base.gpu_lease(WORKSPACE),base.bounded_signals(TOTAL_TIMEOUT):
        report['gpu_preflight']=base.gpu_idle(repo);original=input_receipts(base,plan)
        output.mkdir(parents=True,exist_ok=False);model=None;dataset=None;loaded=None;loader=None;phase_pins={}
        try:
            with (output/'plan.json').open('xb') as stream:stream.write(plan_path.read_bytes())
            for name,value in before.items():
                path=Path(__file__) if name=='independent_oracle_probe.py' else HELPER_PATH if name=='frozen_helper.py' else repo/name
                destination=output/'source'/name;destination.parent.mkdir(parents=True,exist_ok=True)
                with destination.open('xb') as stream:stream.write(path.read_bytes())
                require(base.digest(destination)==value,'source archive differs')
            sys.path.insert(0,str(repo));import torch
            from portable_e2e import losses as loss_module
            from portable_e2e.audit_runtime import _read_checkpoint_for_audit
            from portable_e2e.dataset import load_training_examples
            from portable_e2e.model import ModelConfig,PerspectiveTrajectoryModel,parameter_count
            from portable_e2e.torch_dataset import Common10TorchDataset
            from portable_e2e.train import _nested_tensors_are_finite,_seed_everything
            require(Path(loss_module.__file__).resolve()==repo/'portable_e2e/losses.py' and torch.cuda.device_count()==1
                and torch.get_num_threads()==16,'loaded source/device/thread identity differs')
            report['torch_device_uuid']=base.verify_torch_uuid(torch.cuda.get_device_properties(0))
            projected,proof=projected_costs(loss_module);require(proof==projection,'imported loss projection differs')
            loader=load_training_examples;loaded=loader(dataset_root,split='train',mode='planning',check_image_hashes=True)
            require(loaded.validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA,'full-corpus integrity changed')
            cfg=ModelConfig.from_mapping(base.json_read(repo/MODEL_PATH));dataset=Common10TorchDataset(loaded.examples,cfg,verify_image_sha256=True,split='train')
            episode_ids=sorted({e.episode_id for e in dataset.examples})
            require(len(dataset)==1147 and len(episode_ids)==3 and dataset.fingerprint_sha256==base.TRAIN_SHA
                and cfg.model_id==MODEL_ID and cfg.candidate_count==12,'exact TRAIN/STOPMIX contract required')
            phases,phase_pins=phase_metadata(dataset_root,episode_ids,base);report['capture_phase_metadata']=phases
            report['phase_metadata_sha256']=phase_pins;all_rows={}
            for seed in CHECKPOINTS:
                for endpoint in CAMPAIGNS:
                    require(source_identity(plan,base,repo)==before and input_receipts(base,plan)==original,'source/original input changed between passes')
                    record=dict(seed=seed,endpoint=endpoint,status='STARTED',started_at_utc=base.utc(),checkpoint_sha256=CHECKPOINTS[seed][endpoint])
                    report['passes'].append(record)
                    _seed_everything(0,torch.device('cuda:0'))
                    payload,checkpoint_cfg,training_ids,provenance=_read_checkpoint_for_audit(checkpoint_path=checkpoint_path(seed,endpoint),
                        expected_checkpoint_sha256=record['checkpoint_sha256'],corpus_fingerprint_sha256=base.CORPUS_SHA)
                    steps,seen=(1540,6155) if endpoint=='parent1540' else (2870,11470)
                    require(checkpoint_cfg==cfg and list(training_ids)==episode_ids and payload['dataset_fingerprint_sha256']==base.TRAIN_SHA
                        and payload['state']['global_step']==steps and payload['state']['samples_seen']==seen
                        and payload['train_config']['seed']==int(seed) and payload['loss_config']==base.LOSS_CONFIG,
                        'TRAIN-only checkpoint identity differs')
                    model=PerspectiveTrajectoryModel(cfg).to('cuda:0');model.load_state_dict(payload['model_state_dict'],strict=True)
                    require(_nested_tensors_are_finite(model.state_dict()) and parameter_count(model)==1056362,'model state/capacity differs')
                    model.requires_grad_(False);model.eval();del payload
                    relative=f'seed_{seed}/{endpoint}';item=output/relative;item.mkdir(parents=True)
                    rows=analyze_pass(dataset,model,loss_module,projected,loss_module.TrajectoryLossConfig.from_mapping(base.LOSS_CONFIG),phases,item/'samples.jsonl',torch)
                    require(source_identity(plan,base,repo)==before and input_receipts(base,plan)==original,'source/original input changed during pass')
                    record.update(status='PASS_COMPLETE_NOT_PROMOTED',completed_at_utc=base.utc(),sample_count=len(rows),
                        samples_file=relative+'/samples.jsonl',samples_sha256=base.digest(item/'samples.jsonl'),checkpoint_validation=provenance,
                        summary=summarize_rows(rows));all_rows[(seed,endpoint)]=rows
                    del model;model=None;torch.cuda.empty_cache()
                report['teacher_changes'].append(dict(seed=seed,**compare_teachers(all_rows[(seed,'parent1540')],all_rows[(seed,'continued2870')])))
            require(len(report['passes'])==6 and all(p['status']=='PASS_COMPLETE_NOT_PROMOTED' for p in report['passes']),'six full frozen passes required')
            reference=all_rows[(next(iter(CHECKPOINTS)),'parent1540')]
            for rows in all_rows.values():compare_teachers(reference,rows)
            report['all_six_ordered_model_inputs_and_targets_identical']=True
            report['status']='SIX_TRAIN_PASSES_COMPLETE_NOT_PROMOTED'
        except BaseException as error:
            report.update(status='FAILED_OR_PARTIAL_NOT_PROMOTED',error_type=type(error).__name__,error=str(error))
        finally:
            with base.finalization_signals():
                errors=[]
                if model is not None:del model
                checks={'source':lambda:source_identity(plan,base,repo)==before,
                    'original_checkpoint_and_report_inputs':lambda:input_receipts(base,plan)==original,
                    'plan':lambda:base.digest(plan_path)==plan_sha==base.digest(output/'plan.json'),
                    'source_archives':lambda:all(base.digest(output/'source'/n)==v for n,v in before.items()),
                    'completed_sample_rows':lambda:completed_rows_intact(output,report['passes'],base),
                    'dataset_manifest':lambda:base.digest(dataset_root/'dataset.json')==base.MANIFEST_SHA,
                    'phase_metadata':lambda:all(base.digest(dataset_root/n)==v for n,v in phase_pins.items())}
                if loader is not None:
                    def corpus():
                        after=loader(dataset_root,split='train',mode='planning',check_image_hashes=True)
                        return after.validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA and after.fingerprint_sha256==loaded.fingerprint_sha256
                    checks['full_corpus']=corpus
                for name,operation in checks.items():
                    try:require(operation(),name+' changed')
                    except BaseException as error:errors.append(dict(check=name,error_type=type(error).__name__))
                report.update(completed_at_utc=base.utc(),postcheck_errors=errors,original_input_sha256=original,
                    completed_pass_count=sum(p['status']=='PASS_COMPLETE_NOT_PROMOTED' for p in report['passes']),
                    source_and_original_inputs_unchanged=not errors,deadline_met=datetime.now(timezone.utc)<base.timestamp(plan['finish_before_utc']))
                if errors or not report['deadline_met']:report['status']='FAILED_OR_PARTIAL_NOT_PROMOTED'
                base.write_json(output/'report.json',report)
                with (output/'SHA256SUMS').open('x') as stream:
                    for path in sorted(p for p in output.rglob('*') if p.is_file() and p.name!='SHA256SUMS'):
                        stream.write(base.digest(path)+'  '+path.relative_to(output).as_posix()+'\n')
    return 0 if report['status']=='SIX_TRAIN_PASSES_COMPLETE_NOT_PROMOTED' else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    return run(parser.parse_args(argv),load_helper())


if __name__=='__main__':raise SystemExit(main())
