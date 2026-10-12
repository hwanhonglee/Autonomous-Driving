#!/usr/bin/env python3
"""HH_260906 - Continue matched research scorers from authenticated residual parents without a generator forward."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

WORKSPACE=Path.home()/'personal/hwanhong/portable_e2e'
SOURCE_COMMIT='b478f02e42b94bf04bffec5c8170e05edc33b0f8'
PARENT_ROOT=WORKSPACE/'runs/diagnostics/frozen_stopmix_residual_v1'
PARENT_PLAN_SHA='5bdb3e49d39db3788c6db4ed1b717510d7f53abd0532f3ca66fb01ee006d98b9'
PARENT_REPORT_SHA='6259f0408fdba35ccfda0714603f5e5b6b4566e351a23d657756dfeb34edb2fe'
PARENT_MANIFEST_SHA='c2245a99e60945117c1e7602f921cedde9fd23399dcb90aa64aaf853bd6d3ae3'
RESIDUAL_WORKER_SHA='c1572d06d8718ce9c22f9899ef0c2a2e02d35652d16d95164959fd41bc66e9e6'
RESIDUAL_CORE_SHA='d4bf42fc9f4db930f31ef7b8ccd79900249687642f47bf525dedb240207550b7'
OWNER_SHA='a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
OWNER_PATH=WORKSPACE/'benchmarks/training_profile_v1/profile_portable_training.py'
ARMS=('ce_continue','drive_pair_continue')
BASELINE='parent_candidate_residual'
SCHEMA='portable_e2e.external_frozen_stopmix_drive_rank.v1'
PLAN_SCHEMA='portable_e2e.external_frozen_stopmix_drive_rank_plan.v1'
HEAD_TIMEOUT,AUDIT_TIMEOUT,CACHE_TIMEOUT=150,90,30
INTERNAL_TIMEOUT,EXTERNAL_TIMEOUT,RESERVE=2400,2700,180


def require(value,message):
    if not value:raise ValueError(message)


def sha(raw):return hashlib.sha256(raw).hexdigest()


def encode(value):return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def regular(path):
    path=Path(path).absolute()
    require('..'not in path.parts and path==path.resolve(),'canonical source path required')
    require(path.is_file()and not any(p.is_symlink()for p in(path,*path.parents)),'regular nonsymlink source required')
    return path


def load_external(path,pin,name):
    path=regular(path);raw=path.read_bytes();require(sha(raw)==pin,'external source SHA differs')
    require(name not in sys.modules,'external module already loaded')
    result=ModuleType(name);result.__file__=str(path);result.__package__=name.rpartition('.')[0];sys.modules[name]=result
    try:exec(compile(raw,str(path),'exec'),result.__dict__)
    except BaseException:del sys.modules[name];raise
    require(sha(path.read_bytes())==pin,'external bytes changed during import');return result


def residual_args(args):
    return SimpleNamespace(core=args.residual_core,frozen_worker=args.frozen_worker,frozen_core=args.frozen_core,oracle_helper=args.oracle_helper)


def plan_contract(base,old,r,oracle,projection):
    value=r.plan_contract(base,old,oracle,projection)
    value.update(schema=PLAN_SCHEMA,experiment_id='hh260909-frozen-stopmix-drive-pair-continuation-six-heads-v1',
        arms=list(ARMS),baseline_directory=BASELINE,head_parent_arm='candidate_residual',parent_head_steps=1540,
        parent_residual_plan_sha256=PARENT_PLAN_SHA,parent_residual_report_sha256=PARENT_REPORT_SHA,
        parent_residual_manifest_sha256=PARENT_MANIFEST_SHA,residual_worker_sha256=RESIDUAL_WORKER_SHA,
        residual_core_sha256=RESIDUAL_CORE_SHA,parent_head_files_loaded=3,parent_logit_files_loaded=3,
        residual_parent_pt_files_hash_checked=12,original_cache_run_files_hash_checked=205,residual_parent_run_files_hash_checked=195,
        optimizer_resume=False,initialization='Exact completed candidate_residual parent weights for both arms; fresh AdamW state.',
        pair_scope='All15 unordered DRIVE0..5 pairs on every TRAIN row. Original fullK12 composite costs, masks and coefficients; no future-group filtering, tie tolerance or new oracle.',
        pair_loss_weight={'ce_continue':0.,'drive_pair_continue':1.},
        objective='0.1*native hard composite-oracle cross entropy; only lambda1 adds 0.1*mean(sum15(normalized absolute composite-cost gap * softplus(sign(cost_i-cost_j)*(logit_i-logit_j)))). Lambda0 does not add a zero term, preserving original CE operation order.',
        pair_gap_normalization='Each row divides absolute15 DRIVE cost gaps by their sum; all exact-zero gaps give zero ranking loss.',
        scorer_contract='Same completed candidate_residual449->256->1 parameters and same frozen original-logit prior; both retain normalized candidate geometry. Only the predeclared ranking-loss weight differs.',
        head_timeout_seconds=HEAD_TIMEOUT,paired_audit_timeout_seconds=AUDIT_TIMEOUT,cache_timeout_seconds=CACHE_TIMEOUT,
        internal_wall_timeout_seconds=INTERNAL_TIMEOUT,external_wall_timeout_seconds=EXTERNAL_TIMEOUT,safety_reserve_seconds=RESERVE,
        interpretation='Six head-only continuations with fresh optimizers, not six full-model fits or optimizer resumes. Parent and final gates/groups are retained without automatic adoption.')
    return value


def validate_plan(plan,base,old,r,oracle,projection):
    expected=plan_contract(base,old,r,oracle,projection)
    require(set(plan)==set(expected)|{'declared_at_utc','source_sha256','worker_source_sha256','rank_core_source_sha256'},'exact prospective rank plan fields required')
    require(encode({k:plan[k]for k in expected})==encode(expected),'unreviewed ranking setting')
    require(set(plan['source_sha256'])==set(old.source_paths(base)),'complete native source inventory required')
    for value in [*plan['source_sha256'].values(),plan['worker_source_sha256'],plan['rank_core_source_sha256']]:
        require(type(value)is str and len(value)==64 and all(c in'0123456789abcdef'for c in value),'SHA256 required')
    now=datetime.now(timezone.utc)
    require(base.timestamp(plan['declared_at_utc'])<=now and (base.timestamp(plan['finish_before_utc'])-now).total_seconds()>=EXTERNAL_TIMEOUT+RESERVE,'whole external budget and reserve required')


def checked_output(path,dataset,base,r):
    output=base.checked_output(path,dataset,WORKSPACE)
    require('..'not in output.parts and output==output.resolve(),'canonical output required')
    for parent in(PARENT_ROOT,r.PARENT_ROOT):
        require(not output.is_relative_to(parent.resolve())and not parent.resolve().is_relative_to(output),'output must not overlap either completed parent experiment')
    return output


def source_identity(plan,parent_plan,base,old,r,repo,args):
    original_plan=base.json_read(r.PARENT_ROOT/'plan.json')
    result=r.source_identity(parent_plan,original_plan,base,old,repo,residual_args(args))
    require(plan['source_sha256']==parent_plan['source_sha256'],'rank/native parent source pins differ')
    for name,path,pin in(('external_rank_worker.py',Path(__file__),plan['worker_source_sha256']),
        ('external_rank_core.py',args.core,plan['rank_core_source_sha256'])):
        require(base.digest(path)==pin,'rank source changed');result[name]=pin
    return result


def parent_snapshot(plan,base,old,r,oracle,repo,args):
    require(base.digest(PARENT_ROOT/'plan.json')==PARENT_PLAN_SHA and base.digest(PARENT_ROOT/'report.json')==PARENT_REPORT_SHA
        and base.digest(PARENT_ROOT/'SHA256SUMS')==PARENT_MANIFEST_SHA,'completed residual parent pins differ')
    parent_plan=base.json_read(PARENT_ROOT/'plan.json');report=base.json_read(PARENT_ROOT/'report.json')
    require(report['status']=='SIX_RESIDUAL_HEAD_FITS_COMPLETE_NOT_PROMOTED'and report['completed_head_fit_count']==6
        and report['postcheck_errors']==[]and report['source_and_inputs_unchanged']is True and report['deadline_met']is True,'complete residual parents required')
    require(base.timestamp(report['completed_at_utc'])<=base.timestamp(plan['declared_at_utc']),'rank plan must follow completed residual study')
    original,original_plan,original_report=r.parent_snapshot(parent_plan,base,old,oracle,repo,residual_args(args))
    require(report['original_cache_run_file_sha256']==original['files']and report['original_parent_input_sha256']==original['original_parent_inputs'],'original cache/continued-parent lineage differs')
    actual=old.output_inventory(PARENT_ROOT,base);require(len(actual)==195 and sum(n.endswith('.pt')for n in actual)==12,'all195 residual files and12 PT required')
    manifest={}
    for line in regular(PARENT_ROOT/'SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ')
        require(name not in manifest and name in actual and name!='SHA256SUMS'and '..'not in Path(name).parts and not Path(name).is_absolute(),'invalid residual manifest')
        manifest[name]=value
    require(manifest=={n:v for n,v in actual.items()if n!='SHA256SUMS'},'completed residual bytes changed')
    source=source_identity(plan,parent_plan,base,old,r,repo,args)
    require(report['source_sha256']==original['source_sha256']=={k:v for k,v in source.items()if not k.startswith('external_rank_')}
        and all(actual['source/'+n]==v for n,v in report['source_sha256'].items()),'residual source chain differs')
    require([s['seed']for s in report['seeds']]==list(old.CHECKPOINTS),'all three completed residual parents required')
    for state in report['seeds']:
        require(state['status']=='TWO_RESIDUAL_HEADS_COMPLETE_NOT_PROMOTED'and state['parent_checkpoint_sha256']==old.CHECKPOINTS[state['seed']]
            and all(actual[state['directory']+'/'+n]==v for n,v in state['completed_files'].items()),'completed parent payload differs')
    return dict(original=original,residual_files=actual,source_sha256=source),parent_plan,report,original_report


def load_parent(seed,state,snapshot,base,r,core,torch,reader):
    prefix=f'seed_{seed}/candidate_residual';directory=PARENT_ROOT/prefix
    head_report=base.json_read(directory/'report.json')
    require(head_report['artifact_id']==core.ARTIFACT_ID and head_report['arm']=='candidate_residual'and head_report['seed']==int(seed)
        and head_report['status']=='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED'and head_report['head_parameter_count']==115457
        and head_report['source_checkpoint_sha256']==state['parent_checkpoint_sha256']
        and head_report['final_state']['global_step']==1540 and head_report['final_state']['samples_seen']==6155
        and head_report['cache_sha256_before']==head_report['cache_sha256_after']==state['cache_digest'],'fixed completed candidate-residual parent required')
    values={};proof={}
    for name in('head_only.pt','final_logits.pt'):
        path=directory/name;pin=snapshot['residual_files'][prefix+'/'+name];require(base.digest(path)==pin,'parent tensor changed before secure read')
        value,observed=reader(path,torch.device('cpu'));require(observed==pin==base.digest(path),'secure parent tensor SHA differs')
        values[name]=value;proof[name]=pin
    head=values['head_only.pt'];logits=values['final_logits.pt']
    require(type(head)is dict and set(head)=={'artifact_id','head_state_dict','parent_checkpoint_sha256','parent_cache_report_sha256'}|set(r.DENIALS)
        and head['artifact_id']==core.ARTIFACT_ID and head['parent_checkpoint_sha256']==state['parent_checkpoint_sha256']
        and head['parent_cache_report_sha256']==r.PARENT_REPORT_SHA and all(head[k]==v and type(head[k])is type(v)for k,v in r.DENIALS.items()),'parent head artifact schema/denials differ')
    require(type(logits)is dict and set(logits)=={'artifact_id','logits'}|set(r.DENIALS)
        and logits['artifact_id']=='portable_e2e.frozen_stopmix_residual_logits.research.v1'
        and all(logits[k]==v and type(logits[k])is type(v)for k,v in r.DENIALS.items()),'parent logit artifact schema/denials differ')
    parameters=head['head_state_dict'];shapes={'0.weight':(256,449),'0.bias':(256,),'2.weight':(1,256),'2.bias':(1,)}
    require(type(parameters)is dict and tuple(parameters)==tuple(shapes),'exact parent parameter keys/order required')
    for name,shape in shapes.items():
        value=parameters[name];require(isinstance(value,torch.Tensor)and tuple(value.shape)==shape and value.dtype==torch.float32
            and value.device.type=='cpu'and not value.requires_grad and bool(torch.isfinite(value).all()),'finite CPU parent parameter ABI required')
    require(core._digest_tensors(parameters.items())==head_report['final_head_sha256'],'parent head numeric state digest differs')
    require(type(logits['logits'])is dict and set(logits['logits'])=={'train','val'},'both complete parent logit splits required')
    for split,count in(('train',1147),('val',337)):
        value=logits['logits'][split];require(isinstance(value,torch.Tensor)and tuple(value.shape)==(count,12)and value.dtype==torch.float32
            and value.device.type=='cpu'and not value.requires_grad and bool(torch.isfinite(value).all()),'all finite original parent logits required')
    return parameters,logits['logits'],dict(files=proof,head_state_sha256=head_report['final_head_sha256'],
        logit_sha256={s:core._digest_tensors([('logits',v)])for s,v in logits['logits'].items()},weights_only=True,map_location='cpu',optimizer_loaded=False)


def run(args,base,old,r,oracle):
    repo=WORKSPACE/'autoware_e2e';base.check_environment(WORKSPACE,repo)
    for path in(Path(__file__),args.core,args.residual_worker,args.residual_core,args.frozen_worker,args.frozen_core,args.oracle_helper):
        require(regular(path).is_relative_to(WORKSPACE/'benchmarks'),'external modules must stay outside frozen repository')
    plan_path=regular(args.plan);plan=base.json_read(plan_path);plan_sha=base.digest(plan_path)
    _,projection=oracle.projection_definition(regular(repo/'portable_e2e/losses.py').read_bytes());validate_plan(plan,base,old,r,oracle,projection)
    dataset_root=(WORKSPACE/base.DATASET).resolve(strict=True);require(dataset_root.is_relative_to((WORKSPACE.parent/'dataset').resolve()),'dataset escaped personal storage')
    output=checked_output(args.output_dir,dataset_root,base,r)
    snapshot,parent_plan,parent_report,original_report=parent_snapshot(plan,base,old,r,oracle,repo,args);before=snapshot['source_sha256']
    report=dict(schema=SCHEMA,status='STARTED',started_at_utc=base.utc(),plan_sha256=plan_sha,source_sha256=before,
        parent_residual_report_sha256=PARENT_REPORT_SHA,parent_residual_manifest_sha256=PARENT_MANIFEST_SHA,
        loss_projection=projection,baseline_directory=BASELINE,seeds=[],**r.DENIALS)
    loaded={};datasets={};loader=None;caches={};cache_sha={};records=[];parent_weights=None;parent_logits=None;parent_proof=None
    with base.gpu_lease(WORKSPACE),base.bounded_signals(INTERNAL_TIMEOUT):
        report['gpu_preflight']=base.gpu_idle(repo);output.mkdir(parents=True,exist_ok=False)
        try:
            with(output/'plan.json').open('xb')as stream:stream.write(plan_path.read_bytes())
            external={'external_worker.py':args.frozen_worker,'external_core.py':args.frozen_core,'external_oracle_helper.py':args.oracle_helper,
                'external_owner_helper.py':OWNER_PATH,'external_residual_worker.py':args.residual_worker,'external_residual_core.py':args.residual_core,
                'external_rank_worker.py':Path(__file__),'external_rank_core.py':args.core}
            for name,pin in before.items():
                path=external.get(name,repo/name);dest=output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
                with dest.open('xb')as stream:stream.write(regular(path).read_bytes())
                require(base.digest(dest)==pin,'executed source archive differs')
            sys.path.insert(0,str(repo));import torch
            from portable_e2e import losses as loss_module
            from portable_e2e.dataset import load_training_examples
            from portable_e2e.model import ModelConfig
            from portable_e2e.torch_dataset import Common10TorchDataset
            from portable_e2e.train import _read_checkpoint_file,_seed_everything
            from scripts.e2e import audit_portable_stopmix_behavior as behavior
            require(torch.cuda.device_count()==1 and torch.get_num_threads()==16,'one GPU and fixed16 threads required')
            report['torch_device_uuid']=base.verify_torch_uuid(torch.cuda.get_device_properties(0));report['torch_version']=str(torch.__version__)
            frozen_core=load_external(args.frozen_core,r.FROZEN_CORE_SHA,'portable_e2e.frozen_stopmix_selector')
            residual_core=load_external(args.residual_core,RESIDUAL_CORE_SHA,'portable_e2e.frozen_stopmix_residual')
            core=load_external(args.core,plan['rank_core_source_sha256'],'portable_e2e._frozen_stopmix_drive_rank_external')
            require(tuple(core.ARMS)==ARMS,'rank arms differ');report['imported_native_modules']=old.verify_import_paths(repo)
            projected,proof=oracle.projected_costs(loss_module);require(proof==projection,'original loss projection differs')
            _seed_everything(0,torch.device('cuda:0'));config=ModelConfig.from_mapping(base.json_read(repo/old.MODEL_PATH));loader=load_training_examples
            for split in old.SPLITS:
                loaded[split]=loader(dataset_root,split=split,mode='planning',check_image_hashes=True)
                require(loaded[split].validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA,'whole corpus changed')
                datasets[split]=Common10TorchDataset(loaded[split].examples,config,verify_image_sha256=True,split=split)
            old.validate_datasets(datasets,base)
            for seed,parent in old.CHECKPOINTS.items():
                require(parent_snapshot(plan,base,old,r,oracle,repo,args)[0]==snapshot,'original parent/source changed')
                item=output/f'seed_{seed}';item.mkdir();state=dict(seed=seed,directory=item.name,status='STARTED',parent_checkpoint_sha256=parent,cache_proofs={},arms=[])
                report['seeds'].append(state);records.append(state);caches={};cache_sha={};extras={};identities={};baselines={}
                parent_weights=None;parent_logits=None;parent_proof=None
                original_state=next(s for s in original_report['seeds']if s['seed']==seed);parent_state=next(s for s in parent_report['seeds']if s['seed']==seed)
                for split in old.SPLITS:
                    with old.stage_alarm(CACHE_TIMEOUT):
                        cache,extras[split],identities[split],state['cache_proofs'][split]=r.load_cache(datasets[split],seed,original_state,
                            snapshot['original'],base,old,oracle,frozen_core,torch,_read_checkpoint_file)
                    caches[split]=cache.to('cuda:0')
                cache_sha={k:v.digest()for k,v in caches.items()};state['cache_digest']=cache_sha
                with old.stage_alarm(CACHE_TIMEOUT):parent_weights,parent_logits,parent_proof=load_parent(seed,parent_state,snapshot,base,r,residual_core,torch,_read_checkpoint_file)
                state['parent_head_proof']=parent_proof
                with old.stage_alarm(AUDIT_TIMEOUT):
                    for split in old.SPLITS:
                        baselines[split]=old.audit_cache(caches[split],extras[split],datasets[split],identities[split],parent_logits[split],
                            item/BASELINE/split,oracle,behavior,loss_module,projected,torch)
                        prior=base.json_read(PARENT_ROOT/f'seed_{seed}/candidate_residual/analysis/{split}/summary.json')
                        require(baselines[split]['oracle_and_cost_sha256']==prior['oracle_and_cost_sha256'],'parent fixed composite costs differ')
                state['completed_files']=old.output_inventory(item,base);order_sha=None
                for arm in ARMS:
                    require(source_identity(plan,parent_plan,base,old,r,repo,args)==before and old.completed_outputs_intact(output,records,base),'source or earlier output changed')
                    arm_dir=item/arm;arm_dir.mkdir();arm_state=dict(arm=arm,status='STARTED');state['arms'].append(arm_state)
                    with(arm_dir/'metrics.jsonl').open('x')as history,old.stage_alarm(HEAD_TIMEOUT):
                        def check_deadline():require(datetime.now(timezone.utc)<base.timestamp(plan['finish_before_utc']),'user deadline reached')
                        def progress(row):
                            check_deadline();history.write(encode(row).decode());history.flush()
                            if row.get('global_step',0)%154==0:print(json.dumps(dict(event='DRIVE_RANK_HEAD_PROGRESS',seed=seed,arm=arm,record=row)),flush=True)
                        result=core.fit_frozen_drive_rank(caches['train'],caches['val'],parent_head_state_dict=parent_weights,
                            parent_logits_cpu=parent_logits,projected_costs_fn=projected,projection_proof=proof,
                            arm=arm,seed=int(seed),device='cuda:0',callback=progress,check_deadline=check_deadline)
                    old.verify_history(arm_dir/'metrics.jsonl',result['report']);old.verify_import_paths(repo)
                    require({k:v.digest()for k,v in caches.items()}==cache_sha,'frozen cache changed')
                    head=result['report'];require(head['head_parameter_count']==115457 and head['initial_head_sha256']==state['parent_head_proof']['head_state_sha256']
                        and head['final_state']['global_step']==1540 and head['final_state']['samples_seen']==6155,'matched parent initialization/capacity/exposure required')
                    if order_sha is None:order_sha=head['sampling_order_sha256']
                    require(head['sampling_order_sha256']==order_sha,'paired continuation sampling differs')
                    require(residual_core._digest_tensors(parent_weights.items())==state['parent_head_proof']['head_state_sha256']
                        and all(residual_core._digest_tensors([('logits',v)])==state['parent_head_proof']['logit_sha256'][s]for s,v in parent_logits.items()),'loaded parent tensors changed')
                    with old.stage_alarm(AUDIT_TIMEOUT):
                        for split in old.SPLITS:
                            require(torch.equal(result['baseline_logits_cpu'][split],parent_logits[split]),'initial parent logits changed')
                            measured=old.audit_cache(caches[split],extras[split],datasets[split],identities[split],result['final_logits_cpu'][split],
                                arm_dir/'analysis'/split,oracle,behavior,loss_module,projected,torch)
                            require(measured['oracle_and_cost_sha256']==baselines[split]['oracle_and_cost_sha256']
                                and measured['oracle_indices']==result['oracle_indices_cpu'][split].tolist(),'fixed original candidates/costs/oracle changed')
                    old.save_tensors(arm_dir/'head_only.pt',dict(artifact_id=head['artifact_id'],head_state_dict=result['head_state_dict'],
                        parent_checkpoint_sha256=parent,parent_residual_report_sha256=PARENT_REPORT_SHA,parent_head_sha256=state['parent_head_proof']['files']['head_only.pt'],**r.DENIALS),torch)
                    old.save_tensors(arm_dir/'final_logits.pt',dict(artifact_id='portable_e2e.frozen_stopmix_drive_rank_logits.research.v1',logits=result['final_logits_cpu'],**r.DENIALS),torch)
                    base.write_json(arm_dir/'report.json',head);require(old.completed_outputs_intact(output,records,base),'earlier completed result changed')
                    arm_state.update(status='HEAD_COMPLETE_NOT_PROMOTED',completed_at_utc=base.utc(),files=old.output_inventory(arm_dir,base));state['completed_files']=old.output_inventory(item,base)
                state.update(status='TWO_DRIVE_RANK_HEADS_COMPLETE_NOT_PROMOTED',completed_at_utc=base.utc())
                del result,cache,caches;caches={};parent_weights=None;parent_logits=None;parent_proof=None;torch.cuda.empty_cache()
            require(len(report['seeds'])==3 and all(s['status']=='TWO_DRIVE_RANK_HEADS_COMPLETE_NOT_PROMOTED'for s in report['seeds']),'all six rank continuations required')
            report['status']='SIX_DRIVE_RANK_HEAD_FITS_COMPLETE_NOT_PROMOTED'
        except BaseException as error:report.update(status='FAILED_OR_PARTIAL_NOT_PROMOTED',error_type=type(error).__name__,error=str(error))
        finally:
            with base.finalization_signals():
                errors=[]
                checks={'sources_and_both_original_inventories':lambda:parent_snapshot(plan,base,old,r,oracle,repo,args)[0]==snapshot,
                    'source_archives':lambda:all(base.digest(output/'source'/n)==v for n,v in before.items()),
                    'plan':lambda:base.digest(plan_path)==plan_sha==base.digest(output/'plan.json'),
                    'completed_outputs':lambda:old.completed_outputs_intact(output,records,base),
                    'manifest':lambda:base.digest(dataset_root/'dataset.json')==base.MANIFEST_SHA}
                if caches and cache_sha:checks['active_cache']=lambda:{k:v.digest()for k,v in caches.items()}==cache_sha
                if parent_weights is not None and parent_proof is not None:
                    checks['active_parent_tensors']=lambda:residual_core._digest_tensors(parent_weights.items())==parent_proof['head_state_sha256']and all(
                        residual_core._digest_tensors([('logits',v)])==parent_proof['logit_sha256'][s]for s,v in parent_logits.items())
                if'imported_native_modules'in report:checks['native_import_paths']=lambda:old.verify_import_paths(repo)==report['imported_native_modules']
                if loader is not None:
                    for split in loaded:
                        def corpus(split=split):
                            after=loader(dataset_root,split=split,mode='planning',check_image_hashes=True)
                            return after.validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA and after.fingerprint_sha256==loaded[split].fingerprint_sha256
                        checks['whole_corpus_'+split]=corpus
                for name,operation in checks.items():
                    try:require(operation(),name+' changed')
                    except BaseException as error:errors.append(dict(check=name,error_type=type(error).__name__,error=str(error)))
                report.update(completed_at_utc=base.utc(),postcheck_errors=errors,source_and_inputs_unchanged=not errors,
                    completed_head_fit_count=sum(a['status']=='HEAD_COMPLETE_NOT_PROMOTED'for s in report['seeds']for a in s['arms']),
                    original_cache_run_file_sha256=snapshot['original']['files'],original_residual_run_file_sha256=snapshot['residual_files'],
                    original_parent_input_sha256=snapshot['original']['original_parent_inputs'],deadline_met=datetime.now(timezone.utc)<base.timestamp(plan['finish_before_utc']))
                if errors or not report['deadline_met']:report['status']='FAILED_OR_PARTIAL_NOT_PROMOTED'
                base.write_json(output/'report.json',report);inventory=old.output_inventory(output,base)
                with(output/'SHA256SUMS').open('x')as stream:
                    for name,value in inventory.items():stream.write(value+'  '+name+'\n')
    return 0 if report['status']=='SIX_DRIVE_RANK_HEAD_FITS_COMPLETE_NOT_PROMOTED'else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in('plan','core','residual-worker','residual-core','frozen-worker','frozen-core','oracle-helper','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(argv);r=load_external(args.residual_worker,RESIDUAL_WORKER_SHA,'_drive_rank_residual_worker')
    base=load_external(OWNER_PATH,OWNER_SHA,'_drive_rank_owner_helper')
    old=load_external(args.frozen_worker,r.FROZEN_WORKER_SHA,'_drive_rank_frozen_worker')
    oracle=load_external(args.oracle_helper,r.ORACLE_SHA,'_drive_rank_oracle_helper')
    return run(args,base,old,r,oracle)


if __name__=='__main__':raise SystemExit(main())
