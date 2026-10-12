#!/usr/bin/env python3
"""HH_260906 - Authenticate duration metadata and scalar evidence; never load weights or datasets."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from types import FunctionType, SimpleNamespace

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
PLAN_SHA='5b7c00d86868aaf7e51d36300f653ed2d4acb4df4b76fafae334ce8fbd179bf8'
DURATION_SHA='9d18ba37ddf0b7a790987e12cd7bd16268a61bd895b1eeb4c161343e02b9e9e0'
BASE_SHA='170ddeea85d4a3330229cb0850ec919e8edf9b44a07947f76ebac2d7b1b66e7f'
SCALAR_PATH='scripts/e2e/summarize_stopmix_regret_research.py'
SCALAR_SHA='3ef4e8eccb171f38ac8a05cb29f9ac8339974617dec10f21352026474b7be8ae'
REMOTE_WORKSPACE=Path('/home/pro6000/personal/hwanhong/portable_e2e')
REMOTE_ROOT=REMOTE_WORKSPACE/'runs/diagnostics/stopmix_duration_20260915_v1'
REMOTE_PARENT=REMOTE_WORKSPACE/'runs/diagnostics/stopmix_regret_20260914_v1'
DENIALS=dict(training_data_approved=False,vehicle_control_approved=False,automatic_promotion=False,
    test_read_or_use=False,checkpoint_payloads_loaded=False,neural_inference=False,
    trajectory_math_replayed=False,target_groups_reclassified=False,closed_loop_claim=False)


def require(value,message):
    if not value: raise ValueError(message)


def canonical(path):
    path=Path(path).absolute()
    require('..' not in path.parts and path==path.resolve()
        and not any(p.is_symlink() for p in (path,*path.parents)),'canonical nonsymlink path required')
    return path


def sha(path):
    path=canonical(path);require(path.is_file(),'regular file required')
    result=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def finite(value):
    if isinstance(value,dict):
        for v in value.values():finite(v)
    elif isinstance(value,list):
        for v in value:finite(v)
    elif isinstance(value,float):require(math.isfinite(value),'nonfinite JSON')


def decode(value):
    def pairs(items):
        result={}
        for k,v in items:
            require(k not in result,'duplicate JSON key');result[k]=v
        return result
    result=json.loads(value,object_pairs_hook=pairs,parse_constant=lambda _:require(False,'nonfinite JSON'))
    finite(result);return result


def read(path):
    path=canonical(path);require(path.is_file(),'regular report required')
    require(path.suffix!='.pt','checkpoint reads forbidden in metadata verifier')
    return decode(path.read_bytes())


def encoded(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def write(path,value):
    with canonical(path).open('xb') as stream:
        stream.write(encoded(value));stream.flush();os.fsync(stream.fileno())


def stamp(value):
    result=datetime.fromisoformat(value.replace('Z','+00:00'))
    require(result.tzinfo is not None and result.utcoffset().total_seconds()==0,'UTC timestamp required')
    return result


def safe_inventory(mapping):
    require(isinstance(mapping,dict) and mapping,'nonempty inventory required')
    for name,value in mapping.items():
        path=Path(name)
        require(isinstance(name,str) and name and not path.is_absolute() and '..' not in path.parts
            and path.as_posix()==name,'unsafe inventory path')
        require(isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value),'SHA256 required')


def scan(root,expected=None,exclude_checkpoints=False):
    root=canonical(root);require(root.is_dir(),'evidence directory missing')
    if expected is not None:safe_inventory(expected)
    observed={}
    for parent,dirs,files in os.walk(root,followlinks=False):
        for name in [*dirs,*files]:canonical(Path(parent)/name)
        for name in files:
            path=Path(parent)/name;key=path.relative_to(root).as_posix()
            require(not exclude_checkpoints or path.suffix!='.pt','local checkpoint payload forbidden; refuse open')
            if expected is not None:require(key in expected,'unplanned local file; refuse open')
            observed[key]=sha(path)
    if expected is not None:require(observed==expected,'complete evidence inventory differs')
    return observed


def load_module(path,expected,name):
    require(sha(path)==expected,'analysis helper source differs')
    spec=importlib.util.spec_from_file_location(name,canonical(path))
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    require(sha(path)==expected,'analysis helper changed during import')
    return module


def inventory_only(results):
    root=canonical(results);require(root==REMOTE_ROOT,'inventory requires fixed remote campaign root')
    own=sha(__file__);plan=read(root/'plan.json')
    require(sha(root/'plan.json')==PLAN_SHA,'fixed prospective plan differs')
    summary=read(root/'summary.json')
    require(summary['status']=='COMPLETE_NOT_PROMOTED' and summary['plan_sha256']==PLAN_SHA,
        'final campaign completion required before inventory')
    first=scan(root);parent=scan(REMOTE_PARENT,plan['parent_file_sha256'])
    require(scan(root)==first and scan(REMOTE_PARENT,parent)==parent and sha(__file__)==own,
        'remote evidence changed while hashing')
    return dict(schema='portable_e2e.stopmix_duration_remote_inventory.v1',status='COMPLETE_INVENTORY',
        remote_root=str(root),remote_parent=str(REMOTE_PARENT),plan_sha256=PLAN_SHA,
        captured_at_utc=datetime.now(timezone.utc).isoformat(),verifier_sha256=own,
        remote_file_sha256=first,parent_file_sha256=parent,remote_inventory_unchanged=True,
        parent_inventory_unchanged=True,checkpoint_files_hashed=sum(n.endswith('.pt') for n in first),
        checkpoint_payloads_loaded=False,training_data_approved=False,vehicle_control_approved=False)


def verify_transport(results,parent,receipt_path,expected_sha):
    require(isinstance(expected_sha,str) and len(expected_sha)==64
        and all(c in '0123456789abcdef' for c in expected_sha),'external inventory SHA256 required')
    require(sha(receipt_path)==expected_sha,'externally pinned remote inventory changed')
    receipt=read(receipt_path)
    require(receipt['schema']=='portable_e2e.stopmix_duration_remote_inventory.v1'
        and receipt['status']=='COMPLETE_INVENTORY' and receipt['plan_sha256']==PLAN_SHA
        and receipt['remote_root']==str(REMOTE_ROOT) and receipt['remote_parent']==str(REMOTE_PARENT)
        and receipt['verifier_sha256']==sha(__file__)
        and receipt['remote_inventory_unchanged'] is True and receipt['parent_inventory_unchanged'] is True
        and receipt['checkpoint_payloads_loaded'] is False
        and receipt['training_data_approved'] is False and receipt['vehicle_control_approved'] is False,
        'remote inventory scope/source differs')
    stamp(receipt['captured_at_utc'])
    remote,parents=receipt['remote_file_sha256'],receipt['parent_file_sha256']
    safe_inventory(remote);safe_inventory(parents)
    checkpoints={n for n in remote if n.endswith('.pt')}
    runs=tuple(f'seed_{seed}/{arm}' for seed in (20260903,20260904,20260905) for arm in ('A_original_loss','B_cost_aware'))
    expected={f'epochs/epoch_{epoch:03d}/{run}/training/checkpoints/latest.pt' for epoch in range(6,21) for run in runs}
    require(checkpoints==expected and receipt['checkpoint_files_hashed']==90,'exact 90 endpoint checkpoint inventory required')
    require(len(parents)==293 and {n for n in parents if n.endswith('.pt')}==
        {f'{run}/training/checkpoints/latest.pt' for run in runs},'six-parent complete inventory required')
    copied={n:h for n,h in remote.items() if not n.endswith('.pt')}
    parent_copied={n:h for n,h in parents.items() if not n.endswith('.pt')}
    scan(results,copied,True);scan(parent,parent_copied,True)
    return receipt


def digest_adapter(results,parent,remote,parents):
    def digest(path):
        path=Path(path)
        if path.suffix=='.pt':
            for root,pins in ((results,remote),(parent,parents),(REMOTE_ROOT,remote),(REMOTE_PARENT,parents)):
                if path.is_relative_to(root):
                    key=path.relative_to(root).as_posix()
                    require('..' not in path.parts and key in pins,'unattested checkpoint reference')
                    return pins[key]
            require(False,'checkpoint reference outside attested roots')
        return sha(path)
    return digest


def behavior_check(item,run,epoch,scalar,identities,entropy):
    behavior=read(item/'behavior/summary.json');evaluation=read(item/'evaluation/metrics.json')
    audit=read(item/'gate_v8.json');train=read(item/'training/run.json');measured={}
    for split,count in scalar.COUNTS.items():
        path=item/'behavior'/f'{split}_samples.jsonl'
        rows=[decode(line) for line in canonical(path).read_bytes().splitlines()]
        measured[split]=scalar.verify_rows(rows,count)
        scalar.compare_aggregates(behavior['splits'][split],dict(measured[split],samples_sha256=sha(path)),
            f'{epoch}/{run}/{split}',entropy)
        identity=[{k:r[k] for k in scalar.IDENTITY} for r in rows]
        require(split not in identities or identities[split]==identity,'behavior identity/groups changed across epochs/lineages')
        identities[split]=identity
        ids={r['episode_id'] for r in rows}
        require(ids==set(train['training_episode_ids']) if split=='train' else not ids.intersection(train['training_episode_ids']),
            'behavior TRAIN/VAL identities overlap or changed')
        require(measured[split]['target_motion_groups']['stationary_hold']['sample_count']==(118 if split=='train' else 0),
            'fixed stopping subgroup denominator changed')
        if split=='val':
            scalar.compare_aggregates(audit['geometry'],measured[split]['all_samples']['geometry'],f'{epoch}/{run}/native/geometry',entropy)
            for key in scalar.METRICS[:-1]:
                require(math.isclose(measured[split]['all_samples']['metrics'][key],evaluation['metrics'][key],
                    rel_tol=1e-5,abs_tol=1e-5),'native versus scalar metric reduction differs')
            expected={f'{index:04d}_'+''.join(c if c.isalnum() or c in '-_.' else '_' for c in row['sample_id'])[:120]+'.png'
                for index,row in enumerate(rows[:12])}
            require({p.name for p in (item/'evaluation/trajectories').glob('*.png')}==expected,'native first12 VAL PNG identities differ')
    return measured


def verify(results,parent_local,inventory_path,expected_inventory_sha):
    results,parent,inventory_path=map(canonical,(results,parent_local,inventory_path))
    require(not results.is_relative_to(parent) and not parent.is_relative_to(results),'disjoint current/parent evidence required')
    own=sha(__file__);inventory_sha=sha(inventory_path)
    transport=verify_transport(results,parent,inventory_path,expected_inventory_sha)
    remote,parents=transport['remote_file_sha256'],transport['parent_file_sha256']
    plan=read(results/'plan.json');require(sha(results/'plan.json')==PLAN_SHA,'fixed plan bytes differ')
    duration=load_module(results/'source/run_stopmix_duration_learning.py',DURATION_SHA,'_verified_duration_worker')
    base=load_module(results/'source/run_stopmix_regret_research.py',BASE_SHA,'_verified_duration_base')
    duration.validate_plan(plan,DURATION_SHA,base)
    require(plan['parent_file_sha256']==parents,'remote parent differs from prospectively pinned parents')
    duration.verify_archive(plan,results,base)
    digest=digest_adapter(results,parent,remote,parents)
    stage=FunctionType(base.verify_stage.__code__,dict(vars(base),digest=digest))
    command=FunctionType(base.command.__code__,dict(vars(base),digest=digest))
    adapted_base=SimpleNamespace(**(vars(base)|dict(verify_stage=stage,command=command)))
    def parent_inventory(path,expected):
        require(path==parent and expected==parents,'unplanned parent inventory request')
        scan(parent,{n:h for n,h in parents.items() if not n.endswith('.pt')},True)
        return parents
    # HH_260906 - Only file-digest lookup is adapted; the frozen resume/stage validators and saved scalars are unchanged.
    parent_check=FunctionType(duration.verify_parent.__code__,dict(vars(duration),inventory=parent_inventory))
    parent_check(plan,parent,adapted_base)
    train_check=FunctionType(duration.verify_training.__code__,dict(vars(duration),digest=digest))
    pure=('portable_e2e/__init__.py','portable_e2e/contract.py','portable_e2e/dataset.py','portable_e2e/runtime_contract.py')
    local_pins={ROOT/n:plan['source_sha256'][n] for n in pure}
    local_pins[ROOT/'scripts/e2e/run_stopmix_regret_research.py']=BASE_SHA
    local_pins[ROOT/SCALAR_PATH]=SCALAR_SHA
    for path,pin in local_pins.items():require(sha(path)==pin,'local pure source differs')
    if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
    scalar=load_module(ROOT/SCALAR_PATH,SCALAR_SHA,'_duration_scalar_verifier')
    started,summary=read(results/'started.json'),read(results/'summary.json')
    expected_epochs={str(e):{r:'COMPLETE' for r in duration.RUNS} for e in duration.EPOCHS}
    require(started['schema']==summary['schema']=='portable_e2e.stopmix_duration_learning.v1'
        and started['status']=='RUNNING' and summary['status']=='COMPLETE_NOT_PROMOTED'
        and started['plan_sha256']==summary['plan_sha256']==PLAN_SHA
        and summary['source_view_parent_postcheck_pass'] is True,'authoritative completion missing')
    require(summary['epochs']==expected_epochs and started['epochs']==
        {str(e):{r:'NOT_RUN' for r in duration.RUNS} for e in duration.EPOCHS}
        and started['stages']==[] and len(summary['stages'])==282
        and summary['stage_counts']=={'train':90,'evaluate':90,'audit':90,'behavior':12}
        and summary['total_new_optimizer_steps']==25200 and summary['total_new_sample_exposures']==100710,'campaign denominator differs')
    for document in (started,summary):
        require(all(document[k] is v for k,v in duration.DENIALS.items()),'runner research denial differs')
    idle=dict(physical_index=0,uuid=base.GPU_UUID,pre_cuda_compute_processes=0)
    require(started['pre_cuda_gpu']==summary['pre_cuda_gpu']==summary['final_gpu']==idle,'GPU0 boundary receipts differ')
    previous=stamp(started['started_at_utc'])
    require(stamp(plan['declared_at_utc'])<=previous and started['started_at_utc']==summary['started_at_utc'],'prospective start differs')
    behavior_start=read(results/'epochs/epoch_010'/duration.RUNS[0]/'behavior_started.json')
    remote_script=Path(behavior_start['command'][1]);remote_plan=Path(behavior_start['command'][3])
    for path in (remote_script,remote_plan):
        require(path.is_relative_to(REMOTE_WORKSPACE/'benchmarks') and '..' not in path.parts,'external command source escaped personal benchmarks')
    receipts=[];identities={};entropy=[];checkpoints={};behavior_results={};history_rows=0
    owner_pid=None
    for epoch in duration.EPOCHS:
        root=duration.epoch_root(results,epoch)
        require(sha(root/'source'/base.MODEL_PATH)==base.MODEL_SHA,'epoch model archive differs')
        for run_name in duration.RUNS:
            item=root/run_name;resume=read(item/'resume_source.json')
            source=parent/run_name/'training' if epoch==6 else duration.epoch_root(results,epoch-1)/run_name/'training'
            remote_source=REMOTE_PARENT/run_name/'training' if epoch==6 else duration.epoch_root(REMOTE_ROOT,epoch-1)/run_name/'training'
            previous_steps=1540 if epoch==6 else (epoch-1)*287
            source_pins={n:digest(source/n) for n in duration.RESUME_FILES}
            expected_resume=dict(schema='portable_e2e.duration_resume_copy.v1',run=run_name,epoch=epoch,
                source_training_path=str(remote_source),source_file_sha256=source_pins,copied_file_sha256=source_pins,
                source_global_step=previous_steps,target_global_step=epoch*287,
                inherited_history=dict(steps=previous_steps,bytes=(source/'metrics.jsonl').stat().st_size,sha256=source_pins['metrics.jsonl']),
                independent_byte_exact_copies=True,parent_checkpoint_not_modified=True,
                checkpoint_cursor_note='Native checkpoint may retain completed epoch cursor (epoch-1,287); native trainer validates and normalizes it.')
            require(resume==expected_resume,'resume source/history provenance differs')
            input_sha=source_pins['checkpoints/latest.pt']
            for stage_name in ('train','evaluate','audit',*(['behavior'] if epoch in (10,20) else [])):
                initial=read(item/f'{stage_name}_started.json');completed=read(item/f'{stage_name}_completed.json')
                args=duration.command(adapted_base,plan,REMOTE_WORKSPACE,REMOTE_WORKSPACE/'autoware_e2e',REMOTE_ROOT,
                    run_name,epoch,stage_name,remote_plan,remote_script)
                if owner_pid is None:owner_pid=initial['owner_pid']
                require(type(owner_pid) is int and owner_pid>1,'invalid owner PID')
                require(initial==dict(epoch=epoch,run=run_name,stage=stage_name,command=args,
                    timeout_seconds=base.TIMEOUTS[stage_name],started_at_utc=initial['started_at_utc'],status='RUNNING',
                    owner_pid=owner_pid,plan_sha256=PLAN_SHA,input_checkpoint_sha256=input_sha,pre_cuda_gpu=idle),
                    'stage command/ownership/input checkpoint differs')
                if stage_name=='train':
                    proof=train_check(base,plan,results,parent,run_name,epoch,resume)
                    history_rows+=proof['history']['row_count']
                else:
                    proof=stage(plan,root,run_name,stage_name)
                    proof['path']=(root/proof['path']).relative_to(results).as_posix()
                require(completed==dict(initial,status='COMPLETE',ended_at_utc=completed['ended_at_utc'],**proof),
                    'stage completion/report/checkpoint binding differs')
                begin,end=stamp(initial['started_at_utc']),stamp(completed['ended_at_utc'])
                require(previous<=begin<=end and (end-begin).total_seconds()<=base.TIMEOUTS[stage_name]+30,
                    'stage execution chronology/timeout differs')
                previous=end;receipts.append(completed);input_sha=proof['checkpoint_sha256']
            checkpoints[f'{epoch}/{run_name}']=input_sha
            evaluation=read(item/'evaluation/metrics.json');audit=read(item/'gate_v8.json')
            require(evaluation['rendered_trajectory_png']==12 and evaluation['training_episode_count']==3
                and evaluation['evaluation_episode_count']==1,'native render/episode denominator differs')
            require(audit['gate']==read(parent/run_name/'gate_v8.json')['gate'],'native gate changed')
            require(len(list((item/'evaluation/trajectories').glob('*.png')))==12,'native VAL PNG denominator differs')
            for horizon in ('1p0','3p0','6p4'):
                for name in (f'ade_{horizon}s_m',f'fde_{horizon}s_m',f'speed_mae_{horizon}s_mps'):
                    require(evaluation['metric_counts'][name]==337,'native horizon denominator differs')
            if epoch in (10,20):
                behavior_results[f'{epoch}/{run_name}']=behavior_check(item,run_name,epoch,scalar,identities,entropy)
    require(summary['stages']==receipts,'authoritative stage order/content differs')
    require(previous<=stamp(summary['ended_at_utc'])<=stamp(transport['captured_at_utc'])
        and (stamp(summary['ended_at_utc'])-stamp(started['started_at_utc'])).total_seconds()<=21630,'finish/inventory chronology differs')
    require(read(results/'progress.json')['status']=='COMPLETE_NOT_PROMOTED'
        and read(results/'progress.json')['completed_stages']==282,'final progress differs')
    for run_name in duration.RUNS:
        for name in ('training/run.json','training/metrics.jsonl','evaluation/metrics.json','gate_v8.json','behavior/summary.json'):
            require(sha(results/'parents'/run_name/name)==parents[f'{run_name}/{name}'],'archived parent evidence differs')
    verify_transport(results,parent,inventory_path,expected_inventory_sha)
    require(sha(__file__)==own and sha(inventory_path)==inventory_sha,'verifier or inventory changed')
    for path,pin in local_pins.items():require(sha(path)==pin,'local pure source changed')
    return dict(schema='portable_e2e.stopmix_duration_verification.v1',status='PASS_NOT_PROMOTED',
        campaign_completion_verified=True,plan_sha256=PLAN_SHA,remote_inventory_sha256=inventory_sha,
        verifier_sha256=own,duration_worker_sha256=DURATION_SHA,scalar_helper_sha256=SCALAR_SHA,
        copied_file_count=len(remote)-90,parent_copied_file_count=287,epoch_runs=90,
        stage_counts=summary['stage_counts'],optimizer_history_rows_including_repeated_prefixes=history_rows,
        distinct_optimizer_history_rows_across_six_lineages=34440,new_optimizer_steps=25200,new_sample_exposures=100710,
        endpoint_checkpoint_sha256=checkpoints,behavior_rows=17808,behavior=behavior_results,
        entropy_roundoff=dict(absolute_tolerance=1e-15,affected_fields=entropy),
        native_scalar_reduction_tolerance=dict(relative=1e-5,absolute=1e-5),
        limitations=['Verification PASS authenticates execution/evidence, not accuracy or runtime gate success.',
            'Checkpoints were remotely hashed, never locally opened or deserialized; native secure resume is source/receipt bound.',
            'Scalar errors, groups and geometry flags are reaggregated; raw trajectories and target group classification are not replayed.',
            'Repeated VAL is development evidence; TRAIN groups are fitted, VAL stationary-hold count is zero.',
            'All epochs and failure metrics remain retained; this result selects or promotes no checkpoint.'],**DENIALS)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--results',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--parent-local',type=Path);parser.add_argument('--remote-inventory',type=Path)
    parser.add_argument('--remote-inventory-sha256')
    parser.add_argument('--inventory-only',action='store_true');args=parser.parse_args(argv)
    output=canonical(args.output);require(not output.exists(),'fresh output required')
    inputs=[canonical(args.results)]
    if args.inventory_only:
        require(args.parent_local is None and args.remote_inventory is None and args.remote_inventory_sha256 is None,
            'inventory-only arguments differ')
        inputs.append(REMOTE_PARENT)
    else:
        require(args.parent_local is not None and args.remote_inventory is not None and args.remote_inventory_sha256 is not None,
            'parent/inventory/external SHA required')
        inputs.extend((canonical(args.parent_local),canonical(args.remote_inventory)))
    for path in inputs:
        require(not output.is_relative_to(path) and not path.is_relative_to(output),'output must be disjoint from evidence')
    result=inventory_only(args.results) if args.inventory_only else verify(
        args.results,args.parent_local,args.remote_inventory,args.remote_inventory_sha256)
    write(output,result);print(json.dumps(dict(status=result['status'],output=str(output),sha256=sha(output))))
    return 0


if __name__=='__main__':raise SystemExit(main())
