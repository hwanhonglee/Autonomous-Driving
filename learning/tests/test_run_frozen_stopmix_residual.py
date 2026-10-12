"""HH_260906 - Exercise authenticated-cache orchestration with synthetic CPU data, never a live GPU or parent payload."""
from contextlib import nullcontext
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import run_frozen_stopmix_residual as run
from scripts.e2e import run_frozen_stopmix_selector as old
from scripts.e2e import profile_portable_training as base
from scripts.e2e import diagnose_stopmix_training_oracles as oracle
from portable_e2e import frozen_stopmix_selector as core, losses


class FrozenTime(datetime):
    @classmethod
    def now(cls,tz=None):return cls(2026,9,8,21,0,tzinfo=timezone.utc)


def plan():
    _,projection=oracle.projection_definition(Path(losses.__file__).read_bytes())
    value=run.plan_contract(base,old,oracle,projection)
    value.update(declared_at_utc='2026-09-08T20:59:00Z',source_sha256={n:'a'*64 for n in old.source_paths(base)},
        worker_source_sha256='b'*64,residual_core_source_sha256='c'*64)
    return value,projection


def test_fixed_matched_residual_plan_and_no_generator_scope(monkeypatch):
    value,projection=plan();monkeypatch.setattr(run,'datetime',FrozenTime)
    run.validate_plan(value,base,old,oracle,projection)
    assert value['head_parameter_count']==115457 and value['samples_seen_per_head']==6155
    assert value['arms']==['prior_only_residual','candidate_residual'] and value['expected_head_fit_count']==6
    assert value['generator_neural_forwards']==0 and value['generator_constructed'] is False
    assert value['cache_files_loaded']==6 and value['original_pt_files_hash_checked']==18
    assert value['original_pt_files_copied']==0 and value['expected_val_png_count']==108
    assert 'whole-corpus' in value['integrity_scope'] and 'held-out test bytes' in value['integrity_scope']
    assert value['finish_before_utc']=='2026-09-09T01:00:00Z'


@pytest.mark.parametrize('key,value',[
    ('candidate_count',6),('score_input_width',448),('head_parameter_count',115201),('steps',1541),
    ('batch_size',8),('learning_rate',.001),('samples_seen_per_head',6156),('expected_head_fit_count',5),
    ('generator_constructed',True),('generator_neural_forwards',1),('test_neural_inference',True),
    ('cache_files_loaded',18),('internal_wall_timeout_seconds',2100),('parent_cache_report_sha256','a'*64),
    ('residual_core_source_sha256','bad'),('declared_at_utc','2026-09-08T22:00:00Z')])
def test_plan_fixed_scope_cannot_change(monkeypatch,key,value):
    proposed,projection=plan();proposed[key]=value;monkeypatch.setattr(run,'datetime',FrozenTime)
    with pytest.raises(ValueError):run.validate_plan(proposed,base,old,oracle,projection)


def test_plan_requires_source_inventory_and_whole_external_reserve(monkeypatch):
    proposed,projection=plan();monkeypatch.setattr(run,'datetime',FrozenTime)
    proposed['source_sha256'].pop(next(iter(proposed['source_sha256'])))
    with pytest.raises(ValueError,match='inventory'):run.validate_plan(proposed,base,old,oracle,projection)
    class Late(FrozenTime):
        @classmethod
        def now(cls,tz=None):return cls(2026,9,9,0,23,tzinfo=timezone.utc)
    proposed,projection=plan();monkeypatch.setattr(run,'datetime',Late)
    with pytest.raises(ValueError,match='reserve'):run.validate_plan(proposed,base,old,oracle,projection)


def test_external_preexecution_pin_failed_import_cleanup_and_no_alias(tmp_path):
    path=tmp_path/'module.py';path.write_text("raise RuntimeError('must not execute without pin')\n")
    name='_residual_test_external'
    with pytest.raises(ValueError,match='SHA'):run.load_external(path,'a'*64,name)
    assert name not in sys.modules
    with pytest.raises(RuntimeError):run.load_external(path,run.sha(path.read_bytes()),name)
    assert name not in sys.modules
    alias=tmp_path/'alias.py';alias.symlink_to(path)
    with pytest.raises(ValueError,match='canonical|nonsymlink'):run.load_external(alias,run.sha(path.read_bytes()),name)


def test_external_module_has_canonical_package_and_no_replacement(tmp_path):
    path=tmp_path/'module.py';path.write_text('VALUE = 12\n');name='portable_e2e._residual_test_external'
    try:
        module=run.load_external(path,run.sha(path.read_bytes()),name)
        assert module.__package__=='portable_e2e' and module.VALUE==12
        with pytest.raises(ValueError,match='already'):run.load_external(path,run.sha(path.read_bytes()),name)
    finally:sys.modules.pop(name,None)


@pytest.mark.parametrize('fault',['inside_original','original','ancestor','parent_traversal','symlink'])
def test_output_preflight_cannot_change_original_or_escape_namespace(tmp_path,monkeypatch,fault):
    workspace=tmp_path/'workspace';parent=workspace/'runs/diagnostics/old';parent.mkdir(parents=True)
    (parent/'original.txt').write_bytes(b'unchanged');dataset=tmp_path/'dataset';dataset.mkdir()
    monkeypatch.setattr(run,'WORKSPACE',workspace);monkeypatch.setattr(run,'PARENT_ROOT',parent)
    paths={'inside_original':parent/'new','original':parent,'ancestor':parent.parent,
        'parent_traversal':parent.parent/'..'/'..'/'escaped'}
    alias=tmp_path/'alias';alias.symlink_to(parent.parent,target_is_directory=True);paths['symlink']=alias/'new'
    before=old.output_inventory(parent,base)
    with pytest.raises(ValueError):run.checked_output(paths[fault],dataset,base)
    assert old.output_inventory(parent,base)==before
    assert run.checked_output(parent.parent/'new_sibling',dataset,base)==parent.parent/'new_sibling'


def test_external_parent_traversal_cannot_bypass_benchmarks_boundary(tmp_path):
    child=tmp_path/'child';child.mkdir();source=tmp_path/'module.py';source.write_text('VALUE=1')
    with pytest.raises(ValueError,match='traversal'):run.regular(child/'..'/'module.py')


class MetadataDataset:
    split='train';fingerprint_sha256=old.SPLITS['train'][1]
    config=SimpleNamespace(route_points=4)
    def __init__(self):
        self.examples=[SimpleNamespace(token=f'train-{i}',episode_id=f'episode-{i}',index=i)for i in range(3)]
    def __len__(self):return len(self.examples)
    def __getitem__(self,index):raise AssertionError('No image decoding or forward inputs allowed')
    def _targets(self,example):
        xy=torch.full((64,2),float(example.index));yaw=torch.zeros(64);speed=torch.ones(64)
        valid=torch.ones(64,dtype=torch.bool);valid[60:]=False
        xy[60:]=0.;speed[60:]=0.
        return xy,yaw,speed,valid
    def _route(self,example):return torch.full((4,2),float(example.index)),torch.ones(4,dtype=torch.bool)


def cache_fixture():
    dataset=MetadataDataset();seed='20260903';targets=[dataset._targets(e)for e in dataset.examples]
    cache=core.CachedStopmixSelectorData(split='train',sample_ids=tuple(e.token for e in dataset.examples),
        episode_ids=tuple(e.episode_id for e in dataset.examples),source_checkpoint_sha256=old.CHECKPOINTS[seed],
        corpus_fingerprint_sha256=base.CORPUS_SHA,dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        fused=torch.zeros(3,256),candidate_xy=torch.zeros(3,12,64,2),candidate_speed=torch.zeros(3,12,64),
        original_logits=torch.zeros(3,12),target_xy=torch.stack([x[0]for x in targets]),
        target_yaw_rad=torch.stack([x[1]for x in targets]),target_speed_mps=torch.stack([x[2]for x in targets]),
        target_valid=torch.stack([x[3]for x in targets]))
    payload=dict(artifact_id=old.CACHE_ID,split='train',digest=cache.digest(),sample_ids=list(cache.sample_ids),
        episode_ids=list(cache.episode_ids),source_checkpoint_sha256=old.CHECKPOINTS[seed],
        tensors={n:getattr(cache,n)for n in old.CACHE_FIELDS},
        extra=dict(route_xy=torch.stack([dataset._route(e)[0]for e in dataset.examples]),
            route_mask=torch.stack([dataset._route(e)[1]for e in dataset.examples])),**old.DENIALS)
    return dataset,seed,payload,dict(cache_digest={'train':cache.digest()})


def test_cache_reconstruction_exact_cpu_owns_clones_without_image_access():
    dataset,seed,payload,state=cache_fixture()
    cache,extra=run.validate_cache_payload(payload,dataset,seed,state,base,old,core,torch)
    assert len(cache)==3 and cache.digest()==state['cache_digest']['train']
    payload['tensors']['fused'][0,0]=10.
    assert cache.fused[0,0]==0. and extra['route_xy'].shape==(3,4,2)


@pytest.mark.parametrize('fault',['artifact','denial','extra_key','split','source','sample_id','episode_id','digest',
    'tensor_key','tensor_shape','tensor_dtype','tensor_nonfinite','tensor_grad','route_shape','route_dtype','route_nonfinite',
    'route_grad','route_values','route_mask','target_resealed'])
def test_cache_corruption_and_resealed_metadata_rejected(fault):
    dataset,seed,payload,state=cache_fixture()
    if fault=='artifact':payload['artifact_id']='portable_e2e.checkpoint.v1'
    elif fault=='denial':payload[next(iter(old.DENIALS))]=True
    elif fault=='extra_key':payload['unsafe_override']=True
    elif fault=='split':payload['split']='test'
    elif fault=='source':payload['source_checkpoint_sha256']='a'*64
    elif fault=='sample_id':payload['sample_ids'][0]='wrong'
    elif fault=='episode_id':payload['episode_ids'][0]='wrong'
    elif fault=='digest':payload['digest']='a'*64
    elif fault=='tensor_key':payload['tensors']['unknown']=torch.zeros(1)
    elif fault=='tensor_shape':payload['tensors']['fused']=torch.zeros(3,255)
    elif fault=='tensor_dtype':payload['tensors']['fused']=payload['tensors']['fused'].double()
    elif fault=='tensor_nonfinite':payload['tensors']['fused'][0,0]=float('nan')
    elif fault=='tensor_grad':payload['tensors']['fused'].requires_grad_(True)
    elif fault=='route_shape':payload['extra']['route_xy']=torch.zeros(3,3,2)
    elif fault=='route_dtype':payload['extra']['route_mask']=torch.ones(3,4)
    elif fault=='route_nonfinite':payload['extra']['route_xy'][0,0,0]=float('inf')
    elif fault=='route_grad':payload['extra']['route_xy'].requires_grad_(True)
    elif fault=='route_values':payload['extra']['route_xy'][0,0,0]=.01
    elif fault=='route_mask':payload['extra']['route_mask'][0,0]=False
    else:
        payload['tensors']['target_xy'][0,0,0]=.01
        forged=core.CachedStopmixSelectorData(split='train',sample_ids=tuple(payload['sample_ids']),episode_ids=tuple(payload['episode_ids']),
            source_checkpoint_sha256=old.CHECKPOINTS[seed],corpus_fingerprint_sha256=base.CORPUS_SHA,
            dataset_fingerprint_sha256=dataset.fingerprint_sha256,**payload['tensors'])
        payload['digest']=forged.digest();state['cache_digest']['train']=forged.digest()
    with pytest.raises(ValueError):run.validate_cache_payload(payload,dataset,seed,state,base,old,core,torch)


def cache_disk_fixture(tmp_path,monkeypatch):
    dataset,seed,payload,state=cache_fixture();monkeypatch.setattr(run,'PARENT_ROOT',tmp_path)
    directory=tmp_path/f'seed_{seed}';directory.mkdir();cache_path=directory/'train_cache.pt';cache_path.write_bytes(b'fake secure-reader bytes')
    rows=[]
    for i,e in enumerate(dataset.examples):
        tensors={k:payload['tensors'][k][i:i+1]for k in('target_xy','target_speed_mps','target_yaw_rad','target_valid')}
        rows.append(dict(index=i,sample_id=e.token,model_input_sha256='a'*64,target_sha256=oracle.tensor_sha(tensors,0,tuple(tensors))))
    journal=directory/'train_cache_progress.jsonl';journal.write_bytes(b''.join(run.encode(r)for r in rows))
    snapshot={'files':old.output_inventory(tmp_path,base)}
    def reader(path,device):
        assert path==cache_path and device==torch.device('cpu')
        return payload,base.digest(path)
    return dataset,seed,payload,state,snapshot,reader,journal


def test_secure_cache_reader_passes_cpu_and_original_identity_hashes(tmp_path,monkeypatch):
    dataset,seed,payload,state,snapshot,reader,journal=cache_disk_fixture(tmp_path,monkeypatch)
    cache,extra,rows,proof=run.load_cache(dataset,seed,state,snapshot,base,old,oracle,core,torch,reader)
    assert len(cache)==len(rows)==3 and proof['weights_only'] is True
    assert proof['targets_and_routes_reconstructed_exactly'] is True and proof['model_input_sha_recomputed'] is False


@pytest.mark.parametrize('fault',['decoder_sha','decoder_mutation','journal_sha','journal_count','journal_order','target_sha'])
def test_secure_cache_and_journal_mutation_cannot_pass(tmp_path,monkeypatch,fault):
    dataset,seed,payload,state,snapshot,reader,journal=cache_disk_fixture(tmp_path,monkeypatch)
    original_reader=reader
    if fault=='decoder_sha':
        def reader(path,device):return payload,'a'*64
    elif fault=='decoder_mutation':
        def reader(path,device):
            result=original_reader(path,device);path.write_bytes(b'mutated');return result
    else:
        rows=[json.loads(line)for line in journal.read_bytes().splitlines()]
        if fault=='journal_count':rows.pop()
        elif fault=='journal_order':rows.reverse()
        else:rows[0]['target_sha256']='f'*64
        journal.write_bytes(b''.join(run.encode(r)for r in rows))
        if fault!='journal_sha':snapshot['files'][journal.relative_to(tmp_path).as_posix()]=base.digest(journal)
    with pytest.raises(ValueError):run.load_cache(dataset,seed,state,snapshot,base,old,oracle,core,torch,reader)


def test_cache_reader_failure_has_no_unsafe_fallback(tmp_path,monkeypatch):
    dataset,seed,payload,state,snapshot,reader,journal=cache_disk_fixture(tmp_path,monkeypatch)
    def denied(*args):raise ValueError('weights_only unavailable')
    with pytest.raises(ValueError,match='weights_only'):run.load_cache(dataset,seed,state,snapshot,base,old,oracle,core,torch,denied)


def test_source_identity_keeps_old_pins_and_adds_only_residual_modules(tmp_path,monkeypatch):
    names=['worker','core','frozen_worker','frozen_core','oracle_helper'];paths={n:tmp_path/(n+'.py')for n in names}
    for n,p in paths.items():p.write_text(n)
    monkeypatch.setattr(run,'__file__',str(paths['worker']))
    monkeypatch.setattr(run,'FROZEN_WORKER_SHA',base.digest(paths['frozen_worker']))
    monkeypatch.setattr(run,'FROZEN_CORE_SHA',base.digest(paths['frozen_core']))
    native={'portable_e2e/losses.py':'a'*64};parent_plan={'source_sha256':native}
    proposed=dict(source_sha256=native,worker_source_sha256=base.digest(paths['worker']),residual_core_source_sha256=base.digest(paths['core']))
    old_view=SimpleNamespace(source_identity=lambda *args:dict(native))
    args=SimpleNamespace(**{k:v for k,v in paths.items()if k!='worker'})
    result=run.source_identity(proposed,parent_plan,base,old_view,tmp_path,args)
    assert set(result)==set(native)|{'external_residual_worker.py','external_residual_core.py'}
    paths['core'].write_text('changed')
    with pytest.raises(ValueError,match='source changed'):run.source_identity(proposed,parent_plan,base,old_view,tmp_path,args)


def parent_fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(run,'PARENT_ROOT',tmp_path)
    sources={}
    for i in range(22):
        name=f'file_{i:02d}.py';path=tmp_path/'source'/name;path.parent.mkdir(exist_ok=True);path.write_text(name)
        sources[name]=base.digest(path)
    states=[]
    for seed in old.CHECKPOINTS:
        directory=tmp_path/f'seed_{seed}';directory.mkdir()
        for i in range(6):(directory/f'cache_{i}.pt').write_bytes(b'PT bytes are hashed, never decoded')
        states.append(dict(seed=seed,directory=directory.name,status='TWO_HEADS_COMPLETE_NOT_PROMOTED',
            parent_checkpoint_sha256=old.CHECKPOINTS[seed],generator_state_sha256='a'*64,generator_state_after_sha256='a'*64,
            completed_files=old.output_inventory(directory,base)))
    original_inputs={f'parent_{i}':'c'*64 for i in range(17)}
    report=dict(status='SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED',completed_head_fit_count=6,
        source_and_inputs_unchanged=True,postcheck_errors=[],deadline_met=True,completed_at_utc='2026-09-08T20:40:53Z',
        source_sha256=sources,seeds=states,original_input_sha256=dict(original_inputs))
    (tmp_path/'plan.json').write_bytes(run.encode({'source_sha256':{}}))
    (tmp_path/'report.json').write_bytes(run.encode(report))
    for i in range(204-len(old.output_inventory(tmp_path,base))):(tmp_path/f'payload_{i:03d}.txt').write_text(str(i))
    def seal():
        manifest=old.output_inventory(tmp_path,base);manifest.pop('SHA256SUMS',None)
        (tmp_path/'SHA256SUMS').write_text(''.join(v+'  '+n+'\n'for n,v in manifest.items()))
        monkeypatch.setattr(run,'PARENT_PLAN_SHA',base.digest(tmp_path/'plan.json'))
        monkeypatch.setattr(run,'PARENT_REPORT_SHA',base.digest(tmp_path/'report.json'))
        monkeypatch.setattr(run,'PARENT_MANIFEST_SHA',base.digest(tmp_path/'SHA256SUMS'))
    seal();monkeypatch.setattr(run,'source_identity',lambda *args:dict(sources))
    old_view=SimpleNamespace(CHECKPOINTS=old.CHECKPOINTS,output_inventory=old.output_inventory,input_receipts=lambda *args:dict(original_inputs))
    return {'declared_at_utc':'2026-09-08T20:59:00Z'},old_view,report,sources,seal


def test_all205_parent_files18_pt_and17_original_inputs_are_retained_and_hashed(tmp_path,monkeypatch):
    proposed,old_view,report,sources,seal=parent_fixture(tmp_path,monkeypatch)
    before=old.output_inventory(tmp_path,base)
    result,parent_plan,observed=run.parent_snapshot(proposed,base,old_view,oracle,tmp_path,SimpleNamespace())
    assert result['files']==before and len(before)==205 and sum(n.endswith('.pt')for n in before)==18
    assert len(result['original_parent_inputs'])==17 and len(result['source_sha256'])==22
    assert old.output_inventory(tmp_path,base)==before and observed==report


@pytest.mark.parametrize('fault',['payload','new_file','source','partial','postcheck','generator','original_inputs','chronology','manifest_self'])
def test_parent_source_completion_and_inventory_fail_closed(tmp_path,monkeypatch,fault):
    proposed,old_view,report,sources,seal=parent_fixture(tmp_path,monkeypatch)
    if fault=='payload':(tmp_path/'payload_000.txt').write_text('mutated')
    elif fault=='new_file':(tmp_path/'new.txt').write_text('unreviewed')
    elif fault=='source':
        (tmp_path/'source/file_00.py').write_text('changed');seal()
    elif fault=='chronology':proposed['declared_at_utc']='2026-09-08T20:00:00Z'
    elif fault=='manifest_self':
        with (tmp_path/'SHA256SUMS').open('a')as stream:stream.write('a'*64+'  SHA256SUMS\n')
        monkeypatch.setattr(run,'PARENT_MANIFEST_SHA',base.digest(tmp_path/'SHA256SUMS'))
    else:
        if fault=='partial':report['completed_head_fit_count']=5
        elif fault=='postcheck':report['postcheck_errors']=['failed']
        elif fault=='generator':report['seeds'][0]['generator_state_after_sha256']='f'*64
        else:report['original_input_sha256']['parent_0']='f'*64
        (tmp_path/'report.json').write_bytes(run.encode(report));seal()
    with pytest.raises(ValueError):run.parent_snapshot(proposed,base,old_view,oracle,tmp_path,SimpleNamespace())


def test_startup_core_failure_keeps_partial_report_and_original_snapshot(tmp_path,monkeypatch):
    workspace=tmp_path/'portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    dataset=tmp_path/'dataset/data';dataset.mkdir(parents=True);(dataset/'dataset.json').write_text('{}')
    (repo/'portable_e2e').mkdir();(repo/'portable_e2e/losses.py').write_text('fixture')
    benchmarks=workspace/'benchmarks';benchmarks.mkdir()
    paths={n:benchmarks/(n+'.py')for n in('worker','core','frozen_worker','frozen_core','oracle_helper')}
    for n,p in paths.items():p.write_text(n)
    proposed=dict(finish_before_utc='2026-09-09T01:00:00Z',residual_core_source_sha256='c'*64)
    plan_path=benchmarks/'plan.json';plan_path.write_bytes(run.encode(proposed));output=workspace/'runs/diagnostics/fixture'
    snapshot=dict(files={'retained_cache.pt':'a'*64},original_parent_inputs={'old_parent.pt':'b'*64},source_sha256={})
    monkeypatch.setattr(run,'WORKSPACE',workspace);monkeypatch.setattr(run,'__file__',str(paths['worker']))
    monkeypatch.setattr(run,'datetime',FrozenTime);monkeypatch.setattr(run,'validate_plan',lambda *a:None)
    monkeypatch.setattr(run,'source_identity',lambda *a:{})
    monkeypatch.setattr(run,'parent_snapshot',lambda *a:(deepcopy(snapshot),{},{}))
    monkeypatch.setattr(run,'load_external',lambda *a:(_ for _ in ()).throw(RuntimeError('core fixture failure')))
    monkeypatch.setattr(torch.cuda,'device_count',lambda:1);monkeypatch.setattr(torch,'get_num_threads',lambda:16)
    monkeypatch.setattr(torch.cuda,'get_device_properties',lambda _:SimpleNamespace(uuid='fixture'))
    owner=SimpleNamespace(**{k:getattr(base,k)for k in('json_read','digest','write_json','timestamp','utc','checked_output')})
    owner.check_environment=lambda *a:None;owner.DATASET=str(dataset);owner.MANIFEST_SHA=base.digest(dataset/'dataset.json')
    owner.gpu_lease=lambda _:nullcontext();owner.bounded_signals=lambda _:nullcontext();owner.finalization_signals=lambda:nullcontext()
    owner.gpu_idle=lambda _:dict(fixture_only=True);owner.verify_torch_uuid=lambda _:'fixture'
    args=SimpleNamespace(plan=plan_path,output_dir=output,**{n:p for n,p in paths.items()if n!='worker'})
    assert run.run(args,owner,old,SimpleNamespace(projection_definition=lambda _:(None,{})))==2
    report=json.loads((output/'report.json').read_bytes())
    assert report['status']=='FAILED_OR_PARTIAL_NOT_PROMOTED' and report['error_type']=='RuntimeError'
    assert report['completed_head_fit_count']==0 and report['postcheck_errors']==[]
    assert report['original_cache_run_file_sha256']==snapshot['files'] and report['generator_neural_forwards']==0
    names=[]
    for line in (output/'SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ');assert base.digest(output/name)==value;names.append(name)
    assert set(names)=={'plan.json','report.json'}
    with pytest.raises(ValueError,match='exist'):run.run(args,owner,old,SimpleNamespace(projection_definition=lambda _:(None,{})))


def test_cli_abbreviations_and_missing_required_bindings_rejected():
    with pytest.raises(SystemExit):run.main(['--pla','missing'])
