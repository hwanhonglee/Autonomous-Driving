"""HH_260906 - Test continuation orchestration on CPU fixtures without opening real parents or using a GPU."""
from contextlib import nullcontext
from copy import deepcopy
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import run_frozen_stopmix_drive_rank as run
from scripts.e2e import run_frozen_stopmix_residual as r
from scripts.e2e import run_frozen_stopmix_selector as old
from scripts.e2e import profile_portable_training as base
from scripts.e2e import diagnose_stopmix_training_oracles as oracle
from portable_e2e import frozen_stopmix_residual as residual,losses


class FrozenTime(datetime):
    @classmethod
    def now(cls,tz=None):return cls(2026,9,8,22,0,tzinfo=timezone.utc)


def plan():
    _,proof=oracle.projection_definition(Path(losses.__file__).read_bytes());value=run.plan_contract(base,old,r,oracle,proof)
    value.update(declared_at_utc='2026-09-08T21:59:00Z',source_sha256={n:'a'*64 for n in old.source_paths(base)},
        worker_source_sha256='b'*64,rank_core_source_sha256='c'*64)
    return value,proof


def test_fixed_parent_continuation_plan_no_optimizer_resume_or_generator(monkeypatch):
    value,proof=plan();monkeypatch.setattr(run,'datetime',FrozenTime);run.validate_plan(value,base,old,r,oracle,proof)
    assert value['arms']==['ce_continue','drive_pair_continue'] and value['pair_loss_weight']=={'ce_continue':0.,'drive_pair_continue':1.}
    assert value['head_parent_arm']=='candidate_residual'and value['parent_head_steps']==1540 and value['optimizer_resume']is False
    assert value['head_parameter_count']==115457 and value['steps']==1540 and value['samples_seen_per_head']==6155
    assert value['cache_files_loaded']==6 and value['parent_head_files_loaded']==value['parent_logit_files_loaded']==3
    assert value['generator_neural_forwards']==0 and value['baseline_directory']=='parent_candidate_residual'
    assert value['internal_wall_timeout_seconds']==2400 and value['external_wall_timeout_seconds']==2700
    assert 'sum' in value['pair_gap_normalization'] and 'zero term' in value['objective']


@pytest.mark.parametrize('key,value',[('steps',2870),('batch_size',8),('head_parameter_count',115201),('parent_head_steps',2870),
    ('head_parent_arm','prior_only_residual'),('optimizer_resume',True),('baseline_directory','original'),
    ('pair_loss_weight',{'ce_continue':0.,'drive_pair_continue':.1}),('generator_neural_forwards',1),('parent_logit_files_loaded',0),
    ('internal_wall_timeout_seconds',3000),('rank_core_source_sha256','bad'),('declared_at_utc','2026-09-08T23:00:00Z')])
def test_plan_fixed_scope_rejects_unreviewed_settings(monkeypatch,key,value):
    proposal,proof=plan();proposal[key]=value;monkeypatch.setattr(run,'datetime',FrozenTime)
    with pytest.raises(ValueError):run.validate_plan(proposal,base,old,r,oracle,proof)


def test_full_source_inventory_and_remaining_deadline_reserve(monkeypatch):
    proposal,proof=plan();monkeypatch.setattr(run,'datetime',FrozenTime);proposal['source_sha256'].clear()
    with pytest.raises(ValueError,match='inventory'):run.validate_plan(proposal,base,old,r,oracle,proof)
    class Late(FrozenTime):
        @classmethod
        def now(cls,tz=None):return cls(2026,9,9,0,13,tzinfo=timezone.utc)
    proposal,proof=plan();monkeypatch.setattr(run,'datetime',Late)
    with pytest.raises(ValueError,match='reserve'):run.validate_plan(proposal,base,old,r,oracle,proof)


@pytest.mark.parametrize('fault',['original_child','residual_child','original','residual','traversal','alias'])
def test_output_must_be_fresh_separate_from_both_immutable_parent_roots(tmp_path,monkeypatch,fault):
    workspace=tmp_path/'workspace';diagnostics=workspace/'runs/diagnostics';original=diagnostics/'original';parent=diagnostics/'residual'
    original.mkdir(parents=True);parent.mkdir();dataset=tmp_path/'dataset';dataset.mkdir()
    monkeypatch.setattr(run,'WORKSPACE',workspace);monkeypatch.setattr(run,'PARENT_ROOT',parent)
    previous=SimpleNamespace(PARENT_ROOT=original)
    alias=tmp_path/'alias';alias.symlink_to(diagnostics,target_is_directory=True)
    output={'original_child':original/'new','residual_child':parent/'new','original':original,'residual':parent,
        'traversal':diagnostics/'..'/'..'/'escape','alias':alias/'new'}[fault]
    with pytest.raises(ValueError):run.checked_output(output,dataset,base,previous)
    assert list(original.iterdir())==list(parent.iterdir())==[]
    assert run.checked_output(diagnostics/'new_sibling',dataset,base,previous)==diagnostics/'new_sibling'


def test_external_hash_before_import_and_failed_import_removed(tmp_path):
    path=tmp_path/'module.py';path.write_text("raise RuntimeError('fixture')");name='_drive_rank_failed_fixture'
    with pytest.raises(ValueError,match='SHA'):run.load_external(path,'a'*64,name)
    with pytest.raises(RuntimeError):run.load_external(path,run.sha(path.read_bytes()),name)
    assert name not in sys.modules
    alias=tmp_path/'alias';alias.symlink_to(path)
    with pytest.raises(ValueError):run.load_external(alias,run.sha(path.read_bytes()),name)


def test_external_canonical_package_explicit_loading(tmp_path):
    path=tmp_path/'module.py';path.write_text('VALUE=1');name='portable_e2e._drive_rank_fixture'
    try:
        module=run.load_external(path,run.sha(path.read_bytes()),name);assert module.VALUE==1 and module.__package__=='portable_e2e'
        with pytest.raises(ValueError,match='already'):run.load_external(path,run.sha(path.read_bytes()),name)
    finally:sys.modules.pop(name,None)


def parent_tensors_fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(run,'PARENT_ROOT',tmp_path);seed='20260903';directory=tmp_path/f'seed_{seed}/candidate_residual';directory.mkdir(parents=True)
    weights={name:torch.zeros(shape)for name,shape in {'0.weight':(256,449),'0.bias':(256,),'2.weight':(1,256),'2.bias':(1,)}.items()}
    state=dict(parent_checkpoint_sha256=old.CHECKPOINTS[seed],cache_digest={'train':'a'*64,'val':'b'*64})
    head_report=dict(artifact_id=residual.ARTIFACT_ID,arm='candidate_residual',seed=int(seed),
        status='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',head_parameter_count=115457,
        source_checkpoint_sha256=old.CHECKPOINTS[seed],final_state=dict(global_step=1540,samples_seen=6155),
        cache_sha256_before=dict(state['cache_digest']),cache_sha256_after=dict(state['cache_digest']),
        final_head_sha256=residual._digest_tensors(weights.items()))
    (directory/'report.json').write_bytes(run.encode(head_report))
    values={'head_only.pt':dict(artifact_id=residual.ARTIFACT_ID,head_state_dict=weights,parent_checkpoint_sha256=old.CHECKPOINTS[seed],
        parent_cache_report_sha256=r.PARENT_REPORT_SHA,**r.DENIALS),
        'final_logits.pt':dict(artifact_id='portable_e2e.frozen_stopmix_residual_logits.research.v1',
            logits={'train':torch.zeros(1147,12),'val':torch.ones(337,12)},**r.DENIALS)}
    for name in values:(directory/name).write_bytes(('opaque safe-reader fixture '+name).encode())
    snapshot={'residual_files':old.output_inventory(tmp_path,base)}
    def reader(path,device):
        assert device==torch.device('cpu')and path.parent==directory
        return values[path.name],base.digest(path)
    return seed,state,snapshot,values,head_report,reader,directory


def test_secure_parent_cpu_state_and_logits_exact_abi_without_checkpoint_or_image_access(tmp_path,monkeypatch):
    seed,state,snapshot,values,head,reader,directory=parent_tensors_fixture(tmp_path,monkeypatch)
    parameters,logits,proof=run.load_parent(seed,state,snapshot,base,r,residual,torch,reader)
    assert sum(v.numel()for v in parameters.values())==115457
    assert logits['train'].shape==(1147,12)and logits['val'].shape==(337,12)
    assert proof['weights_only']is True and proof['optimizer_loaded']is False and proof['head_state_sha256']==head['final_head_sha256']


@pytest.mark.parametrize('fault',['artifact','head_extra','denial','denial_type','parent_source','cache_report','key','order','shape',
    'dtype','nonfinite','gradient','state_digest','logit_artifact','logit_split','logit_shape','logit_dtype','logit_nonfinite',
    'logit_gradient','report_seed','report_status','report_capacity','report_exposure','report_cache','decode_sha','decode_mutation','unsafe_loader'])
def test_parent_tensor_or_provenance_corruption_fail_closed(tmp_path,monkeypatch,fault):
    seed,state,snapshot,values,report,reader,directory=parent_tensors_fixture(tmp_path,monkeypatch)
    head=values['head_only.pt'];logits=values['final_logits.pt'];weights=head['head_state_dict'];original_reader=reader
    if fault=='artifact':head['artifact_id']='runtime.checkpoint'
    elif fault=='head_extra':head['optimizer_state_dict']={}
    elif fault=='denial':head['generator_training']=True
    elif fault=='denial_type':head['generator_neural_forwards']=False
    elif fault=='parent_source':head['parent_checkpoint_sha256']='f'*64
    elif fault=='cache_report':head['parent_cache_report_sha256']='f'*64
    elif fault=='key':weights['bad']=torch.zeros(1)
    elif fault=='order':head['head_state_dict']=dict(reversed(list(weights.items())))
    elif fault=='shape':weights['0.weight']=torch.zeros(256,448)
    elif fault=='dtype':weights['0.weight']=weights['0.weight'].double()
    elif fault=='nonfinite':weights['0.weight'][0,0]=float('nan')
    elif fault=='gradient':weights['0.weight'].requires_grad_(True)
    elif fault=='state_digest':weights['0.weight'][0,0]=1.
    elif fault=='logit_artifact':logits['artifact_id']='wrong'
    elif fault=='logit_split':logits['logits']['test']=torch.zeros(1,12)
    elif fault=='logit_shape':logits['logits']['val']=torch.zeros(336,12)
    elif fault=='logit_dtype':logits['logits']['val']=logits['logits']['val'].double()
    elif fault=='logit_nonfinite':logits['logits']['val'][0,0]=float('inf')
    elif fault=='logit_gradient':logits['logits']['val'].requires_grad_(True)
    elif fault=='report_seed':report['seed']=20260904
    elif fault=='report_status':report['status']='STARTED'
    elif fault=='report_capacity':report['head_parameter_count']=115201
    elif fault=='report_exposure':report['final_state']['samples_seen']=11470
    elif fault=='report_cache':report['cache_sha256_after']['train']='f'*64
    elif fault=='decode_sha':
        def reader(path,device):return values[path.name],'f'*64
    elif fault=='decode_mutation':
        def reader(path,device):
            answer=original_reader(path,device);path.write_bytes(b'mutated');return answer
    else:
        def reader(path,device):raise ValueError('weights_only unavailable; no fallback')
    (directory/'report.json').write_bytes(run.encode(report))
    with pytest.raises(ValueError):run.load_parent(seed,state,snapshot,base,r,residual,torch,reader)


def snapshot_fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(run,'PARENT_ROOT',tmp_path);sources={}
    for i in range(24):
        name=f'file{i}.py';path=tmp_path/'source'/name;path.parent.mkdir(exist_ok=True);path.write_text(name);sources[name]=base.digest(path)
    states=[]
    for seed in old.CHECKPOINTS:
        path=tmp_path/f'seed_{seed}';path.mkdir()
        for i in range(4):(path/f'tensor{i}.pt').write_bytes(b'opaque')
        states.append(dict(seed=seed,directory=path.name,status='TWO_RESIDUAL_HEADS_COMPLETE_NOT_PROMOTED',
            parent_checkpoint_sha256=old.CHECKPOINTS[seed],completed_files=old.output_inventory(path,base)))
    original=dict(files={f'file{i}':'a'*64 for i in range(205)},original_parent_inputs={f'parent{i}':'b'*64 for i in range(17)},source_sha256=sources)
    report=dict(status='SIX_RESIDUAL_HEAD_FITS_COMPLETE_NOT_PROMOTED',completed_head_fit_count=6,postcheck_errors=[],
        source_and_inputs_unchanged=True,deadline_met=True,completed_at_utc='2026-09-08T21:14:03Z',seeds=states,
        source_sha256=sources,original_cache_run_file_sha256=dict(original['files']),original_parent_input_sha256=dict(original['original_parent_inputs']))
    (tmp_path/'plan.json').write_bytes(b'{}\n');(tmp_path/'report.json').write_bytes(run.encode(report))
    for i in range(194-len(old.output_inventory(tmp_path,base))):(tmp_path/f'payload{i}').write_bytes(str(i).encode())
    def seal():
        files=old.output_inventory(tmp_path,base);files.pop('SHA256SUMS',None)
        (tmp_path/'SHA256SUMS').write_text(''.join(v+'  '+n+'\n'for n,v in files.items()))
        for name,file in(('PARENT_PLAN_SHA','plan.json'),('PARENT_REPORT_SHA','report.json'),('PARENT_MANIFEST_SHA','SHA256SUMS')):
            monkeypatch.setattr(run,name,base.digest(tmp_path/file))
    seal();source=dict(sources,external_rank_worker='c'*64)
    monkeypatch.setattr(run,'source_identity',lambda *a:dict(sources,**{'external_rank_worker.py':'c'*64,'external_rank_core.py':'d'*64}))
    residual_view=SimpleNamespace(parent_snapshot=lambda *a:(deepcopy(original),{},{}))
    args=SimpleNamespace(residual_core='core',frozen_core='frozen',frozen_worker='worker',oracle_helper='oracle')
    return {'declared_at_utc':'2026-09-08T21:59:00Z'},residual_view,args,report,original,seal


def test_both_parent_inventories_195_and205_and17_lineage_inputs_bound(tmp_path,monkeypatch):
    proposed,view,args,report,original,seal=snapshot_fixture(tmp_path,monkeypatch)
    result,*_=run.parent_snapshot(proposed,base,old,view,oracle,tmp_path,args)
    assert len(result['residual_files'])==195 and sum(n.endswith('.pt')for n in result['residual_files'])==12
    assert len(result['original']['files'])==205 and len(result['original']['original_parent_inputs'])==17
    assert len(result['source_sha256'])==26


@pytest.mark.parametrize('fault',['original_bytes','parent_inputs','residual_bytes','partial','seed','source','chronology'])
def test_resealed_or_mutated_parent_chain_rejected(tmp_path,monkeypatch,fault):
    proposed,view,args,report,original,seal=snapshot_fixture(tmp_path,monkeypatch)
    if fault=='original_bytes':report['original_cache_run_file_sha256']['file0']='f'*64
    elif fault=='parent_inputs':report['original_parent_input_sha256']['parent0']='f'*64
    elif fault=='residual_bytes':(tmp_path/'payload0').write_bytes(b'changed')
    elif fault=='partial':report['completed_head_fit_count']=5
    elif fault=='seed':report['seeds'].reverse()
    elif fault=='source':report['source_sha256']=dict(report['source_sha256'],unreviewed='f'*64)
    else:proposed['declared_at_utc']='2026-09-08T21:00:00Z'
    if fault not in('residual_bytes','chronology'):(tmp_path/'report.json').write_bytes(run.encode(report));seal()
    with pytest.raises(ValueError):run.parent_snapshot(proposed,base,old,view,oracle,tmp_path,args)


def test_startup_failure_keeps_report_manifest_and_both_parent_proofs_without_gpu(tmp_path,monkeypatch):
    workspace=tmp_path/'portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    dataset=tmp_path/'dataset/data';dataset.mkdir(parents=True);(dataset/'dataset.json').write_text('{}')
    (repo/'portable_e2e').mkdir();(repo/'portable_e2e/losses.py').write_text('fixture')
    directory=workspace/'benchmarks';directory.mkdir();paths={n:directory/(n+'.py')for n in('worker','core','residual_worker','residual_core','frozen_worker','frozen_core','oracle_helper')}
    for name,path in paths.items():path.write_text(name)
    proposed=dict(finish_before_utc='2026-09-09T01:00:00Z',rank_core_source_sha256='c'*64)
    plan_path=directory/'plan.json';plan_path.write_bytes(run.encode(proposed));output=workspace/'runs/diagnostics/new'
    snapshot=dict(original=dict(files={'original.pt':'a'*64},original_parent_inputs={'parent.pt':'b'*64}),residual_files={'head.pt':'c'*64},source_sha256={})
    monkeypatch.setattr(run,'WORKSPACE',workspace);monkeypatch.setattr(run,'__file__',str(paths['worker']));monkeypatch.setattr(run,'datetime',FrozenTime)
    monkeypatch.setattr(run,'validate_plan',lambda *a:None);monkeypatch.setattr(run,'parent_snapshot',lambda *a:(deepcopy(snapshot),{},{},{}))
    monkeypatch.setattr(run,'load_external',lambda *a:(_ for _ in()).throw(RuntimeError('fixture import failure')))
    monkeypatch.setattr(torch.cuda,'device_count',lambda:1);monkeypatch.setattr(torch,'get_num_threads',lambda:16)
    monkeypatch.setattr(torch.cuda,'get_device_properties',lambda _:SimpleNamespace(uuid='fixture'))
    owner=SimpleNamespace(**{n:getattr(base,n)for n in('json_read','digest','write_json','timestamp','utc','checked_output')})
    owner.check_environment=lambda *a:None;owner.DATASET=str(dataset);owner.MANIFEST_SHA=base.digest(dataset/'dataset.json')
    owner.gpu_lease=lambda _:nullcontext();owner.bounded_signals=lambda _:nullcontext();owner.finalization_signals=lambda:nullcontext()
    owner.gpu_idle=lambda _:dict(fixture_only=True);owner.verify_torch_uuid=lambda _:'fixture'
    args=SimpleNamespace(plan=plan_path,output_dir=output,**{n:p for n,p in paths.items()if n!='worker'})
    assert run.run(args,owner,old,r,SimpleNamespace(projection_definition=lambda _:(None,{})))==2
    report=json.loads((output/'report.json').read_bytes());assert report['completed_head_fit_count']==0 and report['postcheck_errors']==[]
    assert report['status']=='FAILED_OR_PARTIAL_NOT_PROMOTED'and report['generator_neural_forwards']==0
    assert report['original_cache_run_file_sha256']==snapshot['original']['files']and report['original_residual_run_file_sha256']==snapshot['residual_files']
    names=[]
    for line in(output/'SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ');assert base.digest(output/name)==value;names.append(name)
    assert set(names)=={'plan.json','report.json'}


def test_cli_does_not_accept_abbreviation_or_missing_source_bindings():
    with pytest.raises(SystemExit):run.main(['--pla','missing'])
