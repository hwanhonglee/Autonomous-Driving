"""HH_260906 - Verify external frozen scorer orchestration with CPU-only synthetic data and mocked ownership."""
from contextlib import nullcontext
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import run_frozen_stopmix_selector as run
from scripts.e2e import profile_portable_training as base
from scripts.e2e import diagnose_stopmix_training_oracles as oracle
from portable_e2e import losses
from portable_e2e import frozen_stopmix_selector as core


class FrozenTime(datetime):
    @classmethod
    def now(cls, tz=None): return cls(2026, 9, 8, 20, 0, tzinfo=timezone.utc)


def plan():
    _, projection = oracle.projection_definition(Path(losses.__file__).read_bytes())
    value = run.plan_contract(base, oracle, projection)
    value.update(declared_at_utc='2026-09-08T19:59:00Z', source_sha256={n: 'a' * 64 for n in run.source_paths(base)},
        worker_source_sha256='b' * 64, core_source_sha256='c' * 64)
    return value, projection


def test_fixed_plan_six_head_only_fits_and_test_integrity_boundary(monkeypatch):
    value, projection = plan(); monkeypatch.setattr(run, 'datetime', FrozenTime)
    run.validate_plan(value, base, oracle, projection)
    assert value['expected_head_fit_count'] == 6 and value['sample_exposures_per_head'] == 6155
    assert value['parent_steps'] == 2870 and value['parent_samples_seen'] == 11470
    assert list(value['parent_checkpoint_sha256']) == ['20260903', '20260904', '20260905']
    assert value['arms'] == ['linear_pair_reset', 'candidate_reset']
    assert value['generator_training'] is False and value['test_neural_inference'] is False
    assert 'held-out test bytes for integrity' in value['integrity_scope']
    assert value['fixed_val_render_indices'] == list(range(0)) + [0,31,61,92,122,153,183,214,244,275,305,336]


@pytest.mark.parametrize('key,value', [('candidate_count',6),('steps',1541),('batch_size',8),('learning_rate',.001),
    ('parent_steps',1540),('expected_head_fit_count',5),('generator_training',True),('test_neural_inference',True),
    ('internal_wall_timeout_seconds',3600),('core_source_sha256','bad'),('declared_at_utc','2026-09-08T21:00:00Z')])
def test_plan_cannot_expand_or_relabel_scope(monkeypatch, key, value):
    proposal, projection = plan(); proposal[key] = value; monkeypatch.setattr(run, 'datetime', FrozenTime)
    with pytest.raises(ValueError): run.validate_plan(proposal, base, oracle, projection)


def test_plan_requires_full_deadline_reserve(monkeypatch):
    class Late(FrozenTime):
        @classmethod
        def now(cls,tz=None): return cls(2026,9,9,0,0,tzinfo=timezone.utc)
    proposal, projection = plan(); monkeypatch.setattr(run,'datetime',Late)
    with pytest.raises(ValueError, match='reserve'): run.validate_plan(proposal,base,oracle,projection)


def test_external_bytes_hash_checked_before_execution_and_symlinks_rejected(tmp_path):
    path = tmp_path/'module.py'; path.write_text("raise RuntimeError('must not execute')\n")
    with pytest.raises(ValueError, match='SHA'): run.load_external(path,'a'*64,'_bad_external_fixture')
    assert '_bad_external_fixture' not in sys.modules
    alias = tmp_path/'alias.py'; alias.symlink_to(path)
    with pytest.raises(ValueError, match='nonsymlink'): run.load_external(alias,run.sha(path.read_bytes()),'_bad_external_fixture')


def test_external_load_is_explicit_and_failed_module_removed(tmp_path):
    path = tmp_path/'module.py'; path.write_text("raise RuntimeError('fixture')\n")
    with pytest.raises(RuntimeError): run.load_external(path,run.sha(path.read_bytes()),'_failed_external_fixture')
    assert '_failed_external_fixture' not in sys.modules
    path.write_text('VALUE = 12\n'); name='_successful_external_fixture'
    try:
        loaded=run.load_external(path,run.sha(path.read_bytes()),name); assert loaded.VALUE==12
        with pytest.raises(ValueError,match='already'): run.load_external(path,run.sha(path.read_bytes()),name)
    finally: sys.modules.pop(name,None)


def test_stage_alarm_preserves_original_deadline(monkeypatch):
    calls=[]; moments=iter([10.,25.])
    monkeypatch.setattr(run.signal,'getitimer',lambda _: (100.,0.))
    monkeypatch.setattr(run.signal,'setitimer',lambda *args:calls.append(args))
    monkeypatch.setattr(run.time,'monotonic',lambda:next(moments))
    with run.stage_alarm(20): pass
    assert calls==[(signal.ITIMER_REAL,20),(signal.ITIMER_REAL,85.,0.)]


def test_stage_alarm_expired_original_budget_does_not_restart(monkeypatch):
    calls=[]; moments=iter([0.,20.])
    monkeypatch.setattr(run.signal,'getitimer',lambda _: (10.,0.))
    monkeypatch.setattr(run.signal,'setitimer',lambda *args:calls.append(args))
    monkeypatch.setattr(run.time,'monotonic',lambda:next(moments))
    with pytest.raises(RuntimeError):
        with run.stage_alarm(20): raise RuntimeError('partial')
    assert calls[-1][1]==1e-6


def test_stage_requires_whole_study_timer(monkeypatch):
    monkeypatch.setattr(run.signal,'getitimer',lambda _: (0.,0.))
    with pytest.raises(ValueError,match='whole-study'):
        with run.stage_alarm(20): pass


class DatasetView:
    def __init__(self, split):
        count,self.fingerprint_sha256=run.SPLITS[split]; self.split=split
        self.examples=[SimpleNamespace(token=f'{split}-{i}',episode_id=f'{split}-episode-{i%3 if split=="train" else 0}')for i in range(count)]
    def __len__(self):return len(self.examples)


def datasets():return {s:DatasetView(s)for s in run.SPLITS}


def test_complete_disjoint_train_val_and_parent_identity():
    data=datasets();run.validate_datasets(data,base)
    payload=dict(dataset_fingerprint_sha256=run.SPLITS['train'][1],state=dict(global_step=2870,samples_seen=11470),
        train_config=dict(seed=20260903),loss_config=base.LOSS_CONFIG)
    config=SimpleNamespace(model_id=run.MODEL_ID,candidate_count=12)
    training_ids=sorted({e.episode_id for e in data['train'].examples})
    run.validate_parent(payload,config,training_ids,'20260903',data,base)
    payload['state']['global_step']=1540
    with pytest.raises(ValueError):run.validate_parent(payload,config,training_ids,'20260903',data,base)


@pytest.mark.parametrize('fault',['count','fingerprint','duplicate','leak'])
def test_split_identity_corruptions_rejected(fault):
    data=datasets()
    if fault=='count':data['val'].examples.pop()
    elif fault=='fingerprint':data['train'].fingerprint_sha256='a'*64
    elif fault=='duplicate':data['val'].examples[1].token=data['val'].examples[0].token
    else:data['val'].examples[0].token=data['train'].examples[0].token
    with pytest.raises(ValueError):run.validate_datasets(data,base)


def test_tensor_state_digest_binds_shape_dtype_and_bytes():
    a=run.tensor_state_sha({'x':torch.zeros(2,3)})
    assert a!=run.tensor_state_sha({'x':torch.zeros(3,2)})
    assert a!=run.tensor_state_sha({'x':torch.zeros(2,3,dtype=torch.float64)})
    assert a!=run.tensor_state_sha({'x':torch.ones(2,3)})


def test_imported_modules_must_resolve_to_each_frozen_source(tmp_path):
    modules={}
    names=('contract','dataset','torch_dataset','losses','model','train','evaluate','audit_runtime','runtime','runtime_contract','frozen_selector','visualize','stop_primitive_research')
    for name in names:modules['portable_e2e.'+name]=SimpleNamespace(__file__=str(tmp_path/'portable_e2e'/f'{name}.py'))
    modules['scripts.e2e.audit_portable_stopmix_behavior']=SimpleNamespace(__file__=str(tmp_path/'scripts/e2e/audit_portable_stopmix_behavior.py'))
    assert len(run.verify_import_paths(tmp_path,modules))==14
    modules['portable_e2e.model'].__file__=str(tmp_path.parent/'wrong_model.py')
    with pytest.raises(ValueError,match='escaped'):run.verify_import_paths(tmp_path,modules)


@pytest.mark.parametrize('fault',['training','requires_grad','gradient'])
def test_generator_eval_grad_and_frozen_state_are_enforced(fault):
    model=TinyGenerator();run.frozen_generator_sha(model)
    if fault=='training':model.train()
    elif fault=='requires_grad':next(model.parameters()).requires_grad_(True)
    else:next(model.parameters()).grad=torch.zeros_like(next(model.parameters()))
    with pytest.raises(ValueError,match='eval-only'):run.frozen_generator_sha(model)


class TinyDataset(torch.utils.data.Dataset):
    split='train';fingerprint_sha256=run.SPLITS['train'][1]
    def __init__(self):
        self.examples=[SimpleNamespace(token=f'train-{i}',episode_id=f'train-{i%3}',targets_xy=[(0.,0.)]*64)for i in range(5)]
    def __len__(self):return len(self.examples)
    def __getitem__(self,i):
        return dict(images=torch.ones(1),calibration=torch.ones(1),ego_history=torch.full((1,13),float(i)),
            ego_history_mask=torch.ones(1,dtype=torch.bool),route_xy=torch.zeros(64,2),route_mask=torch.ones(64,dtype=torch.bool),
            target_xy=torch.zeros(64,2),target_speed_mps=torch.zeros(64),target_yaw_rad=torch.zeros(64),
            target_valid=torch.ones(64,dtype=torch.bool),sample_id=self.examples[i].token)


class TinyGenerator(torch.nn.Module):
    def __init__(self):
        super().__init__();self.fusion=torch.nn.Identity();self.drive=torch.nn.Linear(256,6);self.stop=torch.nn.Linear(256,6)
        self.requires_grad_(False);self.eval()
    def forward(self,images,calibration,ego_history,history_mask,route_xy,route_mask):
        context=self.fusion(ego_history[:,-1,:1].expand(-1,256).contiguous())
        logits=torch.cat((self.drive(context),self.stop(context)),1)
        return torch.zeros(len(context),12,64,2),torch.zeros(len(context),12,64),logits


def test_cache_extraction_retains_all_ordered_rows_masks_and_frozen_state_cpu(tmp_path,monkeypatch):
    original_to=torch.Tensor.to
    def cpu_only(tensor,*args,**kwargs):
        if args and args[0]=='cuda:0':args=('cpu',*args[1:])
        return original_to(tensor,*args,**kwargs)
    monkeypatch.setattr(torch.Tensor,'to',cpu_only);monkeypatch.setattr(run,'stage_alarm',lambda _:nullcontext())
    model=TinyGenerator();before=run.tensor_state_sha(model.state_dict());journal=tmp_path/'cache.jsonl'
    data,extra,ids,observed=run.extract_cache(model,TinyDataset(),core,oracle,base,'a'*64,journal,torch)
    assert len(data)==len(ids)==5 and data.candidate_xy.shape==(5,12,64,2)
    assert observed==before==run.tensor_state_sha(model.state_dict())
    assert [r['sample_id']for r in ids]==[f'train-{i}'for i in range(5)]
    assert data.target_valid.all() and not data.fused.requires_grad
    assert len(journal.read_text().splitlines())==5 and not model.fusion._forward_hooks


def test_completed_persisted_files_cannot_be_resealed_by_later_work(tmp_path):
    directory=tmp_path/'seed';directory.mkdir();file=directory/'samples.jsonl';file.write_bytes(b'first\n')
    records=[dict(directory='seed',completed_files={'samples.jsonl':base.digest(file)})]
    assert run.completed_outputs_intact(tmp_path,records,base)
    file.write_bytes(b'changed\n');assert not run.completed_outputs_intact(tmp_path,records,base)


def history_fixture():
    records=[];seen=0
    for step in range(1,1541):
        count=3 if step%287==0 else 4;seen+=count
        records.append(dict(global_step=step,batch_samples=count,samples_seen=seen))
    return records


@pytest.mark.parametrize('fault',[None,'truncated','changed','nonfinite'])
def test_exact_persisted_history_and_exposures(tmp_path,fault):
    rows=history_fixture();path=tmp_path/'metrics.jsonl';payload=b''.join(run.encoded(r)for r in rows)
    if fault=='truncated':payload=b''.join(run.encoded(r)for r in rows[:-1])
    elif fault=='changed':payload=payload.replace(b'"global_step":1,',b'"global_step":2,',1)
    elif fault=='nonfinite':payload+=b'{"x":NaN}\n'
    path.write_bytes(payload)
    if fault is None:run.verify_history(path,{'history':rows});assert rows[-1]['samples_seen']==6155
    else:
        with pytest.raises(ValueError):run.verify_history(path,{'history':rows})


def test_research_tensor_save_never_overwrites_or_uses_runtime_checkpoint_id(tmp_path):
    path=tmp_path/'head.pt';run.save_tensors(path,{'artifact_id':run.CACHE_ID,'x':torch.ones(1)},torch)
    before=path.read_bytes()
    with pytest.raises(ValueError):run.save_tensors(path,{},torch)
    assert path.read_bytes()==before and 'research' in run.CACHE_ID


def test_source_guard_disables_fetch_and_rejects_dirty_source(monkeypatch,tmp_path):
    def fake(cmd,**kwargs):
        assert cmd[:3]==['git','-c','protocol.allow=never'];assert kwargs['timeout']==10
        assert kwargs['env']['GIT_NO_LAZY_FETCH']=='1' and kwargs['env']['GIT_ALLOW_PROTOCOL']==''
        return (run.SOURCE_COMMIT+'\n').encode()if'rev-parse'in cmd else b'?? changed.py\n'
    monkeypatch.setattr(run.subprocess,'check_output',fake)
    with pytest.raises(ValueError,match='clean'):run.source_identity({},base,tmp_path,tmp_path/'core',tmp_path/'oracle')


def test_cli_abbreviation_is_not_accepted():
    with pytest.raises(SystemExit):run.main(['--pla','missing'])


def test_startup_core_failure_keeps_failed_report_and_manifest_without_gpu(tmp_path,monkeypatch):
    workspace=tmp_path/'portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    dataset=tmp_path/'dataset/data';dataset.mkdir(parents=True);(dataset/'dataset.json').write_text('{}')
    (repo/'portable_e2e').mkdir();(repo/'portable_e2e/losses.py').write_text('frozen fixture')
    benchmarks=workspace/'benchmarks';benchmarks.mkdir()
    worker=benchmarks/'worker.py';worker.write_text('safe worker fixture')
    core_path=benchmarks/'core.py';core_path.write_text('safe core fixture')
    oracle_path=benchmarks/'oracle.py';oracle_path.write_text('safe oracle fixture')
    proposal={'finish_before_utc':'2026-09-09T01:00:00Z','core_source_sha256':'c'*64}
    plan_path=benchmarks/'plan.json';plan_path.write_bytes(run.encoded(proposal));output=workspace/'runs/diagnostics/fixture'
    monkeypatch.setattr(run,'WORKSPACE',workspace);monkeypatch.setattr(run,'__file__',str(worker))
    monkeypatch.setattr(run,'datetime',FrozenTime);monkeypatch.setattr(run,'validate_plan',lambda *a:None)
    monkeypatch.setattr(run,'source_identity',lambda *a:{});monkeypatch.setattr(run,'input_receipts',lambda *a:{})
    monkeypatch.setattr(run,'load_external',lambda *a:(_ for _ in ()).throw(RuntimeError('core fixture failure')))
    monkeypatch.setattr(torch.cuda,'device_count',lambda:1);monkeypatch.setattr(torch,'get_num_threads',lambda:16)
    monkeypatch.setattr(torch.cuda,'get_device_properties',lambda _:SimpleNamespace(uuid='fixture'))
    owner=SimpleNamespace(**{k:getattr(base,k)for k in ('json_read','digest','write_json','timestamp','utc','checked_output')})
    owner.check_environment=lambda *a:None;owner.DATASET=str(dataset);owner.MANIFEST_SHA=base.digest(dataset/'dataset.json')
    owner.gpu_lease=lambda _:nullcontext();owner.bounded_signals=lambda _:nullcontext();owner.finalization_signals=lambda:nullcontext()
    owner.gpu_idle=lambda _:dict(physical_index=0,fixture_only=True);owner.verify_torch_uuid=lambda _:'fixture'
    helper=SimpleNamespace(projection_definition=lambda _:(None,{}))
    args=SimpleNamespace(plan=plan_path,core=core_path,oracle_helper=oracle_path,output_dir=output)
    assert run.run(args,owner,helper)==2
    report=json.loads((output/'report.json').read_bytes())
    assert report['status']=='FAILED_OR_PARTIAL_NOT_PROMOTED' and report['error_type']=='RuntimeError'
    assert report['completed_head_fit_count']==0 and report['postcheck_errors']==[]
    names=[]
    for line in (output/'SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ');assert base.digest(output/name)==value;names.append(name)
    assert set(names)=={'plan.json','report.json'}
