"""HH_260906 - Test frozen TRAIN-only teacher diagnosis with CPU tensors and synthetic metadata, never real inference."""

import ast
from copy import deepcopy
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest
import torch

from portable_e2e import losses
from scripts.e2e import diagnose_stopmix_training_oracles as module
from scripts.e2e import profile_portable_training as base


def fixture(dtype=torch.float32):
    xy=torch.zeros(2,12,64,2,dtype=dtype)
    xy[:,:6,:,0]=torch.arange(6,dtype=dtype)[None,:,None]*.1
    speed=torch.zeros(2,12,64,dtype=dtype)
    logits=torch.arange(12,dtype=dtype).expand(2,-1)
    target=torch.zeros(2,64,2,dtype=dtype);target_speed=torch.zeros(2,64,dtype=dtype)
    valid=torch.ones(2,64,dtype=torch.bool);yaw=torch.full((2,64),.2,dtype=dtype)
    return [xy,speed,logits,target,target_speed,valid,yaw]


def projection():return module.projected_costs(losses)[0]


@pytest.mark.parametrize('dtype',[torch.float32,torch.float64])
def test_exact_prefix_and_native_oracle_regression_with_all64_stop_ties(dtype):
    values=fixture(dtype);projected,proof=module.projected_costs(losses)
    original=losses.trajectory_loss
    rows=module.candidate_rows(*values,losses.TrajectoryLossConfig(),losses,projected)
    assert losses.trajectory_loss is original
    assert proof['original_loss_sha256']==module.LOSS_SHA and len(proof['derived_ast_sha256'])==64
    assert rows[0]['exact_minimum_indices']==[0,6,7,8,9,10,11]
    assert rows[0]['composite_oracle_index']==0 and rows[0]['selected_candidate_index']==11
    assert rows[0]['selected_in_exact_minimum'] and rows[0]['second_minus_first_composite_cost']==0
    assert all(r['native_oracle_and_regression_exact'] for r in rows)


@pytest.mark.parametrize('coefficient',['xy_weight','speed_weight','yaw_weight','kinematic_speed_weight','final_displacement_weight'])
def test_each_original_composite_coefficient_matches_known_numeric_fixture(coefficient):
    xy,speed,logits,target,target_speed,valid,yaw=fixture(torch.float64)
    speed[:]=.3;target_speed[:]=.1
    fields={name:0. for name in losses.TrajectoryLossConfig().to_dict()};fields['xy_weight']=1.;fields[coefficient]=1.
    cfg=losses.TrajectoryLossConfig(**fields)
    costs=projection()(xy,speed,logits,target,target_speed,valid,cfg,target_yaw=yaw)
    c=.3
    expected={'xy_weight':.5*c*c,'speed_weight':.5*.2*.2,'yaw_weight':.2,
        'kinematic_speed_weight':((2.7-.5)+63*.5*.3*.3)/64,'final_displacement_weight':c}[coefficient]
    if coefficient!='xy_weight':expected+=.5*c*c
    assert float(costs[0,3])==pytest.approx(expected,rel=1e-14,abs=1e-14)


def test_masked_target_changes_do_not_change_costs_and_partial_masks_stay_visible():
    values=fixture();values[5][:,10:]=False
    projected=projection();cfg=losses.TrajectoryLossConfig()
    a=module.candidate_rows(*values,cfg,losses,projected)
    changed=[v.clone() for v in values];changed[3][:,10:]=999.;changed[4][:,10:]=99.;changed[6][:,10:]=2.
    b=module.candidate_rows(*changed,cfg,losses,projected)
    assert a==b and sum(a[0]['original_target_valid'])==10
    assert module.motion_group(0.,[0.]*64,a[0]['original_target_valid'])=='unavailable_masks'
    assert module.motion_group(0.,[0.]*10+[None]*54,a[0]['original_target_valid'])=='unavailable_masks'


def test_original_yaw_cannot_be_omitted():
    values=fixture();values[-1]=None
    with pytest.raises(ValueError,match='yaw'):module.candidate_rows(*values,losses.TrajectoryLossConfig(),losses,projection())


def test_no_near_tie_tolerance_and_small_positive_cost_preserved():
    values=fixture(torch.float64);values[0][:,6,0,0]=1e-12
    rows=module.candidate_rows(*values,losses.TrajectoryLossConfig(),losses,projection())
    assert 6 not in rows[0]['exact_minimum_indices'] and rows[0]['candidate_composite_costs'][6]>0


def test_mismatched_projected_oracle_is_rejected():
    projected=projection()
    def corrupt(*args,**kwargs):
        costs=projected(*args,**kwargs);costs[:,0]=-1.;return costs
    with pytest.raises(ValueError,match='differs'):module.candidate_rows(*fixture(),losses.TrajectoryLossConfig(),losses,corrupt)


@pytest.mark.parametrize('fault',['source','boundary','duplicate','decorator'])
def test_source_and_exact_ast_boundary_drift_rejected(monkeypatch,fault):
    raw=Path(losses.__file__).read_bytes()
    if fault=='source':raw+=b'\n'
    else:
        if fault=='boundary':raw=raw.replace(b'per_candidate.detach().argmin(dim=1)',b'per_candidate.detach().argmin(dim=0)')
        elif fault=='duplicate':raw=raw.replace(b'    oracle_candidate =',b'    oracle_candidate = None\n    oracle_candidate =',1)
        else:raw=raw.replace(b'def trajectory_loss(',b'@staticmethod\ndef trajectory_loss(')
        monkeypatch.setattr(module,'LOSS_SHA',module.sha(raw))
    with pytest.raises(ValueError):module.projection_definition(raw)


@pytest.mark.parametrize('index',[0,1,2,3,4,6])
def test_nonfinite_tensors_rejected(index):
    values=fixture();values[index]=values[index].contiguous().clone();values[index].reshape(-1)[0]=float('nan')
    with pytest.raises((ValueError,RuntimeError,FloatingPointError)):module.candidate_rows(*values,losses.TrajectoryLossConfig(),losses,projection())


@pytest.mark.parametrize('count',[6,11,13])
def test_only_full_twelve_candidate_axis_allowed(count):
    values=fixture();values[0]=torch.zeros(2,count,64,2)
    with pytest.raises(ValueError):module.candidate_rows(*values,losses.TrajectoryLossConfig(),losses,projection())


def test_group_boundaries_raw_nonpositive_separate_from_future_group():
    valid=[True]*64
    assert module.motion_group(-.1,[.1]*64,valid)=='stationary_hold'
    assert module.motion_group(.1,[.1]*64,valid)=='stationary_hold'
    assert module.motion_group(.1000001,[1.]*54+[.1]*10,valid)=='moving_to_stop'
    assert module.motion_group(0.,[.5]*64,valid)=='continuing_motion'
    assert module.motion_group(0.,[0.]*30+[1.]*34,valid)=='other_motion'
    assert module.motion_group(-.100001,[.1]*64,valid)=='other_motion'
    with pytest.raises(ValueError):module.motion_group(float('nan'),[0.]*64,valid)


def annotated_rows():
    rows=module.candidate_rows(*fixture(),losses.TrajectoryLossConfig(),losses,projection())
    for i,row in enumerate(rows):row.update(sample_id=str(i),episode_id='e',sequence_index=i,anchor_timestamp_ns=i,
        model_input_sha256='a'*64,target_sha256='b'*64,raw_current_vx_mps=0.,target_motion_group='stationary_hold',capture_phase='stationary_warmup')
    return rows


def test_summary_keeps_exact_ties_and_empty_group_denominators():
    report=module.summarize_rows(annotated_rows())
    assert report['all_samples']['sample_count']==report['all_samples']['multiple_exact_minimum_count']==2
    assert report['raw_nonpositive']['sample_count']==2
    assert report['target_motion_group']['moving_to_stop']['sample_count']==0
    assert report['target_motion_group']['moving_to_stop']['means']['selected_ade_m'] is None


def test_teacher_changes_require_every_fixed_input_and_target_identity():
    template=annotated_rows()[0];a=[]
    for i in range(1147):a.append(dict(deepcopy(template),sample_id=str(i),sequence_index=i))
    b=deepcopy(a);b[30]['composite_oracle_index']=6
    result=module.compare_teachers(a,b)
    assert result['oracle_index_changed_indices']==[30] and result['exact_minimum_sets_disjoint_count']==0
    b[30]['target_sha256']='c'*64
    with pytest.raises(ValueError):module.compare_teachers(a,b)


def test_input_hash_does_not_include_any_future_label():
    batch={name:torch.ones(1,2) for name in module.INPUT_KEYS}
    a=module.tensor_sha(batch,0,module.INPUT_KEYS);batch['target_xy']=torch.zeros(1,64,2)
    assert module.tensor_sha(batch,0,module.INPUT_KEYS)==a
    batch['ego_history'][0,0]=2.
    assert module.tensor_sha(batch,0,module.INPUT_KEYS)!=a


def make_plan():
    _,proof=module.projection_definition(Path(losses.__file__).read_bytes())
    return module.plan_contract(base,proof)|dict(declared_at_utc='2026-09-08T19:50:00Z',
        source_sha256={n:'a'*64 for n in module.source_paths(base)},worker_source_sha256='b'*64),proof


class FrozenTime(datetime):
    @classmethod
    def now(cls,tz=None):return cls(2026,9,8,20,0,tzinfo=timezone.utc)


def test_exact_six_train_only_fixed_plan(monkeypatch):
    plan,proof=make_plan();monkeypatch.setattr(module,'datetime',FrozenTime);module.validate_plan(plan,base,proof)
    assert plan['expected_pass_count']==6 and plan['num_workers']==0 and plan['expected_samples_per_pass']==1147
    assert plan['checkpoint_sha256']==module.CHECKPOINTS and len(module.source_paths(base))==15
    assert all(plan[k] is False for k in module.DENIALS)


@pytest.mark.parametrize('field,value',[('batch_size',8),('evaluation_split','val'),('num_workers',2),('near_tie_tolerance_used',True),
    ('model_training',True),('expected_samples_per_pass',337),('expected_pass_count',3),('source_commit','f'*40)])
def test_plan_rejects_scope_change(monkeypatch,field,value):
    plan,proof=make_plan();monkeypatch.setattr(module,'datetime',FrozenTime);plan[field]=value
    with pytest.raises(ValueError):module.validate_plan(plan,base,proof)


def test_plan_exact_checkpoint_and_projection_pins(monkeypatch):
    plan,proof=make_plan();monkeypatch.setattr(module,'datetime',FrozenTime)
    for key in ('checkpoint_sha256','loss_projection','original_receipt_sha256'):
        changed=deepcopy(plan);changed[key]={}
        with pytest.raises(ValueError):module.validate_plan(changed,base,proof)


def test_per_pass_alarm_restores_whole_remaining_deadline():
    old=signal.getsignal(signal.SIGALRM)
    with base.bounded_signals(300):
        with module.inference_alarm():
            assert 0<signal.getitimer(signal.ITIMER_REAL)[0]<=120
        assert 120<signal.getitimer(signal.ITIMER_REAL)[0]<=300
    assert signal.getsignal(signal.SIGALRM)==old


def test_inference_alarm_requires_supervisor_budget():
    assert signal.getitimer(signal.ITIMER_REAL)[0]==0
    with pytest.raises(ValueError):
        with module.inference_alarm():pass


def test_cli_abbreviations_rejected_before_any_helper_import(monkeypatch):
    monkeypatch.setattr(module,'load_helper',lambda:pytest.fail('no helper'))
    with pytest.raises(SystemExit):module.main(['--pla','bad'])


def test_source_has_no_training_optimizer_call_or_future_model_input():
    tree=ast.parse(Path(module.__file__).read_text())
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):assert node.func.attr not in ('train_model','backward','step')
    assert module.INPUT_KEYS==('images','calibration','ego_history','ego_history_mask','route_xy','route_mask')


@pytest.fixture
def cpu_pass(tmp_path,monkeypatch):
    # HH_260906 - Fake CUDA transfers are intercepted before execution; tensors, model and synthetic loader remain exclusively CPU.
    original_to=torch.Tensor.to
    def cpu_to(self,*args,**kwargs):
        if args and args[0]=='cuda:0':return self
        return original_to(self,*args,**kwargs)
    monkeypatch.setattr(torch.Tensor,'to',cpu_to)
    examples=[SimpleNamespace(token=f'synthetic-{i}',episode_id='e',sequence_index=i,anchor_timestamp_ns=i,
        features=(1.,0.)+(0.,)*11,target_speed_mps=(0.,)*64,
        camera_sha256=('a'*64,)*6,source_manifest_sha256='b'*64) for i in range(1147)]
    dataset=SimpleNamespace(examples=examples)
    def batches(*args,**kwargs):
        assert kwargs==dict(batch_size=4,shuffle=False,num_workers=0,pin_memory=False)
        for start in range(0,1147,4):
            count=min(4,1147-start)
            batch={k:torch.zeros(count,2) for k in module.INPUT_KEYS}
            batch['ego_history']=torch.zeros(count,2,13)
            batch.update(sample_id=[e.token for e in examples[start:start+count]],target_xy=torch.zeros(count,64,2),
                target_speed_mps=torch.zeros(count,64),target_yaw_rad=torch.zeros(count,64),target_valid=torch.ones(count,64,dtype=torch.bool))
            yield batch
    monkeypatch.setattr(torch.utils.data,'DataLoader',batches)
    calls=[]
    def model(*args):
        assert len(args)==6 and all(t.device.type=='cpu' for t in args)
        calls.append(len(args[0]));n=len(args[0])
        return torch.zeros(n,12,64,2),torch.zeros(n,12,64),torch.zeros(n,12)
    return dataset,model,calls,{'e':{'stationary_warmup':[0,34],'driving':[35,1146]}},tmp_path/'rows.jsonl'


def test_complete_cpu_fixture_preserves1147_rows_last3_and_native_phase_identity(cpu_pass):
    dataset,model,calls,phases,path=cpu_pass
    with base.bounded_signals(300):
        rows=module.analyze_pass(dataset,model,losses,projection(),losses.TrajectoryLossConfig(),phases,path,torch)
    assert len(rows)==1147 and calls==[4]*286+[3]
    assert sum(r['capture_phase']=='stationary_warmup' for r in rows)==35
    assert all(r['raw_current_vx_nonpositive'] and r['exact_minimum_count']==12 for r in rows)
    assert len(path.read_text().splitlines())==1147 and not torch.cuda.is_initialized()


def test_midpass_failure_retains_every_completed_row_and_restores_alarm(cpu_pass):
    dataset,model,calls,phases,path=cpu_pass
    def fail(*args):
        if calls:raise RuntimeError('synthetic second batch failure')
        return model(*args)
    with base.bounded_signals(300):
        with pytest.raises(RuntimeError):module.analyze_pass(dataset,fail,losses,projection(),losses.TrajectoryLossConfig(),phases,path,torch)
        assert signal.getitimer(signal.ITIMER_REAL)[0]>120
    assert len(path.read_text().splitlines())==4


def test_groups_keep_source_double_not_rounded_model_tensor(cpu_pass):
    dataset,model,calls,phases,path=cpu_pass
    for example in dataset.examples:
        example.features=(1.,.099999999)+(0.,)*11
        example.target_speed_mps=(.099999999,)*64
    assert float(torch.tensor(.099999999))>.1
    with base.bounded_signals(300):
        rows=module.analyze_pass(dataset,model,losses,projection(),losses.TrajectoryLossConfig(),phases,path,torch)
    assert rows[0]['raw_current_vx_mps']==.099999999 and rows[0]['target_motion_group']=='stationary_hold'
    assert rows[0]['model_tensor_current_vx_mps']==0. and rows[0]['raw_target_speed_mps']==[.099999999]*64


def test_parent_input_receipt_tamper_rejects_before_checkpoint_decode(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'WORKSPACE',tmp_path)
    values={name:'f'*64 for name in module.RECEIPTS}
    fake=SimpleNamespace(digest=lambda path:values[str(path.relative_to(tmp_path))])
    with pytest.raises(ValueError,match='receipts'):module.input_receipts(fake,{})


def test_missing_exact_helper_never_imports_untrusted_bytes(tmp_path,monkeypatch):
    path=tmp_path/'helper.py';path.write_text('raise AssertionError("must not execute")')
    monkeypatch.setattr(module,'HELPER_PATH',path)
    with pytest.raises(ValueError,match='mismatch'):module.load_helper()


def test_late_mutation_of_completed_rows_invalidates_completion(tmp_path):
    path=tmp_path/'samples.jsonl';path.write_text('{}\n')
    passes=[dict(status='PASS_COMPLETE_NOT_PROMOTED',samples_file='samples.jsonl',samples_sha256=base.digest(path))]
    assert module.completed_rows_intact(tmp_path,passes,base)
    path.write_text('{"changed":true}\n')
    assert not module.completed_rows_intact(tmp_path,passes,base)
