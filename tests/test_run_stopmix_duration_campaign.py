"""HH_260906 - Test fixed-duration private ownership with synthetic files and child processes, never GPU training."""

from contextlib import nullcontext
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

PATH = Path(__file__).resolve().parents[1]/'scripts/e2e/run_stopmix_duration_campaign.py'
SPEC = importlib.util.spec_from_file_location('private_stopmix_duration',PATH)
module = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(module)
PARENT_BEHAVIOR_PROOF = module.parent_behavior_proof


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,sort_keys=True)+'\n')


def metric(step):
    expected=module.training_state(step)
    return {key:expected[key] for key in ('global_step','epoch','samples_seen','domain_samples_seen')} | {
        'batch_domain_sample_counts':{'carla':3 if step%287==0 else 4},'loss':1.,'regression_loss':.9,
        'candidate_score_loss':1.,'selected_ade_m':1.,'selected_fde_m':2.,'selected_speed_mae_mps':.2,'gradient_norm':1.}


def history(steps):
    return ''.join(json.dumps(metric(i),sort_keys=True)+'\n'for i in range(1,steps+1)).encode()


def test_fixed_budget_and_resume_cursor_are_exact():
    end=datetime(2026,9,9,1,tzinfo=timezone.utc)
    assert module.remaining_budget(0,end-timedelta(seconds=6120))==6120
    assert module.remaining_budget(24,end-timedelta(seconds=900))==900
    with pytest.raises(RuntimeError,match='budget'):module.remaining_budget(0,end-timedelta(seconds=6119.9))
    for invalid in (-1,25,True):
        with pytest.raises(RuntimeError):module.remaining_budget(invalid,end-timedelta(hours=3))
    assert module.training_state(1540)=={'epoch':5,'next_batch_index':105,'global_step':1540,'samples_seen':6155,'domain_samples_seen':{'carla':6155}}
    assert module.training_state(2870)['epoch']==9 and module.training_state(2870)['next_batch_index']==287
    assert module.training_state(2870,final_report=True)=={'epoch':10,'next_batch_index':0,'global_step':2870,'samples_seen':11470,'domain_samples_seen':{'carla':11470}}


@pytest.mark.parametrize('field,value',[('total_steps',3000),('total_samples_seen',11471),('checkpoint_interval',287),
    ('num_workers',1),('num_workers',False),('automatic_retry',True),('automatic_promotion',True),
    ('test_nn_inference_or_training_or_selection',True),('source_commit','f'*40),('runs',list(module.RUNS)[:-1]),
    ('stage_timeout_seconds',{'train':450,'evaluate':120,'audit':120,'behavior':179})])
def test_plan_rejects_unapproved_changes(field,value):
    plan=module.plan_contract('a'*64)|{'declared_at_utc':'2026-09-08T19:00:00Z'}
    plan=deepcopy(plan);plan[field]=value
    with pytest.raises(RuntimeError,match='settings'):module.validate_plan(plan,'a'*64,datetime(2026,9,8,20,tzinfo=timezone.utc))


def test_plan_exact_keys_parent_hashes_and_timestamp():
    plan=module.plan_contract('a'*64)|{'declared_at_utc':'2026-09-08T19:00:00Z'}
    module.validate_plan(plan,'a'*64,datetime(2026,9,8,20,tzinfo=timezone.utc))
    assert len(plan['parent_training_file_sha256'])==6
    assert sum(len(v)for v in plan['parent_training_file_sha256'].values())==18
    for mutation in ('future','extra','hash'):
        changed=deepcopy(plan)
        if mutation=='future':changed['declared_at_utc']='2026-09-08T21:00:00Z'
        elif mutation=='extra':changed['override']='unsafe'
        else:changed['parent_training_file_sha256'][module.RUNS[0]]['run.json']='b'*64
        with pytest.raises(RuntimeError):module.validate_plan(changed,'a'*64,datetime(2026,9,8,20,tzinfo=timezone.utc))


@pytest.fixture
def workflow(tmp_path,monkeypatch):
    workspace=tmp_path/'personal/hwanhong/portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    parent=workspace/'runs/campaigns'/module.PARENT_ID;parent.mkdir(parents=True)
    lease=parent.parent/'.gpu0_training.lock';lease.touch()
    write(parent/'plan.json',{'dataset':'datasets/prepared/fixed'})
    write(parent/'status.json',{'status':'synthetic parent complete'})
    owner_file=workspace/'runs/diagnostics/stopmix_behavior_v1/provenance/stdin_worker.py'
    owner_file.parent.mkdir(parents=True);owner_file.write_bytes(b'fixed synthetic helper; not executed')
    monkeypatch.setattr(module,'WORKSPACE',workspace)
    monkeypatch.setattr(module,'OWNER_SHA',module.digest(owner_file))
    monkeypatch.setattr(module,'PARENT_PLAN_SHA',module.digest(parent/'plan.json'))
    monkeypatch.setattr(module,'PARENT_STATUS_SHA',module.digest(parent/'status.json'))
    monkeypatch.setattr(module.sys,'prefix',str(workspace/'venvs/py312'))
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES',module.GPU_UUID);monkeypatch.setenv('PYTHONNOUSERSITE','1')
    for key in ('PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE','PYTHONPATH'):monkeypatch.delenv(key,raising=False)
    monkeypatch.setattr(module,'now',lambda:datetime(2026,9,8,20,tzinfo=timezone.utc))
    source=b'synthetic duration worker; never executed'
    monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SOURCE',source,raising=False)
    worker_sha=hashlib.sha256(source).hexdigest();monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SHA256',worker_sha,raising=False)
    parent_pins={};models={}
    for run in module.RUNS:
        seed=int(run.split('/')[0][5:]);mixed=run.endswith(module.ARMS[1]);k=12 if mixed else 6
        model_id='portable_e2e.perspective_trajectory.physical_stopmix.v1'if mixed else 'portable_e2e.perspective_trajectory.physical.v1'
        models[run]=model_id
        report={'status':'TRAINING_TARGET_REACHED','completed_at_utc':'2026-09-08T18:30:00Z',
            'created_at_utc':'2026-09-08T18:00:00Z','train_config':module.train_config(seed,1540),
            'loss_config':deepcopy(module.LOSS),'state':module.training_state(1540),'last_metrics':metric(1540),
            'model_config':{'model_id':model_id,'candidate_count':k},'model_parameter_count':1056362 if mixed else 954590,
            'trainer_id':'portable_e2e.pytorch_trainer.v1','corpus_fingerprint_sha256':'c'*64,
            'dataset_fingerprint_sha256':'d'*64,'dataset_size':1147,'training_split':'train','training_episode_ids':['train_a','train_b','train_c'],
            'runtime':{'synthetic':'same'},'hardware':{'device_uuid':module.GPU_UUID},'device':'cuda:0',
            'sampling_plan':{'batch_size':4},'sampling_plan_sha256':'a'*64}
        write(parent/run/'training/run.json',report)
        (parent/run/'training/metrics.jsonl').write_bytes(history(1540))
        checkpoint=parent/run/'training/checkpoints/latest.pt';checkpoint.parent.mkdir();checkpoint.write_bytes((run+'synthetic checkpoint').encode())
        parent_pins[run]={name:module.digest(parent/run/'training'/name)for name in ('run.json','metrics.jsonl','checkpoints/latest.pt')}
        common={'corpus_fingerprint_sha256':'c'*64,'dataset_fingerprint_sha256':'e'*64,
            'training_dataset_fingerprint_sha256':'d'*64,'training_episode_ids':report['training_episode_ids'],
            'evaluation_episode_ids':['val'],'model_config_sha256':'f'*64,'model_parameter_count':report['model_parameter_count'],
            'evaluation_split':'val','vehicle_control_approved':False,'checkpoint_sha256':module.digest(checkpoint)}
        write(parent/run/'evaluation/metrics.json',common|{'status':'OPEN_LOOP_EVALUATION_COMPLETE','sample_count':337})
        write(parent/run/'gate_v8.json',common|{'status':'RUNTIME_GEOMETRY_AUDIT_COMPLETE','geometry':{'sample_count':337},
            'gate':{'source':'portable_e2e.runtime_geometry_gate.v8','threshold_overrides':False,'thresholds':{'candidate_count':k}}})
    monkeypatch.setattr(module,'PARENT_FILES',parent_pins)
    snapshot={'source_sha256':{'fixture':'a'*64},'normal_input_sha256':{},'checkpoint_sha256':{r:p['checkpoints/latest.pt']for r,p in parent_pins.items()}}
    calls=[]
    runner=SimpleNamespace(termination_guard=lambda:nullcontext(),assert_gpu_idle=lambda _:None,
        stage_environment=lambda:{'CUDA_VISIBLE_DEVICES':module.GPU_UUID})
    def base_commands(plan,campaign,actual_repo):
        for run in module.RUNS:
            item=campaign/run;checkpoint=item/'training/checkpoints/latest.pt'
            for stage in ('train','evaluate','audit'):
                command=['python','-m','portable_e2e.'+('audit_runtime'if stage=='audit'else stage)]
                if stage=='train':command+=['--run-dir',str(item/'training'),'--max-steps','1540','--checkpoint-interval','154']
                elif stage=='evaluate':command+=['--checkpoint',str(checkpoint),'--output-dir',str(item/'evaluation')]
                else:command+=['--checkpoint',str(checkpoint),'--output-json',str(item/'gate_v8.json')]
                yield item,checkpoint,command,stage
    runner.commands=base_commands
    def behavior_command(run,model_id,checksum,output,dataset,campaign):
        return ['python','-m','scripts.e2e.audit_portable_stopmix_behavior','--checkpoint',str(campaign/run/'training/checkpoints/latest.pt'),
                '--checkpoint-sha256',checksum,'--output-dir',str(output/run)]
    def verify_behavior(item,model_id,checksum,sources):
        report=module.read(item/'summary.json')
        assert report=={'status':'fixture behavior complete','checkpoint_sha256':checksum,'count':337}
        return {'summary_sha256':module.digest(item/'summary.json')}
    owner=SimpleNamespace(RUNS=tuple(models.items()),load_runner=lambda _:runner,normal_snapshot=lambda *_:deepcopy(snapshot),
        ignore_cleanup_signals=lambda:nullcontext(),command_for=behavior_command,verify_result=verify_behavior)
    monkeypatch.setattr(module,'load_owner',lambda:owner)
    monkeypatch.setattr(module,'parent_behavior_proof',lambda *_:{'finished_at_utc':'2026-09-08T18:40:00Z','fixture':'six complete passes'})
    def child(command,actual_repo,env,log,timeout):
        calls.append((list(command),timeout))
        if '--run-dir'in command:
            training=Path(command[command.index('--run-dir')+1]);item=training.parent
            report=module.read(training/'run.json');report['train_config']['max_steps']=2870
            report.update(state=module.training_state(2870,final_report=True),last_metrics=metric(2870),resumed_at_utc='2026-09-08T20:00:00Z')
            write(training/'run.json',report);(training/'metrics.jsonl').write_bytes(history(2870))
            (training/'checkpoints/latest.pt').write_bytes(b'synthetic new checkpoint:'+str(item).encode())
        else:
            checkpoint=Path(command[command.index('--checkpoint')+1]);item=checkpoint.parents[2]
            run=str(item.relative_to(workspace/'runs/campaigns'/module.CAMPAIGN_ID))
            if 'scripts.e2e.audit_portable_stopmix_behavior'in command:
                output=Path(command[command.index('--output-dir')+1])
                write(output/'summary.json',{'status':'fixture behavior complete','checkpoint_sha256':module.digest(checkpoint),'count':337})
            else:
                stage='audit'if '--output-json'in command else 'evaluate'
                name='gate_v8.json'if stage=='audit'else 'evaluation/metrics.json'
                report=module.read(parent/run/name);report['checkpoint_sha256']=module.digest(checkpoint);write(item/name,report)
        return 0
    runner.run_owned_stage=child
    plan=module.plan_contract(worker_sha)|{'declared_at_utc':'2026-09-08T19:00:00Z'}
    plan_path=workspace/'runs/plans/duration.json';write(plan_path,plan)
    args=['--plan',str(plan_path),'--expected-plan-sha256',module.digest(plan_path)]
    originals={str(p.relative_to(parent)):module.digest(p)for p in parent.rglob('*')if p.is_file()}
    return SimpleNamespace(workspace=workspace,parent=parent,runner=runner,owner=owner,calls=calls,args=args,
        plan=plan_path,output=workspace/'runs/campaigns'/module.CAMPAIGN_ID,originals=originals)


def test_complete24_fake_stages_preserve_all_parent_bytes_and_history_prefix(workflow):
    f=workflow
    assert module.main(f.args)==0
    state=module.read(f.output/'status.json')
    assert state['status']=='COMPLETE_NOT_PROMOTED'and state['completed_stage_count']==24
    assert [(r['run'],r['stage'])for r in state['stages']]==list(module.ORDER)
    assert [timeout for _,timeout in f.calls]==[module.TIMEOUTS[s]for _,s in module.ORDER]
    assert {str(p.relative_to(f.parent)):module.digest(p)for p in f.parent.rglob('*')if p.is_file()}==f.originals
    assert state['parent_source_and_output_postcheck_pass']is True
    for run in module.RUNS:
        assert state['copies'][run]==module.PARENT_FILES[run]
        train=next(r for r in state['stages']if r['run']==run and r['stage']=='train')
        assert train['command'][-1]=='--resume'
        assert train['command'][train['command'].index('--max-steps')+1]=='2870'
        assert train['report']['history']['rows']==2870
    for line in (f.output/'WORKFLOW_SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ',1);assert module.digest(f.output/name)==value


@pytest.mark.parametrize('fault',['exit','timeout','signal','parent_mutation','prefix_mutation','setting_mutation','worker_archive'])
def test_child_failure_stops_without_retry_and_retains_evidence(workflow,fault):
    f=workflow;original=f.runner.run_owned_stage;count=[]
    def child(*args,**kwargs):
        count.append(1)
        if fault=='timeout':raise TimeoutError('fixture timeout; underlying scoped cleanup tested separately')
        if fault=='signal':raise KeyboardInterrupt('fixture signal')
        result=original(*args,**kwargs)
        if fault=='exit':return 1
        training=f.output/module.RUNS[0]/'training'
        if fault=='parent_mutation':(f.parent/module.RUNS[0]/'training/metrics.jsonl').write_bytes(b'bad')
        elif fault=='prefix_mutation':
            path=training/'metrics.jsonl';path.write_bytes(b' '+path.read_bytes())
        elif fault=='setting_mutation':
            path=training/'run.json';report=module.read(path);report['train_config']['checkpoint_interval']=287;write(path,report)
        elif fault=='worker_archive':(f.output/'provenance/stdin_worker.py').write_bytes(b'bad')
        return result
    f.runner.run_owned_stage=child
    assert module.main(f.args)==1 and len(count)==1
    state=module.read(f.output/'status.json')
    assert state['status']=='STOPPED_FAILURE_NO_PROMOTION'and state['completed_stage_count']==0
    assert state['stages'][0]['status']=='FAILED'
    assert (f.output/'WORKFLOW_SHA256SUMS').is_file()


@pytest.mark.parametrize('fault',['environment','gpu','bootstrap','plan_sha','deadline','lease','parent_alias','helper'])
def test_preflight_fails_before_training(workflow,monkeypatch,fault):
    f=workflow
    if fault=='environment':monkeypatch.setenv('PYTHONPATH','/unexpected')
    elif fault=='gpu':monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0')
    elif fault=='bootstrap':monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SHA256','f'*64)
    elif fault=='plan_sha':f.args[-1]='f'*64
    elif fault=='deadline':monkeypatch.setattr(module,'now',lambda:datetime(2026,9,9,1,tzinfo=timezone.utc))
    elif fault=='lease':
        def held(*_):raise BlockingIOError('foreign lease held')
        monkeypatch.setattr(module.fcntl,'flock',held)
    elif fault=='parent_alias':f.output.symlink_to(f.parent,target_is_directory=True)
    else:monkeypatch.setattr(module,'load_owner',lambda:(_ for _ in ()).throw(RuntimeError('helper pin changed')))
    with pytest.raises((RuntimeError,BlockingIOError)):module.main(f.args)
    assert not f.calls


def test_copy_uses_real_distinct_files_and_rejects_parent_path(workflow):
    f=workflow;run=module.RUNS[0]
    with pytest.raises(RuntimeError,match='alias'):module.copy_training(f.parent,f.parent/'nested',run)
    f.output.mkdir();module.copy_training(f.parent,f.output,run)
    for name in module.PARENT_FILES[run]:
        a=f.parent/run/'training'/name;b=f.output/run/'training'/name
        assert a.read_bytes()==b.read_bytes()and a.stat().st_ino!=b.stat().st_ino and b.stat().st_nlink==1
    with pytest.raises(RuntimeError,match='namespace'):module.copy_training(f.parent,f.output,run)


@pytest.mark.parametrize('mutation',['nonfinite','count','cursor','original_prefix'])
def test_history_proof_rejects_invalid_appended_stream(tmp_path,mutation):
    parent=tmp_path/'parent';parent.mkdir();new=tmp_path/'new';new.mkdir()
    (parent/'metrics.jsonl').write_bytes(history(1540));rows=[metric(i)for i in range(1,2871)]
    if mutation=='nonfinite':rows[-1]['loss']=float('nan')
    elif mutation=='count':rows.pop()
    elif mutation=='cursor':rows[-1]['samples_seen']=11471
    else:rows[0]['loss']=2.
    (new/'metrics.jsonl').write_text(''.join(json.dumps(r,sort_keys=True)+'\n'for r in rows))
    with pytest.raises(RuntimeError):module.history_proof(parent,new,2870)


@pytest.mark.parametrize('fault',[None,'missing_pass','failed_pass','wrong_exit','different_normal',
    'payload','source','status_sha'])
def test_all_six_parent_behavior_passes_are_a_strict_launch_prerequisite(tmp_path,monkeypatch,fault):
    workspace=tmp_path/'personal/hwanhong/portable_e2e';root=workspace/'runs/diagnostics/stopmix_behavior_v1'
    root.mkdir(parents=True);monkeypatch.setattr(module,'WORKSPACE',workspace)
    normal={'source_sha256':{'fixture':'a'*64},'normal_input_sha256':{'fixture':'b'*64}}
    model_ids=tuple((run,'synthetic_model')for run in module.RUNS)
    proofs={run:{'summary_sha256':hashlib.sha256(run.encode()).hexdigest()}for run in module.RUNS}
    state={'status':'COMPLETE_NOT_PROMOTED','completed_behavior_count':6,'source_commit':module.SOURCE_COMMIT,
        'worker_source_sha256':module.OWNER_SHA,'source_and_normal_inputs_unchanged':True,'vehicle_control_approved':False,
        'before':deepcopy(normal),'after':deepcopy(normal),'finished_at_utc':'2026-09-08T18:40:00Z',
        'stages':[{'run':run,'status':'COMPLETE','returncode':0,'report':proofs[run]}for run in module.RUNS]}
    if fault=='missing_pass':state['stages'].pop()
    elif fault=='failed_pass':state['stages'][0]['status']='FAILED'
    elif fault=='wrong_exit':state['stages'][0]['returncode']=False
    elif fault=='different_normal':state['after']['source_sha256']={'changed':'f'*64}
    elif fault=='source':state['source_commit']='f'*40
    write(root/'workflow_status.json',state);(root/'WORKFLOW_SHA256SUMS').write_bytes(b'pinned fixture manifest')
    monkeypatch.setattr(module,'PARENT_BEHAVIOR_STATUS_SHA','f'*64 if fault=='status_sha'else module.digest(root/'workflow_status.json'))
    monkeypatch.setattr(module,'PARENT_BEHAVIOR_MANIFEST_SHA',module.digest(root/'WORKFLOW_SHA256SUMS'))
    def verify(item,model_id,checksum,sources):
        run=str(item.relative_to(root));assert checksum==module.PARENT_FILES[run]['checkpoints/latest.pt']
        return {'changed':'x'}if fault=='payload'else deepcopy(proofs[run])
    owner=SimpleNamespace(RUNS=model_ids,verify_result=verify)
    if fault:
        with pytest.raises(RuntimeError):PARENT_BEHAVIOR_PROOF(owner,normal)
    else:
        result=PARENT_BEHAVIOR_PROOF(owner,normal)
        assert list(result['reports'])==list(module.RUNS)


def test_original_completed_outputs_cannot_change_after_later_children(workflow):
    f=workflow;original=f.runner.run_owned_stage;calls=[]
    def child(*args,**kwargs):
        code=original(*args,**kwargs);calls.append(1)
        if len(calls)==24:
            path=f.output/module.RUNS[0]/'evaluation/metrics.json'
            report=module.read(path);report['sample_count']=336;write(path,report)
        return code
    f.runner.run_owned_stage=child
    assert module.main(f.args)==1
    state=module.read(f.output/'status.json')
    assert state['completed_stage_count']==24 and state['status']=='STOPPED_FAILURE_NO_PROMOTION'
    assert 'validation337'in state['final_integrity_error']


def test_nonzero_owned_child_uses_only_frozen_cleanup_implementation():
    # HH_260906 - The wrapper delegates child lifetime to the pinned existing owner, never a broad kill command.
    source=PATH.read_text()
    assert "runner.run_owned_stage(command,repo,runner.stage_environment(),log,timeout=TIMEOUTS[stage])"in source
    assert 'os.kill'not in source and 'pkill'not in source and 'killall'not in source
