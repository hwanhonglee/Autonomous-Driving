"""HH_260906 - Test the private stdin owner with synthetic processes and files only, never a GPU or live audit."""

from contextlib import nullcontext
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from test_summarize_portable_stopmix import stopmix, no_accel, expansion, campaign  # noqa: F401

PATH = Path(__file__).resolve().parents[1] / 'artifacts/training/2026-09-09/run_stopmix_behavior_campaign.py'
if not PATH.is_file():
    pytest.skip('private executed-owner source is not distributed in a normal clone', allow_module_level=True)
SPEC = importlib.util.spec_from_file_location('private_stopmix_behavior_owner', PATH)
module = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(module)


def test_deadline_reserves_all_six_slots_plus_cleanup():
    end=datetime(2026,9,9,1,tzinfo=timezone.utc)
    module.remaining_budget(6,end-timedelta(seconds=1380))
    with pytest.raises(RuntimeError,match='budget'):module.remaining_budget(6,end-timedelta(seconds=1379.99))
    module.remaining_budget(0,end-timedelta(seconds=300))
    for count in (-1,7,True):
        with pytest.raises(RuntimeError):module.remaining_budget(count,end-timedelta(hours=1))


def test_exact_six_commands_match_frozen_model_source_checkpoint():
    workspace=module.WORKSPACE;campaign=workspace/'runs/campaigns'/module.CAMPAIGN_ID
    for run,model_id in module.RUNS:
        command=module.command_for(run,model_id,'a'*64,workspace/'runs/diagnostics/stopmix_behavior_v1',workspace/'datasets/prepared/fixed',campaign)
        assert command[0]==str(workspace/'venvs/py312/bin/python')
        assert command[command.index('-m')+1]=='scripts.e2e.audit_portable_stopmix_behavior'
        assert command[command.index('--expected-source-commit')+1]=='b478f02e42b94bf04bffec5c8170e05edc33b0f8'
        assert command[command.index('--checkpoint-sha256')+1]=='a'*64
        assert command[command.index('--expected-model-id')+1]==model_id
        assert command[-4:]==['--device','cuda:0','--batch-size','4']
    assert len(module.RUNS)==6


@pytest.fixture
def owner(tmp_path,monkeypatch):
    workspace=tmp_path/'personal/hwanhong/portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    campaign=workspace/'runs/campaigns'/module.CAMPAIGN_ID;campaign.mkdir(parents=True)
    lease=workspace/'runs/campaigns/.gpu0_training.lock';lease.touch()
    (campaign/'plan.json').write_text(json.dumps({'dataset':'datasets/prepared/fixed'}))
    original_status=b'normal campaign remains byte-identical';(campaign/'status.json').write_bytes(original_status)
    monkeypatch.setattr(module,'WORKSPACE',workspace);monkeypatch.setattr(module.sys,'prefix',str(workspace/'venvs/py312'))
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES',module.GPU_UUID);monkeypatch.setenv('PYTHONNOUSERSITE','1')
    for key in ('PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE'):monkeypatch.delenv(key,raising=False)
    source=b'isolated fixture source, never executed';monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SOURCE',source,raising=False)
    monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SHA256',hashlib.sha256(source).hexdigest(),raising=False)
    snapshot={'source_sha256':{'fixture':'b'*64},'normal_input_sha256':{'runs/campaigns/'+module.CAMPAIGN_ID+'/status.json':hashlib.sha256(original_status).hexdigest()},
        'checkpoint_sha256':{run:'a'*64 for run,_ in module.RUNS},'dataset_manifest_sha256':'c'*64}
    events=[]
    runner=SimpleNamespace(termination_guard=lambda:nullcontext(),assert_gpu_idle=lambda _:events.append('idle'),
        stage_environment=lambda:{'CUDA_VISIBLE_DEVICES':module.GPU_UUID})
    def stage(command,_repo,_env,_log,timeout):
        events.append(('child',command,timeout));return 0
    runner.run_owned_stage=stage
    monkeypatch.setattr(module,'load_runner',lambda _:runner)
    monkeypatch.setattr(module,'normal_snapshot',lambda *_:deepcopy(snapshot))
    monkeypatch.setattr(module,'remaining_budget',lambda count:events.append(('budget',count)))
    monkeypatch.setattr(module,'verify_result',lambda *_:{'fixture':'result-only validation tested separately'})
    return workspace,campaign,original_status,snapshot,events,runner


def test_full_six_sequential_owned_stages_leave_normal_status_unchanged(owner):
    workspace,campaign,original,snapshot,events,runner=owner
    assert module.main()==0
    root=workspace/'runs/diagnostics/stopmix_behavior_v1';state=json.loads((root/'workflow_status.json').read_text())
    assert state['status']=='COMPLETE_NOT_PROMOTED' and state['completed_behavior_count']==6
    assert state['before']==state['after']==snapshot and state['source_and_normal_inputs_unchanged'] is True
    assert [r['run'] for r in state['stages']]==[r for r,_ in module.RUNS]
    calls=[event for event in events if isinstance(event,tuple) and event[0]=='child']
    assert len(calls)==6 and all(c[2]==180 for c in calls)
    assert [c for c in events if isinstance(c,tuple) and c[0]=='budget']==[('budget',6),('budget',6),
        *[('budget',i) for i in range(6,0,-1) for _ in range(2)],('budget',0)]
    assert (campaign/'status.json').read_bytes()==original
    assert (root/'provenance/stdin_worker.py').read_bytes()==module.VERIFIED_STDIN_WORKER_SOURCE
    for line in (root/'WORKFLOW_SHA256SUMS').read_text().splitlines():
        checksum,name=line.split('  ');assert name!='WORKFLOW_SHA256SUMS' and module.digest(root/name)==checksum


@pytest.mark.parametrize('fault',['child_exit','timeout','interrupt','post_input','post_source','post_idle'])
def test_failure_retains_partial_receipt_no_retry_or_normal_write(owner,monkeypatch,fault):
    workspace,campaign,original,snapshot,events,runner=owner
    count=[]
    def stage(*_,**__):
        count.append(1)
        if fault=='timeout':raise TimeoutError('synthetic stage timeout')
        if fault=='interrupt':raise KeyboardInterrupt('synthetic interruption')
        return 1 if fault=='child_exit' else 0
    runner.run_owned_stage=stage
    if fault in ('post_input','post_source'):
        def changed(*_):
            result=deepcopy(snapshot)
            if count:result['normal_input_sha256' if fault=='post_input' else 'source_sha256']={'changed':'f'*64}
            return result
        monkeypatch.setattr(module,'normal_snapshot',changed)
    elif fault=='post_idle':
        def idle(_):
            if count:raise RuntimeError('GPU not idle after owned stage')
        runner.assert_gpu_idle=idle
    assert module.main()==1 and len(count)==1
    state=json.loads((workspace/'runs/diagnostics/stopmix_behavior_v1/workflow_status.json').read_text())
    assert state['status']=='STOPPED_FAILURE_NO_PROMOTION' and state['completed_behavior_count']==0
    assert state['stages'][0]['status']=='FAILED' and (campaign/'status.json').read_bytes()==original


@pytest.mark.parametrize('fault',['bootstrap','venv','gpu','lease','existing'])
def test_preflight_scope_fails_before_child(owner,monkeypatch,fault):
    workspace,_,_,_,events,_=owner
    if fault=='bootstrap':monkeypatch.setattr(module,'VERIFIED_STDIN_WORKER_SHA256','f'*64)
    elif fault=='venv':monkeypatch.setattr(module.sys,'prefix','/not-approved')
    elif fault=='gpu':monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0')
    elif fault=='lease':
        def locked(*_):raise BlockingIOError('fixture lease held')
        monkeypatch.setattr(module.fcntl,'flock',locked)
    else:(workspace/'runs/diagnostics/stopmix_behavior_v1').mkdir(parents=True)
    with pytest.raises((RuntimeError,BlockingIOError)):module.main()
    assert not any(isinstance(e,tuple) and e[0]=='child' for e in events)


def test_completed_child_output_mutation_discovered_at_finalization(owner,monkeypatch):
    workspace,_,_,_,_,_=owner;calls=[]
    def verify(*_):
        calls.append(1)
        return {'sha256':'a'*64 if len(calls)<=6 else 'b'*64}
    monkeypatch.setattr(module,'verify_result',verify)
    assert module.main()==1
    state=json.loads((workspace/'runs/diagnostics/stopmix_behavior_v1/workflow_status.json').read_text())
    assert state['status']=='STOPPED_FAILURE_NO_PROMOTION' and state['completed_behavior_count']==6
    assert 'completed behavior output changed' in state['final_integrity_error']


def test_git_source_reads_cannot_fetch(monkeypatch,tmp_path):
    def run(command,**kwargs):
        assert command[:3]==['git','-c','protocol.allow=never'] and kwargs['timeout']==15
        assert kwargs['env']['GIT_NO_LAZY_FETCH']=='1' and kwargs['env']['GIT_ALLOW_PROTOCOL']==''
        assert kwargs['env']['GIT_TERMINAL_PROMPT']=='0'
        return SimpleNamespace(stdout=b'fixture')
    monkeypatch.setattr(module.subprocess,'run',run)
    assert module.git(tmp_path,'show',module.SOURCE_COMMIT+':file')==b'fixture'


def test_normal_snapshot_all18_and_authorized_dataset_symlink(stopmix,tmp_path,monkeypatch):
    # HH_260906 - Resolve only the approved dataset alias; ordinary input hashing must continue rejecting symlink traversal.
    from scripts.e2e import run_portable_training_campaign as runner
    workspace=tmp_path/'personal/hwanhong/portable_e2e';workspace.mkdir(parents=True)
    target=workspace/'runs/campaigns'/module.CAMPAIGN_ID;target.parent.mkdir(parents=True);stopmix.rename(target)
    repo=workspace/'autoware_e2e';repo.mkdir()
    plan=json.loads((target/'plan.json').read_text());state=json.loads((target/'status.json').read_text())
    for name,record in state['source_files'].items():
        path=repo/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((target/record['archive_path']).read_bytes())
    dataset=workspace.parent/'dataset'/Path(plan['dataset']).relative_to('datasets');dataset.mkdir(parents=True)
    manifest=dataset/'dataset.json';manifest.write_text('{"fixture":"manifest-only"}')
    (workspace/'datasets').symlink_to(workspace.parent/'dataset',target_is_directory=True)
    monkeypatch.setattr(module,'WORKSPACE',workspace);monkeypatch.setattr(runner,'WORKSPACE',workspace)
    monkeypatch.setattr(module,'PLAN_SHA256',module.digest(target/'plan.json'))
    monkeypatch.setattr(runner,'run_inventory',lambda command,_:module.SOURCE_COMMIT if command[1]=='rev-parse' else '')
    def verify_manifest(p,actual):
        assert p==plan and actual==dataset and not actual.is_symlink()
        return runner.EXPANDED_MANIFEST_SHA256
    monkeypatch.setattr(runner,'verify_dataset_manifest',verify_manifest)
    monkeypatch.setattr(module,'source_snapshot',lambda actual:{'fixture_verified_repo':str(actual.relative_to(workspace))})
    for run,_ in module.RUNS:
        checkpoint=target/run/'training/checkpoints/latest.pt';checkpoint.parent.mkdir();checkpoint.write_bytes(run.encode())
        checksum=module.digest(checkpoint)
        for stage,name in [('evaluate','evaluation/metrics.json'),('audit','gate_v8.json')]:
            path=target/run/name;value=json.loads(path.read_text());value['checkpoint_sha256']=checksum;path.write_text(json.dumps(value))
            record=next(r for r in state['stages'] if r['run']==run and r['stage']==stage);record['report']['sha256']=module.digest(path)
        for record in (r for r in state['stages'] if r['run']==run):
            record['command']=[v.replace('/synthetic/personal/hwanhong/portable_e2e',str(workspace)) for v in record['command']]
            if record['stage']=='audit':record['command'][-1]=checksum
    (target/'status.json').write_text(json.dumps(state));before=(target/'status.json').read_bytes()
    snapshot=module.normal_snapshot(runner,repo,target)
    assert len(snapshot['checkpoint_sha256'])==6 and len(snapshot['normal_input_sha256'])==37
    assert snapshot['dataset_manifest_sha256']==module.digest(manifest)
    assert not any(key.startswith('datasets/') for key in snapshot['normal_input_sha256'])
    assert (target/'status.json').read_bytes()==before
    with pytest.raises(RuntimeError,match='nonsymlink'):module.digest(workspace/plan['dataset']/'dataset.json')
