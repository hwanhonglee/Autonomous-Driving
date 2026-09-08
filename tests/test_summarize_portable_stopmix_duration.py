"""HH_260906 - Exercise duration evidence with explicitly synthetic files, never checkpoints, training or a GPU."""

from copy import deepcopy
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import shutil

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_stopmix_duration as module
from test_summarize_portable_stopmix import stopmix, behavior_root, no_accel, expansion, campaign, seal_behavior  # noqa: F401
from test_summarize_portable_stopmix import old as fixture_old


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True)+'\n')


def checksum_tree(root, name):
    paths=sorted(p for p in root.rglob('*') if p.is_file() and p.name != name)
    (root/name).write_text(''.join(f'{module.digest(p)}  {p.relative_to(root).as_posix()}\n' for p in paths))


def metric(worker, step):
    state=worker.training_state(step)
    return {k:state[k] for k in ('global_step','epoch','samples_seen','domain_samples_seen')} | dict(
        batch_domain_sample_counts={'carla':3 if step%287==0 else 4},loss=1.,regression_loss=.9,candidate_score_loss=1.,
        selected_ade_m=1.,selected_fde_m=2.,selected_speed_mae_mps=.2,gradient_norm=1.)


@pytest.fixture
def duration(stopmix, behavior_root, tmp_path, monkeypatch):
    # HH_260906 - Only provenance pins are synthetic here; the production reader still validates the entire 24-stage workflow.
    source=module.ROOT/'scripts/e2e/run_stopmix_duration_campaign.py'
    if not source.exists(): source=module.ROOT/'artifacts/training/2026-09-09/run_stopmix_duration_campaign.py'
    spec=importlib.util.spec_from_file_location('hh_synthetic_duration_contract',source)
    worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)
    root=tmp_path/'continued';root.mkdir();(root/'provenance').mkdir()
    (root/'provenance/stdin_worker.py').write_bytes(source.read_bytes())
    assert module.digest(source)==module.WORKER_SHA
    monkeypatch.setattr(module,'frozen_worker',lambda path:worker)
    parent_state=module.read(stopmix/'status.json')
    parent_files={}
    for run in worker.RUNS:
        training=stopmix/run/'training'; training.mkdir(exist_ok=True)
        history=''.join(json.dumps(metric(worker,i),sort_keys=True)+'\n' for i in range(1,1541))
        (training/'metrics.jsonl').write_text(history)
        report=module.read(training/'run.json')
        report.update(completed_at_utc='2026-09-08T18:16:00Z',state=worker.training_state(1540))
        write(training/'run.json',report)
        next(r for r in parent_state['stages'] if r['run']==run and r['stage']=='train')['report']['sha256']=module.digest(training/'run.json')
        checkpoint=module.read(stopmix/run/'evaluation/metrics.json')['checkpoint_sha256']
        parent_files[run]={'run.json':module.digest(training/'run.json'),'metrics.jsonl':module.digest(training/'metrics.jsonl'),
            'checkpoints/latest.pt':checkpoint}
    write(stopmix/'status.json',parent_state)
    worker.PARENT_FILES=parent_files;worker.PARENT_PLAN_SHA=module.digest(stopmix/'plan.json');worker.PARENT_STATUS_SHA=module.digest(stopmix/'status.json')
    parent_summary=module.old.summarize_campaign(stopmix,expected_source_commit=module.SOURCE,expected_plan_sha256=worker.PARENT_PLAN_SHA,behavior_root=behavior_root)
    sources={n:module.old.base._sha(module.old.git_bytes(module.SOURCE,n)) for n in module.old.BEHAVIOR_SOURCES}
    normal=dict(source_sha256=sources,normal_input_sha256={f'runs/campaigns/{worker.PARENT_ID}/{r["path"]}':r['sha256'] for r in parent_summary['input_manifest']},
        dataset_manifest_sha256=module.old.expansion.MANIFEST_SHA256,checkpoint_sha256={r:p['checkpoints/latest.pt'] for r,p in parent_files.items()})
    (behavior_root/'provenance').mkdir();(behavior_root/'provenance/stdin_worker.py').write_bytes(b'Synthetic owner archive, never executed.')
    worker.OWNER_SHA=module.digest(behavior_root/'provenance/stdin_worker.py')
    parent_workflow=dict(status='COMPLETE_NOT_PROMOTED',completed_behavior_count=6,source_commit=module.SOURCE,
        worker_source_sha256=worker.OWNER_SHA,source_and_normal_inputs_unchanged=True,before=normal,after=normal,
        finished_at_utc='2026-09-08T18:18:00Z',stages=[dict(run=r,status='COMPLETE',returncode=0,report=module.behavior_receipt(behavior_root/r)) for r in worker.RUNS])
    write(behavior_root/'workflow_status.json',parent_workflow);checksum_tree(behavior_root,'WORKFLOW_SHA256SUMS')
    worker.PARENT_BEHAVIOR_STATUS_SHA=module.digest(behavior_root/'workflow_status.json')
    worker.PARENT_BEHAVIOR_MANIFEST_SHA=module.digest(behavior_root/'WORKFLOW_SHA256SUMS')
    plan=worker.plan_contract(module.WORKER_SHA)|{'declared_at_utc':'2026-09-08T19:00:00Z'}
    write(root/'plan.json',plan);monkeypatch.setattr(module,'PLAN_SHA',module.digest(root/'plan.json'))
    before=module.parent_proof(stopmix,behavior_root,worker,parent_summary)
    state=dict(schema='portable_e2e.stopmix_duration_workflow.v1',status='COMPLETE_NOT_PROMOTED',
        started_at_utc='2026-09-08T19:01:00Z',finished_at_utc='2026-09-08T19:10:00Z',source_commit=module.SOURCE,
        worker_sha256=module.WORKER_SHA,plan_sha256=module.PLAN_SHA,plan=plan,before=before,copies=parent_files,
        automatic_promotion=False,training_data_approved=False,vehicle_control_approved=False,original_parent_modified=False,
        additional_fresh_initializations=0,parent_source_and_output_postcheck_pass=True,completed_stage_count=24,stages=[])
    base_runs={r['run']:r for pair in parent_summary['pairs'] for r in (pair['baseline'],pair['candidate'])}
    for run in worker.RUNS:
        item=root/run;shutil.copytree(stopmix/run,item)
        checkpoint=module.old.base._sha(('continued '+run).encode())
        training=item/'training';history=''.join(json.dumps(metric(worker,i),sort_keys=True)+'\n' for i in range(1,2871))
        (training/'metrics.jsonl').write_text(history)
        report=module.read(training/'run.json');report.update(state=worker.training_state(2870,final_report=True),
            train_config=worker.train_config(base_runs[run]['seed'],2870),resumed_at_utc='2026-09-08T19:01:00Z',
            completed_at_utc='2026-09-08T19:02:00Z',last_metrics=metric(worker,2870))
        write(training/'run.json',report)
        for name in ('evaluation/metrics.json','gate_v8.json'):
            value=module.read(item/name);value.update(checkpoint_sha256=checkpoint,training_domain_samples_seen={'carla':11470});write(item/name,value)
        shutil.copytree(behavior_root/run,item/'behavior')
        started=datetime(2026,9,8,19,1,tzinfo=timezone.utc)+timedelta(seconds=20*len(state['stages'])+61)
        for name in ('started.json','summary.json'):
            value=module.read(item/'behavior'/name);value.update(checkpoint_sha256=checkpoint,started_at_utc=started.isoformat())
            if name=='summary.json':
                value['ended_at_utc']=(started+timedelta(seconds=1)).isoformat()
                value['checkpoint_validation'].update(checkpoint_sha256=checkpoint,training_domain_samples_seen={'carla':11470})
            write(item/'behavior'/name,value)
        seal_behavior(item/'behavior')
        for stage in worker.STAGES:
            when=datetime(2026,9,8,19,1,tzinfo=timezone.utc)+timedelta(seconds=20*len(state['stages']))
            original=next(r for r in parent_state['stages'] if r['run']==run and r['stage']==('train' if stage=='behavior' else stage))
            python=Path(original['command'][0]);workspace=python.parents[3];output=workspace/'runs/campaigns'/worker.CAMPAIGN_ID
            if stage=='behavior':
                command=[str(python),'-m','scripts.e2e.audit_portable_stopmix_behavior',str(workspace/module.old.common.DATASET),
                    '--checkpoint',str(output/run/'training/checkpoints/latest.pt'),'--checkpoint-sha256',checkpoint,
                    '--expected-source-commit',module.SOURCE,'--expected-model-id',module.old.MODEL_IDS[run.split('/')[1]],
                    '--output-dir',str(output/run/'behavior'),'--device','cuda:0','--batch-size','4']
                proof=module.behavior_receipt(item/'behavior')
            else:
                command=[v.replace(worker.PARENT_ID,worker.CAMPAIGN_ID) for v in original['command']]
                if stage=='train':command[command.index('--max-steps')+1]='2870';command.append('--resume')
                if stage=='audit':command[command.index('--checkpoint-sha256')+1]=checkpoint
                proof=dict(path=module.STAGE_FILES[stage],sha256=module.digest(item/module.STAGE_FILES[stage]),checkpoint_sha256=checkpoint)
                if stage=='train':proof['history']=worker.history_proof(stopmix/run/'training',training,2870)[0]
            state['stages'].append(dict(run=run,stage=stage,status='COMPLETE',returncode=0,command=command,
                timeout_seconds=worker.TIMEOUTS[stage],started_at_utc=when.isoformat(),finished_at_utc=(when+timedelta(seconds=3)).isoformat(),report=proof))
    write(root/'status.json',state);reseal(root,state,worker)
    return root,stopmix,behavior_root,worker


def reseal(root,state,worker):
    write(root/'status.json',state);checksum_tree(root,'WORKFLOW_SHA256SUMS')
    with (root/'WORKFLOW_SHA256SUMS').open('a') as stream:
        for run in worker.RUNS:
            checksum=module.read(root/run/'evaluation/metrics.json')['checkpoint_sha256']
            stream.write(f'{checksum}  {run}/training/checkpoints/latest.pt\n')


def summarize(fixture):
    return module.summarize_campaign(*fixture[:3])


def test_complete_all_six_prefixes_24_stages_and_no_fresh_fit(duration):
    report=summarize(duration)
    assert report['status']=='COMPLETE_NOT_PROMOTED' and report['completed_stage_count']==24
    assert len(report['lineages'])==report['completed_lineage_count']==6
    assert report['budget']['additional_steps_all_six']==7980 and report['budget']['additional_samples_seen_all_six']==31890
    assert report['budget']['additional_fresh_initializations']==0 and report['budget']['total_samples_seen']==11470
    assert report['all_lineages_relative_checks_pass'] is False
    assert report['checkpoint_tensors_loaded'] is report['parent_reevaluated'] is report['automatic_promotion'] is False
    assert all(not r['continued']['checkpoint_bytes_locally_verified'] for r in report['lineages'])
    assert all(r['continued']['geometry']['selected_failure_counts']=={'speed':1} for r in report['lineages'])
    assert str(duration[0]) not in json.dumps(report) and '/synthetic/' not in json.dumps(report)


@pytest.mark.parametrize('fault',['prefix','last','seed','steps','loss','sampling','runtime','checkpoint','gate','behavior','source'])
def test_resealed_semantic_corruption_rejected(duration,fault):
    root,_,_,worker=duration;run=worker.RUNS[0];state=module.read(root/'status.json')
    if fault=='prefix':
        path=root/run/'training/metrics.jsonl';value=path.read_text();path.write_text(value.replace('"loss": 1.0','"loss": 2.0',1))
    elif fault in ('last','seed','steps','loss','sampling'):
        path=root/run/'training/run.json';value=module.read(path)
        if fault=='last':value['last_metrics']['loss']=2.
        elif fault=='seed':value['train_config']['seed']=20260906
        elif fault=='steps':value['train_config']['max_steps']=2871
        elif fault=='loss':value['loss_config']['candidate_score_weight']=.2
        else:value['sampling_plan_sha256']='f'*64
        write(path,value);state['stages'][0]['report']['sha256']=module.digest(path)
    else:
        name='evaluation/metrics.json' if fault in ('runtime','checkpoint') else 'gate_v8.json' if fault in ('gate','source') else 'behavior/summary.json'
        path=root/run/name;value=module.read(path)
        if fault=='runtime':value['device']='cpu'
        elif fault=='checkpoint':value['checkpoint_sha256']='f'*64
        elif fault=='gate':value['gate']['thresholds']['maximum_speed_mps']=10.
        elif fault=='source':value['implementation']['audit_runtime_sha256']='f'*64
        else:value['checkpoint_validation']['training_domain_samples_seen']={'carla':6155}
        write(path,value)
        index=1 if fault in ('runtime','checkpoint') else 2 if fault in ('gate','source') else 3
        if index==3:seal_behavior(path.parent);state['stages'][index]['report']=module.behavior_receipt(path.parent)
        else:state['stages'][index]['report']['sha256']=module.digest(path)
    reseal(root,state,worker)
    with pytest.raises((ContractError,RuntimeError)):summarize(duration)


@pytest.mark.parametrize('fault',['order','resume','time','timeout','copy','postcheck','checkpoint_receipt','wrong_source','workflow_deadline'])
def test_workflow_false_completion_fails_closed(duration,fault):
    root,_,_,worker=duration;state=module.read(root/'status.json')
    if fault=='order':state['stages'][0],state['stages'][1]=state['stages'][1],state['stages'][0]
    elif fault=='resume':state['stages'][0]['command'].remove('--resume')
    elif fault=='time':state['stages'][0]['started_at_utc']='2026-09-09T00:59:00Z'
    elif fault=='timeout':state['stages'][0]['timeout_seconds']=999
    elif fault=='copy':state['copies'][worker.RUNS[0]]['run.json']='f'*64
    elif fault=='postcheck':state['parent_source_and_output_postcheck_pass']=False
    elif fault=='checkpoint_receipt':state['stages'][0]['report']['checkpoint_sha256']='f'*64
    elif fault=='wrong_source':state['source_commit']='f'*40
    else:state['finished_at_utc']='2026-09-09T01:00:00.000001Z'
    reseal(root,state,worker)
    with pytest.raises((ContractError,RuntimeError)):summarize(duration)


def test_incomplete_preserves_finished_lineages_without_false_pass(duration):
    root,_,_,worker=duration;state=module.read(root/'status.json')
    state.update(status='RUNNING',stages=state['stages'][:4]);write(root/'status.json',state)
    report=summarize(duration)
    assert report['status']=='INCOMPLETE' and report['completed_stage_count']==4 and report['completed_lineage_count']==1
    assert report['all_lineages_absolute_checks_pass'] is report['all_lineages_relative_checks_pass'] is False


def test_missing_inputs_return_incomplete(tmp_path):
    report=module.summarize_campaign(tmp_path/'missing',tmp_path/'parent',tmp_path/'behavior')
    assert report['status']=='INCOMPLETE' and len(report['missing_artifacts'])==3


def test_output_guard_blocks_all_inputs_and_direct_dataset_alias(tmp_path,monkeypatch):
    root=tmp_path/'repo';root.mkdir();data=tmp_path/'actual-data';data.mkdir();(root/'datasets').symlink_to(data,target_is_directory=True)
    monkeypatch.setattr(module,'ROOT',root)
    inputs=[tmp_path/'a',tmp_path/'b',tmp_path/'c']
    for path in [*(p/'new' for p in inputs),data/'new',root/'datasets'/'new']:
        with pytest.raises(ContractError):module.checked_output(path,inputs)
    assert module.checked_output(tmp_path/'new',inputs)==tmp_path/'new'


def test_actual_worker_source_authenticated_before_import(tmp_path):
    (tmp_path/'provenance').mkdir();(tmp_path/'provenance/stdin_worker.py').write_text('raise AssertionError("must never execute")')
    with pytest.raises(ContractError,match='archive'):module.frozen_worker(tmp_path)


def test_actual_authenticated_worker_is_imported_without_input_writes(tmp_path):
    source=module.ROOT/'scripts/e2e/run_stopmix_duration_campaign.py'
    (tmp_path/'provenance').mkdir();shutil.copyfile(source,tmp_path/'provenance/stdin_worker.py')
    before=module.tree_pins(tmp_path)
    worker=module.frozen_worker(tmp_path)
    assert worker.SOURCE_COMMIT==module.SOURCE and worker.training_state(2870,final_report=True)['epoch']==10
    assert module.tree_pins(tmp_path)==before and not (tmp_path/'provenance/__pycache__').exists()


def test_two_actual_numeric_plots_and_every_history_row(duration,tmp_path):
    report=summarize(duration);output=tmp_path/'plots';output.mkdir()
    info=module.plot_results(report,duration[0],output)
    assert info['learning_history_points']==17220 and info['last_bin_size']==20
    from PIL import Image
    for name in info['files']:
        with Image.open(output/name) as image:assert image.format=='PNG' and image.width>=2000
    report['status']='INCOMPLETE'
    with pytest.raises(ContractError):module.plot_results(report,duration[0],output)


def test_numeric_bins_preserve_batch_three_weight_final_twenty_and_boundary():
    values=[dict(global_step=i,loss=2. if i%287==0 else 1.,batch_domain_sample_counts={'carla':3 if i%287==0 else 4}) for i in range(1,2871)]
    bins=module.loss_bins(values)
    assert sum(row['updates'] for row in bins)==2870 and sum(row['exposures'] for row in bins)==11470
    assert bins[5]['first_step']==251 and bins[5]['last_step']==300 and bins[5]['exposures']==199
    assert bins[5]['mean_loss']==202/199
    assert bins[30]['first_step']==1501 and bins[30]['last_step']==1550
    assert bins[-1]['updates']==20 and bins[-1]['exposures']==79 and bins[-1]['mean_loss']==82/79


def test_publication_checks_inputs_after_plotting(duration,tmp_path,monkeypatch):
    def mutate(report,campaign,output):
        (campaign/'status.json').write_text('{}')
        return {'files':[]}
    monkeypatch.setattr(module,'plot_results',mutate)
    root,parent,behavior,_=duration;output=tmp_path/'published'
    assert module.main([str(root),'--parent',str(parent),'--parent-behavior',str(behavior),'--output-dir',str(output),'--plots'])==2
    assert not (output/'SHA256SUMS').exists()
