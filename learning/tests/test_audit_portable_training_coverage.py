# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Exercise metadata-only coverage without dataset payload, checkpoint, Torch or GPU reads."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
spec = importlib.util.spec_from_file_location('coverage_audit_test', _hh_source_path(ROOT, 'scripts/e2e/audit_portable_training_coverage.py'))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(m.encoded(value))


@pytest.fixture
def view(tmp_path):
    root=tmp_path/'view';refs=[]
    for i,(split,n) in enumerate((('train',400),('train',400),('train',347),('val',337))):
        eid=f'episode_{i}';name=f'episodes/{eid}/episode.json'
        ep=dict(schema_id='autoware-e2e.common10.episode.v1',episode_id=eid,domain='carla',split=split,
            start_timestamp_ns=1000000000,end_timestamp_ns=1000000000+(n+64)*100000000,
            sample_count=n,capture_accounting={'raw_anchor_count':n+65},source_session_id=f'session_{i}',
            split_group_id=f'group_{i}',map_id=f'map_{i}',site_id=f'map_{i}',route_id=f'route_{i}',
            weather='ClearNoon',traffic_seed=i%2,sample_jsonl='samples.jsonl',
            source_provenance={'source_manifest_file':'FORBIDDEN_provenance.json'})
        write(root/name,ep);refs.append(dict(episode_id=eid,manifest=name,sha256=m.sha((root/name).read_bytes())))
        (root/name).parent.joinpath('samples.jsonl').write_text('DO NOT READ\n')
    write(root/'dataset.json',dict(schema_id='autoware-e2e.common10.dataset.v1',contract_id='common_10hz_v1',
        dataset_id='synthetic_trainval',camera_order=m.CAMERAS,episodes=refs))
    return root


def test_metadata_coverage_never_reads_payloads(view,monkeypatch):
    original=Path.read_bytes;opened=[]
    def guarded(path):
        opened.append(path)
        assert path.name in ('dataset.json','episode.json'), 'payload access forbidden'
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',guarded)
    inputs=m.Inputs();manifest,report=m.coverage(view,inputs);inputs.verify()
    assert report['train']['sample_count']==1147 and report['train']['recorded_episode_count']==3
    assert report['train']['unique_map_count']==report['train']['unique_route_count']==3
    assert report['train']['unique_weather_count']==1
    assert report['train']['recorded_duration_seconds']==pytest.approx(133.9)
    assert report['train']['recorded_duration_minutes']==pytest.approx(133.9/60)
    assert report['val']['recorded_episode_count']==1 and report['val']['sample_count']==337
    assert len(inputs.hashes)==5 and len(opened)==10


@pytest.mark.parametrize('split',['test','unknown',None,True])
def test_forbidden_split_rejected_without_payload_touch(view,monkeypatch,split):
    path=view/'episodes/episode_3/episode.json';ep=json.loads(path.read_text());ep['split']=split;write(path,ep)
    original=Path.read_bytes
    def guarded(path):
        assert path.name in ('dataset.json','episode.json'), 'TEST payload must never be opened'
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',guarded)
    with pytest.raises(ValueError,match='TEST and unknown'):m.coverage(view,m.Inputs())


@pytest.mark.parametrize('kind',['traversal','absolute','symlink','duplicate','wronghash','nonfinite','backward_time','split_leak'])
def test_metadata_references_and_fields_fail_closed(view,tmp_path,kind):
    manifest=json.loads((view/'dataset.json').read_text());ref=manifest['episodes'][0]
    path=view/ref['manifest'];ep=json.loads(path.read_text())
    if kind=='traversal':ref['manifest']='../external.json'
    elif kind=='absolute':ref['manifest']=str(tmp_path/'external.json')
    elif kind=='symlink':
        other=tmp_path/'external.json';other.write_bytes(path.read_bytes());path.unlink();path.symlink_to(other)
    elif kind=='duplicate':manifest['episodes'].append(ref)
    elif kind=='wronghash':ref['sha256']='a'*64
    elif kind=='nonfinite':
        path.write_text(json.dumps(ep).replace('"traffic_seed": 0','"traffic_seed": 1e309'))
        ref['sha256']=m.sha(path.read_bytes())
    elif kind=='backward_time':
        ep['end_timestamp_ns']=ep['start_timestamp_ns'];write(path,ep);ref['sha256']=m.sha(path.read_bytes())
    else:
        path=view/manifest['episodes'][-1]['manifest'];ep=json.loads(path.read_text());ep['split_group_id']='group_0'
        write(path,ep);manifest['episodes'][-1]['sha256']=m.sha(path.read_bytes())
    write(view/'dataset.json',manifest)
    with pytest.raises(ValueError):m.coverage(view,m.Inputs())


def test_dataset_root_symlink_rejected(view,tmp_path):
    alias=tmp_path/'alias';alias.symlink_to(view,target_is_directory=True)
    with pytest.raises(ValueError,match='nonsymlink'):m.canonical(alias,directory=True)


@pytest.mark.parametrize('payload',['{"x":1,"x":2}','{"x":NaN}','{"x":Infinity}','{"x":1e309}'])
def test_strict_json_rejects_duplicate_and_nonfinite(payload):
    with pytest.raises(ValueError):m.loads(payload)


def history(sample_count=1147,batch_size=4,steps=1540,arm='A_original_loss'):
    rows=[];batches=(sample_count+batch_size-1)//batch_size;seen=0
    for step in range(1,steps+1):
        epoch,index=divmod(step-1,batches);size=min(batch_size,sample_count-index*batch_size);seen+=size
        row=dict(global_step=step,epoch=epoch,samples_seen=seen,batch_domain_sample_counts={'carla':size},domain_samples_seen={'carla':seen},
            **{key:1/(epoch+1) for key in m.LOSS_FIELDS})
        if arm=='B_cost_aware':row['candidate_regret_loss']=2/(epoch+1)
        rows.append(row)
    return rows


@pytest.mark.parametrize('arm',['A_original_loss','B_cost_aware'])
def test_real_exposure_budget_complete_epochs_and_partial(arm):
    result=m.exposure(history(arm=arm),1147,4,1540,arm)
    assert result['optimizer_steps']==1540 and result['sample_exposures']==6155
    assert result['complete_epochs']==5 and result['partial_epoch_sample_exposures']==420
    assert result['equivalent_dataset_passes']==pytest.approx(6155/1147)
    assert [ep['sample_exposures'] for ep in result['epochs']]==[1147]*5+[420]


def test_weighted_epoch_means_use_actual_short_batch_sizes():
    rows=history(5,2,3);rows[-1]['regression_loss']=10
    result=m.exposure(rows,5,2,3,'A_original_loss')
    assert result['epochs'][0]['exposure_weighted_metrics']['regression_loss']==pytest.approx(14/5)


@pytest.mark.parametrize('kind',['missing','step','shortbatch','seen','boolstep','nan','negative','wrongaux'])
def test_history_failure_is_not_relabelled_complete(kind):
    rows=history()
    if kind=='missing':rows.pop()
    elif kind=='step':rows[1]['global_step']=1
    elif kind=='shortbatch':rows[286]['batch_domain_sample_counts']={'carla':4}
    elif kind=='seen':rows[-1]['samples_seen']=6156
    elif kind=='boolstep':rows[0]['global_step']=True
    elif kind=='nan':rows[0]['gradient_norm']=float('nan')
    elif kind=='negative':rows[0]['loss']=-1
    else:rows[0]['candidate_regret_loss']=.1
    with pytest.raises(ValueError):m.exposure(rows,1147,4,1540,'A_original_loss')


def full_fixture(view,tmp_path):
    inputs=m.Inputs();manifest,covered=m.coverage(view,inputs)
    plan=dict(schema='portable_e2e.stopmix_regret_research_plan.v1',runs=list(m.RUNS),steps=1540,batch_size=4,samples_seen=6155,
        arms={'A_original_loss':0.,'B_cost_aware':.1},resume=False,test_read_or_use=False,
        dataset_manifest_sha256=m.sha((view/'dataset.json').read_bytes()),
        view_file_sha256={Path(path).relative_to(view).as_posix():value for path,value in inputs.hashes.items()},
        split_contract={split:dict(fingerprint_sha256=letter*64) for split,letter in (('train','a'),('val','b'))},
        corpus_fingerprint_sha256='c'*64)
    plan_path=tmp_path/'prospective.json';plan_path.write_text(json.dumps(plan,indent=2)+'\n');plan_sha=m.sha(plan_path.read_bytes())
    campaign=tmp_path/'campaign';write(campaign/'plan.json',plan)
    ledger=[]
    for run in m.RUNS:
        arm=run.split('/')[1];rows=history(arm=arm);cp='d'*64
        train=dict(status='TRAINING_TARGET_REACHED',training_split='train',training_episode_ids=['episode_0','episode_1','episode_2'],
            dataset_size=1147,dataset_fingerprint_sha256='a'*64,corpus_fingerprint_sha256='c'*64,
            last_metrics=rows[-1],state={'global_step':1540,'samples_seen':6155},model_config={'future_points':64,'camera_count':6})
        evaluation=dict(status='OPEN_LOOP_EVALUATION_COMPLETE',evaluation_split='val',sample_count=337,
            dataset_fingerprint_sha256='b'*64,corpus_fingerprint_sha256='c'*64,checkpoint_sha256=cp,
            metrics={'selected_ade_m':4.},metric_counts={'selected_ade_m':337})
        for stage,relative,report in (('train','training/run.json',train),('evaluate','evaluation/metrics.json',evaluation),
                ('audit','gate_v8.json',{}),('behavior','behavior/summary.json',{})):
            path=campaign/run/relative;write(path,report)
            receipt=dict(run=run,stage=stage,status='COMPLETE',path=run+'/'+relative,sha256=m.sha(path.read_bytes()),checkpoint_sha256=cp)
            write(campaign/run/(stage+'_completed.json'),receipt);ledger.append(receipt)
        (campaign/run/'training/metrics.jsonl').write_bytes(b''.join(m.encoded(row) for row in rows))
    write(campaign/'summary.json',dict(status='COMPLETE_NOT_PROMOTED',plan_sha256=plan_sha,
        runs={name:'COMPLETE' for name in m.RUNS},test_read_or_use=False,automatic_promotion=False,stages=ledger))
    return campaign,plan_path,plan_sha


def test_six_fit_metadata_history_audit_no_payload_or_torch(view,tmp_path,monkeypatch):
    campaign,plan_path,pin=full_fixture(view,tmp_path)
    original=Path.read_bytes
    def guard(path):
        assert path.suffix!='.pt' and path.name!='samples.jsonl'
        if path.is_relative_to(view):assert path.name in ('dataset.json','episode.json')
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',guard)
    report=m.audit(view,campaign,plan_path,pin)
    assert report['total_optimizer_steps']==9240 and report['total_sample_exposures']==36930
    assert report['nominal_future_window_overlap']['adjacent_shared_fraction']==63/64
    assert all(report[key] is False for key in m.DENIALS)
    assert report['input_sha256_before']==report['input_sha256_after']
    assert all(fit['per_epoch_val_curve_available'] is False for fit in report['fits'])


def test_no_output_on_failed_plan_or_partial_run(view,tmp_path):
    campaign,plan_path,pin=full_fixture(view,tmp_path);output=tmp_path/'audit.json'
    args=['--dataset-root',str(view),'--campaign-root',str(campaign),'--plan',str(plan_path),
          '--plan-sha256','f'*64,'--output-json',str(output)]
    with pytest.raises(ValueError,match='plan SHA'):m.main(args)
    assert not output.exists()
    (campaign/m.RUNS[-1]/'training/metrics.jsonl').write_text('')
    args[args.index('--plan-sha256')+1]=pin
    with pytest.raises(ValueError,match='complete optimizer'):m.main(args)
    assert not output.exists()


def test_fresh_disjoint_output_only(view,tmp_path):
    campaign,plan_path,pin=full_fixture(view,tmp_path)
    args=['--dataset-root',str(view),'--campaign-root',str(campaign),'--plan',str(plan_path),'--plan-sha256',pin,'--output-json']
    with pytest.raises(ValueError,match='disjoint'):m.main(args+[str(view/'audit.json')])
    output=tmp_path/'output.json';m.main(args+[str(output)])
    with pytest.raises(ValueError,match='fresh'):m.main(args+[str(output)])


def test_input_change_detected(view):
    inputs=m.Inputs();inputs.read(view/'dataset.json');(view/'dataset.json').write_text('{}')
    with pytest.raises(ValueError,match='changed'):inputs.verify()
