"""HH_260906 - Test publication with synthetic ABBA receipts, not training, checkpoints, GPU work or private datasets."""

import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import curate_portable_dataloader_abba as curate
from scripts.e2e import profile_portable_dataloader_abba as worker
from scripts.e2e import profile_portable_training as helper


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(value if isinstance(value,bytes) else curate.encoded(value))


@pytest.fixture
def evidence(tmp_path,monkeypatch):
    root=tmp_path/'input';root.mkdir();model={'model_id':'synthetic-fixture-only'}
    source_hashes={}
    for name in helper.SOURCE_PATHS:
        raw=curate.encoded(model if name==helper.MODEL_CONFIG else {'synthetic':True}) if name.endswith('.json') else b'# synthetic source bytes\n'
        write(root/'source'/name,raw);source_hashes[name]=curate.sha(raw)
    for name in ('independent_worker.py','frozen_helper.py'):write(root/'source'/name,b'# synthetic owned code fixture\n')
    monkeypatch.setattr(curate,'WORKER_SHA',curate.sha((root/'source/independent_worker.py').read_bytes()))
    monkeypatch.setattr(curate,'HELPER_SHA',curate.sha((root/'source/frozen_helper.py').read_bytes()))
    plan=worker.plan_contract(helper)
    plan.update(declared_at_utc='2026-09-08T19:00:00Z',source_sha256=source_hashes,worker_source_sha256=curate.WORKER_SHA)
    write(root/'plan.json',plan);monkeypatch.setattr(curate,'PLAN_SHA',curate.sha((root/'plan.json').read_bytes()))
    monkeypatch.setattr(curate,'load_owned_code',lambda path,expected,pins:worker if path.name=='independent_worker.py' else helper)
    sources=dict(source_hashes,**{'frozen_helper.py':curate.HELPER_SHA,'independent_worker.py':curate.WORKER_SHA})
    report=dict(schema=worker.SCHEMA,status='ABBA_COMPLETE_EXACT_PARITY_NOT_PROMOTED',source_sha256=sources,
        started_at_utc='2026-09-08T19:01:00Z',completed_at_utc='2026-09-08T19:09:00Z',plan_sha256=curate.PLAN_SHA,
        approval=worker.DENIALS,deadline_met=True,postcheck_errors=[],unperformed_arms=[],arms=[],checkpoint_parity=[],
        metrics_parity=[dict(reference=worker.ARMS[0][0],candidate=arm,exact=True,first_difference=None,retained_rows_per_arm=287) for arm,_ in worker.ARMS[1:]])
    state=dict(global_step=287,epoch=1,next_batch_index=0,samples_seen=1147,domain_samples_seen={'carla':1147})
    rows=[dict(global_step=i,epoch=0,samples_seen=min(i*4,1147),domain_samples_seen={'carla':min(i*4,1147)},
        batch_domain_sample_counts={'carla':3 if i==287 else 4},loss=1.,gradient_norm=2.,regression_loss=.9,candidate_score_loss=1.) for i in range(1,288)]
    for index,(arm,workers) in enumerate(worker.ARMS):
        pid=1000+index;began=f'2026-09-08T19:0{index*2+1}:00Z';ended=f'2026-09-08T19:0{index*2+2}:00Z'
        train_time=20.+index if workers else 30.+index;process_time=train_time+10
        proofs=[]
        for k in range(workers):
            ppid=2000+index*10+k
            proof=dict(pid=ppid,parent_pid=pid,process_group=pid,source_sha256=curate.WORKER_SHA,start_method='spawn',
                cuda_visible_devices='',torch_imported_before_guard=False)
            write(root/arm/'worker_proofs'/(str(ppid)+'.json'),proof);proofs.append(proof)
        fit=dict(schema=worker.SCHEMA,status='FIT_COMPLETE_NOT_PROMOTED',arm=arm,num_workers=workers,state=state,
            source_sha256=sources,approval=worker.DENIALS,postcheck_errors=[],pid=pid,process_group=pid,child_cleanup=[],
            started_at_utc=began,completed_at_utc=ended,worker_proofs=proofs,**{curate.TRAIN_TIME:train_time})
        write(root/arm/'fit_report.json',fit)
        report['arms'].append(dict(arm=arm,num_workers=workers,status='FIT_COMPLETE_NOT_PROMOTED',exit_code=0,pid=pid,
            cleanup=dict(pid=pid,process_group=pid,group_absent=True),started_at_utc=began,completed_at_utc=ended,
            fit_report_sha256=curate.sha((root/arm/'fit_report.json').read_bytes()),**{curate.TRAIN_TIME:train_time,curate.PROCESS_TIME:process_time}))
        run=dict(status='TRAINING_TARGET_REACHED',state=state,dataset_size=1147,training_split='train',
            dataset_fingerprint_sha256=helper.TRAIN_SHA,corpus_fingerprint_sha256=helper.CORPUS_SHA,
            loss_config=helper.LOSS_CONFIG,train_config=worker.train_config(helper,workers),model_parameter_count=954590,
            training_episode_ids=['one','two','three'],model_config=model,last_metrics=rows[-1])
        write(root/arm/'fit/run.json',run)
        write(root/arm/'fit/metrics.jsonl',b''.join((__import__('json').dumps(r)+'\n').encode() for r in rows))
        for name in ('console.log','.fit.portable-e2e.lock'):write(root/arm/name,b'')
        report['checkpoint_parity'].append(dict(arm=arm,checkpoint_sha256=str(index+1)*64,exact=True,first_difference=None,
            ignored_fields=['created_at_utc','train_config.num_workers'],weights_only=True))
    e=SimpleNamespace(root=root,output=tmp_path/'public',report=report,plan=plan,monkeypatch=monkeypatch)
    reseal(e);return e


def reseal(e):
    root=e.root
    for item in e.report['arms']:item['fit_report_sha256']=curate.sha((root/item['arm']/'fit_report.json').read_bytes())
    files={p.relative_to(root).as_posix():dict(sha256=curate.sha(p.read_bytes()),bytes=p.stat().st_size)
        for p in root.rglob('*') if p.is_file() and p.name not in ('report.json','SHA256SUMS','transport_verification.json')}
    missing=[]
    for item in e.report['checkpoint_parity']:
        name=item['arm']+'/fit/checkpoints/latest.pt';files[name]=dict(sha256=item['checkpoint_sha256'],bytes=123);missing.append(name)
    e.report['files']=files;write(root/'report.json',e.report)
    e.monkeypatch.setattr(curate,'REPORT_SHA',curate.sha((root/'report.json').read_bytes()))
    entries={n:v['sha256'] for n,v in files.items()}|{'report.json':curate.REPORT_SHA}
    write(root/'SHA256SUMS',''.join(h+'  '+n+'\n' for n,h in sorted(entries.items())).encode())
    e.monkeypatch.setattr(curate,'MANIFEST_SHA',curate.sha((root/'SHA256SUMS').read_bytes()))
    inventory={n:dict(sha256=v['sha256'],size_bytes=v['bytes']) for n,v in files.items()}
    inventory.update({n:dict(sha256=curate.sha((root/n).read_bytes()),size_bytes=(root/n).stat().st_size) for n in ('report.json','SHA256SUMS')})
    transport=dict(status='VERIFIED_ABBA_EXACT_PARITY_NOT_PROMOTED',regular_file_inventory=inventory,omitted_checkpoints=sorted(missing),
        all_original_sha_payloads_pass=43,remote_pre_post_bytes_unchanged=True,source_pre_post_bytes_unchanged=True,
        metrics_four_byte_identical=True,cooperative_gpu0_lease_free=True,checkpoint_tensors_loaded_by_collector=False,
        data_admission=False,model_promotion=False)
    write(root/'transport_verification.json',transport)
    e.monkeypatch.setattr(curate,'TRANSPORT_SHA',curate.sha((root/'transport_verification.json').read_bytes()))


def test_synthetic_complete_binding_and_two_different_timing_denominators(evidence):
    r,p,t,measurements,pins,m=curate.bind(evidence.root)
    assert len(measurements)==4 and sum(x['metric_rows'] for x in measurements)==1148
    assert sum(x['sample_exposures'] for x in measurements)==4588
    s=curate.statistics(measurements)
    assert s['train_call_seconds']['workers0_mean']==31.5 and s['train_call_seconds']['workers2_mean']==21.5
    assert s['train_call_seconds']['observed_reduction_percent']==100*(1-21.5/31.5)
    assert s['process_seconds']['observed_reduction_percent']==100*(1-31.5/41.5)
    assert not list(evidence.root.rglob('*.pt')) and not (evidence.root/'A1_workers0/worker_proofs').exists()


@pytest.mark.parametrize('kind',['status','order','checkpoint','cleanup','scope','clock','missing_arm','timing','nan','steps'])
def test_resealed_false_completion_or_bad_timing_rejected(evidence,kind):
    r=evidence.report
    if kind=='status':r['status']='FAILED'
    if kind=='order':r['arms'][1],r['arms'][2]=r['arms'][2],r['arms'][1]
    if kind=='checkpoint':r['checkpoint_parity'][0]['exact']=False
    if kind=='cleanup':r['arms'][0]['cleanup']['group_absent']=False
    if kind=='scope':r['approval']=dict(r['approval'],model_promotion=True)
    if kind=='clock':r['arms'][0]['started_at_utc']='2026-09-08T18:00:00Z'
    if kind=='missing_arm':r['arms'].pop()
    if kind=='timing':r['arms'][0][curate.TRAIN_TIME]+=1
    if kind=='nan':r['arms'][0][curate.PROCESS_TIME]=-1
    if kind=='steps':r['metrics_parity'][0]['retained_rows_per_arm']=286
    reseal(evidence)
    with pytest.raises(ValueError):curate.bind(evidence.root)


@pytest.mark.parametrize('kind',['different','missing_row','counter','last_metric','worker_missing','worker_gpu','wrong_loss','wrong_split'])
def test_resealed_history_and_child_evidence_inconsistency_rejected(evidence,kind):
    root=evidence.root;path=root/'B1_workers2/fit/metrics.jsonl'
    rows=[curate.read_json(line) for line in path.read_bytes().splitlines()]
    if kind=='different':rows[1]['loss']=2.
    if kind=='missing_row':rows.pop()
    if kind=='counter':rows[0]['global_step']=2
    if kind in ('different','missing_row','counter'):write(path,b''.join((__import__('json').dumps(r)+'\n').encode() for r in rows))
    if kind in ('last_metric','wrong_loss','wrong_split'):
        p=root/'B1_workers2/fit/run.json';r=curate.read_json(p.read_bytes())
        if kind=='last_metric':r['last_metrics']['loss']=9.
        if kind=='wrong_loss':r['loss_config']['candidate_score_weight']=.5
        if kind=='wrong_split':r['training_split']='val'
        write(p,r)
    if kind=='worker_missing':next((root/'B1_workers2/worker_proofs').iterdir()).unlink()
    if kind=='worker_gpu':
        p=next((root/'B1_workers2/worker_proofs').iterdir());r=curate.read_json(p.read_bytes());r['cuda_visible_devices']='0';write(p,r)
    reseal(evidence)
    with pytest.raises(ValueError):curate.bind(root)


def test_unauthenticated_owned_source_is_never_executed(tmp_path):
    p=tmp_path/'malicious.py';p.write_text('raise AssertionError("must not execute")')
    with pytest.raises(ValueError):curate.load_owned_code(p,'0'*64,{})


def test_source_text_redaction_and_no_private_uuid_survives():
    value={'path':'/home/account/personal/owner/portable_e2e/runs/x','uuid':'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',
        'code':'device_uuid = "59f374a4-53f5-c050-34b2-56aab0e3c7e5"'}
    rendered=curate.encoded(curate.redact(value));curate.safe.privacy(rendered)
    assert b'${PORTABLE_E2E_ROOT}' in rendered and rendered.count(b'${GPU0_UUID}')==2


@pytest.mark.parametrize('value',[float('nan'),float('inf'),float('-inf'),True,0,-1,'1'])
def test_timing_values_must_be_finite_positive_real_scalars(value):
    assert not curate.finite_positive(value)


def test_create_only_publication_has_one_measured_png_exact_histories_and_hashes(evidence):
    before={p:p.read_bytes() for p in evidence.root.rglob('*') if p.is_file()}
    result=curate.publish(evidence.root,evidence.output)
    assert not result['checkpoint_tensors_loaded'] and result['all_four_metric_streams_independently_byte_equal']
    assert all(p.read_bytes()==v for p,v in before.items())
    from PIL import Image
    with Image.open(evidence.output/'01_measured_abba_timing.png') as image:assert image.size==(1920,840)
    assert len(list(evidence.output.rglob('*.png')))==1 and not list(evidence.output.rglob('*.pt'))
    entries=curate.safe.manifest_entries((evidence.output/'SHA256SUMS').read_bytes())
    assert all(curate.sha((evidence.output/n).read_bytes())==h for n,h in entries.items())
    for arm,_ in curate.ARMS:assert (evidence.output/arm/'fit/metrics.jsonl').read_bytes()==(evidence.root/arm/'fit/metrics.jsonl').read_bytes()
    before_public={p:p.read_bytes() for p in evidence.output.rglob('*') if p.is_file()}
    with pytest.raises(ValueError):curate.publish(evidence.root,evidence.output)
    assert all(p.read_bytes()==v for p,v in before_public.items())


def test_postrender_input_mutation_cannot_get_success_manifest(evidence,monkeypatch):
    original=curate.render
    def mutate(rows,output):
        result=original(rows,output);(evidence.root/'A1_workers0/fit/metrics.jsonl').write_bytes(b'changed');return result
    monkeypatch.setattr(curate,'render',mutate)
    with pytest.raises(ValueError):curate.publish(evidence.root,evidence.output)
    assert evidence.output.exists() and not (evidence.output/'publication_manifest.json').exists()


def test_symlink_inputs_or_unsafe_output_rejected(evidence,tmp_path):
    with pytest.raises(ValueError):curate.publish(evidence.root,evidence.root/'new')
    p=evidence.root/'A1_workers0/console.log';p.unlink();p.symlink_to(tmp_path/'outside')
    with pytest.raises(ValueError):curate.bind(evidence.root)


def test_cli_no_abbreviations():
    with pytest.raises(SystemExit):curate.main(['--input-r','x'])
