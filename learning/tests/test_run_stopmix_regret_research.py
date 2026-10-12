# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU-only protocol tests; no CARLA, remote calls, or GPU training."""
from copy import deepcopy
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, 'scripts/e2e/run_stopmix_regret_research.py')
spec = importlib.util.spec_from_file_location('regret_research_test', PATH)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(m.encoded(value))


def plan():
    return m.plan_contract() | dict(declared_at_utc='2026-09-14T00:00:00Z', worker_sha256=m.digest(PATH),
        source_sha256={name: m.digest(_hh_source_path(ROOT, name)) for name in m.SOURCE_PATHS},
        dataset='datasets/prepared/trainval_research', dataset_manifest_sha256='a'*64,
        view_report_path='benchmarks/view.json', view_report_sha256='b'*64,
        view_file_sha256={'dataset.json': 'a'*64}, corpus_fingerprint_sha256='c'*64,
        split_contract={s: dict(sample_count=n, fingerprint_sha256=h*64) for s,n,h in (('train',1147,'d'),('val',337,'e'))})


def test_plan_and_command_expose_exact_new_interaction(tmp_path):
    p = plan(); m.validate_plan(p, m.digest(PATH), datetime(2026,9,14,1,tzinfo=timezone.utc))
    assert len(m.RUNS) == 6 and len(m.SOURCE_PATHS) == 18
    for name in m.RUNS:
        args = m.command(p, tmp_path, ROOT, tmp_path/'out', name, 'train', tmp_path/'plan.json', PATH)
        # HH_260906 - This command targets the frozen remote layout, not a local source-file read.
        assert args[args.index('--model-config')+1] == str(ROOT / m.MODEL_PATH)
        assert args[args.index('--candidate-regret-weight')+1] == str(m.ARMS[name.split('/')[1]])
        assert '--resume' not in args and '--limit-samples' not in args
        assert args[args.index('--split')+1] == 'train'
        assert args[args.index('--max-steps')+1] == '1540'
        evaluate = m.command(p, tmp_path, ROOT, tmp_path/'out', name, 'evaluate', tmp_path/'plan.json', PATH)
        assert evaluate[evaluate.index('--split')+1] == 'val'
        assert 'test' not in evaluate


@pytest.mark.parametrize('field,value', [
    ('steps',1541), ('batch_size',True), ('automatic_retry',True), ('resume',True),
    ('test_read_or_use',True), ('training_data_approved',True), ('automatic_promotion',True),
    ('source_commit','f'*40), ('gpu_uuid','GPU-foreign'), ('runs',list(m.RUNS)[:-1]),
    ('arms',{'A_original_loss':0.,'B_cost_aware':.2}), ('whole_timeout_seconds',7201),
    ('stage_timeout_seconds',{'train':601,'evaluate':120,'audit':120,'behavior':180}),
    ('dataset','../escape'), ('dataset','/absolute'), ('worker_sha256','f'*64),
    ('declared_at_utc','2100-01-01T00:00:00Z'), ('declared_at_utc','2026-09-14T00:00:00'),
])
def test_plan_rejects_unreviewed_changes(field,value):
    p=plan();p[field]=value
    with pytest.raises(ValueError):m.validate_plan(p,m.digest(PATH))


def test_plan_rejects_extra_missing_source_and_test_reference():
    for kind in ('extra','source','test','count','fingerprint'):
        p=plan()
        if kind=='extra':p['silent_override']=True
        elif kind=='source':p['source_sha256'].pop(m.SOURCE_PATHS[0])
        elif kind=='test':p['view_file_sha256']['episodes/test/samples.jsonl']='f'*64
        elif kind=='count':p['split_contract']['val']['sample_count']=336
        else:p['split_contract']['train']['fingerprint_sha256']='short'
        with pytest.raises(ValueError):m.validate_plan(p,m.digest(PATH))


def test_strict_json_and_regular_paths(tmp_path):
    path=tmp_path/'data.json'
    path.write_text('{"a":1,"a":2}')
    with pytest.raises(ValueError,match='duplicate'):m.read(path)
    path.write_text('{"a":NaN}')
    with pytest.raises(ValueError,match='nonfinite'):m.read(path)
    link=tmp_path/'link';link.symlink_to(path)
    with pytest.raises(ValueError,match='nonsymlink'):m.digest(link)


@pytest.fixture
def view(tmp_path):
    p=plan();root=tmp_path/p['dataset'];root.mkdir(parents=True)
    refs=[]
    for index,(split,n) in enumerate((('train',400),('train',400),('train',347),('val',337))):
        name=f'episodes/e{index}/episode.json'
        write(root/name,dict(split=split,sample_count=n))
        refs.append(dict(manifest=name,sha256=m.digest(root/name)))
    write(root/'dataset.json',dict(episodes=refs))
    p['view_file_sha256']={x.relative_to(root).as_posix():m.digest(x) for x in root.rglob('*') if x.is_file()}
    p['dataset_manifest_sha256']=p['view_file_sha256']['dataset.json']
    proof=dict(schema='portable_e2e.trainval_research_view.v1',status='VALIDATED_LEGACY_RESEARCH_VIEW_NOT_NEW_ADMISSION',
        counts={'train':1147,'val':337},copied_payloads_byte_exact=True,source_inputs_unchanged=True,
        source_dataset_manifest_sha256='18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c',
        fresh_fits_only=True,optimizer_resume_allowed=False,training_data_approved=False,new_quality_approval=False,
        labels_modified=False,split_reassignment=False,test_payloads_opened=False,test_targets_opened=False,
        test_images_opened=False,test_provenance_opened=False,output_file_sha256=p['view_file_sha256'],
        output_dataset_manifest_sha256=p['dataset_manifest_sha256'],corpus_fingerprint_sha256=p['corpus_fingerprint_sha256'],
        split_tensor_fingerprint_sha256={s:p['split_contract'][s]['fingerprint_sha256'] for s in ('train','val')})
    write(tmp_path/p['view_report_path'],proof);p['view_report_sha256']=m.digest(tmp_path/p['view_report_path'])
    return p,tmp_path,root


def test_view_authenticates_complete_tree_without_parent_access(view,monkeypatch):
    p,workspace,root=view;observed=[];original=m.digest
    monkeypatch.setattr(m,'digest',lambda path:(observed.append(Path(path)),original(path))[1])
    assert m.verify_view(p,workspace)==p['view_file_sha256']
    assert all(path.is_relative_to(root) or path==workspace/p['view_report_path'] for path in observed)


@pytest.mark.parametrize('mutation',['changed','extra','missing','symlink','test','proof'])
def test_view_fails_closed(view,mutation):
    p,workspace,root=view
    if mutation=='changed':(root/'episodes/e0/episode.json').write_text('{}')
    elif mutation=='extra':(root/'unlisted.json').write_text('{}')
    elif mutation=='missing':(root/'episodes/e0/episode.json').unlink()
    elif mutation=='symlink':(root/'link').symlink_to(root/'dataset.json')
    elif mutation=='test':(root/'test').mkdir()
    else:(workspace/p['view_report_path']).write_text('{}')
    with pytest.raises(ValueError):m.verify_view(p,workspace)


def test_only_declared_personal_dataset_alias_is_allowed(view):
    p,workspace,root=view
    approved=workspace.parent/'dataset'
    (workspace/'datasets').rename(approved)
    (workspace/'datasets').symlink_to(approved,target_is_directory=True)
    assert m.verify_view(p,workspace)==p['view_file_sha256']
    real=approved/'prepared/trainval_research'
    real.rename(approved/'prepared/other')
    real.symlink_to(approved/'prepared/other',target_is_directory=True)
    with pytest.raises(ValueError,match='below approved'):m.verify_view(p,workspace)


def stage_fixture(tmp_path, arm='B_cost_aware'):
    p=plan();output=tmp_path/'output';run=f'seed_20260903/{arm}';item=output/run
    config=json.loads((_hh_source_path(ROOT, m.MODEL_PATH)).read_text())
    target=output/'source'/m.MODEL_PATH;target.parent.mkdir(parents=True);target.write_bytes((_hh_source_path(ROOT, m.MODEL_PATH)).read_bytes())
    cp=item/'training/checkpoints/latest.pt';cp.parent.mkdir(parents=True);cp.write_bytes(b'synthetic checkpoint, never loaded')
    canonical=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    common=dict(corpus_fingerprint_sha256=p['corpus_fingerprint_sha256'],checkpoint_sha256=m.digest(cp),
        model_config_sha256=canonical,training_dataset_fingerprint_sha256=p['split_contract']['train']['fingerprint_sha256'],
        dataset_fingerprint_sha256=p['split_contract']['val']['fingerprint_sha256'],evaluation_split='val',
        vehicle_control_approved=False,device='cuda:0',hardware={'device_uuid':m.GPU_UUID.removeprefix('GPU-')})
    return p,output,run,item,config,common


def test_native_audit_uses_geometry_sample_count(tmp_path):
    p,out,run,item,config,common=stage_fixture(tmp_path)
    report=common|dict(status='RUNTIME_GEOMETRY_AUDIT_COMPLETE',geometry={'sample_count':337},
        gate={'source':'portable_e2e.runtime_geometry_gate.v8','threshold_overrides':False,'thresholds':{'candidate_count':12}},
        implementation={'audit_runtime_sha256':p['source_sha256']['portable_e2e/audit_runtime.py'],
                        'runtime_contract_sha256':p['source_sha256']['portable_e2e/runtime_contract.py']})
    write(item/'gate_v8.json',report)
    assert m.verify_stage(p,out,run,'audit')['checkpoint_sha256']==common['checkpoint_sha256']
    report['geometry']['sample_count']=336;write(item/'gate_v8.json',report)
    with pytest.raises(ValueError,match='denominator'):m.verify_stage(p,out,run,'audit')


@pytest.mark.parametrize('arm',list(m.ARMS))
def test_native_eval_retains_research_auxiliary_and_provenance(tmp_path,arm):
    p,out,run,item,config,common=stage_fixture(tmp_path,arm)
    report=common|dict(status='OPEN_LOOP_EVALUATION_COMPLETE',sample_count=337,model_parameter_count=1056362,
        metrics={'selected_ade_m':1.})
    if m.ARMS[arm]:report.update(loss_config=dict(m.LOSS,candidate_regret_weight=.1),
        auxiliary_loss_metrics={'candidate_regret_loss':2.},auxiliary_loss_metric_counts={'candidate_regret_loss':337})
    write(item/'evaluation/metrics.json',report);m.verify_stage(p,out,run,'evaluate')
    report['training_dataset_fingerprint_sha256']='f'*64;write(item/'evaluation/metrics.json',report)
    with pytest.raises(ValueError,match='provenance'):m.verify_stage(p,out,run,'evaluate')


@pytest.mark.parametrize('arm',list(m.ARMS))
def test_native_training_warning_config_and_fresh_exposure(tmp_path,arm):
    p,out,run,item,config,common=stage_fixture(tmp_path,arm)
    report=dict(status='TRAINING_TARGET_REACHED',warning='RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL',
        corpus_fingerprint_sha256=p['corpus_fingerprint_sha256'],dataset_size=1147,model_parameter_count=1056362,
        dataset_fingerprint_sha256=p['split_contract']['train']['fingerprint_sha256'],model_config=config,
        state=dict(global_step=1540,samples_seen=6155,domain_samples_seen={'carla':6155}),
        loss_config=dict(m.LOSS,**({'candidate_regret_weight':.1} if m.ARMS[arm] else {})),
        device='cuda:0',hardware=common['hardware'],train_config=dict(seed=20260903,batch_size=4,learning_rate=1e-4,
            weight_decay=1e-4,max_steps=1540,checkpoint_interval=154,num_workers=0,maximum_gradient_norm=5.,
            verify_image_sha256=True,sampling_policy='uniform_without_replacement',domain_ratios=[]))
    write(item/'training/run.json',report);m.verify_stage(p,out,run,'train')
    report['resumed_at_utc']='2026-09-14T00:00:00Z';write(item/'training/run.json',report)
    with pytest.raises(ValueError,match='fresh'):m.verify_stage(p,out,run,'train')


@contextmanager
def nothing(*args):yield


@pytest.mark.parametrize('failure',[True,False,'swap'])
def test_owned_workflow_retains_failure_and_never_retries(tmp_path,monkeypatch,failure):
    workspace=tmp_path/'personal/hwanhong/portable_e2e';repo=workspace/'autoware_e2e';repo.mkdir(parents=True)
    p=plan()
    for name in m.SOURCE_PATHS:
        target=repo/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((_hh_source_path(ROOT, name)).read_bytes())
    owner=workspace/m.OWNER_PATH;owner.parent.mkdir(parents=True);owner.write_bytes(b'fixture helper')
    monkeypatch.setattr(m,'OWNER_SHA',m.digest(owner));p['owner_sha256']=m.OWNER_SHA
    plan_path=workspace/'plan.json';write(plan_path,p)
    output=workspace/'runs/diagnostics/stopmix_regret_20260914_v1'
    monkeypatch.setattr(m,'WORKSPACE',workspace)
    monkeypatch.setattr(m,'verify_sources',lambda *args:None)
    monkeypatch.setattr(m,'verify_view',lambda *args:None)
    calls=[]
    def stage(args,repo,env,log,timeout):
        calls.append(args)
        if failure=='swap' and len(calls)==2:
            (output/m.RUNS[0]/'training/checkpoints/latest.pt').write_bytes(b'swapped authentic-looking checkpoint')
        return 1 if failure is True else 0
    helper=SimpleNamespace(check_environment=lambda *args:None,gpu_lease=nothing,gpu_idle=lambda *args:{},
        bounded_signals=nothing,finalization_signals=nothing)
    monkeypatch.setattr(m,'load_module',lambda path,sha,name:helper if 'gpu_owner' in name else SimpleNamespace(run_owned_stage=stage))
    def receipt(plan,output,run,stage):
        cp=output/run/'training/checkpoints/latest.pt'
        if stage=='train':cp.parent.mkdir(parents=True);cp.write_bytes(b'fresh cp '+run.encode())
        report=output/run/(stage+'.json');write(report,{'synthetic':True})
        return dict(path=str(report.relative_to(output)),sha256=m.digest(report),checkpoint_sha256=m.digest(cp))
    monkeypatch.setattr(m,'verify_stage',receipt)
    monkeypatch.setattr(m,'paired_results',lambda *args:[])
    if failure:
        with pytest.raises(ValueError,match='exited 1' if failure is True else 'checkpoint changed during'):m.run(plan_path,output)
        final=m.read(output/'failed.json')
        assert len(calls)==(1 if failure is True else 2) and final['runs'][m.RUNS[0]]=='FAIL'
        assert list(final['runs'].values()).count('NOT_RUN')==5 and not (output/'summary.json').exists()
    else:
        m.run(plan_path,output);final=m.read(output/'summary.json')
        assert len(calls)==24 and final['status']=='COMPLETE_NOT_PROMOTED'
        assert len(final['stages'])==24 and set(final['runs'].values())=={'COMPLETE'}
        assert final['automatic_promotion'] is False and final['test_read_or_use'] is False
    with pytest.raises(ValueError,match='fresh'):m.run(plan_path,output)
