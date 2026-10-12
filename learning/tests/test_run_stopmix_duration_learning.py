# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU-only continuation protocol tests; never allocate CUDA or load checkpoint tensors."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT=next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH=_hh_source_path(ROOT, 'scripts/e2e/run_stopmix_duration_learning.py')
spec=importlib.util.spec_from_file_location('duration_learning_test',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
spec=importlib.util.spec_from_file_location('duration_regret_test',_hh_source_path(ROOT, 'scripts/e2e/run_stopmix_regret_research.py'))
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
MIRROR=_hh_source_path(ROOT, 'artifacts/training/2026-09-14/stopmix_regret_research_v1/remote_results')


def put(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(m.encoded(value))


def plan():
    original=m.read(MIRROR/'plan.json')
    p=m.plan_contract(b)|{k:original[k] for k in m.VIEW_KEYS}
    pins={'plan.json':'a'*64,'summary.json':'b'*64,'source/'+m.BASE_NAME:m.BASE_SHA}
    for run in m.RUNS:
        for name in m.RESUME_FILES:pins[f'{run}/training/{name}']='c'*64
    for i in range(293-len(pins)):pins[f'fixture_{i}.json']='d'*64
    p.update(declared_at_utc='2026-09-15T00:00:00Z',worker_sha256=m.digest(PATH),parent_file_sha256=pins,
        parent_plan_sha256=pins['plan.json'],parent_summary_sha256=pins['summary.json'])
    return p


def rows(steps):
    for step in range(1,steps+1):
        samples=step*4-step//287
        yield dict(global_step=step,epoch=(step-1)//287,samples_seen=samples,
            batch_domain_sample_counts={'carla':3 if step%287==0 else 4},domain_samples_seen={'carla':samples},
            loss=1.,regression_loss=.5,candidate_score_loss=.1,gradient_norm=18.)


def save_history(path,steps):
    path.parent.mkdir(parents=True,exist_ok=True)
    values=list(rows(steps));path.write_bytes(b''.join(m.encoded(r) for r in values))
    return values[-1]


def fixture(tmp_path,epoch=6,run=m.RUNS[0]):
    p=plan();parent=tmp_path/'parent';output=tmp_path/'output'
    original=m.read(MIRROR/run/'training/run.json')
    original['last_metrics']=save_history(parent/run/'training/metrics.jsonl',1540)
    put(parent/run/'training/run.json',original)
    cp=parent/run/'training/checkpoints/latest.pt';cp.parent.mkdir();cp.write_bytes(b'opaque checkpoint never deserialized')
    for name in m.RESUME_FILES:p['parent_file_sha256'][f'{run}/training/{name}']=m.digest(parent/run/'training'/name)
    resume=m.prepare_resume(p,output,parent,run,6)
    report=deepcopy(original);report['train_config']['max_steps']=epoch*287
    report['state']=dict(epoch=epoch,next_batch_index=0,global_step=epoch*287,samples_seen=epoch*1147,domain_samples_seen={'carla':epoch*1147})
    report['resumed_at_utc']='2026-09-15T00:01:00Z'
    item=m.epoch_root(output,6)/run
    report['last_metrics']=save_history(item/'training/metrics.jsonl',epoch*287)
    (item/'training/run.json').write_bytes(m.encoded(report))
    return p,parent,output,run,resume,report,item


def test_prospective_contract_and_epoch_major_commands(tmp_path):
    p=plan();m.validate_plan(p,m.digest(PATH),b)
    assert len(p['view_file_sha256'])==8926
    assert p['continuation_authorization']['exact_same_view_six_parents_only']
    assert p['continuation_authorization']['resume_original_corpus'] is False
    assert p['whole_timeout_seconds']==21600 and p['behavior_epochs']==[10,20]
    for epoch in m.EPOCHS:
        for run in m.RUNS:
            command=m.command(b,p,tmp_path,ROOT,tmp_path/'out',run,epoch,'train',tmp_path/'plan.json',PATH)
            assert command.count('--resume')==1
            assert command[command.index('--max-steps')+1]==str(epoch*287)
            assert command[command.index('--checkpoint-interval')+1]=='154'
            assert command[command.index('--candidate-regret-weight')+1]==str(b.ARMS[run.split('/')[1]])
            assert command[command.index('--run-dir')+1]==str(tmp_path/'out/epochs'/f'epoch_{epoch:03d}'/run/'training')
            assert '--limit-samples' not in command and 'test' not in command
    assert 20*287==5740 and 20*1147==22940 and 22940-6155==16785


@pytest.mark.parametrize('key,value',[
    ('epochs',[6,20]),('order','lineage_major'),('behavior_epochs',[6,10,20]),
    ('whole_timeout_seconds',21601),('automatic_promotion',True),('test_read_or_use',True),
    ('parent_steps',1541),('final_steps',5741),('runs',list(m.RUNS)[:1]),
    ('source_commit','f'*40),('declared_at_utc','2100-01-01T00:00:00Z'),('worker_sha256','a'*64)])
def test_fixed_plan_rejects_changes(key,value):
    p=plan();p[key]=value
    with pytest.raises(ValueError):m.validate_plan(p,m.digest(PATH),b)


def test_plan_inventory_and_test_denial():
    for mutation in ('parent','view','test','split','source','extra'):
        p=plan()
        if mutation=='parent':p['parent_file_sha256'].pop('source/'+m.BASE_NAME)
        elif mutation=='view':p['view_file_sha256'].pop(next(k for k in p['view_file_sha256'] if k!='dataset.json'))
        elif mutation=='test':
            key=next(k for k in p['view_file_sha256'] if k!='dataset.json')
            p['view_file_sha256']['episodes/test/payload.json']=p['view_file_sha256'].pop(key)
        elif mutation=='split':p['split_contract']['test']=p['split_contract']['val']
        elif mutation=='source':p['source_sha256'].pop(next(iter(p['source_sha256'])))
        else:p['silent_override']=True
        with pytest.raises(ValueError):m.validate_plan(p,m.digest(PATH),b)


def test_strict_json_paths_and_create_only(tmp_path):
    for value in ('{"a":1,"a":2}','{"a":NaN}','{"a":1e999}'):
        with pytest.raises(ValueError):m.decode(value)
    path=tmp_path/'x';m.write(path,{'a':1})
    with pytest.raises(FileExistsError):m.write(path,{})
    link=tmp_path/'link';link.symlink_to(path)
    with pytest.raises(ValueError):m.digest(link)
    with pytest.raises(ValueError):m.canonical(tmp_path/'../escape')


def test_exact_resume_copies_are_independent_and_history_preserved(tmp_path):
    p,parent,output,run,resume,report,item=fixture(tmp_path)
    result=m.verify_training(b,p,output,parent,run,6,resume)
    assert result['history']['new_steps']==182
    assert result['history']['new_samples_seen']==727
    assert result['history']['samples_seen']==6882
    assert result['history']['inherited_prefix_verified']
    assert m.read(parent/run/'training/run.json')['state']['global_step']==1540
    assert (parent/run/'training/checkpoints/latest.pt').stat().st_ino!=(item/'training/checkpoints/latest.pt').stat().st_ino
    with pytest.raises(FileExistsError):m.prepare_resume(p,output,parent,run,6)


@pytest.mark.parametrize('mutation',['prefix','steps','samples','nonfinite','last','checkpoint_interval','loss','runtime','state','resume'])
def test_resume_fail_closed(tmp_path,mutation):
    p,parent,output,run,resume,report,item=fixture(tmp_path)
    history_path=item/'training/metrics.jsonl'
    if mutation in ('prefix','steps','samples','nonfinite'):
        values=[m.decode(line) for line in history_path.read_bytes().splitlines()]
        if mutation=='prefix':values[0]['loss']=.9
        elif mutation=='steps':values[-1]['global_step']-=1
        elif mutation=='samples':values[-1]['batch_domain_sample_counts']['carla']=4
        else:values[-1]['gradient_norm']=float('inf')
        history_path.write_text(''.join(json.dumps(x)+'\n' for x in values))
    elif mutation=='last':report['last_metrics']['loss']=.9
    elif mutation=='checkpoint_interval':report['train_config']['checkpoint_interval']=287
    elif mutation=='loss':report['loss_config']['candidate_regret_weight']=0.
    elif mutation=='runtime':report['runtime']['torch']='foreign'
    elif mutation=='state':report['state']['epoch']=5
    else:report.pop('resumed_at_utc')
    (item/'training/run.json').write_bytes(m.encoded(report))
    with pytest.raises(ValueError):m.verify_training(b,p,output,parent,run,6,resume)


def test_full_epoch_checkpoint_is_opaque_and_native_cursor_is_not_guessed(tmp_path,monkeypatch):
    p,parent,output,run,resume,report,item=fixture(tmp_path)
    proof=m.verify_training(b,p,output,parent,run,6,resume)
    put(item/'train_completed.json',dict(status='COMPLETE',epoch=6,run=run,**proof))
    for stage in ('evaluate','audit'):put(item/f'{stage}_completed.json',dict(status='COMPLETE'))
    next_resume=m.prepare_resume(p,output,parent,run,7)
    assert next_resume['source_global_step']==1722 and next_resume['target_global_step']==2009
    next_item=m.epoch_root(output,7)/run
    assert (next_item/'training/checkpoints/latest.pt').read_bytes()==b'opaque checkpoint never deserialized'
    assert next_resume['inherited_history']['steps']==1722


def test_changed_source_checkpoint_cannot_be_copied(tmp_path):
    p,parent,output,run,resume,report,item=fixture(tmp_path)
    source=parent/run/'training/checkpoints/latest.pt';expected=m.digest(source)
    source.write_bytes(b'swapped')
    with pytest.raises(ValueError):m.copy_exact(source,tmp_path/'new',expected)
    link=tmp_path/'hardlink';__import__('os').link(source,link)
    with pytest.raises(ValueError):m.copy_exact(source,tmp_path/'new',m.digest(source))


def test_parent_inventory_refuses_unplanned_payload_before_read(tmp_path,monkeypatch):
    put(tmp_path/'safe.json',{});put(tmp_path/'unknown.json',{})
    expected={'safe.json':m.digest(tmp_path/'safe.json')};opened=[];original=m.digest
    monkeypatch.setattr(m,'digest',lambda p:(opened.append(Path(p).name),original(p))[1])
    with pytest.raises(ValueError,match='unplanned'):m.inventory(tmp_path,expected)
    assert 'unknown.json' not in opened


def test_progress_only_mutable_receipt_and_behavior_schedule(tmp_path):
    m.progress(tmp_path,{'stage':'train'});m.progress(tmp_path,{'stage':'evaluate'})
    assert m.read(tmp_path/'progress.json')=={'stage':'evaluate'}
    assert not (tmp_path/'progress.json.partial').exists()
    with pytest.raises(ValueError):m.command(b,plan(),tmp_path,ROOT,tmp_path/'out',m.RUNS[0],6,'behavior',tmp_path/'p',PATH)
    cmd=m.command(b,plan(),tmp_path,ROOT,tmp_path/'out',m.RUNS[0],10,'behavior',tmp_path/'p',PATH)
    assert cmd[cmd.index('--behavior-epoch')+1]=='10'


def test_module_import_does_not_load_torch_or_write_sealed_cache():
    import ast
    tree=ast.parse(PATH.read_text())
    assert not any(isinstance(n,ast.Import) and any(a.name=='torch' for a in n.names) for n in ast.walk(tree))
    assert m.sys.dont_write_bytecode is True
    assert m.digest(_hh_source_path(ROOT, 'scripts/e2e/run_stopmix_regret_research.py'))==m.BASE_SHA


def test_archived_helper_import_does_not_create_parent_files(tmp_path):
    target=tmp_path/'source'/m.BASE_NAME;target.parent.mkdir()
    target.write_bytes((_hh_source_path(ROOT, 'scripts/e2e/run_stopmix_regret_research.py')).read_bytes())
    before=m.inventory(tmp_path)
    loaded=m.load_base(tmp_path)
    assert loaded.MODEL_SHA==b.MODEL_SHA
    assert m.inventory(tmp_path)==before
    assert not (target.parent/'__pycache__').exists()
