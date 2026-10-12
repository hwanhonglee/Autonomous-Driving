# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Metadata verification tests; no checkpoint deserialization, GPU, dataset, or network access."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import FunctionType

import pytest

ROOT=next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH=ROOT/'tools/e2e/verify_stopmix_duration_results.py'
spec=importlib.util.spec_from_file_location('duration_integrity_test',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def put(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(m.encoded(value))


def transport(tmp_path):
    root=tmp_path/'results';parent=tmp_path/'parent';root.mkdir();parent.mkdir()
    put(root/'plan.json',{});put(parent/'plan.json',{})
    runs=tuple(f'seed_{seed}/{arm}' for seed in (20260903,20260904,20260905) for arm in ('A_original_loss','B_cost_aware'))
    remote={'plan.json':m.sha(root/'plan.json')}
    for epoch in range(6,21):
        for run in runs:remote[f'epochs/epoch_{epoch:03d}/{run}/training/checkpoints/latest.pt']='a'*64
    parents={'plan.json':m.sha(parent/'plan.json')}
    for run in runs:parents[f'{run}/training/checkpoints/latest.pt']='b'*64
    for index in range(286):
        name=f'metadata_{index}.json';put(parent/name,dict(index=index));parents[name]=m.sha(parent/name)
    receipt=dict(schema='portable_e2e.stopmix_duration_remote_inventory.v1',status='COMPLETE_INVENTORY',
        remote_root=str(m.REMOTE_ROOT),remote_parent=str(m.REMOTE_PARENT),plan_sha256=m.PLAN_SHA,
        captured_at_utc='2026-09-15T16:00:00Z',verifier_sha256=m.sha(PATH),
        remote_file_sha256=remote,parent_file_sha256=parents,remote_inventory_unchanged=True,
        parent_inventory_unchanged=True,checkpoint_files_hashed=90,checkpoint_payloads_loaded=False,
        training_data_approved=False,vehicle_control_approved=False)
    path=tmp_path/'inventory.json';put(path,receipt)
    return root,parent,path,receipt


def test_complete_transport_hashes_all_metadata_and_never_opens_weights(tmp_path,monkeypatch):
    root,parent,path,receipt=transport(tmp_path)
    original=Path.open;opened=[]
    def guarded(self,*args,**kwargs):
        assert self.suffix!='.pt','checkpoint must never be opened locally'
        opened.append(self);return original(self,*args,**kwargs)
    monkeypatch.setattr(Path,'open',guarded)
    assert m.verify_transport(root,parent,path,m.sha(path))==receipt
    assert len(receipt['parent_file_sha256'])==293
    assert all(p.is_relative_to(root) or p.is_relative_to(parent) or p in (path,PATH) for p in opened)


@pytest.mark.parametrize('mutation',['missing_cp','extra_cp','source','status','count','parent','plan','approval','changed','extra','local_cp'])
def test_transport_fails_closed(tmp_path,mutation):
    root,parent,path,receipt=transport(tmp_path)
    if mutation=='missing_cp':receipt['remote_file_sha256'].pop(next(k for k in receipt['remote_file_sha256'] if k.endswith('.pt')))
    elif mutation=='extra_cp':receipt['remote_file_sha256']['elsewhere/latest.pt']='c'*64
    elif mutation=='source':receipt['verifier_sha256']='c'*64
    elif mutation=='status':receipt['status']='PARTIAL'
    elif mutation=='count':receipt['checkpoint_files_hashed']=89
    elif mutation=='parent':receipt['parent_file_sha256'].pop('metadata_0.json')
    elif mutation=='plan':receipt['plan_sha256']='c'*64
    elif mutation=='approval':receipt['training_data_approved']=True
    elif mutation=='changed':(root/'plan.json').write_text('{"tampered":true}')
    elif mutation=='extra':put(root/'unplanned.json',{})
    else:(root/'unplanned.pt').write_bytes(b'must not read')
    path.write_bytes(m.encoded(receipt))
    with pytest.raises(ValueError):m.verify_transport(root,parent,path,m.sha(path))


@pytest.mark.parametrize('pin',['short','A'*64,'f'*64])
def test_inventory_external_pin_required_before_mirror_read(tmp_path,monkeypatch,pin):
    root,parent,path,receipt=transport(tmp_path)
    monkeypatch.setattr(m,'read',lambda _:pytest.fail('untrusted inventory must not be parsed'))
    monkeypatch.setattr(m,'scan',lambda *_args,**_kwargs:pytest.fail('mirror must not be opened'))
    with pytest.raises(ValueError):m.verify_transport(root,parent,path,pin)


def test_hash_adapter_returns_only_attested_weights_without_open(tmp_path,monkeypatch):
    root,parent,path,receipt=transport(tmp_path)
    adapted=m.digest_adapter(root,parent,receipt['remote_file_sha256'],receipt['parent_file_sha256'])
    key=next(k for k in receipt['remote_file_sha256'] if k.endswith('.pt'))
    assert adapted(root/key)==adapted(m.REMOTE_ROOT/key)=='a'*64
    parent_key=next(k for k in receipt['parent_file_sha256'] if k.endswith('.pt'))
    assert adapted(parent/parent_key)==adapted(m.REMOTE_PARENT/parent_key)=='b'*64
    assert adapted(root/'plan.json')==m.sha(root/'plan.json')
    for bad in (tmp_path/'foreign.pt',root/'missing.pt',root/'../foreign.pt'):
        with pytest.raises(ValueError):adapted(bad)


def test_nonsymlink_no_unplanned_read_and_strict_json(tmp_path,monkeypatch):
    put(tmp_path/'safe.json',{});put(tmp_path/'unlisted.json',{})
    pins={'safe.json':m.sha(tmp_path/'safe.json')};opened=[];original=m.sha
    monkeypatch.setattr(m,'sha',lambda p:(opened.append(Path(p).name),original(p))[1])
    with pytest.raises(ValueError):m.scan(tmp_path,pins,True)
    assert 'unlisted.json' not in opened
    for value in ('{"a":1,"a":2}','{"a":NaN}','{"a":1e999}'):
        with pytest.raises(ValueError):m.decode(value)
    link=tmp_path/'link';link.symlink_to(tmp_path/'safe.json')
    with pytest.raises(ValueError):m.scan(tmp_path)
    with pytest.raises(ValueError):m.read(tmp_path/'anything.pt')


def test_inventory_only_requires_completion_and_two_stable_passes(tmp_path,monkeypatch):
    root=tmp_path/'remote';parent=tmp_path/'original';root.mkdir();parent.mkdir()
    put(parent/'source.json',{})
    plan=dict(parent_file_sha256=m.scan(parent));put(root/'plan.json',plan)
    monkeypatch.setattr(m,'REMOTE_ROOT',root);monkeypatch.setattr(m,'REMOTE_PARENT',parent)
    monkeypatch.setattr(m,'PLAN_SHA',m.sha(root/'plan.json'))
    put(root/'summary.json',dict(status='COMPLETE_NOT_PROMOTED',plan_sha256=m.PLAN_SHA))
    cp=root/'checkpoint.pt';cp.write_bytes(b'opaque bytes only hashed')
    result=m.inventory_only(root)
    assert result['checkpoint_files_hashed']==1
    assert result['remote_file_sha256']['checkpoint.pt']==m.sha(cp)
    assert result['checkpoint_payloads_loaded'] is False
    before=m.scan(root);original=m.scan;calls=[]
    def unstable(path,*args,**kwargs):
        calls.append(path);value=original(path,*args,**kwargs)
        if path==root and calls.count(root)==2:value['checkpoint.pt']='f'*64
        return value
    monkeypatch.setattr(m,'scan',unstable)
    with pytest.raises(ValueError,match='changed while hashing'):m.inventory_only(root)
    assert original(root)==before
    (root/'summary.json').write_bytes(m.encoded(dict(status='RUNNING',plan_sha256=m.PLAN_SHA)))
    with pytest.raises(ValueError,match='completion'):m.inventory_only(root)


def test_inventory_cli_does_not_overwrite_or_write_within_evidence(tmp_path,monkeypatch):
    root=tmp_path/'root';root.mkdir();out=tmp_path/'out.json'
    monkeypatch.setattr(m,'inventory_only',lambda _:dict(status='COMPLETE_INVENTORY'))
    assert m.main(['--inventory-only','--results',str(root),'--output',str(out)])==0
    with pytest.raises(ValueError,match='fresh'):m.main(['--inventory-only','--results',str(root),'--output',str(out)])
    with pytest.raises(ValueError,match='disjoint'):m.main(['--inventory-only','--results',str(root),'--output',str(root/'out.json')])


def test_frozen_sources_and_no_torch_import():
    import ast
    assert m.sha(ROOT/'tools/e2e/run_stopmix_duration_learning.py')==m.DURATION_SHA
    assert m.sha(ROOT/'tools/e2e/run_stopmix_regret_research.py')==m.BASE_SHA
    # HH_260906 - Historical manifest identity is unchanged; validate bytes at the real relocated path.
    assert m.sha(ROOT/'tools'/Path(m.SCALAR_PATH).relative_to('scripts'))==m.SCALAR_SHA
    imports=[node for node in ast.walk(ast.parse(PATH.read_text())) if isinstance(node,(ast.Import,ast.ImportFrom))]
    assert not any(getattr(node,'module','')=='torch' or any(alias.name=='torch' for alias in node.names) for node in imports)


def test_scalar_replay_retains_rejects_and_empty_hold_group():
    spec=importlib.util.spec_from_file_location('duration_scalar_fixture',ROOT/'learning/tests/test_summarize_stopmix_regret_research.py')
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    rows=[fixture.row(),fixture.row(1,selected=2)]
    measured=fixture.m.verify_rows(rows,2)
    assert measured['all_samples']['geometry']['selected_result']['geometry_reject_count']==1
    assert measured['target_motion_groups']['stationary_hold']['sample_count']==0
    assert measured['target_motion_groups']['stationary_hold']['metrics'] is None
    changed=deepcopy(rows);changed[1]['runtime_geometry']['selected_failure_codes']=[]
    with pytest.raises(ValueError):fixture.m.verify_rows(changed,2)
