"""HH_261001 - Path transport cannot change endpoint identities or silently choose files."""

from copy import deepcopy
from pathlib import Path
import inspect
import pytest
from scripts.e2e import transport_warmup_duration_w2_replay_20261001 as transport


def mapping():
    return dict(remote_training_root='/remote/training', local_training_root='/local/training',
        remote_parent_checkpoint='/remote/parent.pt', local_parent_checkpoint='/local/parent.pt',
        remote_export_root='/remote/training/exports', local_export_root='/local/training/exports')


def original():
    evidence={'/remote/training/file_'+str(i):f'{i:064x}' for i in range(34)}
    evidence['/remote/parent.pt']='f'*64
    return dict(evidence_file_sha256=evidence,weights='/remote/training/exports/model.npz',
        metadata='/remote/training/exports/model.json',weights_sha256='a'*64,metadata_sha256='b'*64,
        checkpoint_sha256='c'*64,fit={'global_step':11996},nested={'first_block':'fixed'})


def test_exact_path_only_mapping_and_inputs_immutable():
    old=original(); before=deepcopy(old)
    result,rows=transport.make_view(old,mapping())
    assert old==before and len(rows)==37 and len(result['evidence_file_sha256'])==35
    assert '/local/parent.pt' in result['evidence_file_sha256']
    assert result['weights']=='/local/training/exports/model.npz'
    for key in old.keys()-{'evidence_file_sha256','weights','metadata'}:assert result[key]==old[key]


def test_unknown_path_no_basename_search():
    old=original();old['evidence_file_sha256']['/elsewhere/file_0']=old['evidence_file_sha256'].pop('/remote/training/file_0')
    with pytest.raises(ValueError):transport.make_view(old,mapping())


def test_missing_parent_or_file_and_export_root_fail():
    for change in ('parent','count','exports'):
        old=original()
        if change=='parent':old['evidence_file_sha256']['/elsewhere/parent.pt']=old['evidence_file_sha256'].pop('/remote/parent.pt')
        if change=='count':old['evidence_file_sha256'].pop('/remote/training/file_0')
        if change=='exports':old['weights']='/remote/other/model.npz'
        with pytest.raises(ValueError):transport.make_view(old,mapping())


@pytest.mark.parametrize('bad',['relative/path','/remote/../other'])
def test_paths_are_explicit_absolute_normalized(bad):
    with pytest.raises(ValueError):transport.absolute(bad)


def test_destination_collision_rejected():
    values=mapping();values['local_parent_checkpoint']='/local/training/file_0'
    with pytest.raises(ValueError):transport.make_view(original(),values)


def test_all37_bytes_are_checked(tmp_path):
    path=tmp_path/'weights.npz';path.write_bytes(b'finite test bytes')
    row=dict(local=str(path),sha256=transport.replay.digest(path))
    transport.verify_records([row])
    row['sha256']='0'*64
    with pytest.raises(ValueError):transport.verify_records([row])


def test_no_local_checkpoint_loader_or_unpickle_fallback():
    source=inspect.getsource(transport)
    assert '_read_checkpoint_file' not in source and 'torch.load' not in source
    assert 'replay.load_endpoint' in source


def test_frozen_original_three_sources_and_addendum():
    transport.sources()


def test_transport_verification_rejects_nonpath_change(monkeypatch,tmp_path):
    old=original();old_path=tmp_path/'original.json';new_path=tmp_path/'local.json';receipt_path=tmp_path/'receipt.json'
    transport.replay.write_new(old_path,old)
    view,rows=transport.make_view(old,mapping());view['fit']['global_step']=2999
    transport.replay.write_new(new_path,view)
    receipt=dict(schema=transport.SCHEMA,status='VERIFIED_PATH_ONLY_TRANSPORT',addendum_sha256=transport.PLAN_SHA,
        original_replay_plan_sha256=transport.replay.PLAN_SHA,local_binding=str(new_path),
        local_binding_sha256=transport.replay.digest(new_path),copied_data_files=37,
        nonpath_payload_unchanged=True,checkpoint_decode_on_local=False,endpoint_reselected=False,
        training_data_approved=False,vehicle_control_approved=False,model_promoted=False,
        original_binding=str(old_path),original_binding_sha256=transport.replay.digest(old_path),
        explicit_mapping=mapping(),files=rows)
    transport.replay.write_new(receipt_path,receipt)
    with pytest.raises(ValueError,match='only explicitly declared paths'):
        transport.verify_transport(new_path,transport.replay.digest(new_path),receipt_path,transport.replay.digest(receipt_path))
