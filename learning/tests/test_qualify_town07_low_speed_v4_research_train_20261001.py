"""HH_261001 - Keep V4 research admission explicit, one-episode-only and denial preserving."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from scripts.e2e import qualify_town07_low_speed_v4_research_train_20261001 as authority


def pending_manifest():
    # HH_261001 - Model only genuine prospective PENDING fields, never a source approval flag.
    pending = dict(admission_status='PENDING_QA', dataset_conversion_performed=False,
                   vehicle_control_approved=False)
    return dict(status='complete', cleanup=dict(completed=True), **pending,
        result=dict(**pending, task_complete=True, goal_reached=False,
            collision_event_count=0, lane_invasion_event_count=0, moving_task=dict(pending)),
        capture_contract=dict(agent_initialization=dict(pending),
            moving_task=dict(plan_sha256=authority.PLAN_SHA, cohort_slot_id=authority.task.SLOT,
                dataset_split_intent='train', selected_progress_m=[40., 120.])))


def test_actual_pending_does_not_mean_denied_or_approved():
    value = pending_manifest()
    before = deepcopy(value)
    authority.scope_manifest(value)
    assert value == before and 'training_data_approved' not in value


@pytest.mark.parametrize('path', [(), ('result',), ('capture_contract',),
    ('capture_contract', 'agent_initialization'), ('result', 'goal_stop_quality')])
@pytest.mark.parametrize('marker,value', [('training_data_approved', False), ('development_only', True)])
def test_explicit_native_denial_is_never_overridden(path, marker, value):
    manifest = pending_manifest()
    item = manifest
    for key in path:
        item = item.setdefault(key, {})
    item[marker] = value
    with pytest.raises((authority.adapter.AdapterError, authority.contract.ContractError), match='explicitly prohibits'):
        authority.scope_manifest(manifest)


@pytest.mark.parametrize('change', ['goal', 'cleanup', 'events', 'scope', 'pending'])
def test_altered_capture_scope_is_rejected(change):
    value = pending_manifest()
    if change == 'goal': value['result']['goal_reached'] = True
    if change == 'cleanup': value['cleanup']['completed'] = False
    if change == 'events': value['result']['collision_event_count'] = 1
    if change == 'scope': value['capture_contract']['moving_task']['selected_progress_m'] = [10., 30.]
    if change == 'pending': value['result']['dataset_conversion_performed'] = True
    with pytest.raises(authority.adapter.AdapterError):
        authority.scope_manifest(value)


@pytest.mark.parametrize('value', ['', 'A'*64, 'g'*64, None, 1, 'a'*63])
def test_external_hash_cannot_be_guessed(value):
    with pytest.raises(authority.adapter.AdapterError, match='64-hex'):
        authority.hash_value(value)


def test_external_digest_validates():
    assert authority.hash_value('a'*64) == 'a'*64


def test_receipt_or_execution_flag_required_before_evidence(monkeypatch):
    monkeypatch.setattr(authority, 'evidence', lambda **_: pytest.fail('evidence read without authority'))
    with pytest.raises(authority.adapter.AdapterError, match='paired'):
        authority.authorize(SimpleNamespace())
    with pytest.raises(authority.adapter.AdapterError, match='explicit'):
        authority.authorize(SimpleNamespace(moving_qualification='receipt', moving_qualification_sha256='a'*64))


def test_fixed_dataset_scope_cannot_be_renamed():
    assert authority.PREPARED.name == authority.DATASET_ID
    assert authority.task.SLOT == 'T07-LOW-CRUISE-20261001-V4'
    assert authority.LEDGER_SHA.keys() == {'all_camera_futures.jsonl.gz','all_control_checks.jsonl.gz',
        'all_image_checks.jsonl.gz','all_native.jsonl.gz','all_projection_rows.jsonl.gz',
        'all_selected_windows.jsonl.gz'}


def test_safe_rejects_symlink(tmp_path):
    target = tmp_path / 'actual'
    target.mkdir()
    link = tmp_path / 'alias'
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(authority.adapter.AdapterError, match='nonsymlink'):
        authority.safe(link / 'receipt.json', under_root=False)


def test_safe_rejects_lexical_traversal(tmp_path):
    with pytest.raises(authority.adapter.AdapterError, match='nonsymlink'):
        authority.safe(tmp_path / 'future' / '..' / 'receipt.json', under_root=False)


def test_recheck_rejects_changed_receipt_before_inputs(monkeypatch):
    monkeypatch.setattr(authority, 'sha', lambda _: 'b'*64)
    with pytest.raises(authority.adapter.AdapterError, match='receipt changed'):
        authority.recheck(dict(receipt_path='receipt', receipt_sha256='a'*64,
            document=dict(input_sha256={}, source_sha256={})))
