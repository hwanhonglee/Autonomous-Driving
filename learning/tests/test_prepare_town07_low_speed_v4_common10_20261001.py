"""HH_261001 - Verify research-only serial conversion dispatch and immutable native metadata."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import prepare_town07_low_speed_v4_common10_20261001 as driver


@pytest.mark.parametrize('args', [SimpleNamespace(), SimpleNamespace(moving_qualification='receipt'),
    SimpleNamespace(moving_qualification='receipt', moving_qualification_sha256='a'*64)])
def test_unqualified_conversion_has_no_generic_fallback(args, monkeypatch):
    monkeypatch.setattr(driver.base, 'prepare_dataset', lambda _: pytest.fail('generic conversion called'))
    with pytest.raises(driver.base.AdapterError, match='paired'):
        driver.prepare_dataset(args)


@pytest.mark.parametrize('fail', [False, True])
def test_serial_dispatch_restores_all_state(monkeypatch, fail):
    old_module = driver.importlib.import_module('scripts.e2e.qualify_carla_moving_common10')
    old_attribute = getattr(e2e, 'qualify_carla_moving_common10', old_module)
    old_writer = driver.base._write_json
    args = SimpleNamespace(moving_qualification='receipt', moving_qualification_sha256='a'*64,
                           execute_reviewed_research_conversion=True)

    def convert(value):
        from scripts.e2e import qualify_carla_moving_common10 as actual
        assert actual is driver.authority and value is args
        if fail: raise ValueError('test rejection')
        return 'result'

    monkeypatch.setattr(driver.base, 'prepare_dataset', convert)
    if fail:
        with pytest.raises(ValueError, match='test rejection'):
            driver.prepare_dataset(args)
    else:
        assert driver.prepare_dataset(args) == 'result'
    assert e2e.qualify_carla_moving_common10 is old_attribute
    assert driver.base._write_json is old_writer


def test_writer_changes_only_new_identity_and_selection(tmp_path, monkeypatch):
    recorded = {}
    monkeypatch.setattr(driver.base, '_write_json', lambda path, value: recorded.setdefault(path.name, value))
    capture = dict(admission_status='PENDING_QA', vehicle_control_approved=False)
    original = dict(adapter_id='carla_fixed_moving_task_to_common10_v1',
                    future_selection_policy='old text', native_capture_contract=capture, native_result=dict(goal_reached=False))
    before = deepcopy(original)
    with driver.bound_research_authority():
        driver.base._write_json(tmp_path / 'collection_config.json', original)
        driver.base._write_json(tmp_path / 'native_manifest.json', original)
    assert original == before
    assert recorded['collection_config.json']['native_capture_contract'] == capture
    assert recorded['collection_config.json']['native_result'] == before['native_result']
    assert recorded['collection_config.json']['adapter_id'] == driver.ADAPTER_ID
    assert recorded['collection_config.json']['future_selection_policy'] == 'prospective_40_to_120m_all_327_anchors_no_quality_filter'
    assert recorded['native_manifest.json'] is original


@pytest.mark.parametrize('flags', [[], ['--execute-reviewed-research-conversion']*2])
def test_cli_requires_one_explicit_research_execution_flag(flags, monkeypatch):
    monkeypatch.setattr(driver.base, '_parse_args', lambda _: pytest.fail('generic args reached'))
    with pytest.raises(driver.base.AdapterError, match='exactly one'):
        driver.main(flags)
