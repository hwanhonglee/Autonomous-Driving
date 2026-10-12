"""HH_261001 - V3 source evidence cannot launder a failed fixed-window audit."""

from copy import deepcopy
from pathlib import Path
import pytest

from scripts.e2e import audit_town07_low_speed_moving_v3_20261001 as audit


@pytest.fixture
def records():
    snapshot = audit.sources.source_snapshot()
    addendum = audit.read(audit.sources.ADDENDUM)
    modules, classes = audit.recorded_import_paths()
    imported = dict(schema='carla.fixed_agents_import_snapshot.v1', status='PASS',
                    sources=snapshot, imported_module_paths=modules, inherited_classes=classes,
                    observed_at_utc='2026-10-01T00:01:00+00:00')
    command = ['python', '-m', 'scripts.e2e.collect_town07_low_speed_moving_v3_20261001', '--',
               '--source-addendum-sha256', audit.ADDENDUM_SHA256,
               str(audit.sources.task.RUN / 'episode'), str(audit.sources.task.ROUTE)]
    owner_plan = dict(external_sources_v3_addendum_sha256=audit.ADDENDUM_SHA256,
                      external_sources_v3_addendum=addendum,
                      external_agents_bytes_archived_before_server=True,
                      measurement_only_worker_wrapper=True,
                      collector_command=command, source_sha256=addendum['source_sha256'])
    owner_after = dict(status='PASS', addendum_sha256=audit.ADDENDUM_SHA256)
    return dict(owner_plan=owner_plan,
        owner_before=dict(status='PASS', addendum_sha256=audit.ADDENDUM_SHA256, source_snapshot=snapshot),
        worker_before=dict(status='PASS', addendum_sha256=audit.ADDENDUM_SHA256, import_snapshot=imported),
        worker_after=dict(status='PASS', addendum_sha256=audit.ADDENDUM_SHA256,
                          original_worker_exit_code=0,
                          import_snapshot=dict(deepcopy(imported), observed_at_utc='2026-10-01T00:03:00+00:00')),
        owner_after=owner_after, owner_result=dict(collector_exit_code=0),
        owner_moving=dict(external_source_v3_result=deepcopy(owner_after)),
        manifest=dict(created_at='2026-10-01T00:02:00+00:00'),
        addendum=addendum, snapshot=snapshot, recomputed=deepcopy(owner_after))


def test_complete_eight_file_source_records_pass(records):
    actual = audit.validate_records(**records)
    assert actual['status'] == 'PASS'
    assert actual['capture_time_eight_file_agents_closure_verified'] is True
    assert actual['archived_source_file_count'] == 8


@pytest.mark.parametrize('change', [
    lambda r: r['owner_plan']['collector_command'].__setitem__(2, 'scripts.e2e.collect_town07_low_speed_moving_20261001'),
    lambda r: r['worker_before']['import_snapshot']['inherited_classes'].__setitem__(
        'agents.navigation.basic_agent.BasicAgent', '/tmp/shadow_basic_agent.py'),
    lambda r: r['worker_after']['import_snapshot'].__setitem__('sources', {}),
    lambda r: r['owner_after'].__setitem__('source_files', 7),
    lambda r: r['owner_moving'].__setitem__('external_source_v3_result', {'status': 'FAIL'}),
    lambda r: r['worker_after'].__setitem__('original_worker_exit_code', 1),
    lambda r: r['manifest'].__setitem__('created_at', '2026-09-30T23:59:00+00:00'),
])
def test_tampered_records_fail_even_with_pass_labels(records, change):
    changed = deepcopy(records)
    change(changed)
    with pytest.raises((ValueError, IndexError)):
        audit.validate_records(**changed)


def test_numerical_failure_is_never_promoted_by_good_sources(records, monkeypatch):
    monkeypatch.setattr(audit, 'sha', lambda path: audit.NUMERICAL_AUDITOR_SHA256)
    monkeypatch.setattr(audit.numerical, 'audit', lambda *args: (
        dict(status='BLOCKED_NECESSARY_CHECKS', selected_window_count=321,
             scoped_native_failures={'protected': [{'frame': 500}]}), {'all_selected_windows': [{'frame': 500}]}))
    monkeypatch.setattr(audit, 'inspect_sources', lambda run: ({'status': 'PASS'}, {}))
    summary, ledgers = audit.audit(audit.sources.task.RUN, 'owner', 'native')
    assert summary['status'] == 'BLOCKED_NECESSARY_CHECKS'
    assert summary['all_original_numerical_gates_passed'] is False
    assert summary['numerical_audit']['scoped_native_failures']['protected'][0]['frame'] == 500
    assert ledgers['all_selected_windows'][0]['frame'] == 500
    assert summary['training_data_approved'] is False and summary['dataset_conversion_performed'] is False


def test_source_failure_blocks_clear_numerical_result_and_keeps_its_ledger(monkeypatch):
    monkeypatch.setattr(audit, 'sha', lambda path: audit.NUMERICAL_AUDITOR_SHA256)
    monkeypatch.setattr(audit.numerical, 'audit', lambda *args: (
        dict(status=audit.CLEAR, selected_window_count=321), {'all_selected_windows': [{'frame': 390}]}))
    def missing_source(run):
        raise ValueError('external archive missing')
    monkeypatch.setattr(audit, 'inspect_sources', missing_source)
    summary, ledgers = audit.audit(audit.sources.task.RUN, 'owner', 'native')
    assert summary['status'] == 'BLOCKED_SOURCE_PROVENANCE'
    assert summary['v3_source_evidence']['error'] == 'external archive missing'
    assert summary['all_original_numerical_gates_passed'] is True
    assert ledgers['all_selected_windows'][0]['frame'] == 390
    assert summary['admission_authority'] == 'NONE_NEW_SCOPE_REQUIRES_REVIEW'


def test_frozen_inputs_are_pinned_without_launch():
    assert audit.sha(Path(audit.numerical.__file__)) == audit.NUMERICAL_AUDITOR_SHA256
    assert audit.sha(Path(audit.sources.__file__)) == audit.SOURCE_HELPER_SHA256
    assert audit.sha(audit.sources.ADDENDUM) == audit.ADDENDUM_SHA256
