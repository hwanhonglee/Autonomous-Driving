"""HH_261001 - Source-only supplement preserves frozen policy and fails closed on shadowing/tampering."""

from copy import deepcopy
from pathlib import Path
import pytest

from scripts.e2e import town07_moving_external_sources_v3_20261001 as closure
from scripts.e2e import collect_town07_low_speed_moving_v3_20261001 as worker
from scripts.e2e import run_owned_town07_low_speed_moving_v3_20261001 as owner


def test_fixed_package_eight_files_69282_bytes():
    snapshot = closure.source_snapshot()
    assert len(snapshot) == 8 and sum(row['size_bytes'] for row in snapshot.values()) == 69282
    assert closure.sha(closure.task.PLAN) == closure.PLAN_SHA
    for name, digest in closure.read(closure.task.PLAN)['source_sha256'].items():
        assert closure.sha(closure.task.ROOT / name) == digest


def test_actual_imported_paths_and_classes_no_simulator(monkeypatch):
    egg = closure.AGENTS_ROOT.parent / 'dist/carla-0.9.15-py3.10-linux-x86_64.egg'
    monkeypatch.syspath_prepend(str(egg))
    snapshot = closure.imported_snapshot()
    assert snapshot['status'] == 'PASS' and len(snapshot['inherited_classes']) == 4
    import agents.navigation.basic_agent as basic
    monkeypatch.setattr(basic, 'BasicAgent', object)
    with pytest.raises(ValueError):
        closure.imported_snapshot()


def test_safe_paths_reject_escape_symlink_and_unlisted(tmp_path):
    for name in ('../outside.py', '/tmp/outside.py', 'navigation/behavior_agent.py'):
        with pytest.raises(ValueError):
            closure.safe_path(tmp_path, name)
    (tmp_path / '__init__.py').symlink_to(closure.AGENTS_ROOT / '__init__.py')
    with pytest.raises(ValueError):
        closure.safe_path(tmp_path, '__init__.py')


def test_create_only_archive_and_all_bytes_rechecked(tmp_path):
    run = tmp_path / 'owned_run'
    run.mkdir()
    snapshot = closure.source_snapshot()
    closure.archive_sources(run, snapshot)
    closure.verify_archive(run, snapshot)
    with pytest.raises(ValueError):
        closure.archive_sources(run, snapshot)
    # HH_261001 - Synthetic archive corruption does not modify any package source.
    (run / 'external_agents_source/__init__.py').write_text('changed')
    with pytest.raises(ValueError):
        closure.verify_archive(run, snapshot)


def test_prepare_preserves_original_command_and_resource_preflight(monkeypatch):
    argv = ['python', 'workspace.py', '-m', 'scripts.e2e.collect_town07_low_speed_moving_20261001',
            '--', 'episode', 'route', '--target-speed-kmh', '9', '--max-duration-sec', '180']
    original_argv = list(argv)
    original_plan = dict(collector_command=argv, resource_guard={'floor': 5}, selected_progress_m=[40., 120.])
    calls = []
    monkeypatch.setattr(owner.original, 'prepare', lambda: calls.append('original_resource_preflight') or original_plan)
    monkeypatch.setattr(closure, 'verify_addendum', lambda digest: {'digest': digest})
    monkeypatch.setattr(closure, 'sha', lambda path: 'a' * 64)
    actual = owner.prepare('a' * 64)
    assert calls == ['original_resource_preflight']
    stripped = list(actual['collector_command'])
    index = stripped.index('--source-addendum-sha256')
    del stripped[index:index + 2]
    stripped[3] = original_argv[3]
    assert stripped == original_argv
    assert actual['resource_guard'] == {'floor': 5} and actual['selected_progress_m'] == [40., 120.]


def test_worker_logs_both_sides_and_preserves_original_exception(tmp_path, monkeypatch):
    monkeypatch.setattr(closure.task, 'RUN', tmp_path)
    (tmp_path / 'owner_plan.json').write_text('{}')
    monkeypatch.setattr(closure, 'verify_addendum', lambda digest: {})
    monkeypatch.setattr(closure, 'verify_archive', lambda *args: None)
    monkeypatch.setattr(closure, 'read', lambda path: {'external_sources': {}})
    monkeypatch.setattr(closure, 'sha', lambda path: 'a' * 64)
    monkeypatch.setattr(closure, 'imported_snapshot', lambda: {'status': 'PASS', 'sources': {}})
    def failure(argv):
        assert argv == ['episode', 'route', '--target-speed-kmh', '9']
        raise ValueError('original retained failure')
    monkeypatch.setattr(worker.original, 'main', failure)
    with pytest.raises(ValueError, match='original retained failure'):
        worker.main(['--source-addendum-sha256', 'a' * 64, 'episode', 'route', '--target-speed-kmh', '9'])
    assert (tmp_path / 'external_agents_worker_before.json').is_file()
    assert (tmp_path / 'external_agents_worker_after.json').is_file()


def test_caller_pin_rejects_changed_addendum(tmp_path, monkeypatch):
    path = tmp_path / 'addendum.json'
    path.write_text('{}')
    monkeypatch.setattr(closure, 'ADDENDUM', path)
    with pytest.raises(ValueError, match='caller-pinned'):
        closure.verify_addendum('0' * 64)


def test_declaration_preserves_pending_no_control_or_resource_change():
    plan = closure.build_addendum()
    assert plan['task_admission_status'] == 'PENDING_QA' and plan['training_data_approved'] is False
    assert plan['fixed_task_plan_sha256'] == closure.PLAN_SHA
    assert all(plan[key] is False for key in ('capture_policy_changed', 'route_or_window_changed',
        'numerical_gates_changed', 'resource_thresholds_changed'))


def test_owner_archives_before_simulator_boundary_and_does_not_change_original_result(tmp_path, monkeypatch):
    run = tmp_path / 'owned'
    monkeypatch.setattr(closure.task, 'RUN', run)
    monkeypatch.setattr(closure.task, 'available_disk_bytes', lambda: 6 * 1024**3)
    declaration = tmp_path / 'addendum.json'
    declaration.write_text('{}')
    monkeypatch.setattr(closure, 'ADDENDUM', declaration)
    snapshot = closure.source_snapshot()
    plan = dict(source_sha256={}, external_sources_v3_addendum_sha256=closure.sha(declaration),
                external_sources_v3_addendum=dict(external_sources=snapshot))
    monkeypatch.setattr(closure, 'verify_addendum', lambda digest: plan['external_sources_v3_addendum'])
    monkeypatch.setattr(closure, 'postcheck', lambda path: {'status': 'PASS'})
    original_result = dict(status='FINISHED_RESEARCH_ONLY', collector_exit_code=0, cleanup_errors=[], ports_released=True, error=None)
    boundary = []
    def fake_owned_execute(value):
        run.mkdir()
        owner.original.owner.write_json(run / 'owner_plan.json', value)
        closure.verify_archive(run, snapshot)
        assert (run / 'external_agents_owner_before.json').exists()
        boundary.append('server_would_start_after_archive')
        (run / 'episode').mkdir()
        closure.write_json(run / 'episode/manifest.json', dict(status='complete', result=dict(task_complete=True, goal_reached=False, moving_task={})))
        closure.write_json(run / 'owner_result.json', original_result)
    monkeypatch.setattr(owner.original.owner, 'execute', fake_owned_execute)
    assert owner.execute(plan) == 0
    assert boundary == ['server_would_start_after_archive']
    assert closure.read(run / 'owner_result.json') == original_result
    result = closure.read(run / 'owner_moving_result.json')
    assert result['training_data_approved'] is False and result['goal_reached'] is False


def test_owner_does_not_write_into_unowned_existing_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(closure.task, 'RUN', tmp_path)
    def reject_existing(plan):
        raise RuntimeError('fresh diagnostic output required')
    monkeypatch.setattr(owner.original.owner, 'execute', reject_existing)
    with pytest.raises(RuntimeError, match='fresh diagnostic'):
        owner.execute({})
    assert not (tmp_path / 'external_agents_owner_after.json').exists()
