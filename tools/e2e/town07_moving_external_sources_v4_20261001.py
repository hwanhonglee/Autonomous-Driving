"""HH_261001 - Prospective V4 capture and inherited-source closure."""

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from scripts.e2e import town07_moving_external_sources_v3_20261001 as agents


ADDENDUM = task.PLAN.parent / 'town07_low_speed_moving_v4_source_and_qa_addendum.json'
QA_SOURCES = (
    'tools/e2e/audit_town07_low_speed_moving_pending_20261001.py',
    'tools/e2e/audit_carla_turn_launch_raw_geometry.py',
    'tools/e2e/audit_town07_low_speed_moving_v4_20261001.py',
    'docs/assets/validation/2026-10-01/town07_low_speed_moving_V4_QA_PROTOCOL.md',
)
sha, read, require = task.original.sha, task.original.read, task.original.require


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def declared_paths():
    return tuple(dict.fromkeys((*task.SOURCE_PATHS, *QA_SOURCES)))


def build_addendum(plan_sha):
    plan = task.verify_plan(plan_sha)
    require(not task.RUN.exists() and not ADDENDUM.exists(), 'fresh V4 addendum and capture required')
    return dict(schema='carla.town07_moving_source_and_qa_addendum.20261001.v4',
        declared_at_utc=datetime.now(timezone.utc).isoformat(),
        fixed_task_plan=str(task.PLAN), fixed_task_plan_sha256=plan_sha,
        output=str(task.RUN), external_root=str(agents.AGENTS_ROOT),
        external_sources=agents.source_snapshot(),
        source_sha256={name: sha(task.ROOT / name) for name in declared_paths()},
        fixed_selected_window_count=327, fixed_selected_progress_m=[40., 120.],
        controller_or_physics_changed=False, route_or_window_changed=False,
        numerical_thresholds_changed=False, resource_thresholds_changed=False,
        original_v3_records_immutable=True, capture_policy_changed=False,
        first_capture_entrypoint='scripts.e2e.run_owned_town07_low_speed_moving_v4_20261001',
        qa_entrypoint='scripts.e2e.audit_town07_low_speed_moving_v4_20261001',
        training_data_approved=False, admission_status='PENDING_QA')


def verify_addendum(expected_sha, expected_plan_sha):
    require(isinstance(expected_sha, str) and len(expected_sha) == 64 and sha(ADDENDUM) == expected_sha,
            'caller-pinned V4 addendum required')
    plan = task.verify_plan(expected_plan_sha)
    addendum = read(ADDENDUM)
    require(addendum['schema'] == 'carla.town07_moving_source_and_qa_addendum.20261001.v4'
            and addendum['fixed_task_plan_sha256'] == expected_plan_sha
            and addendum['fixed_task_plan'] == str(task.PLAN)
            and addendum['output'] == str(task.RUN)
            and addendum['external_root'] == str(agents.AGENTS_ROOT)
            and addendum['fixed_selected_window_count'] == 327
            and addendum['fixed_selected_progress_m'] == [40., 120.]
            and addendum['admission_status'] == 'PENDING_QA'
            and addendum['training_data_approved'] is False,
            'V4 addendum scope differs')
    require(addendum['external_sources'] == agents.source_snapshot(), 'inherited agents source changed')
    require(set(addendum['source_sha256']) == set(declared_paths()), 'V4 declared source membership differs')
    for name, digest in addendum['source_sha256'].items():
        path = task.ROOT / name
        require(not path.is_symlink() and path.is_file() and sha(path) == digest,
                'V4 declared source changed: ' + name)
    for name, digest in plan['source_sha256'].items():
        require(addendum['source_sha256'][name] == digest, 'capture source and QA addendum disagree: ' + name)
    return addendum


def archive_declared_sources(run, owner_plan, addendum):
    """HH_261001 - Close every declared source before owner writes its plan or starts CARLA."""
    require(run == task.RUN and run.is_dir(), 'only fresh V4 output may receive source archive')
    declared = addendum['source_sha256']
    for name, digest in declared.items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe declared source path')
        source, archived = task.ROOT / relative, run / 'source' / relative
        require(not source.is_symlink() and source.is_file() and sha(source) == digest,
                'declared source changed before archive: ' + name)
        if not archived.exists():
            archived.parent.mkdir(parents=True, exist_ok=True)
            with source.open('rb') as src, archived.open('xb') as dst:
                shutil.copyfileobj(src, dst)
        require(not archived.is_symlink() and archived.is_file() and sha(archived) == digest,
                'declared archive differs: ' + name)
        existing = owner_plan['source_sha256'].get(name)
        require(existing in (None, digest), 'owner source disagrees with declared source: ' + name)
        owner_plan['source_sha256'][name] = digest
    require(set(declared).issubset(owner_plan['source_sha256']), 'declared source archive incomplete')


def verify_owner_archive(run, owner_plan, addendum):
    require(run == task.RUN, 'wrong V4 source archive')
    require(set(addendum['source_sha256']).issubset(owner_plan['source_sha256']),
            'V4 source missing from owner archive')
    for name, digest in owner_plan['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts,
                'unsafe owner source archive path')
        current, archived = task.ROOT / relative, run / 'source' / relative
        require(not current.is_symlink() and not archived.is_symlink()
                and current.is_file() and archived.is_file()
                and sha(current) == sha(archived) == digest,
                'owner source changed or archive missing: ' + name)
    for name, digest in addendum['source_sha256'].items():
        require(owner_plan['source_sha256'][name] == digest,
                'declared V4 source differs from owner: ' + name)


def postcheck(run, expected_addendum_sha, expected_plan_sha):
    addendum = verify_addendum(expected_addendum_sha, expected_plan_sha)
    owner_plan = read(run / 'owner_plan.json')
    require(owner_plan['external_sources_v4_addendum_sha256'] == expected_addendum_sha
            and owner_plan['task_plan_sha256'] == expected_plan_sha,
            'owner did not bind V4 declarations')
    verify_owner_archive(run, owner_plan, addendum)
    agents.verify_archive(run, addendum['external_sources'])
    before, after = (read(run / name) for name in
                     ('external_agents_worker_before.json', 'external_agents_worker_after.json'))
    for item in (before, after):
        require(item['status'] == 'PASS' and item['addendum_sha256'] == expected_addendum_sha
                and item['import_snapshot']['sources'] == addendum['external_sources'],
                'worker inherited-source snapshot differs')
        require(datetime.fromisoformat(addendum['declared_at_utc'])
                <= datetime.fromisoformat(item['import_snapshot']['observed_at_utc']),
                'source declaration was not prospective')
    require(before['import_snapshot']['imported_module_paths'] == after['import_snapshot']['imported_module_paths']
            and before['import_snapshot']['inherited_classes'] == after['import_snapshot']['inherited_classes'],
            'worker inherited classes or paths changed')
    return dict(schema='carla.town07_moving_source_postcheck.v4', status='PASS',
        addendum_sha256=expected_addendum_sha, fixed_task_plan_sha256=expected_plan_sha,
        declared_workspace_source_files=len(addendum['source_sha256']),
        archived_owner_source_files=len(owner_plan['source_sha256']),
        inherited_agents_source_files=len(addendum['external_sources']),
        complete_declared_source_archive=True, capture_time_inherited_class_identity_verified=True,
        training_data_approved=False,
        notice='Source identity only; numerical QA and data admission remain separate.')
