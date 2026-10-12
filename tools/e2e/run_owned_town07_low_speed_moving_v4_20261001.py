"""HH_261001 - Declare or own one fresh V4 moving capture with complete source closure."""

import argparse
import json

from scripts.e2e import run_owned_town07_low_speed_moving_20261001 as original
from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from scripts.e2e import town07_moving_external_sources_v4_20261001 as closure
from scripts.e2e import town07_moving_external_sources_v3_20261001 as agents


def prepare(plan_sha, addendum_sha):
    addendum = closure.verify_addendum(addendum_sha, plan_sha)
    with task.activated():
        plan = original.prepare()
    argv = plan['collector_command']
    old = 'scripts.e2e.collect_town07_low_speed_moving_20261001'
    closure.require(argv.count(old) == 1, 'original moving worker command differs')
    argv[argv.index(old)] = 'scripts.e2e.collect_town07_low_speed_moving_v4_20261001'
    offset = argv.index('--') + 1
    argv[offset:offset] = ['--v4-addendum-sha256', addendum_sha,
                           '--v4-task-plan-sha256', plan_sha]
    closure.require(argv[offset + 4] == str(task.RUN / 'episode'),
                    'V4 worker output differs from fresh plan')
    plan.update(external_sources_v4_addendum_sha256=addendum_sha,
        external_sources_v4_addendum=addendum,
        complete_declared_source_archive_required=True,
        external_agents_bytes_archived_before_server=True,
        measurement_only_worker_wrapper=True,
        interpretation='One fresh fixed 9 kph TRAIN moving capture; V4 QA pending, no data admission.')
    return plan


def execute(plan):
    owner = original.owner
    saved_write = owner.write_json
    archived = False

    def archive_before_owner_plan(path, value):
        nonlocal archived
        if path == task.RUN / 'owner_plan.json':
            closure.require(not archived, 'V4 source archive repeated')
            addendum = closure.verify_addendum(value['external_sources_v4_addendum_sha256'],
                                                 value['task_plan_sha256'])
            closure.archive_declared_sources(task.RUN, value, addendum)
            agents.archive_sources(task.RUN, addendum['external_sources'])
            closure.write_json(task.RUN / 'external_agents_owner_before.json', dict(status='PASS',
                addendum_sha256=value['external_sources_v4_addendum_sha256'],
                declared_source_count=len(addendum['source_sha256']),
                source_snapshot=agents.source_snapshot()))
            archived = True
        return saved_write(path, value)

    owner.write_json = archive_before_owner_plan
    try:
        with task.activated():
            owner.execute(plan)
    finally:
        owner.write_json = saved_write
        if archived:
            try:
                evidence = closure.postcheck(task.RUN, plan['external_sources_v4_addendum_sha256'],
                                             plan['task_plan_sha256'])
            except (ValueError, OSError, KeyError, AssertionError) as error:
                evidence = dict(status='FAIL', error_type=type(error).__name__, error=str(error),
                                training_data_approved=False)
            closure.write_json(task.RUN / 'external_agents_owner_after.json', evidence)
    closure.require(archived, 'V4 owner never established a complete source archive')
    result = closure.read(task.RUN / 'owner_result.json')
    recorded = closure.read(task.RUN / 'owner_plan.json')
    addendum = closure.read(closure.ADDENDUM)
    try:
        closure.verify_owner_archive(task.RUN, recorded, addendum)
        source_match = True
    except (ValueError, OSError, KeyError):
        source_match = False
    episode = task.RUN / 'episode'
    if not episode.exists():
        episode = task.RUN / 'episode.partial'
    manifest = closure.read(episode / 'manifest.json') if (episode / 'manifest.json').is_file() else {}
    moving = dict(schema='carla.owned_town07_low_speed_moving_outcome.v4',
        original_owner_result=result, source_hashes_match=source_match,
        native_status=manifest.get('status'), native_error=manifest.get('error'),
        task_complete=manifest.get('result', {}).get('task_complete', False),
        goal_reached=manifest.get('result', {}).get('goal_reached', False),
        task_result=manifest.get('result', {}).get('moving_task'),
        whole_recording_goal_stop_quality=manifest.get('result', {}).get('whole_recording_goal_stop_quality_diagnostic'),
        external_source_v4_result=evidence,
        existing_qualification_authority_applies=False,
        training_data_approved=False, vehicle_control_approved=False,
        final_free_disk_bytes=task.original.available_disk_bytes())
    moving['final_disk_floor_preserved'] = moving['final_free_disk_bytes'] >= task.original.FREE_SPACE_FLOOR_BYTES
    closure.write_json(task.RUN / 'owner_moving_result.json', moving)
    print(json.dumps(moving, indent=2))
    return 0 if (source_match and result['collector_exit_code'] == 0 and not result['cleanup_errors']
                 and result['ports_released'] and moving['task_complete']
                 and moving['final_disk_floor_preserved'] and evidence['status'] == 'PASS') else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--declare-task', action='store_true')
    action.add_argument('--declare-addendum', action='store_true')
    action.add_argument('--execute', action='store_true')
    parser.add_argument('--task-plan-sha256')
    parser.add_argument('--addendum-sha256')
    args = parser.parse_args()
    if args.declare_task:
        plan = task.build_plan(original.owner.MAPPING, original.owner.CALIBRATION)
        task.PLAN.parent.mkdir(parents=True, exist_ok=True)
        closure.write_json(task.PLAN, plan)
        print(json.dumps(dict(task_plan=str(task.PLAN), sha256=closure.sha(task.PLAN), simulator_started=False)))
        return 0
    if args.declare_addendum:
        addendum = closure.build_addendum(args.task_plan_sha256)
        closure.write_json(closure.ADDENDUM, addendum)
        print(json.dumps(dict(addendum=str(closure.ADDENDUM), sha256=closure.sha(closure.ADDENDUM),
                              simulator_started=False)))
        return 0
    plan = prepare(args.task_plan_sha256, args.addendum_sha256)
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    return execute(plan)


if __name__ == '__main__':
    raise SystemExit(main())
