"""HH_261001 - Prospective source-only V3 owner; identical V2 moving policy and resource gates."""

import argparse
import json
from scripts.e2e import run_owned_town07_low_speed_moving_20261001 as original
from scripts.e2e import town07_moving_external_sources_v3_20261001 as closure
# HH_261001 - Import the measurement wrapper so the unchanged owner archives it before launch.
from scripts.e2e import collect_town07_low_speed_moving_v3_20261001 as measurement_worker


def prepare(expected_sha):
    addendum = closure.verify_addendum(expected_sha)
    # HH_261001 - Preserve the frozen resource preflight and original command parsing first.
    plan = original.prepare()
    argv = plan['collector_command']
    name = 'scripts.e2e.collect_town07_low_speed_moving_20261001'
    closure.require(argv.count(name) == 1, 'original worker command differs')
    argv[argv.index(name)] = 'scripts.e2e.collect_town07_low_speed_moving_v3_20261001'
    offset = argv.index('--') + 1
    argv[offset:offset] = ['--source-addendum-sha256', expected_sha]
    plan.update(external_sources_v3_addendum_sha256=closure.sha(closure.ADDENDUM),
        external_sources_v3_addendum=addendum, external_agents_bytes_archived_before_server=True,
        measurement_only_worker_wrapper=True)
    return plan


def execute(plan):
    owner = original.owner
    saved_write = owner.write_json
    archived = False

    def archive_before_owner_plan(path, value):
        nonlocal archived
        if path == closure.task.RUN / 'owner_plan.json':
            closure.require(not archived, 'owner plan/archive repeated')
            closure.verify_addendum(value['external_sources_v3_addendum_sha256'])
            closure.archive_sources(closure.task.RUN, value['external_sources_v3_addendum']['external_sources'])
            closure.write_json(closure.task.RUN / 'external_agents_owner_before.json', dict(status='PASS',
                addendum_sha256=closure.sha(closure.ADDENDUM), source_snapshot=closure.source_snapshot()))
            archived = True
        return saved_write(path, value)

    owner.write_json = archive_before_owner_plan
    try:
        owner.execute(plan)
    finally:
        owner.write_json = saved_write
        if archived:
            try:
                evidence = closure.postcheck(closure.task.RUN)
            except (ValueError, OSError, KeyError) as error:
                evidence = dict(status='FAIL', error=str(error), training_data_approved=False)
            closure.write_json(closure.task.RUN / 'external_agents_owner_after.json', evidence)
    closure.require(archived, 'owned source archive was never established')
    # HH_261001 - Reproduce only the original owner summary, not its execute/main path a second time.
    result = closure.read(closure.task.RUN / 'owner_result.json')
    recorded = closure.read(closure.task.RUN / 'owner_plan.json')
    source_match = all(closure.sha(closure.task.ROOT / path) == digest
        and closure.sha(closure.task.RUN / 'source' / path) == digest for path, digest in recorded['source_sha256'].items())
    episode = closure.task.RUN / 'episode'
    if not episode.exists():
        episode = closure.task.RUN / 'episode.partial'
    manifest = closure.read(episode / 'manifest.json') if (episode / 'manifest.json').is_file() else {}
    moving = dict(schema='carla.owned_town07_low_speed_moving_outcome.v1', original_owner_result=result,
        source_hashes_match=source_match, native_status=manifest.get('status'), native_error=manifest.get('error'),
        task_complete=manifest.get('result', {}).get('task_complete', False), goal_reached=manifest.get('result', {}).get('goal_reached', False),
        task_result=manifest.get('result', {}).get('moving_task'),
        whole_recording_goal_stop_quality=manifest.get('result', {}).get('whole_recording_goal_stop_quality_diagnostic'),
        existing_qualification_authority_applies=False, training_data_approved=False, vehicle_control_approved=False,
        final_free_disk_bytes=closure.task.available_disk_bytes(), external_source_v3_result=evidence)
    moving['final_disk_floor_preserved'] = moving['final_free_disk_bytes'] >= closure.task.FREE_SPACE_FLOOR_BYTES
    closure.write_json(closure.task.RUN / 'owner_moving_result.json', moving)
    print(json.dumps(moving, indent=2))
    return 0 if (source_match and result['collector_exit_code'] == 0 and not result['cleanup_errors']
        and moving['task_complete'] and moving['final_disk_floor_preserved'] and evidence['status'] == 'PASS') else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    options = parser.add_mutually_exclusive_group()
    options.add_argument('--declare', action='store_true')
    options.add_argument('--execute', action='store_true')
    parser.add_argument('--addendum-sha256')
    args = parser.parse_args()
    if args.declare:
        closure.write_json(closure.ADDENDUM, closure.build_addendum())
        print(json.dumps(dict(addendum=str(closure.ADDENDUM), sha256=closure.sha(closure.ADDENDUM), simulator_started=False)))
        return 0
    plan = prepare(args.addendum_sha256)
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    return execute(plan)


if __name__ == '__main__':
    raise SystemExit(main())
