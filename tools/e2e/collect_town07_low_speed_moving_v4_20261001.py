"""HH_261001 - Source-bound worker for one unchanged-policy V4 moving capture."""

import argparse
import sys

from scripts.e2e import collect_town07_low_speed_moving_20261001 as original
from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from scripts.e2e import town07_moving_external_sources_v4_20261001 as closure
from scripts.e2e import town07_moving_external_sources_v3_20261001 as agents


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--v4-addendum-sha256', required=True)
    parser.add_argument('--v4-task-plan-sha256', required=True)
    parsed, original_argv = parser.parse_known_args(argv)
    addendum = closure.verify_addendum(parsed.v4_addendum_sha256, parsed.v4_task_plan_sha256)
    closure.require((task.RUN / 'owner_plan.json').is_file(), 'V4 owner source archive must precede worker')
    closure.verify_owner_archive(task.RUN, closure.read(task.RUN / 'owner_plan.json'), addendum)
    agents.verify_archive(task.RUN, addendum['external_sources'])
    before = agents.imported_snapshot()
    closure.write_json(task.RUN / 'external_agents_worker_before.json', dict(status='PASS',
        addendum_sha256=parsed.v4_addendum_sha256, import_snapshot=before))
    result = 1
    try:
        with task.activated():
            result = original.main(original_argv)
        return result
    finally:
        try:
            closure.verify_addendum(parsed.v4_addendum_sha256, parsed.v4_task_plan_sha256)
            after = agents.imported_snapshot()
            agents.verify_archive(task.RUN, addendum['external_sources'])
            closure.verify_owner_archive(task.RUN, closure.read(task.RUN / 'owner_plan.json'), addendum)
            evidence = dict(status='PASS', addendum_sha256=parsed.v4_addendum_sha256,
                original_worker_exit_code=result, import_snapshot=after)
        except (ValueError, OSError, KeyError) as error:
            evidence = dict(status='FAIL', error=str(error), original_worker_exit_code=result)
        closure.write_json(task.RUN / 'external_agents_worker_after.json', evidence)
        if evidence['status'] != 'PASS':
            raise ValueError('post-worker V4 source identity failed; original capture retained')


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
