"""HH_261001 - Measurement-only inherited-source wrapper around the frozen moving worker."""

import argparse
import sys
from scripts.e2e import town07_moving_external_sources_v3_20261001 as closure
from scripts.e2e import collect_town07_low_speed_moving_20261001 as original


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--source-addendum-sha256', required=True)
    parsed, original_argv = parser.parse_known_args(argv)
    closure.verify_addendum(parsed.source_addendum_sha256)
    closure.require((closure.task.RUN / 'owner_plan.json').is_file(), 'owned V3 source archive must precede worker')
    closure.verify_archive(closure.task.RUN, closure.read(closure.ADDENDUM)['external_sources'])
    before = closure.imported_snapshot()
    closure.write_json(closure.task.RUN / 'external_agents_worker_before.json', dict(status='PASS',
        addendum_sha256=closure.sha(closure.ADDENDUM), import_snapshot=before))
    result = 1
    try:
        result = original.main(original_argv)
        return result
    finally:
        try:
            closure.verify_addendum(parsed.source_addendum_sha256)
            after = closure.imported_snapshot()
            closure.verify_archive(closure.task.RUN, closure.read(closure.ADDENDUM)['external_sources'])
            evidence = dict(status='PASS', addendum_sha256=closure.sha(closure.ADDENDUM),
                            original_worker_exit_code=result, import_snapshot=after)
        except (ValueError, OSError) as error:
            evidence = dict(status='FAIL', error=str(error), original_worker_exit_code=result)
        closure.write_json(closure.task.RUN / 'external_agents_worker_after.json', evidence)
        if evidence['status'] != 'PASS':
            raise ValueError('post-worker inherited source identity failed; original capture retained')


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
