"""HH_261001 - Join frozen moving-window QA with the prospective V3 source evidence."""

import argparse
from datetime import datetime
import gzip
import json
from pathlib import Path

from scripts.e2e import audit_town07_low_speed_moving_pending_20261001 as numerical
from scripts.e2e import town07_moving_external_sources_v3_20261001 as sources


SCHEMA = 'carla.town07_low_speed_moving_v3_necessary_checks.20261001.v1'
NUMERICAL_AUDITOR_SHA256 = '66dc3c3bdb4c6263606d245c84382c66817a879b63b6a7676aecdfe1eff6444f'
SOURCE_HELPER_SHA256 = 'd33cd5ac2831124fbd1dd5283c036e969830d3b6921c0654795ef73b5e208c1d'
ADDENDUM_SHA256 = 'd25e834f697670320317cec286dbe57f942df869f08b85b90ae425cd5c0f0574'
CLEAR = 'NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW'
require, sha, read = numerical.require, numerical.sha, numerical.read


def instant(value):
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(stamp.tzinfo is not None, 'source-evidence timestamp lacks timezone')
    return stamp


def recorded_import_paths():
    modules = {}
    for relative in sources.PINS:
        suffix = relative.removesuffix('.py').replace('/', '.').removesuffix('.__init__')
        name = 'agents' if suffix == '__init__' else 'agents.' + suffix
        modules[name] = str(sources.AGENTS_ROOT / relative)
    classes = {
        'agents.navigation.basic_agent.BasicAgent': modules['agents.navigation.basic_agent'],
        'agents.navigation.local_planner.LocalPlanner': modules['agents.navigation.local_planner'],
        'agents.navigation.global_route_planner.GlobalRoutePlanner': modules['agents.navigation.global_route_planner'],
        'agents.navigation.controller.VehiclePIDController': modules['agents.navigation.controller'],
    }
    return modules, classes


def validate_records(owner_plan, owner_before, worker_before, worker_after, owner_after,
                     owner_result, owner_moving, manifest, addendum, snapshot, recomputed):
    """HH_261001 - Inspect recorded identities; PASS labels alone carry no authority."""
    require(owner_plan.get('external_sources_v3_addendum_sha256') == ADDENDUM_SHA256
            and owner_plan.get('external_sources_v3_addendum') == addendum
            and owner_plan.get('external_agents_bytes_archived_before_server') is True
            and owner_plan.get('measurement_only_worker_wrapper') is True,
            'owner did not bind the prospective V3 source supplement')
    command = owner_plan.get('collector_command')
    module = 'scripts.e2e.collect_town07_low_speed_moving_v3_20261001'
    require(isinstance(command, list) and command.count('-m') == 1
            and command[command.index('-m') + 1] == module and command.count(module) == 1
            and command.count('--') == 1 and command.count('--source-addendum-sha256') == 1,
            'owner did not execute the single V3 worker')
    flag = command.index('--source-addendum-sha256')
    require(flag == command.index('--') + 1 and command[flag + 1] == ADDENDUM_SHA256
            and command[flag + 2] == str(sources.task.RUN / 'episode')
            and command[flag + 3] == str(sources.task.ROUTE),
            'worker command did not bind the fixed V3 addendum, episode and route')
    require(addendum['external_sources'] == snapshot and set(addendum['source_sha256']) == set(sources.SOURCES),
            'declaration source closure differs from fixed eight-file scope')
    for name, digest in addendum['source_sha256'].items():
        require(owner_plan['source_sha256'].get(name) == digest, 'V3 wrapper absent from owner source archive: ' + name)
    require(owner_before == dict(status='PASS', addendum_sha256=ADDENDUM_SHA256, source_snapshot=snapshot),
            'pre-server owner source record differs')
    modules, classes = recorded_import_paths()
    for item in (worker_before, worker_after):
        require(item.get('status') == 'PASS' and item.get('addendum_sha256') == ADDENDUM_SHA256,
                'worker pre/post source record failed')
        imported = item['import_snapshot']
        require(imported.get('schema') == 'carla.fixed_agents_import_snapshot.v1'
                and imported.get('status') == 'PASS' and imported.get('sources') == snapshot
                and imported.get('imported_module_paths') == modules
                and imported.get('inherited_classes') == classes,
                'actual imported module/class/source identity differs')
    declared = instant(addendum['declared_at_utc'])
    before = instant(worker_before['import_snapshot']['observed_at_utc'])
    after = instant(worker_after['import_snapshot']['observed_at_utc'])
    require(declared <= before <= instant(manifest['created_at']) <= after,
            'source declaration/import snapshots were not prospective and ordered')
    require(worker_after.get('original_worker_exit_code') == owner_result.get('collector_exit_code'),
            'worker/owner return codes disagree')
    require(owner_after == recomputed and owner_after['status'] == 'PASS'
            and owner_moving.get('external_source_v3_result') == owner_after,
            'owner postcheck and preserved moving outcome disagree')
    return dict(status='PASS', addendum_sha256=ADDENDUM_SHA256,
                archived_source_file_count=len(snapshot), imported_module_count=len(modules),
                capture_time_eight_file_agents_closure_verified=True,
                scope='Eight fixed CARLA agents Python files; no binary, OS, or complete dependency attestation.')


def inspect_sources(run):
    require(run.resolve() == sources.task.RUN, 'only the declared prospective run is in scope')
    require(sha(Path(numerical.__file__)) == NUMERICAL_AUDITOR_SHA256,
            'frozen numerical auditor differs')
    require(sha(Path(sources.__file__)) == SOURCE_HELPER_SHA256,
            'prospective source verifier differs')
    require(sha(sources.ADDENDUM) == ADDENDUM_SHA256,
            'prospective V3 source addendum differs')
    addendum = sources.verify_addendum(ADDENDUM_SHA256)
    snapshot = sources.source_snapshot()
    sources.verify_archive(run, snapshot)
    episode = run / 'episode'
    if not episode.exists():
        episode = run / 'episode.partial'
    files = {
        'owner_plan': run / 'owner_plan.json',
        'owner_before': run / 'external_agents_owner_before.json',
        'worker_before': run / 'external_agents_worker_before.json',
        'worker_after': run / 'external_agents_worker_after.json',
        'owner_after': run / 'external_agents_owner_after.json',
        'owner_result': run / 'owner_result.json',
        'owner_moving': run / 'owner_moving_result.json',
        'manifest': episode / 'manifest.json',
    }
    records = {name: read(path) for name, path in files.items()}
    recomputed = sources.postcheck(run)
    assessment = validate_records(**records, addendum=addendum, snapshot=snapshot, recomputed=recomputed)
    inputs = {str(path): sha(path) for path in files.values()}
    inputs[str(sources.ADDENDUM)] = ADDENDUM_SHA256
    for relative in snapshot:
        path = run / 'external_agents_source' / relative
        inputs[str(path)] = sha(path)
    require(all(sha(Path(path)) == digest for path, digest in inputs.items()),
            'V3 source evidence changed during audit')
    return assessment, inputs


def audit(run, owner_plan_sha, manifest_sha):
    require(run.resolve() == sources.task.RUN, 'only the declared prospective run is in scope')
    require(sha(Path(numerical.__file__)) == NUMERICAL_AUDITOR_SHA256,
            'frozen numerical auditor differs')
    # HH_261001 - Always run the original fixed-denominator numerical checks in full.
    numerical_summary, ledgers = numerical.audit(run, owner_plan_sha, manifest_sha)
    try:
        source_summary, source_inputs = inspect_sources(run)
    except (ValueError, AssertionError, KeyError, OSError, IndexError, TypeError) as error:
        source_summary, source_inputs = dict(status='FAIL', error_type=type(error).__name__, error=str(error)), {}
    clear = numerical_summary['status'] == CLEAR and source_summary['status'] == 'PASS'
    blocked = ('BLOCKED_NECESSARY_CHECKS' if numerical_summary['status'] != CLEAR
               else 'BLOCKED_SOURCE_PROVENANCE')
    return dict(schema=SCHEMA, status=CLEAR if clear else blocked,
        checked_run=str(run), owner_plan_sha256=owner_plan_sha,
        native_manifest_sha256=manifest_sha,
        numerical_audit=numerical_summary, v3_source_evidence=source_summary,
        v3_source_input_sha256=source_inputs, selected_window_count=numerical_summary['selected_window_count'],
        all_original_numerical_gates_passed=numerical_summary['status'] == CLEAR,
        training_data_approved=False, dataset_conversion_performed=False,
        admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW',
        original_sealed_15kph_authority_changed=False,
        authority_review_blocks=['New 9 kph fixed-window authority has not been approved.'],
        limitations=['Same-corridor fixed selected windows are not independent route successes.',
            'Original whole-recording failures and outside-protected failures remain in the numerical report.',
            'V3 source closure is limited to the eight recorded CARLA agents Python files.']), ledgers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=sources.task.RUN)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--native-manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.resolve().is_relative_to(args.run.resolve()),
            'fresh external audit output required')
    auditor_sources = {str(Path(module.__file__).resolve()): sha(Path(module.__file__))
                       for module in (numerical, sources)}
    auditor_sources[str(Path(__file__).resolve())] = sha(Path(__file__))
    try:
        summary, ledgers = audit(args.run, args.owner_plan_sha256, args.native_manifest_sha256)
    except (ValueError, AssertionError, KeyError, OSError, IndexError, TypeError) as error:
        summary, ledgers = dict(schema=SCHEMA, status='BLOCKED_UNASSESSED_EVIDENCE',
            error_type=type(error).__name__, error=str(error),
            training_data_approved=False, dataset_conversion_performed=False,
            admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW'), {}
        for relative in ('owner_result.json', 'owner_moving_result.json', 'episode/manifest.json',
                         'episode.partial/manifest.json', 'external_agents_owner_after.json'):
            path = args.run / relative
            if path.is_file():
                summary.setdefault('original_unmodified_records', {})[relative] = dict(
                    sha256=sha(path), record=read(path))
    require(all(sha(Path(path)) == digest for path, digest in auditor_sources.items()),
            'auditor source changed during execution')
    summary['audit_source_sha256'] = auditor_sources
    args.output.mkdir(parents=True)
    for name, rows in ledgers.items():
        with gzip.open(args.output / (name + '.jsonl.gz'), 'wt') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: summary[key] for key in ('status', 'training_data_approved', 'dataset_conversion_performed')}))
    return 0 if summary['status'] == CLEAR else 1


if __name__ == '__main__':
    raise SystemExit(main())
