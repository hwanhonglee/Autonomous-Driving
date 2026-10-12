"""HH_261001 - Full fixed-window V4 QA with corrected physical-speed and future ordering."""

import argparse
from datetime import datetime
import gzip
import json
import math
from pathlib import Path

from scripts.e2e import audit_town07_low_speed_moving_pending_20261001 as frozen
from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import collect_town07_low_speed_start_v2_20261001 as v2
from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from scripts.e2e import town07_moving_external_sources_v4_20261001 as closure
from scripts.e2e import town07_moving_external_sources_v3_20261001 as agents


SCHEMA = 'carla.town07_low_speed_moving_v4_necessary_checks.20261001.v1'
CLEAR = 'NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW'
EXPECTED_SELECTED_WINDOWS = 327
SPEED_ABSOLUTE_TOLERANCE_MPS = 1e-9
ORIGINAL_GEOMETRY_FUTURES = frozen.geometry.geometry_futures
require, sha, read = frozen.require, frozen.sha, frozen.read


def center_speed_mps(state, wheelbase_m):
    """HH_261001 - Undo recorded rear-reference yaw-rate correction to recover actor-center speed."""
    require(wheelbase_m == 2.85, 'fixed Prius wheelbase differs')
    values = (state['vx'], state['vy'], state['yaw_rate'], *state['world_velocity_carla'])
    require(len(state['world_velocity_carla']) == 3
            and all(type(value) in (int, float) and math.isfinite(value) for value in values),
            'center-speed source is malformed')
    reconstructed = math.hypot(state['vx'], state['vy'] + wheelbase_m / 2 * state['yaw_rate'])
    world_center = math.hypot(*state['world_velocity_carla'][:2])
    require(abs(reconstructed - world_center) <= SPEED_ABSOLUTE_TOLERANCE_MPS,
            'rear-reference correction does not match original actor-center velocity')
    return reconstructed


def control_audit(rows, receipts, states, manifest):
    """HH_261001 - Replay the frozen control policy against its causal actor-center speed."""
    expected = [row for row in receipts if row['reason'] in ('initial_drive_control', 'next_driving_control')]
    require([row['ack_receipt'] for row in rows] == expected,
            'expert proposal ledger does not cover exact driving ACKs')
    by_frame = {row['frame']: row for row in states}
    require(len(by_frame) == len(states), 'duplicate native frame')
    policy, result = v2.CombinedControl(), []
    proposal_count = 0
    wheelbase = manifest['coordinate_contract']['wheelbase_m']
    emergency_brake = manifest['capture_contract']['fixed_moving_normal_control']['effective_control'][
        'inherited_basic_agent_max_brake']
    for i, row in enumerate(rows):
        receipt, normal = row['ack_receipt'], row['normal_policy']
        frame = receipt['before_frame']
        observed = by_frame[frame]
        if observed.get('moving_task', {}).get('termination_reason') is not None:
            require(i == len(rows) - 1 and normal is None
                    and receipt['requested_control']['throttle'] == 0.
                    and receipt['requested_control']['brake'] == 1.
                    and receipt['requested_control']['steer'] == 0.,
                    'terminal moving command is not explicit final full-brake cleanup')
            result.append(dict(frame=frame, sequence=receipt['sequence'],
                               emergency_override=False, cleanup_command=True))
            continue
        target = 2.5 if proposal_count == 0 else min(2.5, .075 * proposal_count)
        proposal_count += 1
        require(row['target_ramp']['source_frame'] == frame
                and abs(row['target_ramp']['target_speed_mps'] - target) <= 1e-12,
                'causal target ramp differs')
        require(row['final_basic_agent_proposal'] == receipt['requested_control']
                and row['learned_control'] is False,
                'expert proposal/ACK source differs')
        emergency = normal is None
        if normal is not None:
            center_speed = center_speed_mps(observed, wheelbase)
            require(normal['source_frame'] == frame
                    and abs(normal['source_timestamp'] - observed['timestamp']) <= 1e-9
                    and abs(normal['measured_planar_speed_mps'] - center_speed)
                        <= SPEED_ABSOLUTE_TOLERANCE_MPS,
                    'normal policy is not bound to causal actor-center speed')
            proposal = normal['raw_normal_planner_proposal']
            capped = v2.base.control_dict(frozen.RecordedFloatControl(dict(
                proposal, brake=0., throttle=min(.20, proposal['throttle']))))
            require(capped == normal['revised_normal_capped_proposal'],
                    'normal cap/proposal differs')
            control = frozen.RecordedFloatControl(capped)
            policy.apply(control, speed=normal['measured_planar_speed_mps'],
                         timestamp=normal['source_timestamp'], frame=frame,
                         normal_proposal=proposal)
            require(policy.last == normal, 'fixed native policy replay differs')
            emergency = ack.mismatched_controls(normal['profile_normal_control'],
                                                receipt['requested_control']) != []
        if emergency:
            require(receipt['requested_control']['throttle'] == 0.
                    and receipt['requested_control']['brake'] == emergency_brake,
                    'differing BasicAgent return is not preserved emergency braking')
        result.append(dict(frame=frame, sequence=receipt['sequence'], emergency_override=emergency))
    return dict(status='PASS', commands=len(rows),
                emergency_override_count=sum(row['emergency_override'] for row in result),
                speed_reference='actor_center_reconstructed_from_base_link_and_yaw_rate',
                speed_tolerance_mps=SPEED_ABSOLUTE_TOLERANCE_MPS), result


def geometry_rows_first(timeline, cameras, route, anchors, slot, provider=None):
    """HH_261001 - Underlying geometry returns (summary, rows); the frozen caller expects rows first."""
    summary, rows = (provider or ORIGINAL_GEOMETRY_FUTURES)(timeline, cameras, route, anchors, slot)
    require(isinstance(summary, dict) and isinstance(rows, list)
            and len(rows) == len(cameras), 'geometry future summary/rows contract differs')
    return rows, summary


def numerical_audit(run, plan_sha, owner_sha, manifest_sha):
    """HH_261001 - Keep every frozen gate and denominator, changing only the two identified bugs."""
    task.verify_plan(plan_sha)
    saved_plan = frozen.PLAN_SHA
    saved_control = frozen.control_audit
    saved_geometry = frozen.geometry.geometry_futures
    try:
        with task.activated():
            frozen.PLAN_SHA = plan_sha
            frozen.control_audit = control_audit
            frozen.geometry.geometry_futures = geometry_rows_first
            return frozen.audit(run, owner_sha, manifest_sha)
    finally:
        frozen.PLAN_SHA = saved_plan
        frozen.control_audit = saved_control
        frozen.geometry.geometry_futures = saved_geometry


def instant(value):
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(stamp.tzinfo is not None, 'source timestamp lacks timezone')
    return stamp


def expected_import_paths():
    modules = {}
    for relative in agents.PINS:
        suffix = relative.removesuffix('.py').replace('/', '.').removesuffix('.__init__')
        name = 'agents' if suffix == '__init__' else 'agents.' + suffix
        modules[name] = str(agents.AGENTS_ROOT / relative)
    classes = {
        'agents.navigation.basic_agent.BasicAgent': modules['agents.navigation.basic_agent'],
        'agents.navigation.local_planner.LocalPlanner': modules['agents.navigation.local_planner'],
        'agents.navigation.global_route_planner.GlobalRoutePlanner': modules['agents.navigation.global_route_planner'],
        'agents.navigation.controller.VehiclePIDController': modules['agents.navigation.controller'],
    }
    return modules, classes


def source_audit(run, plan_sha, addendum_sha, owner_sha, manifest_sha):
    require(run.resolve() == task.RUN, 'only fresh V4 run is in scope')
    addendum = closure.verify_addendum(addendum_sha, plan_sha)
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
    require(sha(files['owner_plan']) == owner_sha and sha(files['manifest']) == manifest_sha,
            'caller-pinned V4 capture identity differs')
    records = {name: read(path) for name, path in files.items()}
    owner_plan = records['owner_plan']
    require(owner_plan['external_sources_v4_addendum_sha256'] == addendum_sha
            and owner_plan['external_sources_v4_addendum'] == addendum
            and owner_plan['complete_declared_source_archive_required'] is True
            and owner_plan['external_agents_bytes_archived_before_server'] is True,
            'owner did not bind prospective V4 source/QA declaration')
    command = owner_plan['collector_command']
    module = 'scripts.e2e.collect_town07_low_speed_moving_v4_20261001'
    require(command.count('-m') == 1 and command[command.index('-m') + 1] == module
            and command.count(module) == 1 and command.count('--') == 1,
            'owner did not execute one V4 worker')
    offset = command.index('--') + 1
    require(command[offset:offset + 4] == ['--v4-addendum-sha256', addendum_sha,
                                            '--v4-task-plan-sha256', plan_sha]
            and command[offset + 4] == str(task.RUN / 'episode'),
            'worker command does not bind V4 declarations/output')
    before_owner = records['owner_before']
    require(before_owner == dict(status='PASS', addendum_sha256=addendum_sha,
            declared_source_count=len(addendum['source_sha256']),
            source_snapshot=addendum['external_sources']),
            'pre-server source archive record differs')
    recomputed = closure.postcheck(run, addendum_sha, plan_sha)
    require(records['owner_after'] == recomputed
            and records['owner_moving']['external_source_v4_result'] == recomputed
            and records['owner_moving']['original_owner_result'] == records['owner_result'],
            'post-owner source/outcome records disagree')
    modules, classes = expected_import_paths()
    for name in ('worker_before', 'worker_after'):
        item = records[name]
        imported = item['import_snapshot']
        require(item['status'] == 'PASS' and item['addendum_sha256'] == addendum_sha
                and imported['status'] == 'PASS'
                and imported['sources'] == addendum['external_sources']
                and imported['imported_module_paths'] == modules
                and imported['inherited_classes'] == classes,
                'actual imported CARLA source/class identity differs')
    require(records['worker_after']['original_worker_exit_code'] == records['owner_result']['collector_exit_code'],
            'worker/owner return codes disagree')
    require(instant(addendum['declared_at_utc']) <= instant(records['worker_before']['import_snapshot']['observed_at_utc'])
            <= instant(records['manifest']['created_at'])
            <= instant(records['worker_after']['import_snapshot']['observed_at_utc']),
            'V4 source declaration/import/capture ordering differs')
    inputs = {str(path): sha(path) for path in files.values()}
    inputs[str(closure.ADDENDUM)] = addendum_sha
    inputs[str(task.PLAN)] = plan_sha
    for name in addendum['source_sha256']:
        path = run / 'source' / name
        inputs[str(path)] = sha(path)
    for name in addendum['external_sources']:
        path = run / 'external_agents_source' / name
        inputs[str(path)] = sha(path)
    require(all(sha(Path(name)) == digest for name, digest in inputs.items()),
            'V4 source evidence changed during audit')
    return dict(status='PASS', complete_declared_source_archive=True,
                declared_source_file_count=len(addendum['source_sha256']),
                inherited_agents_file_count=len(addendum['external_sources']),
                scope='Declared workspace sources plus eight inherited CARLA agents Python files; not complete OS/binary attestation.'), inputs


def audit(run, plan_sha, addendum_sha, owner_sha, manifest_sha):
    require(run.resolve() == task.RUN, 'only prospective V4 run is in scope')
    numerical, ledgers = numerical_audit(run, plan_sha, owner_sha, manifest_sha)
    try:
        source, source_inputs = source_audit(run, plan_sha, addendum_sha, owner_sha, manifest_sha)
    except (ValueError, AssertionError, KeyError, OSError, IndexError, TypeError) as error:
        source, source_inputs = dict(status='FAIL', error_type=type(error).__name__, error=str(error)), {}
    denominator_clear = numerical['selected_window_count'] == EXPECTED_SELECTED_WINDOWS
    numerical_clear = numerical['status'] == CLEAR and denominator_clear
    clear = numerical_clear and source['status'] == 'PASS'
    blocked = 'BLOCKED_NECESSARY_CHECKS' if not numerical_clear else 'BLOCKED_SOURCE_PROVENANCE'
    return dict(schema=SCHEMA, status=CLEAR if clear else blocked,
        checked_run=str(run), fixed_selected_window_count=EXPECTED_SELECTED_WINDOWS,
        selected_window_count=numerical['selected_window_count'],
        fixed_denominator_clear=denominator_clear,
        task_plan_sha256=plan_sha, addendum_sha256=addendum_sha,
        owner_plan_sha256=owner_sha, native_manifest_sha256=manifest_sha,
        numerical_audit=numerical, v4_source_evidence=source, v4_source_input_sha256=source_inputs,
        corrected_control_speed_reference='actor_center_from_base_link_vy_plus_half_wheelbase_yaw_rate',
        corrected_geometry_return_order='summary_then_rows_from_provider',
        thresholds_relaxed=False, window_filtering_performed=False,
        training_data_approved=False, dataset_conversion_performed=False,
        admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW',
        original_v3_records_modified=False,
        authority_review_blocks=['New 9 kph fixed-window TRAIN data admission is not approved.'],
        limitations=['One same-corridor 40–120 m steady-cruise capture; not independent route success.',
            'The 40–120 m windows do not cover the observed learned-policy stalls at 15–25 m.',
            'A separate prospective 10–30 m low-throttle/recovery capture is required for that bottleneck.',
            'Bounded snapshot-linear body sweeps do not prove continuous-time physical clearance.']), ledgers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=task.RUN)
    parser.add_argument('--task-plan-sha256', required=True)
    parser.add_argument('--addendum-sha256', required=True)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--native-manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.resolve().is_relative_to(args.run.resolve()),
            'fresh external V4 QA output required')
    modules = (frozen, frozen.geometry, frozen.raw, frozen.body, frozen.ack,
               task, closure, agents)
    auditor_sources = {str(Path(module.__file__).resolve()): sha(Path(module.__file__)) for module in modules}
    auditor_sources[str(Path(__file__).resolve())] = sha(Path(__file__))
    try:
        summary, ledgers = audit(args.run, args.task_plan_sha256, args.addendum_sha256,
                                 args.owner_plan_sha256, args.native_manifest_sha256)
    except (ValueError, AssertionError, KeyError, OSError, IndexError, TypeError) as error:
        summary, ledgers = dict(schema=SCHEMA, status='BLOCKED_UNASSESSED_EVIDENCE',
            error_type=type(error).__name__, error=str(error),
            training_data_approved=False, dataset_conversion_performed=False,
            admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW'), {}
        for relative in ('owner_result.json', 'owner_moving_result.json',
                         'episode/manifest.json', 'episode.partial/manifest.json'):
            path = args.run / relative
            if path.is_file():
                summary.setdefault('original_unmodified_records', {})[relative] = dict(
                    sha256=sha(path), record=read(path))
    require(all(sha(Path(name)) == digest for name, digest in auditor_sources.items()),
            'V4 auditor source changed during execution')
    summary['audit_source_sha256'] = auditor_sources
    args.output.mkdir(parents=True)
    for name, rows in ledgers.items():
        with gzip.open(args.output / (name + '.jsonl.gz'), 'wt') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: summary[key] for key in ('status', 'training_data_approved',
                                                 'dataset_conversion_performed')}))
    return 0 if summary['status'] == CLEAR else 1


if __name__ == '__main__':
    raise SystemExit(main())
