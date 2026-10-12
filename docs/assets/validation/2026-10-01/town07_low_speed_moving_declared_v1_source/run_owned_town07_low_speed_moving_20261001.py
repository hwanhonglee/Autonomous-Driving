"""HH_261001 - Declare, dry-run, or explicitly own one NEW fixed 9 kph expert moving task."""

import argparse
import json
import sys

from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
from scripts.e2e import collect_town07_low_speed_moving_20261001 as worker
from scripts.e2e import town07_low_speed_moving_task_20261001 as task


def command():
    return [sys.executable, str(owner.ROOT / 'tools/workspace_python.py'), '-m',
        'scripts.e2e.collect_town07_low_speed_moving_20261001', '--', str(task.RUN / 'episode'), str(task.ROUTE),
        '--host', '127.0.0.1', '--port', '2440', '--control-transport', 'acknowledged_batch',
        '--agent-initialization', 'after_bootstrap', '--stationary-warmup-sec', '3.5', '--stationary-tail-sec', '6.5',
        '--max-duration-sec', '180', '--target-speed-kmh', '9', '--seed', '2026100101', '--weather', 'ClearNoon',
        '--mapping', str(owner.MAPPING), '--calibration', str(owner.CALIBRATION), '--spawn-z-offset-m', '.5',
        '--expected-live-xodr-sha256', owner.PINS[owner.XODR], '--goal-stop-profile', 'disabled', '--goal-tolerance-m', '1',
        '--basic-agent-base-min-distance-m', '2', '--basic-agent-distance-ratio', '.2',
        '--moving-task-plan', str(task.PLAN), '--moving-task-plan-sha256', task.sha(task.PLAN)]


def prepare():
    owner.OUTPUT = task.RUN
    owner.PINS = {path: digest for path, digest in owner.PINS.items() if path not in (owner.WEIGHTS, owner.METADATA)}
    owner.PINS[task.PLAN] = task.sha(task.PLAN)
    owner.collector_command = command
    with worker.parsing_scope():
        plan = owner.preflight()
    argv = command()
    worker.parse_args(argv[argv.index(str(task.RUN / 'episode')):])
    plan.update(schema='carla.owned_town07_low_speed_moving.v1', source_checkpoint_sha256=None,
        learned_control=False, capture_budget_seconds=180, wall_timeout_seconds=240,
        task_plan_sha256=task.sha(task.PLAN), cohort_slot_id=task.SLOT, selected_progress_m=[40., 120.],
        minimum_selected_camera_span_s=30., normal_control_policy=task.plan_contract()['controller'],
        exact_runtime_tensor_archives=False, existing_qualification_authority_applies=False,
        interpretation='Fresh prospective fixed moving-window TRAIN candidate; PENDING_QA, not goal/launch/stop success or data admission.')
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--declare', action='store_true')
    action.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.declare:
        if task.PLAN.exists() or task.RUN.exists():
            raise ValueError('create-only fresh prospective plan/output required')
        task.PLAN.parent.mkdir(parents=True, exist_ok=True)
        owner.write_json(task.PLAN, task.build_plan(owner.MAPPING, owner.CALIBRATION))
        print(json.dumps(dict(plan=str(task.PLAN), sha256=task.sha(task.PLAN), simulator_started=False)))
        return 0
    plan = prepare()
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    owner.execute(plan)
    original = json.loads((task.RUN / 'owner_result.json').read_text())
    recorded = json.loads((task.RUN / 'owner_plan.json').read_text())
    source_match = all(owner.digest(owner.ROOT / path) == digest
        and owner.digest(task.RUN / 'source' / path) == digest for path, digest in recorded['source_sha256'].items())
    episode = task.RUN / 'episode'
    if not episode.exists():
        episode = task.RUN / 'episode.partial'
    manifest = json.loads((episode / 'manifest.json').read_text()) if (episode / 'manifest.json').exists() else {}
    result = dict(schema='carla.owned_town07_low_speed_moving_outcome.v1', original_owner_result=original,
        source_hashes_match=source_match, native_status=manifest.get('status'), native_error=manifest.get('error'),
        task_complete=manifest.get('result', {}).get('task_complete', False),
        goal_reached=manifest.get('result', {}).get('goal_reached', False),
        task_result=manifest.get('result', {}).get('moving_task'),
        whole_recording_goal_stop_quality=manifest.get('result', {}).get('whole_recording_goal_stop_quality_diagnostic'),
        existing_qualification_authority_applies=False, training_data_approved=False, vehicle_control_approved=False)
    owner.write_json(task.RUN / 'owner_moving_result.json', result)
    print(json.dumps(result, indent=2))
    return 0 if source_match and original['collector_exit_code'] == 0 and not original['cleanup_errors'] and result['task_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
