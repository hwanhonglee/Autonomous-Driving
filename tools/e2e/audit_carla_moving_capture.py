#!/usr/bin/env python3
"""HH_260906 - Diagnose fresh moving-task raw data with unchanged future bounds; never admit a dataset."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_portable_target_feasibility as targets
from scripts.e2e import carla_moving_capture as moving

ROOT = Path(__file__).resolve().parents[2]
require = raw.require
_DYNAMIC_PLAN_FIELDS = {'declared_at_utc', 'output', 'input_paths', 'input_sha256', 'source_sha256'}


def sealed_capture_scope(owner_plan, capture_plan):
    """HH_260906 - Resolve exactly one sealed moving slot without admitting arbitrary maps or routes."""
    slot_id = moving.slot_id_from_plan(capture_plan)
    moving.verify_cohort_slot(slot_id)
    spec = moving.slot_spec(slot_id)
    fixed = moving.plan_contract(slot_id, 15.)
    require(set(capture_plan) == set(fixed) | _DYNAMIC_PLAN_FIELDS,
            'moving plan fields differ from the sealed slot contract')
    require(moving.encoded({key: capture_plan[key] for key in fixed}) == moving.encoded(fixed),
            'fixed moving plan contract changed')
    require(owner_plan.get('capture_mode') == 'expert'
            and owner_plan.get('map') == spec['town']
            and owner_plan.get('route_sha256') == spec['route_sha256']
            and owner_plan.get('quality') == 'Epic', 'unreviewed moving owner scope')
    return dict(slot_id=slot_id, town=spec['town'], route_id=spec['route_id'],
                route_sha256=spec['route_sha256'], plan_schema=fixed['schema'])


def independent_projection(route, row):
    # HH_260906 - Recompute global unclamped projection directly from retained native XY and route arc, not saved membership.
    candidates = []
    planar_arc = 0.
    for i, (a, b) in enumerate(zip(route['route'], route['route'][1:])):
        x, y = b['x'] - a['x'], b['y'] - a['y']
        norm = x * x + y * y
        length = math.sqrt(norm)
        if norm <= 1.e-12:
            planar_arc += length
            continue
        along = max(0., min(1., ((row['x'] - a['x']) * x + (row['y'] - a['y']) * y) / norm))
        distance = math.hypot(row['x'] - a['x'] - along * x, row['y'] - a['y'] - along * y)
        arc = planar_arc + along * length
        candidates.append((distance, -arc, i))
        planar_arc += length
    require(candidates, 'no independently projectable route segment')
    error, negative_arc, _ = min(candidates)
    return -negative_arc, error


def reconstruct_window(route, states, cameras):
    selected, projections, failures = [], [], []
    by_frame = {r['frame']: r for r in cameras}
    require(len(by_frame) == len(cameras), 'duplicate raw camera frame')
    entered = False
    closed = support = previous = None
    for row in states:
        progress, error = independent_projection(route, row)
        stamp = raw.adapter._timestamp_ns(row['timestamp'], 'moving native timestamp')
        saved = row.get('moving_task', {})
        require(abs(progress - saved['measured_route_progress_m']) <= 1.e-8,
                'stored moving projection differs from independent route/pose calculation')
        live = row['capture_phase'] == 'driving' and support is None
        if live:
            if entered and previous is not None and progress < previous:
                failures.append(dict(frame=row['frame'], reason='decreasing_progress'))
            previous = progress
            if progress > 165. and closed is None:
                closed = dict(frame=row['frame'], timestamp_ns=stamp, measured_route_progress_m=progress)
            if saved['original_termination_reason'] is not None:
                failures.append(dict(frame=row['frame'], reason='catalog_or_agent_end_before_support'))
            if closed is not None and selected and stamp >= selected[-1]['timestamp_ns'] + 6_400_000_000:
                support = dict(frame=row['frame'], timestamp_ns=stamp)
        camera = by_frame.get(row['frame'])
        expected = row['capture_phase'] == 'driving' and closed is None and 30. <= progress <= 165.
        if camera is not None:
            # HH_260906 - Every geometrically selected anchor stays in the diagnostic denominator, even after a recorded failure.
            if expected:
                selected.append(dict(frame=row['frame'], timestamp_ns=max(raw.adapter._timestamp_ns(t, 'camera timestamp')
                    for t in camera['source_timestamps'].values()), measured_route_progress_m=progress))
                entered = True
            if camera.get('moving_task_selected') is not expected:
                failures.append(dict(frame=row['frame'], reason='saved_camera_membership_mismatch'))
            require(abs(camera['moving_task_measured_route_progress_m'] - progress) <= 1.e-8,
                    'camera moving projection does not match same-frame pose')
        projections.append(dict(frame=row['frame'], timestamp_ns=stamp, phase=row['capture_phase'],
                                measured_route_progress_m=progress, cross_track_error_m=error))
    driving = [r for r in states if r['capture_phase'] == 'driving']
    tail = [r for r in states if r['capture_phase'] == 'stationary_tail']
    span = selected[-1]['timestamp_ns'] - selected[0]['timestamp_ns'] if selected else None
    complete_future = bool(selected and driving and raw.adapter._timestamp_ns(driving[-1]['timestamp'], 'last driving')
                           >= selected[-1]['timestamp_ns'] + 6_400_000_000)
    flags = dict(nonempty_selection=bool(selected), observed_selected_span_at_least_30_seconds=span is not None and span >= 30_000_000_000,
                 window_closed=closed is not None, actual_full_future_before_tail=complete_future,
                 exact_tail_130_states=len(tail) == 130, no_selection_replay_failure=not failures,
                 support_is_final_driving_observation=bool(support and driving and support['frame'] == driving[-1]['frame']))
    return dict(status='MOVING_SCOPE_REPLAY_CLEAR' if all(flags.values()) else 'MOVING_SCOPE_REPLAY_FAILED', flags=flags,
                selected_camera_anchor_count=len(selected), selected_camera_span_ns=span,
                first_selected_camera=selected[0] if selected else None, last_selected_camera=selected[-1] if selected else None,
                first_closing_observation=closed, independently_observed_support=support,
                selected_camera_frames=[r['frame'] for r in selected], failures=failures), projections


def selected_future_summary(rows, frames):
    # HH_260906 - Keep every predeclared anchor and every original nine-metric failure; do not search for passing slices.
    selected = [row for row in rows if row['frame'] in set(frames)]
    require(len(selected) == len(frames) and len(set(frames)) == len(frames), 'selected anchor denominator differs')
    assessed = [row['diagnostic'] for row in selected if 'diagnostic' in row]
    full = all(row.get('valid_points') == 64 and 'diagnostic' in row for row in selected)
    summary = targets.summarize_samples(assessed, 64)
    violations = {name: record['violating_anchor_count'] for name, record in summary['metrics'].items()}
    return dict(selected_anchor_count=len(selected), all_selected_anchors_have_actual_64_points=bool(selected) and full,
                unchanged_nine_metric_failures=violations, all_selected_necessary_bounds_clear=bool(selected) and full and not any(violations.values()),
                measured_future=summary, dataset_admission_granted=False)


def audit(trial, owner_sha):
    trial = Path(trial)
    ledger = {}
    owner_plan = raw.read_metadata(trial, 'owner_plan.json', ledger)
    require(ledger['owner_plan.json']['sha256'] == owner_sha, 'owner plan SHA differs')
    require(owner_plan.get('capture_mode') == 'expert' and owner_plan.get('quality') == 'Epic'
            and isinstance(owner_plan.get('moving_task_capture'), dict), 'unreviewed moving owner mode')
    owner = raw.read_metadata(trial, 'owner_result.json', ledger)
    require(owner['source_bytes_unchanged_and_archived'] is True
            and set(owner['source_checks']) == set(owner_plan['source_sha256'])
            and all(value is True for value in owner['source_checks'].values()), 'owner source after-check failed')
    for name, pin in owner_plan['source_sha256'].items():
        require(raw.digest(raw.control_audit.checked_bytes(trial, 'provenance/' + name, ledger)) == pin, 'owner archive changed')
    refs = owner_plan['moving_task_capture']
    require(set(refs['inputs']) == {'route_file', 'mapping', 'calibration'}, 'moving source input set differs')
    for name, ref in refs['inputs'].items():
        payload = raw.control_audit.checked_bytes(trial, ref['archived_path'], ledger)
        require(raw.digest(payload) == ref['sha256'], f'archived moving input changed: {name}')
    capture_plan = raw.read_metadata(trial, refs['plan']['archived_path'], ledger)
    require(ledger[refs['plan']['archived_path']]['sha256'] == refs['plan']['sha256'], 'task plan archive differs')
    scope = sealed_capture_scope(owner_plan, capture_plan)
    for stage in ('ready', 'after_capture', 'stopped'):
        observation = raw.read_metadata(trial, f'lifecycle/{stage}.json', ledger)
        require(observation['status'] == 'PASS' and observation['expected_map'] == scope['town'],
                'owned lifecycle check failed')
    require(capture_plan['input_sha256'] == {name: ref['sha256'] for name, ref in refs['inputs'].items()}, 'task/owner input pins differ')
    require(all(owner_plan['source_sha256'].get(name) == pin for name, pin in capture_plan['source_sha256'].items()),
            'task/owner execution source pins differ')
    require(capture_plan['schema'] == scope['plan_schema']
            and capture_plan['route_sha256'] == scope['route_sha256'], 'unexpected prospective selection')
    require(datetime.fromisoformat(capture_plan['declared_at_utc']) <= datetime.fromisoformat(owner_plan['planned_at_utc']),
            'moving plan is not prospective')
    episode_names = [name for name in ('episode', 'episode.partial') if (trial / name).is_dir()]
    require(len(episode_names) == 1, 'exactly one retained native capture required')
    episode_name = episode_names[0]
    episode = trial / episode_name
    manifest = raw.read_metadata(trial, episode_name + '/manifest.json', ledger)
    owner_manifest_sha = owner['moving_task_capture']['final_manifest_sha256']
    if owner_manifest_sha is None:
        # HH_260906 - The legacy owner only pins a final directory; retain stopped failed partial evidence without inventing that receipt.
        require(owner['exit_code'] != 0 and episode_name == 'episode.partial', 'missing successful owner manifest binding')
        manifest_binding = 'FAILED_PARTIAL_DIAGNOSTIC_PRE_POST_HASH_ONLY_NO_OWNER_FINAL_HASH'
    else:
        require(ledger[episode_name + '/manifest.json']['sha256'] == owner_manifest_sha,
                'native manifest no longer matches owner completion')
        manifest_binding = 'OWNER_FINAL_MANIFEST_SHA_MATCH'
    raw.read_metadata(trial, episode_name + '/moving_task_plan.json', ledger)
    require(ledger[episode_name + '/moving_task_plan.json']['sha256'] == refs['plan']['sha256']
            == manifest['provenance']['moving_task_plan_sha256'], 'native moving plan copy differs')
    for name in ('mapping', 'calibration'):
        require(manifest['provenance'][name + '_sha256'] == refs['inputs'][name]['sha256'], 'native sensor provenance differs from plan')
    route = raw.read_metadata(trial, episode_name + '/route.json', ledger)
    states = raw.read_metadata(trial, episode_name + '/states.jsonl', ledger, lines=True)
    cameras = raw.read_metadata(trial, episode_name + '/camera_frames.jsonl', ledger, lines=True)
    receipts = raw.read_metadata(trial, episode_name + '/control_receipts.jsonl', ledger, lines=True)
    require(ledger[episode_name + '/route.json']['sha256'] == scope['route_sha256'],
            'captured route differs from declared route')
    for name in ('portable_e2e/model.py', 'portable_e2e/runtime_contract.py'):
        require(owner_plan['source_sha256'][name] == raw.scalar.sha(ROOT / name), 'numerical bounds changed')
    timeline = raw.measured_timeline(states)
    transport = ack.analyze_transport(states, receipts, manifest['capture_contract']['control_transport']['bootstrap_observation'])
    kinematics = raw.snapshot_kinematics(states, 2.85)
    pixels, image_rows, camera_rows = raw.camera_audit(episode, manifest, cameras, timeline)
    future, future_rows = raw.measured_future_audit(timeline, cameras, route, camera_rows)
    window, projections = reconstruct_window(route, states, cameras)
    selected = selected_future_summary(future_rows, window['selected_camera_frames'])
    events = {name: sum(len(row[name]) for row in states) for name in ('collision', 'lane_invasion')}
    cleanup_brakes = [r for r in receipts if r['reason'] == 'driving_end_brake']
    support = window['independently_observed_support']
    brake_after = bool(support and len(cleanup_brakes) == 1 and cleanup_brakes[0]['before_frame'] == support['frame'])
    facts = dict(original_owner_exit_code=owner['exit_code'], original_capture_status=manifest['status'],
                 manifest_hash_binding=manifest_binding,
                 original_goal_reached=manifest.get('result', {}).get('goal_reached'),
                 original_task_complete=manifest.get('result', {}).get('task_complete'),
                 original_admission_status=manifest.get('result', {}).get('admission_status'),
                 cleanup=manifest['cleanup'], events=events, explicit_cleanup_brake_only_after_support=brake_after)
    clear = (owner['exit_code'] == 0 and manifest['status'] == 'complete' and manifest['cleanup']['completed'] is True
             and facts['original_goal_reached'] is False and facts['original_task_complete'] is True
             and facts['original_admission_status'] == 'PENDING_QA' and not any(events.values()) and brake_after
             and transport['status'] == 'PASS' and pixels['all_pixel_and_metadata_checks_clear']
             and kinematics['coordinate_conversion_status'] == 'RECORDED_FORMULA_MATCH'
             and kinematics['mismatched_state_count'] == 0
             and window['status'] == 'MOVING_SCOPE_REPLAY_CLEAR' and selected['all_selected_necessary_bounds_clear'])
    for image in image_rows:
        require(raw.scalar.sha(episode / image['path']) == image['sha256'], 'native image changed while auditing')
    for name, pin in ledger.items():
        require(raw.scalar.sha(trial / name) == pin['sha256'], 'native metadata changed while auditing')
    report = dict(schema='carla.moving_capture_diagnostic.v1', status='DIAGNOSED_NOT_ADMITTED',
                  moving_scope_necessary_checks_clear=clear, trial=str(trial), owner_plan_sha256=owner_sha,
                  sealed_capture_scope=scope,
                  facts=facts, transport=transport, camera_pixels=pixels, snapshot_kinematics=kinematics,
                  moving_scope=window, selected_future=selected, full_raw_future_all_original_phases=future,
                  source_manifest=[dict(path=name, **pin) for name, pin in sorted(ledger.items())],
                  dataset_admission_granted=False, dataset_conversion_performed=False, vehicle_control_approved=False,
                  limitations=['Read-only necessary-condition checks, not Common10 admission, learned driving, or function approval.',
                    'All warmup/driving future failures remain in the full-raw report; fixed selected membership is not a repaired old episode.',
                    'Catalog goal and stopping were not this moving task. Native tail is retained, not stopping-function evidence.',
                    'ACK is API acceptance/readback evidence, not proven physical actuator timing.'])
    return report, image_rows, future_rows, projections


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink()
            and not args.output.resolve().is_relative_to(args.trial.resolve()), 'fresh output outside raw capture required')
    sources = raw.source_identity()
    sources['files'][str(Path(__file__).resolve().relative_to(ROOT))] = raw.scalar.sha(Path(__file__))
    report, images, futures, projections = audit(args.trial, args.owner_plan_sha256)
    require(all(raw.scalar.sha(ROOT / name) == pin for name, pin in sources['files'].items()), 'diagnostic source changed')
    report['diagnostic_source'] = sources
    args.output.mkdir(parents=True)
    for name, rows in (('jpeg_audit.jsonl', images), ('future_anchor_audit.jsonl', futures), ('native_projection.jsonl', projections)):
        with (args.output / name).open('x') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    with (args.output / 'summary.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(status=report['status'], moving_scope_necessary_checks_clear=report['moving_scope_necessary_checks_clear'],
                         selected_anchors=report['moving_scope']['selected_camera_anchor_count'], output=str(args.output))))


if __name__ == '__main__':
    main()
