"""HH_261001 - Inspect V3 raw moving windows without overriding its failed source-closure gate."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path

from scripts.e2e import audit_town07_low_speed_moving_pending_20261001 as frozen
from scripts.e2e import audit_town07_low_speed_moving_v3_20261001 as v3


def diagnose(run, owner_sha, manifest_sha):
    # HH_261001 - Pin the exact failed capture; this is a diagnostic, never a TRAIN admission.
    frozen.require(run.resolve() == frozen.task.RUN, 'wrong moving capture')
    episode = run / 'episode'
    frozen.require(episode.is_dir(), 'complete native episode required')
    frozen.require(frozen.sha(run / 'owner_plan.json') == owner_sha, 'owner plan changed')
    frozen.require(frozen.sha(episode / 'manifest.json') == manifest_sha, 'native manifest changed')
    plan = frozen.read(frozen.task.PLAN)
    manifest = frozen.read(episode / 'manifest.json')
    owner = frozen.read(run / 'owner_plan.json')
    frozen.validate_identity(run, frozen.PLAN_SHA, plan, manifest)
    missing = {name: digest for name, digest in plan['source_sha256'].items()
               if owner['source_sha256'].get(name) != digest}
    frozen.require(set(missing) == {'tools/e2e/carla_physics_substeps.py'},
                   'unexpected declared source-closure discrepancy')
    frozen.require(frozen.sha(frozen.task.ROOT / 'tools/e2e/carla_physics_substeps.py')
                   == missing['tools/e2e/carla_physics_substeps.py'], 'optional source changed')
    frozen.require(plan['fixed_arguments']['physics_substep_profile'] == 'inherited'
                   and manifest['capture_contract']['moving_task']['physics_settings']['status'] == 'PASS',
                   'not the inherited physics capture')
    source, _ = v3.inspect_sources(run)
    states, cameras, receipts, proposals = [frozen.read_rows(episode / name) for name in
        ('states.jsonl', 'camera_frames.jsonl', 'control_receipts.jsonl', 'expert_moving_control.jsonl')]
    route = frozen.read(episode / 'route.json')
    timeline = frozen.raw.measured_timeline(states)
    pixels, _, anchors = frozen.raw.camera_audit(episode, manifest, cameras, timeline)
    transport = frozen.ack.analyze_transport(states, receipts,
        manifest['capture_contract']['control_transport']['bootstrap_observation'], wheelbase=2.85)
    # HH_261001 - Preserve a control-replay failure while continuing independent raw geometry checks.
    try:
        controls, _ = frozen.control_audit(proposals, receipts, states, manifest)
    except Exception as error:
        controls = dict(status='FAIL', error_type=type(error).__name__, error=str(error))
    # HH_261001 - Reconstruct the CARLA center velocity from the recorded base_link yaw correction.
    wheelbase_m = manifest['coordinate_contract']['wheelbase_m']
    frozen.require(wheelbase_m == 2.85, 'recorded wheelbase differs from fixed audit')
    by_frame = {state['frame']: state for state in states}
    center_speed_errors = []
    for row in proposals:
        normal = row['normal_policy']
        if normal is None:
            continue
        state = by_frame[normal['source_frame']]
        frozen.require(row['ack_receipt']['before_frame'] == state['frame']
                       and abs(normal['source_timestamp'] - state['timestamp']) <= 1e-9,
                       'normal policy is not causally bound to the same frame')
        corrected_center_vy = state['vy'] + wheelbase_m / 2.0 * state['yaw_rate']
        center_speed_errors.append(abs(normal['measured_planar_speed_mps']
                                       - math.hypot(state['vx'], corrected_center_vy)))
    center_speed = dict(proposal_count=len(center_speed_errors),
        maximum_absolute_error_mps=max(center_speed_errors),
        all_within_1e_9_mps=all(error <= 1e-9 for error in center_speed_errors),
        formula='CARLA actor-center planar speed = hypot(base_link vx, base_link vy + wheelbase/2 * yaw_rate)')
    membership, _ = frozen.reconstruct_membership(route, states, cameras)
    future_summary, futures = frozen.geometry.geometry_futures(timeline, cameras, route, anchors, frozen.task.SLOT)
    shapes, body_summary = frozen.body.body_census(manifest, route, states,
        (episode / 'active_map_opendrive.xodr').read_bytes(), receipts[0]['actor_id'])
    native = frozen.native_checks(states, shapes)
    windows, protected = frozen.window_census(membership, cameras, native, anchors, futures)
    failures = frozen.scoped_failures(native, protected)
    flags = dict(source_closure_complete=False, v3_external_agents_pass=source['status'] == 'PASS',
        native_complete=manifest['status'] == 'complete',
        membership_clear=all(membership['flags'].values()),
        selected_windows_clear=bool(windows) and all(row['necessary_window_checks_clear'] for row in windows),
        all_pixels=pixels['all_pixel_and_metadata_checks_clear'],
        all_ack_and_snapshot_checks=all(transport['flags'].values()),
        fixed_control_replay=controls['status'] == 'PASS',
        corrected_center_speed_binding=center_speed['all_within_1e_9_mps'],
        no_collision_or_lane_events=all(not row['collision'] and not row['lane_invasion'] for row in states))
    return dict(schema='carla.town07_moving_v3_raw_forensic.20261001.v1',
        status='FORENSIC_ONLY_SOURCE_CLOSURE_BLOCKED', flags=flags,
        source_closure_discrepancy=missing, physics_substep_profile='inherited',
        owner_plan_sha256=owner_sha, native_manifest_sha256=manifest_sha,
        v3_external_source=source, native_count=len(states), camera_count=len(cameras),
        selected_count=len(windows), selected_span_s=(
            (windows[-1]['timestamp_ns'] - windows[0]['timestamp_ns']) / 1e9 if windows else None),
        membership_flags=membership['flags'], window_failure_counts=dict(Counter(
            reason for row in windows for reason in row['failure_reasons'])),
        failed_window_frames=[row['frame'] for row in windows if not row['necessary_window_checks_clear']],
        protected_native_count=len(protected), scoped_native_failures=failures,
        pixel_summary=pixels, transport_flags=transport['flags'], control_summary=controls,
        center_speed_reconstruction=center_speed,
        whole_capture_body_summary=body_summary, whole_capture_future_summary=future_summary,
        training_data_approved=False, dataset_conversion_performed=False,
        admission_authority='NONE',
        interpretation='The frozen V3 QA remains BLOCKED_UNASSESSED_EVIDENCE. These raw numerical checks are posthoc diagnostics, not a bypass or an admission.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--native-manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    frozen.require(not args.output.exists(), 'fresh diagnostic output required')
    result = diagnose(frozen.task.RUN, args.owner_plan_sha256, args.native_manifest_sha256)
    args.output.mkdir(parents=True)
    (args.output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in ('status', 'flags', 'selected_count')}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
