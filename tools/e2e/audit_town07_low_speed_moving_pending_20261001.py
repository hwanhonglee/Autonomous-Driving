"""HH_261001 - Necessary checks for one prospective moving task; never an admission authority."""

import argparse
from bisect import bisect_left
from collections import Counter
from datetime import datetime
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct

from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_carla_moving_capture as old_audit
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_carla_turn_launch_raw_geometry as geometry
from scripts.e2e import audit_simulation_research_body_lane_20261001 as body
from scripts.e2e import collect_town07_low_speed_start_v2_20261001 as v2
from scripts.e2e import town07_low_speed_moving_task_20261001 as task

PLAN_SHA = 'b8116231db1a5a548b1e890f21d403feb50bcef9403b49cbdcba5e3d055dfee1'
SCHEMA = 'carla.town07_low_speed_moving_necessary_checks.20261001.v1'
NATIVE_CHECKS = ('native_speed', 'native_scalar_rate', 'native_longitudinal_rate',
                 'native_clock', 'snapshot_conversion', 'body_inset', 'sweep_inset', 'driving_phase')
require = raw.require
AUDIT_HELPER_PINS = {
    'tools/e2e/audit_carla_acknowledged_control.py': '9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd',
    'tools/e2e/audit_carla_moving_capture.py': '38e4b952a3b323701293e8c0c769b4e520daba00f282993b1851f55e1deb63d4',
    'tools/e2e/audit_carla_raw_pre_admission.py': '0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00',
    'tools/e2e/audit_carla_turn_launch_raw_geometry.py': '32f1cd04669afc665d28cdf7040ee1517505a6543c3a420806a1816db06fbe49',
    'tools/e2e/audit_simulation_research_body_lane_20261001.py': '6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac',
    'tools/e2e/audit_future_actor_lane_containment.py': '75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d',
    'tools/e2e/prepare_carla_common10_dataset.py': 'eab12ef9d83f7ca72d8396ac46d77082cbafbc9600b44c59d733cb76e0de7e65',
    'tools/e2e/audit_portable_target_feasibility.py': '8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894',
    'learning/portable_e2e/contract.py': 'e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09',
    'learning/portable_e2e/model.py': 'fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e',
    'learning/portable_e2e/config/common_10hz_v1.contract.json': '6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642',
}


class RecordedFloatControl:
    """HH_261001 - Match CARLA control float32 setters for replay; never touch measured states."""
    def __init__(self, values):
        for key, value in values.items():
            setattr(self, key, value)

    def __setattr__(self, key, value):
        if key in ('throttle', 'brake', 'steer'):
            value = struct.unpack('f', struct.pack('f', value))[0]
        object.__setattr__(self, key, value)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def validate_identity(run, plan_sha, plan, manifest):
    """HH_261001 - No caller override can make an old failed episode this prospective task."""
    require(Path(run).resolve() == task.RUN and plan_sha == PLAN_SHA, 'wrong prospective run or plan digest')
    require(plan['cohort_slot_id'] == task.SLOT and plan['selected_progress_m'] == [40., 120.]
            and plan['output'] == str(task.RUN / 'episode'), 'wrong fixed moving cohort')
    require(plan['fixed_arguments']['target_speed_kmh'] == 9., 'wrong speed protocol')
    moving = manifest.get('capture_contract', {}).get('moving_task', {})
    require(moving.get('plan_sha256') == PLAN_SHA and moving.get('cohort_slot_id') == task.SLOT,
            'native manifest is not the prospective moving capture')
    require(manifest.get('control_source') == 'BasicAgent_fixed_v2_normal_policy_new_moving_task',
            'wrong expert source; failed goal captures cannot be repurposed')
    require(manifest.get('admission_status') == 'PENDING_QA'
            and manifest.get('training_data_approved') is not True,
            'native pending status must remain unapproved')
    require('low_speed_start_research' not in manifest['capture_contract'], 'old failed capture prohibited')
    require(datetime.fromisoformat(plan['declared_at_utc'].replace('Z', '+00:00'))
            <= datetime.fromisoformat(manifest['created_at'].replace('Z', '+00:00')),
            'capture predates prospective declaration')


def reconstruct_membership(route, states, cameras):
    """HH_261001 - Independent geometry selects ALL anchors, never a quality-filtered subset."""
    by_frame = {row['frame']: (i, row) for i, row in enumerate(cameras)}
    require(len(by_frame) == len(cameras), 'duplicate camera frame')
    selected, projections, failures = [], [], []
    closed = support = previous = None
    for row in states:
        progress, error = old_audit.independent_projection(route, row)
        stamp = round(row['timestamp'] * 1e9)
        saved = row['moving_task']
        require(abs(progress - saved['measured_route_progress_m']) <= 1e-8, 'stored geometric progress changed')
        if row['capture_phase'] == 'driving' and support is None:
            if selected and previous is not None and progress < previous:
                failures.append(dict(frame=row['frame'], reason='decreasing_progress'))
            previous = progress
            if progress > 120. and closed is None:
                closed = dict(frame=row['frame'], timestamp_ns=stamp)
            if saved['original_termination_reason'] is not None:
                failures.append(dict(frame=row['frame'], reason='catalog_or_agent_end_before_support'))
            if closed is not None and selected and stamp >= selected[-1]['timestamp_ns'] + 6_400_000_000:
                support = dict(frame=row['frame'], timestamp_ns=stamp)
        if row['frame'] in by_frame:
            index, camera = by_frame[row['frame']]
            expected = row['capture_phase'] == 'driving' and closed is None and 40. <= progress <= 120.
            if expected:
                selected.append(dict(camera_index=index, frame=row['frame'], timestamp_ns=max(
                    round(t * 1e9) for t in camera['source_timestamps'].values()), progress_m=progress))
            if camera.get('moving_task_selected') is not expected:
                failures.append(dict(frame=row['frame'], reason='saved_membership_mismatch'))
            require(abs(camera['moving_task_measured_route_progress_m'] - progress) <= 1e-8,
                    'saved camera progress differs')
        projections.append(dict(frame=row['frame'], timestamp_ns=stamp, progress_m=progress,
                                cross_track_error_m=error, capture_phase=row['capture_phase']))
    driving = [row for row in states if row['capture_phase'] == 'driving']
    flags = dict(nonempty=bool(selected), window_closed=closed is not None,
        span_at_least_30s=bool(selected and selected[-1]['timestamp_ns'] - selected[0]['timestamp_ns'] >= 30_000_000_000),
        support_is_last_driving=bool(driving and support and support['frame'] == driving[-1]['frame']),
        exact_70_warmup=sum(row['capture_phase'] == 'stationary_warmup' for row in states) == 70,
        exact_130_tail=sum(row['capture_phase'] == 'stationary_tail' for row in states) == 130,
        saved_membership_and_progress_clear=not failures)
    return dict(flags=flags, selected=selected, closed=closed, support=support, failures=failures), projections


def native_checks(states, geometric):
    """HH_261001 - Record every native interval without smoothing or clipping any measured field."""
    result = []
    maximum, tolerance = raw.targets.PHYSICAL_MAXIMUM_ACCELERATION_MPS2, raw.targets.ROUNDING_TOLERANCE
    for i, (state, shape) in enumerate(zip(states, geometric)):
        speed = math.hypot(state['vx'], state['vy'])
        previous = states[i - 1] if i else None
        dt = state['timestamp'] - previous['timestamp'] if previous else None
        scalar = (speed - math.hypot(previous['vx'], previous['vy'])) / dt if previous else None
        longitudinal = (state['vx'] - previous['vx']) / dt if previous else None
        rebuilt = ack.recompute_snapshot_state(state, 2.85)
        flags = dict(native_speed=speed <= raw.targets.PHYSICAL_MAXIMUM_SPEED_MPS + tolerance,
            native_scalar_rate=scalar is None or abs(scalar) <= maximum + tolerance,
            native_longitudinal_rate=longitudinal is None or abs(longitudinal) <= maximum + tolerance,
            native_clock=previous is None or (state['frame'] == previous['frame'] + 1
                and abs(round(state['timestamp'] * 1e9) - round(previous['timestamp'] * 1e9) - 50_000_000) <= 1),
            snapshot_conversion=max(abs(state[key] - value) for key, value in rebuilt.items()) <= 1e-9,
            body_inset=shape['sampled_body']['inset_10cm_lane']['area_gate_clear'],
            sweep_inset=shape['previous_sweep'] is None or shape['previous_sweep']['regions']['inset_10cm_lane']['area_gate_clear'],
            driving_phase=state['capture_phase'] == 'driving')
        result.append(dict(frame=state['frame'], timestamp_ns=round(state['timestamp'] * 1e9),
            capture_phase=state['capture_phase'], speed_mps=speed, vx_mps=state['vx'],
            raw_api_ax_mps2=state['ax'], scalar_rate_mps2=scalar, longitudinal_dvx_dt_mps2=longitudinal,
            current_control=state['current_control'], checks=flags, geometry=shape))
    return result


def nine_clear(future, first_only=False):
    steps = future.get('discrete_bound_diagnostic', {}).get('steps', [])
    if first_only:
        steps = steps[:1]
    return len(steps) == (1 if first_only else 64) and all(
        set(step['metrics']) == set(raw.targets.METRICS)
        and all(not failed for _, failed in step['metrics'].values()) for step in steps)


def window_census(membership, cameras, native, anchors, futures):
    stamps = [row['timestamp_ns'] for row in native]
    frame_index = {row['frame']: i for i, row in enumerate(native)}
    windows, protected = [], set()
    for selected in membership['selected']:
        i, stamp = selected['camera_index'], selected['timestamp_ns']
        start = max(0, i - 9)
        first_native = frame_index[cameras[start]['frame']]
        lo, hi = max(0, first_native - 1), bisect_left(stamps, stamp + 6_400_000_000)
        reasons = []
        history = cameras[start:i + 1]
        history_ok = i >= 9 and first_native > 0 and len(history) == 10 and all(
            row['capture_phase'] == 'driving' for row in history) and all(
            b['frame'] == a['frame'] + 2 for a, b in zip(history, history[1:])) and (
                stamp - anchors[start]['timestamp_ns'] >= 900_000_000)
        if not history_ok:
            reasons.append('full_ten_camera_history_and_prior_native_guard')
        future = futures[i]
        if hi >= len(native) or future['valid_future_points'] != 64 or future.get('future_crosses_stationary_tail'):
            reasons.append('full_64_actual_future_before_tail')
        hi = min(hi, len(native) - 1)
        indices = set(range(lo, hi + 1))
        protected.update(indices)
        for check in NATIVE_CHECKS:
            if not all(native[index]['checks'][check] for index in indices):
                reasons.append(check)
        if not all(row['same_recorded_frame_and_timestamp'] for row in anchors[start:i + 1]):
            reasons.append('causal_history_camera_native_binding')
        if not nine_clear(future):
            reasons.append('all_nine_future_bounds')
        if not all(nine_clear(row, first_only=True) for row in futures[start:i]):
            reasons.append('all_nine_history_edge_bounds')
        windows.append(dict(**selected, history_camera_indices=[start, i], protected_native_indices=[lo, hi],
            protected_native_frames=[native[lo]['frame'], native[hi]['frame']],
            protected_timestamp_ns=[stamps[lo], stamps[hi]], necessary_window_checks_clear=not reasons,
            failure_reasons=reasons, training_data_approved=False))
    return windows, protected


def scoped_failures(native, protected):
    """HH_261001 - Keep pre-entry/cleanup kinematic failures visible, never silently erase them."""
    return {scope: [dict(frame=row['frame'], capture_phase=row['capture_phase'],
                        failed_checks=[key for key, clear in row['checks'].items()
                                       if not clear and key != 'driving_phase'],
                        scalar_rate_mps2=row['scalar_rate_mps2'], longitudinal_dvx_dt_mps2=row['longitudinal_dvx_dt_mps2'])
                    for i, row in enumerate(native) if (i in protected) == (scope == 'protected')
                    and any(not value for key, value in row['checks'].items() if key != 'driving_phase')]
            for scope in ('protected', 'outside_protected')}


def control_audit(rows, receipts, states, manifest):
    expected = [row for row in receipts if row['reason'] in ('initial_drive_control', 'next_driving_control')]
    require([row['ack_receipt'] for row in rows] == expected, 'expert proposal ledger does not cover exact driving ACKs')
    by_frame = {row['frame']: row for row in states}
    policy, result = v2.CombinedControl(), []
    proposal_count = 0
    emergency_brake = manifest['capture_contract']['fixed_moving_normal_control']['effective_control']['inherited_basic_agent_max_brake']
    for i, row in enumerate(rows):
        receipt, normal = row['ack_receipt'], row['normal_policy']
        frame = receipt['before_frame']
        observed = by_frame[frame]
        # HH_261001 - The last moving tick requests cleanup, not another BasicAgent proposal.
        if observed.get('moving_task', {}).get('termination_reason') is not None:
            require(i == len(rows) - 1 and normal is None and receipt['requested_control']['throttle'] == 0.
                    and receipt['requested_control']['brake'] == 1. and receipt['requested_control']['steer'] == 0.,
                    'terminal moving command is not the explicit final full-brake cleanup')
            result.append(dict(frame=frame, sequence=receipt['sequence'], emergency_override=False, cleanup_command=True))
            continue
        target = 2.5 if proposal_count == 0 else min(2.5, .075 * proposal_count)
        proposal_count += 1
        require(row['target_ramp']['source_frame'] == frame
                and abs(row['target_ramp']['target_speed_mps'] - target) <= 1e-12, 'causal target ramp differs')
        require(row['final_basic_agent_proposal'] == receipt['requested_control']
                and row['learned_control'] is False, 'expert proposal/ACK source differs')
        emergency = normal is None
        if normal is not None:
            require(normal['source_frame'] == frame
                    and abs(normal['source_timestamp'] - observed['timestamp']) <= 1e-9
                    and abs(normal['measured_planar_speed_mps'] - math.hypot(observed['vx'], observed['vy'])) <= 1e-9,
                    'normal policy is not bound to causal same-frame observation')
            proposal = normal['raw_normal_planner_proposal']
            capped = v2.base.control_dict(RecordedFloatControl(dict(proposal, brake=0., throttle=min(.20, proposal['throttle']))))
            require(capped == normal['revised_normal_capped_proposal'], 'normal cap/proposal differs')
            control = RecordedFloatControl(capped)
            policy.apply(control, speed=normal['measured_planar_speed_mps'], timestamp=normal['source_timestamp'],
                         frame=frame, normal_proposal=proposal)
            require(policy.last == normal, 'fixed native policy replay differs')
            emergency = ack.mismatched_controls(normal['profile_normal_control'], receipt['requested_control']) != []
        if emergency:
            require(receipt['requested_control']['throttle'] == 0.
                    and receipt['requested_control']['brake'] == emergency_brake,
                    'differing BasicAgent return is not preserved emergency braking')
        result.append(dict(frame=frame, sequence=receipt['sequence'], emergency_override=emergency))
    return dict(status='PASS', commands=len(rows), emergency_override_count=sum(row['emergency_override'] for row in result)), result


def audit(run, owner_plan_sha, manifest_sha):
    for name, digest in AUDIT_HELPER_PINS.items():
        require(sha(task.ROOT / name) == digest, 'frozen necessary-check helper differs: ' + name)
    require(run.resolve() == task.RUN, 'only the frozen fresh moving-task path is in scope')
    episode = run / 'episode'
    if not episode.exists():
        episode = run / 'episode.partial'
    require(sha(task.PLAN) == PLAN_SHA and sha(episode / 'moving_task_plan.json') == PLAN_SHA, 'prospective plan changed')
    require(sha(run / 'owner_plan.json') == owner_plan_sha and sha(episode / 'manifest.json') == manifest_sha,
            'caller-pinned owner/native identity changed')
    plan, manifest, owner, outcome = read(task.PLAN), read(episode / 'manifest.json'), read(run / 'owner_plan.json'), read(run / 'owner_result.json')
    validate_identity(run, PLAN_SHA, plan, manifest)
    require(owner['schema'] == 'carla.owned_town07_low_speed_moving.v1' and owner['task_plan_sha256'] == PLAN_SHA,
            'wrong owned execution plan')
    for name, digest in owner['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe archived source path')
        require(sha(run / 'source' / relative) == digest, 'archived execution source changed: ' + name)
    for name, digest in plan['source_sha256'].items():
        require(owner['source_sha256'].get(name) == digest, 'declared worker closure differs: ' + name)
    for module in (task, v2):
        relative = Path(module.__file__).resolve().relative_to(task.ROOT).as_posix()
        require(sha(Path(module.__file__)) == plan['source_sha256'][relative], 'replay source differs from capture')
    for name, digest in plan['input_sha256'].items():
        require(sha(Path(plan['input_paths'][name])) == digest, 'declared input changed: ' + name)
    require(sha(episode / 'route.json') == task.ROUTE_SHA, 'recorded route changed')
    require(manifest['provenance']['collector_sha256'] == plan['source_sha256']['tools/e2e/collect_carla_vad_expert.py']
            and manifest['provenance']['moving_capture_helper_sha256'] == plan['source_sha256']['tools/e2e/town07_low_speed_moving_task_20261001.py'],
            'native collector/helper identity differs')
    capture = manifest['capture_contract']
    require(capture['physics_hz'] == 20. and capture['camera_hz'] == 10.
            and capture['camera_interval_ticks'] == 2 and capture['target_speed_kmh'] == 9., 'native timing/speed configuration differs')
    physics = capture['moving_task']['physics_settings']
    require(physics['status'] == 'PASS' and physics['after'] == physics['requested']
            and physics['requested'] == dict(physics['before'], synchronous_mode=True, fixed_delta_seconds=.05)
            and physics['before']['substepping'] is True and physics['before']['max_substep_delta_time'] == .01
            and physics['before']['max_substeps'] == 10 and physics['before']['no_rendering_mode'] is False,
            'original inherited physics/readback differs')
    recorded_policy = manifest['capture_contract']['fixed_moving_normal_control']
    require(recorded_policy['task_worker_sha256'] == plan['source_sha256']['tools/e2e/collect_town07_low_speed_moving_20261001.py']
            and recorded_policy['v2_normal_policy_source_sha256'] == plan['source_sha256']['tools/e2e/collect_town07_low_speed_start_v2_20261001.py']
            and recorded_policy['terminal_goal_governor_enabled'] is False, 'normal policy identity differs')
    names = ('manifest.json', 'route.json', 'states.jsonl', 'camera_frames.jsonl', 'control_receipts.jsonl',
             'expert_moving_control.jsonl', 'active_map_opendrive.xodr', 'moving_task_plan.json')
    inputs = {str(episode / name): sha(episode / name) for name in names}
    inputs.update({str(run / name): sha(run / name) for name in ('owner_plan.json', 'owner_result.json', 'owner_moving_result.json')})
    external = manifest['runtime']['basic_agent_control']['implementation_sources']
    require(external['basic_agent']['source_sha256'] == recorded_policy['task_worker_sha256'], 'subclass implementation source differs')
    navigation = Path('/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/agents/navigation')
    external_pins = {'local_planner': ('local_planner.py', '7a5db9112b0d107038765abfb87b2d7a418b98e4d2c0aa92a4aade53c95b319f'),
                    'vehicle_controller': ('controller.py', 'f95f427a5ad5ba50a96f6971df72a5f3ec078c8457b0ba2a12558396e8b5c402')}
    for key, (name, digest) in external_pins.items():
        path = navigation / name
        require(Path(external[key]['source_file']).resolve() == path and external[key]['source_sha256'] == sha(path) == digest,
                'recorded/current external controller source differs')
        inputs[str(path)] = digest
    inherited = navigation / 'basic_agent.py'
    inherited_sha = '7eb47c0ac30f36e2e71266e918c27fc02c8ff7b74d8f75259c313022e50be2ff'
    require(sha(inherited) == inherited_sha, 'current inherited BasicAgent differs from reviewed source')
    inputs[str(inherited)] = inherited_sha
    states, cameras, receipts, proposals = [read_rows(episode / name) for name in names[2:6]]
    route = read(episode / 'route.json')
    timeline = raw.measured_timeline(states)
    pixels, image_rows, anchors = raw.camera_audit(episode, manifest, cameras, timeline)
    transport = ack.analyze_transport(states, receipts, manifest['capture_contract']['control_transport']['bootstrap_observation'], wheelbase=2.85)
    controls, control_rows = control_audit(proposals, receipts, states, manifest)
    membership, projections = reconstruct_membership(route, states, cameras)
    replay = task.reconstruct(plan, states, cameras, route['route'])
    replay_result = replay.result(tail_states=sum(s['capture_phase'] == 'stationary_tail' for s in states),
                                  goal_reached=manifest['result']['goal_reached'])
    require(replay_result == manifest['result']['moving_task'], 'native task result replay differs')
    futures, future_summary = geometry.geometry_futures(timeline, cameras, route, anchors, task.SLOT)
    actor_id = receipts[0]['actor_id']
    construction = capture['agent_initialization']['construction']
    require(construction['status'] == 'PASS' and all(construction[side]['actor_id'] == actor_id for side in ('before', 'after')),
            'initialization/body/command actor binding differs')
    shapes, body_summary = body.body_census(manifest, route, states, (episode / 'active_map_opendrive.xodr').read_bytes(), actor_id)
    native = native_checks(states, shapes)
    windows, protected = window_census(membership, cameras, native, anchors, futures)
    failures = scoped_failures(native, protected)
    owned_moving = read(run / 'owner_moving_result.json')
    require(owned_moving['original_owner_result'] == outcome and owned_moving['native_status'] == manifest['status']
            and owned_moving['task_result'] == manifest['result']['moving_task'], 'owner/native outcome records disagree')
    flags = dict(native_complete=manifest['status'] == 'complete', native_cleanup=manifest['cleanup']['completed']
        and not manifest['cleanup']['errors'], owner_clean=outcome['collector_exit_code'] == 0
        and not outcome['cleanup_errors'] and outcome['ports_released'] and outcome['error'] is None,
        source_closure_retained=owned_moving['source_hashes_match'], resource_floor=owned_moving['final_disk_floor_preserved'],
        task_complete=manifest['result']['task_complete'] and replay_result['task_complete'],
        all_membership_checks=all(membership['flags'].values()), all_pixels=pixels['all_pixel_and_metadata_checks_clear'],
        all_ack_and_snapshot_checks=all(transport['flags'].values()),
        no_events_anywhere=all(not row['collision'] and not row['lane_invasion'] for row in states),
        every_selected_window_clear=bool(windows) and all(row['necessary_window_checks_clear'] for row in windows))
    summary = dict(schema=SCHEMA, status='NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW' if all(flags.values()) else 'BLOCKED_NECESSARY_CHECKS',
        plan_sha256=PLAN_SHA, input_sha256=inputs, flags=flags, membership=membership,
        all_camera_count=len(cameras), native_count=len(states), selected_window_count=len(windows),
        protected_native_count=len(protected), window_failure_counts=dict(Counter(reason for row in windows for reason in row['failure_reasons'])),
        scoped_native_failures=failures, pixel_summary=pixels, transport=transport, control_summary=controls,
        whole_capture_body_summary=body_summary, whole_capture_future_summary=future_summary,
        whole_capture_original=dict(native_status=manifest['status'], native_error=manifest.get('error'),
            result=manifest['result'], owner=outcome, owner_moving=owned_moving),
        training_data_approved=False, dataset_conversion_performed=False, admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW',
        full_external_capture_time_source_closure_proven=False,
        authority_review_blocks=['New 9 kph fixed-window authority has not been approved.',
            'Inherited external BasicAgent bytes are currently pinned but not separately archived/bound at capture time by the frozen subclass worker.'],
        original_sealed_15kph_authority_changed=False, catalog_goal_success_claim=False,
        launch_or_stop_demonstration_claim=False, window_filtering_performed=False,
        limitations=['Necessary checks of one fixed prospective task, not an admission receipt or route success.',
            'External inherited BasicAgent current bytes are verified; this does not independently prove its capture-time byte identity.',
            'All selected anchors must pass together. Outside-protected raw kinematic/body failures remain recorded limitations.',
            'Collision/lane events, provenance, clock/ACK/pixel errors and capture/cleanup failures block globally.',
            'Body sweep is a conservative snapshot-linear envelope, not a continuous-time physical proof.',
            'Overlapping windows are one same-corridor episode, not independent successes.'])
    require(all(sha(Path(path)) == digest for path, digest in inputs.items()), 'raw input changed during audit')
    require(all(sha(episode / row['path']) == row['sha256'] for row in image_rows), 'raw JPEG changed during audit')
    return summary, dict(all_camera_futures=futures, all_native=native, all_selected_windows=windows,
                         all_image_checks=image_rows, all_projection_rows=projections, all_control_checks=control_rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=task.RUN)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--native-manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.resolve().is_relative_to(args.run.resolve()), 'fresh external audit output required')
    modules = (raw, ack, old_audit, geometry, body, task, v2, raw.adapter, raw.contract, raw.targets, body.body)
    sources = {str(Path(module.__file__).resolve()): sha(Path(module.__file__)) for module in modules}
    sources[str(Path(__file__).resolve())] = sha(Path(__file__))
    sources.update({str(task.ROOT / name): sha(task.ROOT / name) for name in AUDIT_HELPER_PINS})
    try:
        summary, ledgers = audit(args.run, args.owner_plan_sha256, args.native_manifest_sha256)
    except (ValueError, AssertionError, KeyError, OSError, IndexError, TypeError) as error:
        summary, ledgers = dict(schema=SCHEMA, status='BLOCKED_UNASSESSED_EVIDENCE', error_type=type(error).__name__,
            error=str(error), plan_sha256=PLAN_SHA, training_data_approved=False,
            dataset_conversion_performed=False, admission_authority='NONE_NEW_SCOPE_REQUIRES_REVIEW'), {}
        for relative in ('owner_result.json', 'owner_moving_result.json', 'episode/manifest.json', 'episode.partial/manifest.json'):
            path = args.run / relative
            if path.is_file():
                summary.setdefault('original_unmodified_records', {})[relative] = dict(sha256=sha(path), record=read(path))
    require(all(sha(Path(path)) == digest for path, digest in sources.items()), 'auditor source changed during execution')
    summary['audit_source_sha256'] = sources
    args.output.mkdir(parents=True)
    for name, rows in ledgers.items():
        with gzip.open(args.output / (name + '.jsonl.gz'), 'wt') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: summary[key] for key in ('status', 'training_data_approved', 'dataset_conversion_performed')}))
    return 0 if summary['status'] == 'NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW' else 1


if __name__ == '__main__':
    raise SystemExit(main())
