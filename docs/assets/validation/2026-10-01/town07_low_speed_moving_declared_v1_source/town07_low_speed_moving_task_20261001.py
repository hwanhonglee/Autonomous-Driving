"""HH_261001 - New prospective 9 kph TRAIN moving scope; no legacy qualification authority."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from scripts.e2e import carla_moving_capture as original


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1'
PLAN = ROOT / 'storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v1.json'
ROUTE = ROOT / 'storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json'
ROUTE_SHA = '8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9'
SLOT = 'T07-LOW-CRUISE-20261001-V1'
SOURCE_PATHS = tuple('tools/e2e/' + name for name in (
    'town07_low_speed_moving_task_20261001.py', 'collect_town07_low_speed_moving_20261001.py',
    'run_owned_town07_low_speed_moving_20261001.py', 'collect_carla_vad_expert.py',
    'carla_moving_capture.py', 'carla_goal_stop_profile.py', 'carla_physics_substeps.py',
    'collect_town07_low_speed_start_20261001.py', 'collect_town07_low_speed_start_v2_20261001.py'))
FIXED_ARGS = dict(host='127.0.0.1', port=2440, physics_hz=20., capture_hz=10., target_speed_kmh=9.,
    max_duration_sec=180., stationary_warmup_sec=3.5, stationary_tail_sec=6.5,
    control_transport='acknowledged_batch', agent_initialization='after_bootstrap', goal_stop_profile='disabled',
    physics_substep_profile='inherited', low_speed_steering_mode=None, hold_brake_mode=None, terminal_approach_mode=None,
    weather='ClearNoon', seed=2026100101, vehicle_type='vehicle.toyota.prius', role_name='autoware_e2e_expert',
    wheelbase_m=2.85, spawn_z_offset_m=.5, goal_tolerance_m=1., command_lookahead_m=2., command_exit_lookahead_m=2.5,
    allow_map_load=False, allow_stopped_steering=False, wall_timing=False, jpeg_quality=95, timeout=20., sensor_timeout_sec=5.,
    basic_agent_base_min_distance_m=2., basic_agent_distance_ratio=.2, basic_agent_lateral_kp=1.95,
    basic_agent_lateral_ki=.05, basic_agent_lateral_kd=.2, basic_agent_max_steering=.8, basic_agent_lane_offset_m=0.)
canonical = original.canonical
sha = original.sha
read = original.read
encoded = original.encoded
require = original.require
measured_progress = original.measured_progress
validate_original_physics = original.validate_original_physics
validate_applied_physics = original.validate_applied_physics


def pending_scope():
    return dict(admission_status='PENDING_QA', dataset_conversion_performed=False, vehicle_control_approved=False,
        qualification_notice='NEW fixed moving TRAIN task only; no existing sealed qualifier supports this slot. '
            'Future explicit versioned qualification required; no goal, launch, stop, or route-success claim.')


def plan_contract():
    return dict(schema='carla.town07_low_speed_moving_plan.20261001.v1', cohort_slot_id=SLOT,
        dataset_split_intent='train', route_sha256=ROUTE_SHA, fixed_arguments=FIXED_ARGS,
        selected_progress_m=[40., 120.], minimum_selected_camera_span_ns=30_000_000_000,
        required_history_camera_observations=10, required_history_ns=900_000_000,
        required_future_support_ns=6_400_000_000, preceding_native_guard_ns=50_000_000,
        maximum_attempts=1, automatic_retry=False, owner_wall_timeout_sec=240,
        native_cadence_ns=50_000_000, native_timestamp_rounding_tolerance_ns=1,
        selected_anchor_timestamp='Latest of six original camera timestamps, individually rounded to ns.',
        selection_rule='All driving camera anchors in inclusive independent route progress40..120m; close permanently at first native progress>120m.',
        progress_measurement='Unclamped nearest-segment planar arc from immutable recorded ROS rear-reference XY; original route/progress unchanged.',
        raw_outside_selection_retained=True, goal_completion_required=False, catalog_goal_success_claim=False,
        stop_or_dwell_success_claim=False, original_qa_or_runtime_gate_changed=False,
        controller=dict(normal_brake_cap=0., normal_throttle_cap=.20, launch_throttle=.13,
            handoff_measured_planar_speed_mps=.5, launch_timeout_s=8., post_handoff_throttle_ramp_per_s=.05,
            zero_steering_until_speed_mps=.75, target_speed_ramp_mps2=1.5, target_speed_cap_mps=2.5,
            BasicAgent_emergency_unchanged=True, whole_v2_collector_identical=False),
        membership_quality_policy='No post-capture trimming: every fixed anchor and its complete history/native guard/future must pass all original checks or task qualification fails.',
        scope_quality_policy='Whole recording native/goal quality is reported separately without changing failures. '
            'Pre-entry and post-support tail never become training anchors; selected futures cannot cross tail.',
        required_checks=['all_original_nine_10Hz_future_bounds', 'history_edges', 'native20Hz_scalar_speed_and_2p9_rate',
            'six_camera_pixels_calibration_frame_timestamps', 'ACK_snapshot_chain', 'body_and_bounded_sweep_10cm_inset',
            'no_observed_collision_or_lane_events', 'all_fixed_membership_full_history_and_future'],
        quality='Epic', expected_raw_capture_gib_upper_estimate=.5, exact_runtime_tensor_archives=False,
        route_group_independence='NOT_CERTIFIED: same existing TRAIN Town07 corridor', **pending_scope())


def build_plan(mapping, calibration):
    paths = dict(route_file=ROUTE, mapping=canonical(mapping), calibration=canonical(calibration))
    require(sha(ROUTE) == ROUTE_SHA, 'fixed TRAIN route differs')
    return dict(plan_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        output=str(RUN / 'episode'), input_paths={key: str(path) for key, path in paths.items()},
        input_sha256={key: sha(path) for key, path in paths.items()},
        source_sha256={name: sha(ROOT / name) for name in SOURCE_PATHS})


def configuration_from_args(args):
    require(canonical(args.moving_task_plan) == PLAN and sha(PLAN) == args.moving_task_plan_sha256,
            'exact prospective new-task plan required')
    plan, fixed = read(PLAN), plan_contract()
    require(set(plan) == set(fixed) | {'declared_at_utc', 'output', 'input_paths', 'input_sha256', 'source_sha256'},
            'task plan fields differ')
    require(encoded({key: plan[key] for key in fixed}) == encoded(fixed), 'fixed moving task changed')
    require(canonical(args.output) == RUN / 'episode' and plan['output'] == str(RUN / 'episode'), 'fresh task output differs')
    stamp = datetime.fromisoformat(plan['declared_at_utc'])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0 and stamp <= datetime.now(timezone.utc),
            'valid prior UTC declaration required')
    for key, expected in FIXED_ARGS.items():
        require(getattr(args, key, None) == expected, 'fixed new-task CLI differs: ' + key)
    require(set(plan['source_sha256']) == set(SOURCE_PATHS), 'source closure differs')
    for name, digest in plan['source_sha256'].items():
        require(sha(ROOT / name) == digest, 'frozen source changed: ' + name)
    require(set(plan['input_paths']) == set(plan['input_sha256']) == {'route_file', 'mapping', 'calibration'}, 'input closure differs')
    for key, name in plan['input_paths'].items():
        require(canonical(getattr(args, key)) == canonical(name) and sha(Path(name)) == plan['input_sha256'][key],
                'frozen input differs: ' + key)
    return plan


class MovingTask(original.MovingTask):
    """HH_261001 - New closed geometric membership; original sealed 15 kph class remains untouched."""
    def __init__(self, plan):
        super().__init__(plan)
        self.camera_history, self.native_history = [], []
        self.history_support = None

    def observe(self, *, frame, timestamp, phase, progress, base_stop_reason, event_count):
        require(type(frame) is int and phase in ('stationary_warmup', 'driving', 'stationary_tail'), 'invalid native identity')
        import math
        require(all(type(value) in (int, float) and math.isfinite(value) for value in (timestamp, progress)), 'nonfinite native observation')
        now = round(timestamp * 1e9)
        if self.previous_time is not None and abs(now - self.previous_time - 50_000_000) > 1:
            self.fail('moving_native_cadence_failure')
        self.previous_time = now
        self.native_history.append(dict(frame=frame, timestamp_ns=now, phase=phase))
        if event_count:
            self.fail('moving_observed_collision_or_lane_event')
        if phase == 'driving' and self.support is None:
            if self.entered and self.previous_progress is not None and progress < self.previous_progress:
                self.fail('moving_progress_decreased_or_reentered')
            self.previous_progress = progress
            if progress > 120. and self.closed is None:
                self.closed = dict(frame=frame, timestamp_ns=now, measured_route_progress_m=progress)
                if self.first is None or self.last['timestamp_ns'] - self.first['timestamp_ns'] < 30_000_000_000:
                    self.fail('moving_selected_span_under_30_seconds')
            if base_stop_reason is not None:
                self.fail('moving_catalog_or_agent_end_before_support')
            if self.closed is not None and self.last is not None and now >= self.last['timestamp_ns'] + 6_400_000_000 and self.failure is None:
                self.support = dict(frame=frame, timestamp_ns=now, required_timestamp_ns=self.last['timestamp_ns'] + 6_400_000_000)
        reason = self.failure or ('moving_task_future_support_complete' if self.support is not None and phase == 'driving' else None)
        self.latest = dict(schema='carla.town07_low_speed_moving_observation.v1', measured_route_progress_m=progress,
            source_frame=frame, source_timestamp_ns=now, selection_open=self.closed is None,
            selected_camera_saved=False, camera_selection_eligible=phase == 'driving' and self.closed is None and 40. <= progress <= 120.,
            window_closed_at=self.closed, future_support_complete=self.support is not None, termination_reason=reason)
        return self.latest, reason

    def saved_camera(self, camera):
        if self.latest['camera_selection_eligible'] and self.first is None:
            history = [*self.camera_history[-9:], dict(frame=camera['frame'], timestamp_ns=round(camera['timestamp'] * 1e9), phase=camera['capture_phase'])]
            valid = (len(history) == 10 and all(row['phase'] == 'driving' for row in history)
                and all(b['frame'] == a['frame'] + 2 for a, b in zip(history, history[1:]))
                and history[-1]['timestamp_ns'] - history[0]['timestamp_ns'] >= 900_000_000)
            guard = next((row for row in self.native_history if row['frame'] == history[0]['frame'] - 1), None)
            valid = valid and guard is not None and guard['phase'] == 'driving'
            if not valid:
                self.fail('moving_first_anchor_full_history_missing')
            self.history_support = dict(status='PASS' if valid else 'FAIL', camera_observations=history, preceding_native_guard=guard)
        super().saved_camera(camera)
        self.camera_history.append(dict(frame=camera['frame'], timestamp_ns=round(camera['timestamp'] * 1e9), phase=camera['capture_phase']))

    def result(self, *, tail_states=0, goal_reached=False):
        result = super().result(tail_states=tail_states, goal_reached=goal_reached)
        result.update(schema='carla.town07_low_speed_moving_result.v1', history_support=self.history_support,
            cohort_slot_id=SLOT, selected_progress_m=[40., 120.], **pending_scope())
        result['task_complete'] = result['task_complete'] and self.history_support is not None and self.history_support['status'] == 'PASS'
        return result


def reconstruct(plan, states, cameras, route_points):
    by_frame = {row['frame']: row for row in cameras}
    require(len(by_frame) == len(cameras), 'duplicate saved camera frame')
    task = MovingTask(plan)
    for row in states:
        stored = row['moving_task']
        progress = measured_progress(route_points, row['x'], row['y'])
        require(progress == stored['measured_route_progress_m'], 'raw moving projection differs')
        observed, _ = task.observe(frame=row['frame'], timestamp=row['timestamp'], phase=row['capture_phase'],
            progress=progress, base_stop_reason=stored['original_termination_reason'], event_count=stored['observed_event_count'])
        if row['frame'] in by_frame:
            camera = by_frame[row['frame']]
            copied = dict(camera)
            require(camera['capture_phase'] == row['capture_phase'], 'camera/native phase differs')
            task.saved_camera(copied)
            require(copied['moving_task_selected'] == camera['moving_task_selected'], 'saved membership differs')
        require(all(stored[key] == value for key, value in observed.items()), 'native moving replay differs')
    return task
