"""HH_261002 - One prospective full-route manual1 coast-to-goal teacher; no native denial override or TRAIN permission."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace
import math

from scripts.e2e import town07_natural_goal_stop_task_20261002 as previous
from scripts.e2e import calibrate_town07_manual_first_coast_20261002 as measured_coast

ROOT, ROUTE, ROUTE_SHA = previous.ROOT, previous.ROUTE, previous.ROUTE_SHA
FLOOR, RESERVE, QUOTA = previous.FLOOR, previous.RESERVE, previous.QUOTA
require, sha, read, encoded, canonical = previous.require, previous.sha, previous.read, previous.encoded, previous.canonical
original, goal = previous.original, previous.goal
measured_progress = previous.measured_progress
validate_original_physics, validate_applied_physics = previous.validate_original_physics, previous.validate_applied_physics
SCHEMA = "carla.actual_natural_goal_stop_teacher.20261002.v4"
PROFILE = "natural_goal_stop_manual1_1p50_coast11p5_hold0p05_brakefree.20261002.v4"
SEED = 2026102400
FIXED_ARGS = dict(previous.FIXED_ARGS, target_speed_kmh=5.4, max_duration_sec=200., seed=SEED)
CRUISE, COAST_DISTANCE, COAST_ENTRY_BAND, COAST_TIMEOUT = 1.5, 11.5, (1.48, 1.58), 45.
FROZEN = {
    "tools/e2e/town07_natural_goal_stop_brakefree_v3_20261002.py": "d08a03b254eeae25c7dbaef2c424372bf7906887cdd403f64d876b9d3f19ab4f",
    "tools/e2e/collect_town07_natural_goal_stop_brakefree_v3_20261002.py": "3492185749d1cd5874e96c0ec4f08323a68c968ce5628a68b8b258b00a5a6b52",
    "tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v3_20261002.py": "8a89592252c5e8556c068c9f78960075f9220eea46b4ec2875d680505cc82759",
    "tools/e2e/town07_natural_goal_stop_task_20261002.py": "a636058d890c6c28d47bec9de8c134ccaff3594c724ed67860ed8fd16324f35b",
    "tools/e2e/collect_town07_natural_goal_stop_20261002.py": "8eee92c1c83535316ff19189823faa898b10a3344eb6e43eb5844a94f184ce72",
    "tools/e2e/run_owned_town07_natural_goal_stop_20261002.py": "f05fdf610dbd8ec9ec410beb858af1c588b120f5a13b5cc39943d47d0a7eb1ce",
    "tools/e2e/run_owned_town07_natural_goal_stop_v2_20261002.py": "9d5923a7c715d0da536eed248cd1e0b6e49bb2a14fc5f31604297536417365e4",
    "tools/e2e/calibrate_town07_manual_first_coast_20261002.py": "decb509d3c6079861e008a9208c99c4be08bd9db6c8cfd1a83e6db169168abec",
}
COAST_EVIDENCE = {
    "declaration.json": "c0377bfbde57269cf2679c1e2dbfdb1d3c5cac3b491627680e039dfd3f252617",
    "calibration/result.json": "f164e7a9f1e48c43a07f17931076f84386bc45840b5dfe1581b331b5c2eb1dc8",
    "calibration/states.jsonl": "c1fbd84215e0089ff6ca0d37f127e5fc6541d64664e480a15d822d5198c59d4e",
    "calibration/control_receipts.jsonl": "dc9383d3ec4cd53761cbe6b8ba00227c67e7b9c1c2328f12feefda87427f1684",
    "diagnostic_report.json": "60772355a608afb7349eec0fe45cf9f5934bbd08d3cd0a11641a7a663cda4347",
}
EVIDENCE_ROOT = ROOT/"storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1"
_fixed, _configuration, _reconstruct, _pending = previous.fixed_contract, previous.configuration_from_args, previous.reconstruct, previous.pending_scope
_active = False


def verify_frozen():
    for name, value in FROZEN.items(): require(sha(ROOT/name) == value, "frozen goal/coast source changed: "+name)
    for name, value in COAST_EVIDENCE.items(): require(sha(EVIDENCE_ROOT/name) == value, "actual coast reference changed: "+name)


def paths(run_id, case_id=None):
    require(run_id == "v4" and case_id in (None,"pilot_00"), "one fresh explicitly named brakefree v4 pilot only")
    batch = ROOT/"storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v4"
    return (batch,batch.with_name(batch.name+"_plan.json")) if case_id is None else (batch/case_id,batch/(case_id+"_plan.json"))


def goal_configuration(args):
    require(args.goal_stop_profile == "comfortable_v1" and args.target_speed_kmh == 5.4 and args.max_duration_sec == 200.
        and args.physics_hz == 20. and args.capture_hz == 10. and args.goal_tolerance_m == 1. and args.stationary_tail_sec == 6.5,
        "exact NEW 1.5m/s brakefree mission configuration required")
    return replace(goal.ComfortableGoalStopConfig(), profile_id=PROFILE, stopped_speed_mps=.05,
        normal_throttle_cap=.20, normal_brake_cap=0.)


def mission_contract():
    return dict(schema=SCHEMA+".causal_coast_hold", cruise_speed_mps=CRUISE,
        coast_release_remaining_arc_m=COAST_DISTANCE, actual_coast_entry_speed_band_mps=list(COAST_ENTRY_BAND),
        maximum_coast_to_completion_seconds=COAST_TIMEOUT, cruise_and_coast_normal_brake=0.,
        coast_normal_throttle=0., no_throttle_reentry_after_coast_latch=True,
        service_hold_brake=1., service_hold_first_request_requires_measured_goal_and_speed=True,
        measured_goal_tolerance_m=1., measured_stopped_speed_mps=.05, continuous_hold_native_intervals=40,
        fullbrake_before_low_speed_hold_allowed_only_for_original_hazard_abort_or_setup=True,
        actual_remaining_speed_pose_frame_not_rewritten=True, source_speed_reference="actual sameframe rear-base hypot(vx,vy)",
        original_lateral_planner_and_downstream_BasicAgent_hazard_unchanged=True,
        coast_reference_entry_mps=1.5280090765616845, coast_reference45s_progress_m=10.960053314189631,
        empirical_single_condition_reference_not_stopping_distance_guarantee=True,
        reference_not_extrapolated_to2p4_mps=True, reference_raw_sha256={str(EVIDENCE_ROOT/k):v for k,v in COAST_EVIDENCE.items()},
        nominal_speed_and_timeout_changed_explicitly=True, attempts=1, automatic_retry=False,
        semantic_mission_phases=["GO","BRAKE_TO_STOP","HOLD"], brake_intent_is_not_positive_brake_pedal_label=True,
        future64_QA_and_stationary_curvature_masks_separate=True, training_data_approved=False, vehicle_control_approved=False)


def pending_scope():
    result = _pending()
    result["qualification_notice"] = "NEW prospective full-route1.5m/s latched brakefree coast/observed low-speed service HOLD; truefuture64 QA and separate admission required."
    return result


@contextmanager
def activated():
    global _active
    require(not _active, "brakefree task provider cannot nest")
    names = dict(SCHEMA=SCHEMA,PROFILE=PROFILE,SEED=SEED,FIXED_ARGS=FIXED_ARGS,paths=paths,pending_scope=pending_scope,
        goal_configuration=goal_configuration,fixed_contract=fixed_contract,MovingTask=MovingTask)
    saved = {k:getattr(previous,k) for k in names}; _active = True
    try:
        for k,v in names.items():setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active = False


@contextmanager
def bound():
    if _active:yield
    else:
        with activated():yield


def fixed_contract(run_id, case_id):
    with bound():result = _fixed(run_id,case_id)
    result.update(nominal_teacher_cruise_mps=CRUISE, teacher="NEW latched1.5m/s zero-normal-brake coast plus observed near-stop service HOLD; original BasicAgent lateral/hazards.",
        causal_brakefree_terminal_mission=mission_contract(), frozen_goal_and_coast_source_sha256=dict(FROZEN),
        bootstrap_native_separately_retained_in_transport_manifest=True,
        all_captured_warmup_driving_hold_tail_and_bootstrap_assessed=True,
        native_signed_speed_rate_bounds_mps2=[-6.,3.], native_absolute_speed_rate_bound_mps2=2.9)
    result["controller"].update(normal_brake_cap=0.,service_hold_brake=1.,
        governor_math_changed_only_by_disclosed_coast_latch=True,original_measured2s_goal_dwell_math_unchanged=True,
        brake_positive_never_replaced_by_launch_throttle=False,
        original_positive_normal_brake_proposals_retained_before_disclosed_zero_cap=True)
    result["closing_event"] = "Original measured upstream goal<=1m/v<=.05 and continuous40nativeintervals; not BasicAgent.done or first low-speed sample."
    return result


def configuration_from_args(args):
    with bound():return _configuration(args)


class MovingTask(previous.MovingTask):
    """HH_261002 - Preserve every original pending admission, causal anchor and true-future support event."""
    def __init__(self,plan):
        with bound():super().__init__(plan)
    def observe(self,**kwargs):
        with bound():return super().observe(**kwargs)
    def result(self,**kwargs):
        with bound():return super().result(**kwargs)


def reconstruct(plan,states,cameras,route_points):
    with bound():return _reconstruct(plan,states,cameras,route_points)


class CoastToGoalGovernor(goal.GoalStopGovernor):
    """HH_261002 - Keep frozen observed dwell arithmetic; explicitly add one causal coast and low-speed service-hold latch."""
    def __init__(self,config,cruise_speed_mps,physics_hz):
        expected = goal_configuration(type("Args",(),FIXED_ARGS)())
        require(config == expected and cruise_speed_mps == CRUISE and physics_hz == 20., "exact brakefree governor required")
        super().__init__(config,cruise_speed_mps,physics_hz)
        self.coast = self.service_hold = self.failure_reason = self.last_source = None
        self.duplicate_engagement_used = False

    def update(self,remaining_arc_m,planar_distance_m,speed_mps,terminal_overshoot,*,count_hold=True,
               source_frame,source_timestamp,source_state):
        require(type(source_frame) is int and type(count_hold) is bool and type(source_timestamp) in (int,float)
            and source_frame>=0 and math.isfinite(source_timestamp) and source_timestamp>=0
            and all(type(v) in (int,float) and math.isfinite(v) for v in (remaining_arc_m,planar_distance_m,speed_mps,terminal_overshoot))
            and isinstance(source_state,dict) and all(type(source_state.get(k)) in (int,float) and math.isfinite(source_state[k]) for k in ("vx","vy")),
            "actual finite non-Boolean source measurements required")
        require(abs(speed_mps-math.hypot(source_state["vx"],source_state["vy"]))<=1e-12, "governor speed differs from sameframe rear-base state")
        if self.last_source is not None:
            previous_frame,previous_time = self.last_source
            duplicate = source_frame == previous_frame and source_timestamp == previous_time
            if duplicate:
                require(not self.duplicate_engagement_used and not count_hold and self.coast is None,
                    "only one original no-tick initial engagement update may duplicate a frame")
                self.duplicate_engagement_used = True
            else:require(source_frame == previous_frame+1 and abs(source_timestamp-previous_time-.05)<=1e-6, "native governor source chain differs")
        self.last_source = source_frame,source_timestamp
        record = super().update(remaining_arc_m,planar_distance_m,speed_mps,terminal_overshoot,count_hold=count_hold)
        transition = None
        if count_hold and self.coast is None and remaining_arc_m <= COAST_DISTANCE:
            if not COAST_ENTRY_BAND[0] <= speed_mps <= COAST_ENTRY_BAND[1]:self.failure_reason = "brakefree_coast_entry_outside_predeclared_actual_speed_band"
            elif self.failure_reason is None:
                self.coast = dict(frame=source_frame,timestamp=source_timestamp,remaining_route_arc_m=remaining_arc_m,measured_speed_mps=speed_mps)
                transition = "GO_to_latched_zero_pedal_BRAKE_TO_STOP"
        if self.coast is not None and count_hold and self.service_hold is None and self.failure_reason is None \
                and record["goal_window"] and record["upstream_of_endpoint"] and terminal_overshoot<=0. and speed_mps<=.05:
            self.service_hold = dict(frame=source_frame,timestamp=source_timestamp,remaining_route_arc_m=remaining_arc_m,
                planar_goal_distance_m=planar_distance_m,terminal_overshoot_m=terminal_overshoot,measured_speed_mps=speed_mps)
            transition = "measured_near_goal_low_speed_to_service_HOLD"
        elapsed = None if self.coast is None else source_timestamp-self.coast["timestamp"]
        if self.coast is not None and count_hold and elapsed>=COAST_TIMEOUT and not record["complete"] and self.failure_reason is None:
            self.failure_reason = "brakefree_coast_goal_completion_timeout"
        if self.coast is not None or self.failure_reason is not None:self.target_speed_mps = 0.
        phase = "HOLD" if self.service_hold is not None else "BRAKE_TO_STOP" if self.coast is not None else "GO"
        record.update(target_speed_mps=self.target_speed_mps, terminal_mission_phase=phase,coast_source_measurements=self.coast,
            low_speed_service_hold_source=self.service_hold,coast_elapsed_s=elapsed,pilot_transition=transition,
            pilot_failure_reason=self.failure_reason,source_frame=source_frame,source_timestamp=source_timestamp,
            frozen_measured_dwell_math_unchanged=True,coast_target_override_declared=True,
            semantic_receipt_not_TRAIN_authorization=True,training_admission_state="PENDING_QA_NOT_AUTHORIZED")
        return record


def full_native_quality(manifest,states):
    # HH_261002 - Include the independently retained bootstrap-to-first-warmup edge and every captured tail edge.
    bootstrap = dict(manifest["capture_contract"]["control_transport"]["bootstrap_observation"],capture_phase="pre_capture_bootstrap")
    report = measured_coast.quality([bootstrap]+states)
    report.pop("cleanup_excluded_from_quality_but_raw_retained",None)
    report.update(bootstrap_separately_retained_not_TRAIN_or_camera=True,full_captured_warmup_driving_HOLD_tail_included=True,
        bootstrap_native_count=1,recorded_native_count=len(states),native_interval_denominator=len(states),
        scalar_speed_rate_reference="Actual rear-base planar hypot(vx,vy) finite difference; NOT actor-world/API acceleration magnitude",
        coast_motion_encoded_in_original_driving_phase=True)
    return report
