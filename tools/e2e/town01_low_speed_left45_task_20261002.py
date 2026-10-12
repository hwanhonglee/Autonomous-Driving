"""HH_261002 - One prospective actual TRAIN-corridor LEFT teacher; static clearance is not data admission."""
from __future__ import annotations

import ast
import copy
from datetime import datetime, timezone
import math
from pathlib import Path
import shutil

from scripts.e2e import town07_signed_recovery_task_20261001 as parent

ROOT = parent.ROOT
SCHEMA = "carla.actual_town01_left45_teacher.20261002.v1"
ROUTE = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/left/town01_left_s0002_p03.json"
ROUTE_SHA = "fb6741ff24da5ced4a14dd6e97370a710e4a0e426c0ae3b82e8b011d55bf7560"
PARENT_SHA = "fddd47336e5a914f89db60dc7dccfa1dd613c3917125b213754de86a83958ecb"
FLOOR, RESERVE, QUOTA = 5*1024**3, 64*1024**2, 3*1024**3
FIRST_FRAME, LAST_FRAME, ANCHORS = 80, 772, 347
FIRST_NS, LAST_NS = 4_000_000_000, 38_600_000_000
NATIVE_COUNT, CAMERA_COUNT = 1055, 528
CASE = dict(case_id="pilot_00", split="train", seed=2026102500,
    signed_lateral_offset_ros_m=0., signed_yaw_offset_ros_deg=0.)
CASES = (CASE,)
FIXED_ARGS = dict(parent.FIXED_ARGS, max_duration_sec=60.)
require, sha, read, encoded, canonical = parent.require, parent.sha, parent.read, parent.encoded, parent.canonical
original = parent.original
measured_progress = parent.measured_progress
validate_original_physics, validate_applied_physics = parent.validate_original_physics, parent.validate_applied_physics
perturb_actor_center = parent.perturb_actor_center
pending_scope = parent.pending_scope


def verify_parent():
    require(sha(Path(parent.__file__)) == PARENT_SHA, "immutable recovery membership provider changed")


def case(case_id):
    require(case_id == "pilot_00", "only one fixed LEFT teacher pilot is supported")
    return dict(CASE)


def paths(run_id, case_id=None):
    require(run_id == "v1", "fresh LEFT45 v1 only; no retries or arbitrary revisions")
    batch = ROOT/"storage/experiments/validation/2026-10-02/town01_low_speed_left45_teacher_v1"
    if case_id is None: return batch, batch.with_name(batch.name+"_plan.json")
    case(case_id)
    return batch/case_id, batch/(case_id+"_plan.json")


def clone_node(name, kind=ast.FunctionDef):
    verify_parent()
    nodes = [n for n in ast.parse(Path(parent.__file__).read_bytes()).body if isinstance(n, kind) and n.name == name]
    require(len(nodes) == 1, "exact original definition required: "+name)
    return copy.deepcopy(nodes[0])


def fixed_contract(run_id, case_id):
    # HH_261002 - Retain the authenticated old declaration guard, but explicitly declare this new single rolling LEFT protocol.
    node = clone_node("fixed_contract")
    namespace = dict(globals())
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), __file__, "exec"), namespace)
    result = namespace["fixed_contract"](run_id, case_id)
    result.update(town="Town01", scenario="left", train_case_count=1, holdout_case_count=0,
        perturbation="Zero lateral/yaw perturbation; actual original TRAIN route and recorded spawn remain unchanged.",
        selected_driving_native_frames=[FIRST_FRAME, LAST_FRAME], selected_driving_relative_time_ns=[FIRST_NS, LAST_NS],
        observed_selected_span_aggregate_clock_tolerance_ns=LAST_FRAME-FIRST_FRAME,
        expected_fixed_anchor_count=ANCHORS, expected_native_states=NATIVE_COUNT,
        expected_camera_bundles=CAMERA_COUNT, expected_JPEG_count=CAMERA_COUNT*6,
        selection_clock="Native offsets80..772 inclusive, every2frames; actual timestamps NEVER snapped.",
        selection_rule="ALL347 prospective rolling anchors4.0..38.6s; no quality-conditioned selection or retry.",
        closure_rule="Unchanged normal teacher through last actual anchor+6.4s; retain130 diagnostic context ticks, never supervised STOP.",
        first_history_driving_relative_time_ns=[3_100_000_000, FIRST_NS], preceding_history_native_guard_phase="driving",
        dataset_training_scope="PROSPECTIVE_REAL_LOW_SPEED_LEFT_ROLLING_ONLY_NOT_STARTUP_OR_STOP_TRAIN",
        controller=dict(normal_brake_cap=0., normal_throttle_cap=.20, launch_throttle=.11,
            handoff_speed_mps=.5, launch_timeout_s=8., zero_normal_steering_until_mps=.75,
            post_handoff_throttle_ramp_per_second=.025, target_speed_ramp_mps2=1.5,
            target_speed_cap_mps=2.4, manual_gear_shift=True, gear=1, BasicAgent_emergency_unchanged=True),
        teacher="Original BasicAgent lateral/hazards; disclosed .11/.025 manual-first normal control, actual2.40m/s target ramp; no learned inference.",
        initialization_gear_contract=dict(actual_manual_gear_shift=True, actual_gear=1,
            parent_function_source_sha256="815ef4c5806d1914eadfdd12eba1a42c392ceee58521b6f786d3c83349372695",
            all_original_frame_actor_finite_PID_and_full_brake_guards_preserved=True, observed_control_or_snapshot_proxy_used=False),
        diagnostic_tail_policy=dict(native_ticks=130, throttle=0., brake=0., steer=0., gear=1, manual_gear_shift=True,
            context_only=True, stationary_or_STOP_supervision_claimed=False,
            full_brake_cleanup_after_last_recorded_state_or_abort_only=True),
        whole_native_quality=dict(rear_base_planar_speed_ceiling_mps=2.5,
            protected_history_and_future_scalar_dv_dt_physical_abs_mps2=2.9,
            scalar_dv_dt_runtime_bounds_mps2=[-6., 3.], no_tail_or_startup_runtime_exemptions=True,
            all_native_physical_2_9_diagnostic_retained=True),
        future_qualification_required=True, true_future_point_count=ANCHORS*64,
        causal_navigation_support_m=18., actual_turn_minimum_signed_yaw_deg=60.,
        actual_turn_exit_plus_progress_m=10., turn_exit_required_before_last_selected_anchor=True,
        traffic_fixture=dict(state="Green", frozen=True, original_state_and_durations_restored=True,
            ignore_traffic_lights=False, all_original_hazard_detectors_unchanged=True,
            closed_course_only_not_traffic_rule_performance=True),
        full_original_route_retained=True, static_nominal_clearance_is_not_actual_capture_or_TRAIN=True,
        heldout_seeds_reserved=False, route_group_independence="EXISTING_TRAIN_CORRIDOR_NOT_INDEPENDENT_SITE",
        startup_training_or_startup_success_claimed=False, normal_stop_or_full_route_success_claim=False)
    return result


def moving_class():
    node = clone_node("MovingTask", ast.ClassDef)
    counts = dict(end=0, select=0, span=0, clock=0, tolerance=0, guard=0, guard_flag=0, count=0, times=0)
    for method in node.body:
        if not isinstance(method, ast.FunctionDef): continue
        for item in ast.walk(method):
            if method.name == "observe" and isinstance(item, ast.Constant) and type(item.value) is int:
                replacements = {620: (LAST_FRAME, "end"), 602: (LAST_FRAME-FIRST_FRAME, "span"),
                    30_100_000_000: (LAST_NS-FIRST_NS, "clock")}
                if item.value in replacements:
                    item.value, key = replacements[item.value]; counts[key] += 1
            if method.name == "observe" and isinstance(item, ast.keyword) and item.arg == "camera_selection_eligible":
                for value in ast.walk(item.value):
                    if isinstance(value, ast.Constant) and type(value.value) is int and value.value == 18:
                        value.value=FIRST_FRAME; counts["select"]+=1
            if method.name == "saved_camera" and isinstance(item, ast.Constant) and item.value == "stationary_warmup":
                item.value="driving"; counts["guard"]+=1
            if method.name == "saved_camera" and isinstance(item, ast.keyword) and item.arg == "preceding_guard_may_be_stationary_warmup":
                require(isinstance(item.value, ast.Constant) and item.value.value is True, "original guard flag differs")
                item.value.value=False; counts["guard_flag"]+=1
            if method.name == "result" and isinstance(item, ast.Constant) and type(item.value) is int:
                if item.value == 302: item.value=ANCHORS; counts["count"]+=1
                if item.value in (900_000_000, 31_000_000_000):
                    item.value=FIRST_NS if item.value==900_000_000 else LAST_NS; counts["times"]+=1
    require(counts == dict(end=2, select=1, span=2, clock=1, tolerance=0, guard=1, guard_flag=1, count=2, times=2),
        "only exact prospective membership/history literals may change")
    return node, counts


# HH_261002 - No old module globals are patched: authenticated numerical/replay code runs in this new task namespace.
_node, MEMBERSHIP_AST_SUBSTITUTIONS = moving_class()
_definitions = [_node, clone_node("configuration_from_args"), clone_node("reconstruct")]
exec(compile(ast.fix_missing_locations(ast.Module(body=_definitions, type_ignores=[])), __file__, "exec"), globals())
