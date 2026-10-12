"""HH_261002 - One prospective natural goal-stop teacher pilot; no model or TRAIN admission."""

from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import math
from pathlib import Path
import shutil

from scripts.e2e import carla_moving_capture as original
from scripts.e2e import carla_goal_stop_profile as goal
from scripts.e2e import town07_signed_recovery_task_20261001 as geometry_source

ROOT, ROUTE, ROUTE_SHA = geometry_source.ROOT, geometry_source.ROUTE, geometry_source.ROUTE_SHA
SCHEMA = "carla.actual_natural_goal_stop_teacher.20261002.v1"
PROFILE = "natural_goal_stop_manual1_2p40_hold0p05.20261002.v1"
SEED = 2026102200
FLOOR, RESERVE, QUOTA = 5*1024**3, 64*1024**2, 1024**3
require, sha, read, encoded, canonical = original.require, original.sha, original.read, original.encoded, original.canonical
measured_progress = original.measured_progress
validate_original_physics, validate_applied_physics = original.validate_original_physics, original.validate_applied_physics
FIXED_ARGS = dict(geometry_source.FIXED_ARGS, target_speed_kmh=8.64, max_duration_sec=140.,
    goal_stop_profile="comfortable_v1", seed=SEED)


def paths(run_id, case_id=None):
    require(run_id == "v1" and case_id in (None, "pilot_00"), "one fresh explicitly versioned pilot only")
    batch = ROOT/"storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_v1"
    return (batch, batch.with_name(batch.name+"_plan.json")) if case_id is None else (batch/case_id, batch/(case_id+"_plan.json"))


def pending_scope():
    # HH_261002 - Absence of a Boolean admission key means pending, not permission to train.
    return dict(admission_status="PENDING_QA", training_admission_state="PENDING_QA_NOT_AUTHORIZED",
        dataset_conversion_performed=False, learned_control=False, model_promoted=False,
        vehicle_control_approved=False, qualification_notice="Prospective actual normal-braking teacher pilot; full future/body/source QA and separate explicit admission still required.")


def goal_configuration(args):
    # HH_261002 - The NEW wrapper is the opt-in; retain original governor math, not an old development profile.
    require(args.goal_stop_profile == "comfortable_v1" and args.target_speed_kmh == 8.64
        and args.physics_hz == 20. and args.capture_hz == 10. and args.goal_tolerance_m == 1.
        and args.stationary_tail_sec == 6.5, "fixed natural-goal configuration required")
    return replace(goal.ComfortableGoalStopConfig(), profile_id=PROFILE, stopped_speed_mps=.05,
        normal_throttle_cap=.20)


def fixed_contract(run_id, case_id):
    output, plan = paths(run_id, case_id)
    return dict(schema=SCHEMA, run_id=run_id, case_id=case_id, split="pilot_pending_no_fit", seed=SEED,
        output=str(output/"episode"), declaration_path=str(plan), fixed_arguments=dict(FIXED_ARGS), route_sha256=ROUTE_SHA,
        route_length_m=210.5975914062836, town="Town07", scenario="straight", weather="ClearNoon",
        natural_goal_configuration=asdict(goal_configuration(type("Args", (), FIXED_ARGS)())),
        nominal_teacher_cruise_mps=2.4, selected_first_driving_relative_native_frame=80,
        selected_driving_first_seconds=4., minimum_selected_span_ns=30_000_000_000,
        selection_rule="Every six-camera driving anchor from native driving offset80 through actual measured2s goal HOLD completion, inclusive; no quality cropping.",
        closing_event="Original GoalStopGovernor measured stop+dwell termination only; never BasicAgent.done or moving-task support alone.",
        required_future_support_ns=6_400_000_000, stationary_tail_native_ticks=130,
        stationary_tail_can_be_true_future_context=True, stationary_tail_is_training_anchor=False,
        startup_retained_as_raw_context=True, startup_anchors_selected=False,
        stop_reference="Rear-base catalog endpoint minus .75m buffer; catalog goal tolerance <=1m. NOT a bumper-relative stop-line approval.",
        measured_goal_window_m=1., measured_stopped_speed_mps=.05, measured_continuous_hold_seconds=2.,
        actual_speed_or_pose_clipped=False, route_as_expert_future_label=False,
        teacher="Original BasicAgent hazards and GoalStopGovernor target; NEW disclosed manual1/.11 launch with normal PID brakes preserved.",
        controller=dict(launch_throttle=.11, handoff_speed_mps=.5, launch_timeout_s=8.,
            post_handoff_throttle_ramp_per_second=.025, post_handoff_cap=.20,
            normal_brake_cap=.10, startup_only_zero_steering_until_mps=.75,
            governor_target_never_reset_to_cruise=True, brake_positive_never_replaced_by_launch_throttle=True,
            manual_gear_shift=True, gear=1, unchanged_BasicAgent_hazard_override=True),
        native_actor_speed_emergency_mps=2.7, rear_base_absolute_CTE_emergency_m=1.,
        episode_quota_bytes=QUOTA, free_disk_floor_bytes=FLOOR, closing_reserve_bytes=RESERVE,
        attempts=1, automatic_retry=False, all_failed_attempts_retained=True,
        full_body_ten_centimeter_inset_and_sweeps_required=True, actual_BGRA_SHA_and_JPEG_SHA_every_camera=True,
        learned_goal_stop_success_claim=False, full_RTE04_approved=False, main_functions_approved=0,
        subfunctions_approved=0, formal_feature_denominators=dict(main=9, sub=30), **pending_scope())


def configuration_from_args(args):
    run_id = args.natural_goal_run_id
    output, path = paths(run_id, "pilot_00")
    require(canonical(args.moving_task_plan) == path and sha(path) == args.moving_task_plan_sha256, "exact prior pilot declaration required")
    plan, fixed = read(path), fixed_contract(run_id, "pilot_00")
    require(set(plan) == set(fixed)|{"declared_at_utc", "input_paths", "input_sha256", "source_sha256", "external_source_sha256",
        "asset_sha256", "owner_output", "server_command", "worker_command", "owner_wall_timeout_s"}, "pilot declaration fields differ")
    require(encoded({k:plan[k] for k in fixed}) == encoded(fixed), "prospective pilot contract changed")
    require(canonical(args.output) == output/"episode", "pilot output differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0 and stamp <= datetime.now(timezone.utc), "prior UTC declaration required")
    for key, value in FIXED_ARGS.items(): require(type(getattr(args,key,None)) is type(value) and getattr(args,key) == value, "pilot argument differs: "+key)
    require(set(plan["input_paths"]) == set(plan["input_sha256"]) == {"route_file","mapping","calibration"}, "exact input closure required")
    for key,name in plan["input_paths"].items():
        require(canonical(getattr(args,key)) == canonical(name) and sha(name) == plan["input_sha256"][key], "pilot asset differs: "+key)
    require(plan["input_sha256"]["route_file"] == ROUTE_SHA, "original route bytes differ")
    for name, value in plan["source_sha256"].items(): require(sha(ROOT/name) == value, "pilot source changed: "+name)
    for name, value in plan["external_source_sha256"].items(): require(sha(name) == value, "pilot API source changed: "+name)
    goal_configuration(args)
    return plan


class MovingTask(original.MovingTask):
    """HH_261002 - Event-defined true goal/HOLD anchors; no rolling-only terminal-tail denial."""
    def __init__(self, plan):
        super().__init__(plan)
        self.origin_frame = None
        self.origin_ns = None
        self.minimum_free = None
        self.completed_goal = None

    def observe(self, *, frame, timestamp, phase, progress, base_stop_reason, event_count,
                native_actor_speed_mps, rear_base_CTE_m, observed_output_bytes, goal_record, observed_free_disk_bytes=None):
        require(type(frame) is int and frame >= 0 and phase in ("stationary_warmup","driving","stationary_tail"), "native identity required")
        require(all(type(v) in (int,float) and math.isfinite(v) for v in (timestamp,progress,native_actor_speed_mps,rear_base_CTE_m))
            and native_actor_speed_mps >= 0 and type(observed_output_bytes) is int and observed_output_bytes >= 0, "finite actual observations required")
        now = round(timestamp*1e9)
        free = shutil.disk_usage(ROOT).free if observed_free_disk_bytes is None else observed_free_disk_bytes
        require(type(free) is int and free >= 0, "actual free bytes required")
        self.minimum_free = free if self.minimum_free is None else min(self.minimum_free,free)
        if free < FLOOR+RESERVE: self.fail("natural_goal_disk_floor")
        if observed_output_bytes > QUOTA-RESERVE: self.fail("natural_goal_episode_quota")
        if native_actor_speed_mps > 2.7: self.fail("natural_goal_native_speed_guard")
        if abs(rear_base_CTE_m) > 1.: self.fail("natural_goal_rear_CTE_guard")
        if self.previous_time is not None and abs(now-self.previous_time-50_000_000) > 1000: self.fail("natural_goal_native_cadence")
        if event_count: self.fail("natural_goal_collision_or_lane_event")
        self.previous_time = now
        if phase == "driving" and self.origin_frame is None: self.origin_frame,self.origin_ns = frame,now
        offset = None if self.origin_frame is None else frame-self.origin_frame
        require(isinstance(goal_record,dict) and goal_record.get("profile_id") == PROFILE, "actual versioned governor observation required")
        if phase == "driving" and base_stop_reason is not None:
            if base_stop_reason != "comfortable_goal_measured_stop_and_dwell" or goal_record.get("complete") is not True:
                self.fail("natural_goal_original_teacher_terminated_without_measured_dwell")
            else:
                self.completed_goal = dict(frame=frame,timestamp_ns=now,goal_record=dict(goal_record))
                self.closed = dict(frame=frame,timestamp_ns=now,measured_route_progress_m=progress)
        if phase == "stationary_tail" and self.completed_goal is None: self.fail("natural_goal_tail_without_real_driving_hold")
        if phase == "stationary_tail" and self.support is None and self.last is not None and now >= self.last["timestamp_ns"]+6_400_000_000 and self.failure is None:
            self.support = dict(frame=frame,timestamp_ns=now,required_timestamp_ns=self.last["timestamp_ns"]+6_400_000_000)
        eligible = phase == "driving" and offset >= 80 and (self.closed is None or frame == self.closed["frame"])
        reason = self.failure or base_stop_reason
        self.latest = dict(schema=SCHEMA+".observation", source_frame=frame,source_timestamp_ns=now,
            measured_route_progress_m=progress, driving_origin_frame=self.origin_frame,driving_origin_ns=self.origin_ns,
            driving_relative_native_frames=offset,selection_open=self.closed is None,camera_selection_eligible=eligible,
            selected_camera_saved=False,window_closed_at=self.closed,future_support_complete=self.support is not None,
            original_goal_observation=dict(goal_record),native_actor_speed_mps=native_actor_speed_mps,rear_base_CTE_m=rear_base_CTE_m,
            observed_output_bytes=observed_output_bytes,observed_free_disk_bytes=free,termination_reason=reason)
        return self.latest,reason

    def result(self, *, tail_states=0, goal_reached=False):
        result = super().result(tail_states=tail_states,goal_reached=goal_reached)
        span = None if self.first is None else self.last["timestamp_ns"]-self.first["timestamp_ns"]
        complete = result["task_complete"] and self.completed_goal is not None and goal_reached and span is not None and span >= 30_000_000_000
        result.update(schema=SCHEMA+".result",task_complete=bool(complete),case_id="pilot_00",dataset_split_intent="pilot_pending_no_fit",
            actual_goal_completion=self.completed_goal,minimum_observed_free_bytes=self.minimum_free,
            all_event_defined_anchors_retained=True,stationary_tail_only_true_future_context=True,
            startup_anchors_selected=False,minimum_selected_span_ns=30_000_000_000,**pending_scope())
        return result


def reconstruct(plan, states, cameras, route_points):
    from scripts.e2e import collect_carla_vad_expert as collector
    by_frame = {row["frame"]:row for row in cameras}
    require(len(by_frame) == len(cameras) and set(by_frame).issubset({r["frame"] for r in states}), "camera/native frame identities required")
    recovered = MovingTask(plan)
    projector = collector.RouteProjector(dict(route=route_points))
    for row in states:
        stored = row["moving_task"]
        progress = measured_progress(route_points,row["x"],row["y"])
        require(progress == stored["measured_route_progress_m"], "actual pose projection differs")
        observed,_ = recovered.observe(frame=row["frame"],timestamp=row["timestamp"],phase=row["capture_phase"],progress=progress,
            base_stop_reason=stored["original_termination_reason"],event_count=stored["observed_event_count"],
            observed_free_disk_bytes=stored["observed_free_disk_bytes"],native_actor_speed_mps=math.hypot(*row["world_velocity_carla"][:2]),
            rear_base_CTE_m=projector.project(row["x"],row["y"],progress).cross_track_error_m,
            observed_output_bytes=stored["observed_output_bytes"],goal_record=stored["original_goal_observation"])
        # HH_261002 - The frozen engagement reuses the last warmup frame for a fresh initial target update.
        # Its raw pre-engagement target remains in our observation/journal; no TRAIN anchor uses warmup.
        engagement = row["capture_phase"] == "stationary_warmup" and row["goal_stop"].get("next_control_starts_driving") is True
        require(all(row["goal_stop"].get(k) == v for k,v in stored["original_goal_observation"].items()
            if not (engagement and k == "target_speed_mps")), "governor observation differs from native goal row")
        if row["frame"] in by_frame:
            camera=by_frame[row["frame"]];copy=dict(camera);recovered.saved_camera(copy)
            require(camera["moving_task_selected"] == copy["moving_task_selected"] and camera["capture_phase"] == row["capture_phase"], "event membership differs")
        require(all(stored[k] == v for k,v in observed.items()), "pure event-defined native replay differs")
    return recovered
