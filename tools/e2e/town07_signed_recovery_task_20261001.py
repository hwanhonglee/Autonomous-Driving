"""HH_261001 - Fixed actual-pose teacher recovery cohort; no synthetic image or future labels."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys

from scripts.e2e import carla_moving_capture as original

ROOT = Path(__file__).resolve().parents[2]
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
SCHEMA = "carla.actual_signed_teacher_recovery.20261001.v1"
FLOOR, RESERVE, QUOTA = 5*1024**3, 32*1024**2, 512*1024**2
require, sha, read, encoded, canonical = original.require, original.sha, original.read, original.encoded, original.canonical
measured_progress = original.measured_progress
validate_original_physics, validate_applied_physics = original.validate_original_physics, original.validate_applied_physics
CASES = tuple(dict(case_id=f"train_{i:02d}", split="train", seed=2026100110+i,
    signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=yaw)
    for i, (lateral, yaw) in enumerate((lateral, yaw) for lateral in (-.35, 0., .35) for yaw in (-3., 0., 3.))) + (
    dict(case_id="holdout_00", split="dev", seed=2026100210, signed_lateral_offset_ros_m=-.25, signed_yaw_offset_ros_deg=2.),
    dict(case_id="holdout_01", split="dev", seed=2026100211, signed_lateral_offset_ros_m=.25, signed_yaw_offset_ros_deg=-2.))
FIXED_ARGS = dict(host="127.0.0.1", port=2440, physics_hz=20., capture_hz=10., target_speed_kmh=9.,
    max_duration_sec=50., stationary_warmup_sec=1.2, stationary_tail_sec=6.5,
    control_transport="acknowledged_batch", agent_initialization="after_bootstrap", goal_stop_profile="disabled",
    physics_substep_profile="inherited", low_speed_steering_mode=None, hold_brake_mode=None, terminal_approach_mode=None,
    weather="ClearNoon", vehicle_type="vehicle.toyota.prius", role_name="autoware_e2e_expert",
    wheelbase_m=2.85, spawn_z_offset_m=.5, goal_tolerance_m=1., command_lookahead_m=2., command_exit_lookahead_m=2.5,
    allow_map_load=False, allow_stopped_steering=False, wall_timing=False, jpeg_quality=95, timeout=20., sensor_timeout_sec=5.,
    basic_agent_base_min_distance_m=2., basic_agent_distance_ratio=.2, basic_agent_lateral_kp=1.95,
    basic_agent_lateral_ki=.05, basic_agent_lateral_kd=.2, basic_agent_max_steering=.8, basic_agent_lane_offset_m=0.)


def case(case_id):
    matches = [value for value in CASES if value["case_id"] == case_id]
    require(len(matches) == 1, "fixed recovery case required")
    return dict(matches[0])


def paths(run_id, case_id=None):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "safe recovery version required")
    batch = ROOT / ("storage/experiments/validation/2026-10-01/town07_signed_teacher_recovery_" + run_id)
    if case_id is None: return batch, batch.with_name(batch.name + "_plan.json")
    case(case_id)
    output = batch / case_id
    return output, batch / (case_id + "_plan.json")


def pending_scope():
    # HH_261001 - Fresh unqualified data has no Boolean admission marker; False would mean an immutable explicit denial.
    return dict(admission_status="PENDING_QA", training_admission_state="PENDING_QA_NOT_AUTHORIZED", dataset_conversion_performed=False,
        model_promoted=False, vehicle_control_approved=False, learned_control=False,
        qualification_notice="Actual disclosed teacher data; exact fixed membership, native dynamics/ACK/body/future QA required before training.")


def fixed_contract(run_id, case_id):
    spec = case(case_id)
    output, plan = paths(run_id, case_id)
    return dict(schema=SCHEMA, run_id=run_id, **spec, output=str(output / "episode"), declaration_path=str(plan),
        fixed_arguments=dict(FIXED_ARGS, seed=spec["seed"]), route_sha256=ROUTE_SHA,
        perturbation="One actual rear-reference spawn offset/yaw before actor/sensor ticks; nominal route/goal unchanged.",
        sign_convention="ROS positive lateral left / positive yaw counter-clockwise; CARLA y/yaw signs inverted.",
        selected_driving_relative_time_ns=[900_000_000, 31_000_000_000], minimum_selected_span_ns=30_000_000_000, required_future_support_ns=6_400_000_000,
        selection_clock="Exact native frame displacement18..620 from first driving observation, camera every2frames; measured timestamps are NEVER snapped or rewritten.",
        native_timestamp_rounding_tolerance_ns=1, observed_history_aggregate_clock_tolerance_ns=18,
        observed_selected_span_aggregate_clock_tolerance_ns=602,
        required_history_camera_observations=10, required_history_ns=900_000_000, preceding_native_guard_ns=50_000_000,
        selection_rule="ALL six-camera driving anchors in fixed inclusive0.9..31s after first driving observation; no quality-conditioned cropping.",
        closure_rule="Permanently close after31s; continue unchanged teacher until actual last anchor+6.4s, then original130-tick full-brake tail.",
        label_source="Actual future native/camera poses and measured speeds; no route-as-expert-label, future extrapolation, failed learned tail or transformed pixels.",
        raw_original_BGRA_camera_SHA_at_every_bundle=True, saved_original_JPEG_SHA_at_every_bundle=True,
        exact_BGRA_payload_archive_retained=False, original_JPEG_encoding_unchanged=True,
        input_route="Causal navigation only; runtime label/HD lane truth not presented as learned perception.",
        teacher="BasicAgent lateral and original hazard override; original v2 actual-speed launch and normal brake/throttle caps.",
        controller=dict(normal_brake_cap=0., normal_throttle_cap=.20, launch_throttle=.13, handoff_speed_mps=.5,
            launch_timeout_s=8., zero_normal_steering_until_mps=.75, target_speed_ramp_mps2=1.5, target_speed_cap_mps=2.4,
            BasicAgent_emergency_unchanged=True),
        teacher_catalog_request_kph=9., explicit_teacher_controller_target_cap_mps=2.4,
        native_actor_speed_emergency_mps=2.7, rear_base_absolute_CTE_emergency_m=1.,
        original_scalar_future_quality_bounds_unchanged=True, all_failed_attempts_retained=True,
        episode_quota_bytes=QUOTA, free_disk_floor_bytes=FLOOR, closing_reserve_bytes=RESERVE,
        required_checks=["fixed membership/full observed history", "actual nine_future_bounds", "native20Hz dynamics",
            "sixcam pixels/rig/time", "independent ACK/camera audit", "all captured bodies+sweeps raw+10cm inset", "no collision/lane events"],
        attempts=1, automatic_retry=False, train_case_count=9, holdout_case_count=2,
        heldout_seeds_reserved=True, holdout_training_or_checkpoint_selection_allowed=False,
        route_group_independence="NOT_CERTIFIED: same existing TRAIN corridor; holdout is unseen initialization/seed, not independent map.",
        normal_stop_or_full_route_success_claim=False, **pending_scope())


def configuration_from_args(args):
    run_id, case_id = args.recovery_run_id, args.recovery_case_id
    output, path = paths(run_id, case_id)
    require(canonical(args.moving_task_plan) == path and sha(path) == args.moving_task_plan_sha256,
        "exact recovery case declaration required")
    plan, fixed = read(path), fixed_contract(run_id, case_id)
    require(set(plan) == set(fixed) | {"declared_at_utc", "input_paths", "input_sha256", "source_sha256", "external_source_sha256",
        "asset_sha256", "owner_output", "server_command", "worker_command", "owner_wall_timeout_s"},
        "recovery declaration fields differ")
    require(encoded({key: plan[key] for key in fixed}) == encoded(fixed), "fixed recovery declaration changed")
    require(canonical(args.output) == output / "episode", "recovery output differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0 and stamp <= datetime.now(timezone.utc),
        "prospective recovery UTC declaration required")
    for key, value in fixed["fixed_arguments"].items(): require(getattr(args, key, None) == value, "recovery CLI differs: " + key)
    for key, name in plan["input_paths"].items():
        require(key in ("route_file", "mapping", "calibration") and canonical(getattr(args, key)) == canonical(name)
            and sha(Path(name)) == plan["input_sha256"][key], "recovery asset differs: " + key)
    require(set(plan["input_paths"]) == set(plan["input_sha256"]) == {"route_file", "mapping", "calibration"}, "recovery asset closure differs")
    require(plan["input_sha256"]["route_file"] == ROUTE_SHA, "recovery original route pin differs")
    for name, digest in plan["source_sha256"].items(): require(sha(ROOT / name) == digest, "recovery source changed: " + name)
    for name, digest in plan["external_source_sha256"].items(): require(sha(Path(name)) == digest, "recovery external source changed: " + name)
    return plan


def perturb_actor_center(center, *, wheelbase_m, signed_lateral_offset_ros_m, signed_yaw_offset_ros_deg):
    """HH_261001 - Move the actual rear-reference pose first, then rebuild actor center at perturbed yaw."""
    require(wheelbase_m == 2.85 and all(type(value) in (int, float) and math.isfinite(value)
        for value in (*center.values(), signed_lateral_offset_ros_m, signed_yaw_offset_ros_deg)), "finite fixed spawn geometry required")
    require(abs(signed_lateral_offset_ros_m) <= .35 and abs(signed_yaw_offset_ros_deg) <= 3., "spawn perturbation outside declared envelope")
    theta = math.radians(center["yaw"])
    rear = dict(center)
    pitch = math.radians(center.get("pitch", 0.))
    rear["x"] -= wheelbase_m / 2 * math.cos(theta)*math.cos(pitch)
    rear["y"] -= wheelbase_m / 2 * math.sin(theta)*math.cos(pitch)
    rear["z"] -= wheelbase_m / 2 * math.sin(pitch)
    rear["x"] += signed_lateral_offset_ros_m * math.sin(theta)
    rear["y"] -= signed_lateral_offset_ros_m * math.cos(theta)
    rear["yaw"] -= signed_yaw_offset_ros_deg
    result = dict(rear)
    new_theta = math.radians(rear["yaw"])
    result["x"] += wheelbase_m / 2 * math.cos(new_theta)*math.cos(pitch)
    result["y"] += wheelbase_m / 2 * math.sin(new_theta)*math.cos(pitch)
    result["z"] += wheelbase_m / 2 * math.sin(pitch)
    return result, rear


class MovingTask(original.MovingTask):
    """HH_261001 - Select actual recovery states by prospective time, retaining all failures and future support."""
    def __init__(self, plan):
        super().__init__(plan)
        self.driving_origin = None
        self.driving_origin_frame = None
        self.camera_history, self.native_history = [], []
        self.history_support = None
        self.minimum_free = None

    def observe(self, *, frame, timestamp, phase, progress, base_stop_reason, event_count, observed_free_disk_bytes=None,
                native_actor_speed_mps, rear_base_CTE_m, observed_output_bytes):
        require(type(frame) is int and frame >= 0 and phase in ("stationary_warmup", "driving", "stationary_tail"), "invalid recovery native identity")
        require(all(type(value) in (int, float) and math.isfinite(value) for value in (timestamp, progress)), "finite recovery observation required")
        now = round(timestamp * 1e9)
        free = shutil.disk_usage(ROOT).free if observed_free_disk_bytes is None else observed_free_disk_bytes
        require(type(free) is int and free >= 0, "recorded disk bytes required")
        self.minimum_free = free if self.minimum_free is None else min(free, self.minimum_free)
        require(all(type(value) in (int, float) and math.isfinite(value) for value in (native_actor_speed_mps, rear_base_CTE_m))
            and native_actor_speed_mps >= 0 and type(observed_output_bytes) is int and observed_output_bytes >= 0,
            "actual native speed/CTE and quota observation required")
        if free < FLOOR + RESERVE: self.fail("recovery_disk_floor")
        if native_actor_speed_mps > 2.7: self.fail("recovery_native_actor_speed_limit")
        if abs(rear_base_CTE_m) > 1.: self.fail("recovery_rear_base_CTE_limit")
        if observed_output_bytes > QUOTA-RESERVE: self.fail("recovery_episode_disk_quota")
        if self.previous_time is not None and abs(now-self.previous_time-50_000_000) > 1: self.fail("recovery_native_cadence")
        self.previous_time = now
        self.native_history.append(dict(frame=frame, timestamp_ns=now, phase=phase))
        if event_count: self.fail("recovery_collision_or_lane_event")
        if phase == "driving" and self.driving_origin is None:
            self.driving_origin, self.driving_origin_frame = now, frame
        relative = None if self.driving_origin is None else now-self.driving_origin
        relative_frame = None if self.driving_origin_frame is None else frame-self.driving_origin_frame
        if phase == "driving" and self.support is None:
            if base_stop_reason is not None: self.fail("recovery_teacher_or_catalog_ended_before_support")
            if relative_frame > 620 and self.closed is None:
                self.closed = dict(frame=frame, timestamp_ns=now, driving_relative_ns=relative, driving_relative_native_frames=relative_frame,
                    measured_route_progress_m=progress)
                if self.first is None or self.last["frame"]-self.first["frame"] != 602 \
                    or abs(self.last["timestamp_ns"]-self.first["timestamp_ns"]-30_100_000_000) > 602:
                    self.fail("recovery_fixed_anchor_span_incomplete")
            if self.closed is not None and self.last is not None and now >= self.last["timestamp_ns"]+6_400_000_000 and self.failure is None:
                self.support = dict(frame=frame, timestamp_ns=now, required_timestamp_ns=self.last["timestamp_ns"]+6_400_000_000)
        reason = self.failure or ("recovery_future_support_complete" if self.support is not None and phase == "driving" else None)
        self.latest = dict(schema=SCHEMA+".observation", measured_route_progress_m=progress,
            source_frame=frame, source_timestamp_ns=now, selection_open=self.closed is None, selected_camera_saved=False,
            camera_selection_eligible=phase == "driving" and self.closed is None and 18 <= relative_frame <= 620,
            driving_origin_ns=self.driving_origin, driving_relative_ns=relative, observed_free_disk_bytes=free,
            driving_origin_frame=self.driving_origin_frame, driving_relative_native_frames=relative_frame,
            native_actor_speed_mps=native_actor_speed_mps, rear_base_CTE_m=rear_base_CTE_m, observed_output_bytes=observed_output_bytes,
            window_closed_at=self.closed, future_support_complete=self.support is not None, termination_reason=reason)
        return self.latest, reason

    def saved_camera(self, camera):
        if self.latest["camera_selection_eligible"] and self.first is None:
            history = [*self.camera_history[-9:], dict(frame=camera["frame"], timestamp_ns=round(camera["timestamp"]*1e9), phase=camera["capture_phase"])]
            guard = next((row for row in self.native_history if row["frame"] == history[0]["frame"]-1), None) if history else None
            valid = len(history) == 10 and all(row["phase"] == "driving" for row in history) \
                and all(b["frame"] == a["frame"]+2 for a, b in zip(history, history[1:])) \
                and history[-1]["frame"]-history[0]["frame"] == 18 \
                and abs(history[-1]["timestamp_ns"]-history[0]["timestamp_ns"]-900_000_000) <= 18 \
                and guard is not None and guard["phase"] == "stationary_warmup"
            self.history_support = dict(status="PASS" if valid else "FAIL", camera_observations=history, preceding_native_guard=guard,
                preceding_guard_may_be_stationary_warmup=True)
            if not valid: self.fail("recovery_first_anchor_missing_real_history")
        super().saved_camera(camera)
        self.camera_history.append(dict(frame=camera["frame"], timestamp_ns=round(camera["timestamp"]*1e9), phase=camera["capture_phase"]))

    def result(self, *, tail_states=0, goal_reached=False):
        result = super().result(tail_states=tail_states, goal_reached=goal_reached)
        complete = result["task_complete"] and self.history_support is not None and self.history_support["status"] == "PASS" and len(self.selected) == 302
        result.update(schema=SCHEMA+".result", task_complete=complete, case_id=self.plan["case_id"], dataset_split_intent=self.plan["split"],
            selected_driving_relative_time_ns=[900_000_000, 31_000_000_000], history_support=self.history_support,
            minimum_observed_free_bytes=self.minimum_free, expected_fixed_anchor_count=302, **pending_scope())
        return result


def reconstruct(plan, states, cameras, route_points):
    by_frame = {row["frame"]: row for row in cameras}
    require(len(by_frame) == len(cameras), "duplicate recovery camera frame")
    recovered = MovingTask(plan)
    from scripts.e2e import collect_carla_vad_expert as collector
    projector = collector.RouteProjector(dict(route=route_points))
    for row in states:
        stored = row["moving_task"]
        progress = measured_progress(route_points, row["x"], row["y"])
        require(progress == stored["measured_route_progress_m"], "recovery actual pose projection differs")
        observed, _ = recovered.observe(frame=row["frame"], timestamp=row["timestamp"], phase=row["capture_phase"], progress=progress,
            base_stop_reason=stored["original_termination_reason"], event_count=stored["observed_event_count"], observed_free_disk_bytes=stored["observed_free_disk_bytes"],
            native_actor_speed_mps=math.hypot(*row["world_velocity_carla"][:2]),
            rear_base_CTE_m=projector.project(row["x"], row["y"], progress).cross_track_error_m,
            observed_output_bytes=stored["observed_output_bytes"])
        if row["frame"] in by_frame:
            camera = by_frame[row["frame"]]
            copy = dict(camera)
            recovered.saved_camera(copy)
            require(camera["moving_task_selected"] == copy["moving_task_selected"], "recovery membership changed")
        require(all(stored[key] == value for key, value in observed.items()), "recovery observation replay differs")
    return recovered
