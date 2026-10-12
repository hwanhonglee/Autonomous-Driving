"""HH_261001 - Real signed-offset teacher data, actual pixels/poses, no learned actuation or label rewriting."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e import collect_town07_low_speed_start_v2_20261001 as v2
from scripts.e2e import town07_signed_recovery_task_20261001 as task


def moving_provider(args):
    task.configuration_from_args(args)
    return task


def runtime_provider(guarded_task):
    """HH_261001 - Expose every frozen collector moving-module API without replacing independent reconstruction."""
    return SimpleNamespace(**{name: getattr(task, name) for name in ("configuration_from_args", "pending_scope", "reconstruct",
        "validate_original_physics", "validate_applied_physics", "measured_progress", "canonical")},
        MovingTask=guarded_task, __file__=task.__file__)


@contextmanager
def parsing_scope():
    saved = base.moving_capture_module
    base.moving_capture_module = moving_provider
    try: yield
    finally: base.moving_capture_module = saved


def parse_args(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--recovery-run-id", required=True)
    parser.add_argument("--recovery-case-id", required=True)
    ours, remaining = parser.parse_known_args(argv)
    # HH_261001 - Attach only exact cohort identities before base moving-provider validation.
    original_parse = task.configuration_from_args
    def validate(parsed):
        parsed.recovery_run_id, parsed.recovery_case_id = ours.recovery_run_id, ours.recovery_case_id
        return original_parse(parsed)
    task.configuration_from_args = validate
    try:
        with parsing_scope(): args = base.parse_args(remaining)
    finally: task.configuration_from_args = original_parse
    args.recovery_run_id, args.recovery_case_id = ours.recovery_run_id, ours.recovery_case_id
    task.configuration_from_args(args)
    return args


class TargetRamp:
    """HH_261001 - Explicit expert2.40m/s headroom, not clipping observed speeds or relabeling nine-kph driving."""
    def __init__(self): self.initial_call, self.last_frame, self.target = True, None, 0.
    def update(self, frame):
        if self.last_frame is not None and frame != self.last_frame+1:
            raise ValueError("one teacher proposal per contiguous native frame required")
        self.last_frame = frame
        if self.initial_call:
            self.initial_call = False
            return 2.4
        self.target = min(2.4, self.target+1.5*.05)
        return self.target


@contextmanager
def scope(args):
    from agents.navigation import basic_agent as agent_module
    plan = task.configuration_from_args(args)
    original_agent = agent_module.BasicAgent
    names = ("collect_episode", "AcknowledgedControlTransport", "moving_capture_module", "apply_spawn_z_offset", "acknowledged_snapshot_state", "_save_jpeg")
    saved = {name: getattr(base, name) for name in names}
    original_task = task.MovingTask
    session = dict(output=None, states=None, manifest=None, policy=None, target=None, latest_state=None, latest_raw=None, spawn_calls=0)
    projector = base.RouteProjector(base.load_route(task.ROUTE))
    pixel_records = {}

    def save_image(image, path, quality):
        # HH_261001 - Hash actual simulator pixels before unchanged JPEG encoding; never shift/rotate captured images.
        if len(image.raw_data) != image.width*image.height*4: raise ValueError("exact native BGRA payload required")
        raw_sha = hashlib.sha256(image.raw_data).hexdigest()
        saved["_save_jpeg"](image, path, quality)
        name = path.parent.name
        if name not in base.MODEL_CAMERA_ORDER: raise ValueError("declared six-camera source required")
        frame_records = pixel_records.setdefault(int(image.frame), {})
        if name in frame_records: raise ValueError("duplicate actual camera source")
        record = dict(camera=name, frame=int(image.frame), timestamp=float(image.timestamp), width=int(image.width), height=int(image.height),
            encoding="original_carla_bgra8", original_BGRA_sha256=raw_sha, saved_JPEG_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            saved_JPEG_relative_path=str(path.relative_to(session["output"])), JPEG_encoder_unchanged=True, pose_or_image_post_transformed=False)
        frame_records[name] = record
        with (session["output"]/"actual_camera_pixels.jsonl").open("a") as stream:
            stream.write(json.dumps(record, allow_nan=False)+"\n")

    def spawn(center, z_offset):
        if session["spawn_calls"] != 0 or z_offset != .5: raise ValueError("exactly one actual spawn perturbation permitted")
        session["spawn_calls"] += 1
        changed, rear = task.perturb_actor_center(center, wheelbase_m=args.wheelbase_m,
            signed_lateral_offset_ros_m=plan["signed_lateral_offset_ros_m"], signed_yaw_offset_ros_deg=plan["signed_yaw_offset_ros_deg"])
        result = saved["apply_spawn_z_offset"](changed, z_offset)
        session["manifest"]["capture_contract"]["actual_signed_spawn"] = dict(nominal_actor_center_carla=center,
            perturbed_rear_reference_carla=rear, perturbed_actor_center_carla=changed, actual_requested_spawn_carla=result,
            signed_lateral_offset_ros_m=plan["signed_lateral_offset_ros_m"], signed_yaw_offset_ros_deg=plan["signed_yaw_offset_ros_deg"],
            applied_once_before_sensors_and_ticks=True, pixels_or_observed_poses_post_transformed=False)
        return result

    def snapshot(*positional, **kwargs):
        state, raw = saved["acknowledged_snapshot_state"](*positional, **kwargs)
        session.update(latest_state=state, latest_raw=raw)
        return state, raw

    class GuardedTask(original_task):
        def observe(self, **kwargs):
            state, raw = session["latest_state"], session["latest_raw"]
            if state is None or raw is None: raise ValueError("recovery actual native state missing")
            partial = session["output"]
            total = sum(path.stat().st_size for path in partial.rglob("*") if path.is_file())
            return super().observe(**kwargs, native_actor_speed_mps=math.hypot(*raw["world_velocity_carla"][:2]),
                rear_base_CTE_m=projector.project(state["x"], state["y"], kwargs["progress"]).cross_track_error_m,
                observed_output_bytes=total)

        def saved_camera(self, camera):
            records = pixel_records.pop(camera["frame"], {})
            if set(records) != set(base.MODEL_CAMERA_ORDER): raise ValueError("every actual six-camera source hash required")
            if any(records[name]["timestamp"] != camera["source_timestamps"][name] for name in records):
                raise ValueError("camera hash timestamp identity differs")
            camera["original_BGRA_source_sha256"] = {name: row["original_BGRA_sha256"] for name, row in records.items()}
            camera["saved_JPEG_sha256"] = {name: row["saved_JPEG_sha256"] for name, row in records.items()}
            return super().saved_camera(camera)

    class Teacher(original_agent):
        def __init__(self, *positional, **kwargs):
            super().__init__(*positional, **kwargs)
            planner = self.get_local_planner()
            raw_step, observed = planner.run_step, {"value": None}
            def raw(*p, **kw):
                control = raw_step(*p, **kw)
                observed["value"] = base.control_dict(control)
                return control
            planner.run_step = raw
            metadata = base.install_normal_brake_cap(planner, 0., .20)
            normal, policy = planner.run_step, v2.CombinedControl()
            session["policy"] = policy
            def step(*p, **kw):
                control = normal(*p, **kw)
                velocity = planner._vehicle.get_velocity()
                native = planner._vehicle.get_world().get_snapshot()
                return policy.apply(control, speed=math.hypot(velocity.x, velocity.y), timestamp=native.timestamp.elapsed_seconds,
                    frame=native.frame, normal_proposal=observed["value"])
            planner.run_step = step
            self._recovery_ramp = TargetRamp()
            session["manifest"]["capture_contract"]["teacher_controller"] = dict(metadata,
                inherited_BasicAgent_emergency_unchanged=True, explicit_teacher_target_cap_mps=2.4,
                catalog_target_request_kph=9., measured_speed_clipped=False, normal_launch_controls_inherited_v2=True)

        def run_step(self, *positional, **kwargs):
            snapshot = self._vehicle.get_world().get_snapshot()
            target = self._recovery_ramp.update(snapshot.frame)
            self.set_target_speed(target*3.6)
            session["target"] = dict(source_frame=snapshot.frame, actual_teacher_target_mps=target,
                teacher_target_cap_mps=2.4, catalog_target_request_kph=9., actual_observed_speed_clipped=False)
            return super().run_step(*positional, **kwargs)

    class Transport(saved["AcknowledgedControlTransport"]):
        def send(self, control, reason, expected_before_frame=None):
            receipt = super().send(control, reason, expected_before_frame)
            if session["output"] is not None:
                policy = session["policy"]
                normal = policy.last if policy and policy.last and policy.last["source_frame"] == receipt["before_frame"] else None
                record = dict(schema=task.SCHEMA+".actual_teacher_command", reason=reason, normal_policy=normal,
                    target_ramp=session["target"], final_BasicAgent_or_brake_proposal=base.control_dict(control),
                    ack_receipt=receipt, learned_control=False, teacher_control=True, training_data_approved=False)
                with (session["output"]/"actual_teacher_control.jsonl").open("a") as stream:
                    stream.write(json.dumps(record, allow_nan=False)+"\n")
            return receipt

    def collect(*positional, **kwargs):
        _, _, _, partial, states, _, manifest = positional
        session.update(output=partial, states=states, manifest=manifest)
        manifest.update(task.pending_scope(), control_source="Disclosed_BasicAgent_signed_recovery_teacher_actual_v2_launch")
        manifest["capture_contract"]["signed_recovery"] = plan
        try: return saved["collect_episode"](*positional, **kwargs)
        finally:
            manifest.setdefault("result", {})["actual_spawn_perturbation_call_count"] = session["spawn_calls"]
            if session["spawn_calls"] != 1: manifest["result"]["recovery_spawn_exactly_once"] = False

    base.collect_episode, base.AcknowledgedControlTransport = collect, Transport
    # HH_261001 - Bind a module-shaped worker provider; pure replay remains independent of live final state.
    provider = runtime_provider(GuardedTask)
    base.moving_capture_module, base.apply_spawn_z_offset, base.acknowledged_snapshot_state = lambda parsed: provider, spawn, snapshot
    base._save_jpeg = save_image
    agent_module.BasicAgent = Teacher
    try: yield
    finally:
        for name, value in saved.items(): setattr(base, name, value)
        agent_module.BasicAgent = original_agent


def main(argv=None):
    # HH_261001 - Only the owner dispatcher checks parent/PID/socket identity before entering this scoped worker.
    print("ERROR: private recovery worker; use run_owned_town07_signed_recovery_20261001 --mode execute", file=sys.stderr)
    return 1


if __name__ == "__main__": raise SystemExit(main())
