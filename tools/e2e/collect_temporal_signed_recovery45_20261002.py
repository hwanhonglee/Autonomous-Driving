"""HH_261002 - Actual signed-spawn V3 recovery; disclosed teacher launch is never learned recovery."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import math

from . import collect_temporal_route_recovery_fast45_20261001 as previous
from . import run_owned_town07_signed_recovery_v3_20261001 as manual
from . import town07_signed_recovery_task_20261001 as task
from .collect_town07_signed_recovery_20261001 import TargetRamp


frozen = previous.frozen
collector = frozen.collector
SCHEMA = "portable_e2e.actual_signed_recovery45.20261002.v1"
PROFILES = (
    dict(case_id="negative_outward", seed=20261002100, signed_lateral_offset_ros_m=-.20, signed_yaw_offset_ros_deg=-1.5),
    dict(case_id="positive_outward", seed=20261002101, signed_lateral_offset_ros_m=.20, signed_yaw_offset_ros_deg=1.5),
    dict(case_id="centered_setup_comparator", seed=20261002102, signed_lateral_offset_ros_m=0., signed_yaw_offset_ros_deg=0.),
)
FROZEN = {
    "tools/e2e/collect_temporal_route_recovery_fast45_20261001.py": "4b8e3d5ffbab373fa714c3e854858b9262343ad5a3d28c7eae7717eb5bc38447",
    "tools/e2e/town07_signed_recovery_task_20261001.py": "fddd47336e5a914f89db60dc7dccfa1dd613c3917125b213754de86a83958ecb",
    "tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py": "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a",
}
_active = False


def profile(seed):
    rows = [row for row in PROFILES if type(seed) is int and row["seed"] == seed]
    frozen._require(len(rows) == 1, "exact prospective signed-recovery seed required")
    return dict(rows[0])


def verify_frozen():
    for name, sha in FROZEN.items():
        frozen._require(hashlib.sha256((frozen.ROOT/name).read_bytes()).hexdigest() == sha,
                        "frozen signed-recovery dependency changed: "+name)
    manual.verify_frozen_dependencies()


def validate_capture(args, route):
    """HH_261002 - Only the explicit three fresh initialization seeds differ from the fixed 45-second runtime."""
    profile(args.seed)
    frozen._require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.agent_initialization == "after_bootstrap"
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 45 and args.target_speed_kmh == 9
        and args.weather == "ClearNoon" and args.spawn_z_offset_m == .5
        and args.research_speed_cap_mps == 2.5 and args.research_max_cte_m == 1.
        and route["town"] == "Town07" and route["scenario"] == "straight"
        and isinstance(args.research_source_sha256, str) and len(args.research_source_sha256) == 64
        and all(x in "0123456789abcdef" for x in args.research_source_sha256)
        and args.research_longitudinal_pi is True,
        "fixed isolated signed-recovery45 Town07 contract differs")


def teacher_class(original_agent, session):
    """HH_261002 - Same qualified manual-first-gear normal startup hook, before original hazard override."""
    class DisclosedRecoveryStartup(original_agent):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            planner = self.get_local_planner()
            original_step, observed = planner.run_step, {"value": None}
            def raw(*positional, **keywords):
                control = original_step(*positional, **keywords)
                observed["value"] = collector.control_dict(control)
                return control
            planner.run_step = raw
            metadata = collector.install_normal_brake_cap(planner, 0., .20)
            normal, policy = planner.run_step, manual.FirstGearRecoveryLaunchControl()
            session["policy"] = policy
            def step(*positional, **keywords):
                control = normal(*positional, **keywords)
                velocity = planner._vehicle.get_velocity()
                snapshot = planner._vehicle.get_world().get_snapshot()
                result = policy.apply(control, speed=math.hypot(velocity.x, velocity.y),
                    timestamp=snapshot.timestamp.elapsed_seconds, frame=snapshot.frame,
                    normal_proposal=observed["value"])
                with (session["output"]/"signed_startup_normal_teacher.jsonl").open("a") as stream:
                    stream.write(json.dumps(dict(schema=SCHEMA+".normal_teacher_proposal", normal_policy=policy.last,
                        applied_learned_control_claimed=False, hazard_override_occurs_after_this_normal_hook=True),allow_nan=False)+"\n")
                return result
            planner.run_step = step
            self._signed_recovery_ramp = TargetRamp()
            session["manifest"]["capture_contract"]["signed_recovery_startup_teacher"] = dict(metadata,
                schema=SCHEMA+".startup", source_manual_controller_revision=manual.TASK_SCHEMA,
                launch_throttle=.11, post_handoff_native_time_pedal_slew_per_second=.025,
                normal_throttle_cap=.20, zero_normal_steering_until_speed_mps=.75,
                normal_manual_gear_shift=True, normal_gear=1, actual_teacher_target_cap_mps=2.4,
                original_BasicAgent_hazard_return_unchanged=True,
                startup_teacher_is_not_learned_recovery=True, observed_states_or_camera_pixels_changed=False)

        def run_step(self, *args, **kwargs):
            snapshot = self._vehicle.get_world().get_snapshot()
            self.set_target_speed(self._signed_recovery_ramp.update(snapshot.frame)*3.6)
            return super().run_step(*args, **kwargs)
    return DisclosedRecoveryStartup


@contextmanager
def startup_spawn_scope(args, *, agent_module=None):
    """HH_261002 - Perturb the true actor spawn once, before cameras/ticks; never mutate a captured frame."""
    if agent_module is None:
        from agents.navigation import basic_agent as agent_module
    spec = profile(args.seed)
    names = ("collect_episode", "apply_spawn_z_offset", "_blank_brake_control", "checked_agent_initialization_observation")
    saved = {name: getattr(collector, name) for name in names}
    original_agent = agent_module.BasicAgent
    session = dict(manifest=None, output=None, spawn_calls=0, policy=None)

    def spawn(center, z_offset):
        frozen._require(session["manifest"] is not None and session["spawn_calls"] == 0
            and z_offset == .5 and args.wheelbase_m == 2.85, "one declared pre-sensor actor spawn required")
        session["spawn_calls"] += 1
        changed, rear = task.perturb_actor_center(center, wheelbase_m=2.85,
            signed_lateral_offset_ros_m=spec["signed_lateral_offset_ros_m"],
            signed_yaw_offset_ros_deg=spec["signed_yaw_offset_ros_deg"])
        actual = saved["apply_spawn_z_offset"](changed, z_offset)
        session["manifest"]["capture_contract"]["actual_signed_spawn"] = dict(schema=SCHEMA+".spawn", **spec,
            nominal_actor_center_carla=dict(center), perturbed_rear_reference_carla=rear,
            perturbed_actor_center_carla=changed, actual_requested_spawn_carla=actual,
            applied_once_before_sensors_and_ticks=True, pixels_or_observed_poses_post_transformed=False)
        return actual

    def brake(carla):
        control = saved["_blank_brake_control"](carla)
        frozen._require(control.throttle == 0. and control.brake == 1. and control.steer == 0.
            and not control.reverse and not control.hand_brake, "original pure full-service brake required")
        control.gear, control.manual_gear_shift = 1, True
        return control

    def collect(*positional, **keywords):
        session["manifest"] = positional[6]
        session["output"] = positional[3]
        contract = session["manifest"]["capture_contract"]
        contract["signed_recovery_initialization_gear_contract"] = dict(actual_manual_gear_shift=True, actual_gear=1,
            checker_function_source_sha256=manual.PARENT_CHECKER_SOURCE_SHA,
            original_frame_actor_PID_finite_and_full_brake_guards_preserved=True,
            observed_controls_or_snapshots_proxied=False)
        try:
            return saved["collect_episode"](*positional, **keywords)
        finally:
            session["manifest"].setdefault("result", {})["actual_spawn_perturbation_call_count"] = session["spawn_calls"]
            if session["spawn_calls"] != 1:
                raise collector.CollectionError("actual signed spawn was not applied exactly once")

    collector.collect_episode, collector.apply_spawn_z_offset = collect, spawn
    collector._blank_brake_control = brake
    collector.checked_agent_initialization_observation = manual.checked_first_gear_initialization_observation
    agent_module.BasicAgent = teacher_class(original_agent, session)
    try:
        yield
    finally:
        for name, value in saved.items(): setattr(collector, name, value)
        agent_module.BasicAgent = original_agent


@contextmanager
def activated(model, args):
    """HH_261002 - Preserve V3 learned activation, ACK clocks, immutable inputs and no-fallback behavior."""
    global _active
    frozen._require(not _active, "signed-recovery45 activation cannot nest")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    old_validate = previous.previous.validate_capture
    _active = True
    previous.previous.validate_capture = validate_capture
    try:
        with startup_spawn_scope(args), previous.activated(model, args):
            yield
    finally:
        previous.previous.validate_capture = old_validate
        _active = False
