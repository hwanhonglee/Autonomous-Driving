"""HH_261001 - One real expert launch and measured goal stop; no learned rollout relabeling or admission."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e.carla_goal_stop_profile import ComfortableGoalStopConfig


ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
ORIGINAL_CONFIGURATION = base.configuration_from_args
ORIGINAL_INSTALLER = base.install_normal_brake_cap


@dataclass(frozen=True)
class LowSpeedStartConfig(ComfortableGoalStopConfig):
    """HH_261001 - Preserve goal-stop bounds; only this fixed normal-control launch insertion is new."""
    launch_throttle: float = .13
    launch_handoff_speed_mps: float = .5
    maximum_launch_seconds: float = 8.
    post_handoff_throttle_ramp_per_second: float = .05
    development_only: bool = True
    training_data_approved: bool = False
    policy_source_sha256: str = ""


def configuration(args):
    expected = dict(goal_stop_profile="comfortable_v2", target_speed_kmh=9., physics_hz=20., capture_hz=10.,
        stationary_warmup_sec=3.5, stationary_tail_sec=6.5, max_duration_sec=180., goal_tolerance_m=1.,
        agent_initialization="after_bootstrap", control_transport="acknowledged_batch", weather="ClearNoon",
        seed=2026100101, spawn_z_offset_m=.5, vehicle_type="vehicle.toyota.prius", physics_substep_profile="inherited",
        basic_agent_base_min_distance_m=2., basic_agent_distance_ratio=.2, allow_stopped_steering=False)
    if any(getattr(args, key) != value for key, value in expected.items()):
        raise ValueError("fixed Town07 low-speed expert contract differs")
    if args.host not in ("127.0.0.1", "localhost") or args.allow_map_load:
        raise ValueError("local owned server without client map loading required")
    if args.moving_task_plan is not None or args.moving_task_plan_sha256 is not None:
        raise ValueError("this episode must start physically stopped, not from moving-task crops")
    if hashlib.sha256(args.route_file.read_bytes()).hexdigest() != ROUTE_SHA:
        raise ValueError("exact original TRAIN Town07 route required")
    values = asdict(ORIGINAL_CONFIGURATION(args))
    values.update(profile_id="town07_low_speed_start_20261001_v1",
                  policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return LowSpeedStartConfig(**values)


class LaunchControl:
    """HH_261001 - Use measured native time and speed; normal launch never intercepts BasicAgent emergency returns."""
    def __init__(self):
        self.started_at = None
        self.handoff_at = None
        self.previous_time = None
        self.last = None

    def apply(self, control, *, speed, timestamp, frame, normal_proposal):
        if not math.isfinite(speed) or not math.isfinite(timestamp) or speed < 0:
            raise ValueError("finite nonnegative actual planar speed and native time required")
        if self.previous_time is not None and timestamp < self.previous_time:
            raise ValueError("native launch clock moved backwards")
        self.previous_time = timestamp
        if self.started_at is None:
            self.started_at = timestamp
        if self.handoff_at is None and speed >= .5:
            self.handoff_at = timestamp
        if self.handoff_at is None and timestamp - self.started_at >= 8.:
            raise ValueError("fixed low-speed launch did not reach 0.5 m/s within 8 seconds")
        before = base.control_dict(control)
        cap = .13 if self.handoff_at is None else min(.20, .13 + .05 * (timestamp - self.handoff_at + .05))
        if self.handoff_at is None:
            control.throttle, control.brake = .13, 0.
        else:
            control.throttle = min(float(control.throttle), cap)
        self.last = dict(schema="carla.low_speed_start_normal_control.v1", source_frame=int(frame),
            source_timestamp=timestamp, measured_planar_speed_mps=speed, handoff_latched=self.handoff_at is not None,
            launch_started_at=self.started_at, handoff_at=self.handoff_at, normal_throttle_cap=cap,
            raw_normal_planner_proposal=normal_proposal, comfortable_v2_capped_proposal=before,
            profile_normal_control=base.control_dict(control),
            insertion="LocalPlanner normal request before unchanged BasicAgent emergency override",
            training_data_approved=False, learned_control=False)
        return control


@contextmanager
def scope(args):
    original_collect = base.collect_episode
    original_transport = base.AcknowledgedControlTransport
    if base.configuration_from_args is not ORIGINAL_CONFIGURATION or base.install_normal_brake_cap is not ORIGINAL_INSTALLER:
        raise ValueError("unexpected shared expert providers")
    session = dict(policy=None, states=None, output=None, counts=dict(driving_ack_commands=0, normal_policy_commands=0))

    def installer(planner, cap, throttle_cap=None):
        if cap != .15 or throttle_cap != .20:
            raise ValueError("unchanged comfortable_v2 pedal caps required")
        raw_step = planner.run_step
        raw_proposal = None

        def observe_normal(*positional, **kwargs):
            nonlocal raw_proposal
            control = raw_step(*positional, **kwargs)
            raw_proposal = base.control_dict(control)
            return control

        planner.run_step = observe_normal
        metadata = ORIGINAL_INSTALLER(planner, cap, throttle_cap)
        normal_step = planner.run_step
        policy = LaunchControl()
        session["policy"] = policy

        def step(*positional, **kwargs):
            control = normal_step(*positional, **kwargs)
            velocity = planner._vehicle.get_velocity()
            snapshot = planner._vehicle.get_world().get_snapshot()
            return policy.apply(control, speed=math.hypot(float(velocity.x), float(velocity.y)),
                timestamp=float(snapshot.timestamp.elapsed_seconds), frame=snapshot.frame, normal_proposal=raw_proposal)

        planner.run_step = step
        metadata.update(launch_throttle=.13, launch_handoff_mps=.5, maximum_launch_seconds=8.,
            post_handoff_throttle_ramp_per_second=.05, post_handoff_cap=.20,
            launch_speed_source="actual CARLA planar actor velocity", launch_clock="actual world snapshot time",
            emergency_return_control_modified=False, learned_control=False, training_data_approved=False)
        return metadata

    class Transport(original_transport):
        def send(self, control, reason, expected_before_frame=None):
            receipt = super().send(control, reason, expected_before_frame)
            if session["output"] is not None and reason in ("initial_drive_control", "next_driving_control"):
                states, policy = session["states"], session["policy"]
                frame = states[-1]["frame"] if states else None
                normal = policy.last if policy and policy.last and policy.last["source_frame"] == frame else None
                record = dict(schema="carla.low_speed_start_ack.v1", command_reason=reason, source_frame=frame,
                    normal_policy=normal, final_basic_agent_proposal=base.control_dict(control),
                    ack_receipt=receipt, learned_control=False, training_data_approved=False)
                if states:
                    states[-1]["expert_launch_control"] = record
                with (session["output"] / "expert_launch_control.jsonl").open("a") as stream:
                    stream.write(json.dumps(record, allow_nan=False) + "\n")
                session["counts"]["driving_ack_commands"] += int(receipt["server_accepted"])
                session["counts"]["normal_policy_commands"] += int(normal is not None)
            return receipt

    def collect(*positional, **kwargs):
        _, _, _, partial, states, _, manifest = positional
        session.update(states=states, output=partial)
        manifest.update(training_data_approved=False, vehicle_control_approved=False,
                        control_source="BasicAgent_with_fixed_native_low_speed_launch")
        manifest.setdefault("capture_contract", {})["low_speed_start_research"] = dict(
            schema="carla.low_speed_start_capture.v1", profile=asdict(configuration(args)),
            learned_control=False, warmup_is_not_a_launch_demonstration=True,
            restart_demonstration=False, original_goal_stop_quality_unchanged=True,
            full_future_geometry_approval="NOT_ASSESSED_BY_COLLECTOR", training_data_approved=False)
        try:
            return original_collect(*positional, **kwargs)
        finally:
            manifest.setdefault("result", {})["low_speed_start_counts"] = dict(session["counts"])
            manifest["result"].update(training_data_approved=False, development_only=True)

    base.configuration_from_args, base.install_normal_brake_cap = configuration, installer
    base.collect_episode, base.AcknowledgedControlTransport = collect, Transport
    try:
        yield
    finally:
        base.configuration_from_args, base.install_normal_brake_cap = ORIGINAL_CONFIGURATION, ORIGINAL_INSTALLER
        base.collect_episode, base.AcknowledgedControlTransport = original_collect, original_transport


def parse_args(argv=None):
    args = base.parse_args(argv)
    configuration(args)
    return args


def main(argv=None):
    try:
        args = parse_args(argv)
        with scope(args):
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        return 1
    print("capture=" + str(output) + "; original scalar QA retained; full future QA and admission still required")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
