"""HH_261001 - One combined expert-only zero-normal-brake/low-speed-steer diagnostic, without data admission."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

from scripts.e2e import collect_town07_low_speed_start_20261001 as previous


base = previous.base
ORIGINAL_CONFIGURATION = previous.ORIGINAL_CONFIGURATION
ORIGINAL_INSTALLER = previous.ORIGINAL_INSTALLER
PROFILE = "town07_low_speed_start_20261001_v2"


@dataclass(frozen=True)
class CombinedConfig(previous.LowSpeedStartConfig):
    """HH_261001 - Controller interventions only; measured stop and all quality bounds remain original."""
    normal_brake_cap: float = 0.
    desired_deceleration_mps2: float = .5
    low_speed_zero_steering_mps: float = .75


def configuration(args):
    values = asdict(previous.configuration(args))
    values.update(profile_id=PROFILE, normal_brake_cap=0., desired_deceleration_mps2=.5,
                  policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return CombinedConfig(**values)


class CombinedControl(previous.LaunchControl):
    """HH_261001 - Retain original PID execution/memory and launch; zero only its normal low-speed steering."""
    def apply(self, control, *, speed, timestamp, frame, normal_proposal):
        if float(control.brake) != 0.:
            raise ValueError("normal planner braking must already be capped to zero")
        control = super().apply(control, speed=speed, timestamp=timestamp, frame=frame,
                                normal_proposal=normal_proposal)
        before_steering = base.control_dict(control)
        intervention = speed <= .75
        if intervention:
            control.steer = 0.
        self.last["revised_normal_capped_proposal"] = self.last.pop("comfortable_v2_capped_proposal")
        self.last.update(schema="carla.low_speed_start_normal_control.v2", profile_id=PROFILE,
            launch_control_before_steering_intervention=before_steering,
            low_speed_steering_intervention=intervention, low_speed_zero_steering_mps=.75,
            profile_normal_control=base.control_dict(control), normal_brake_cap=0.,
            desired_deceleration_mps2=.5, causal_resolution_claimed=False)
        return control


@contextmanager
def scope(args):
    original_collect, original_transport = base.collect_episode, base.AcknowledgedControlTransport
    if base.configuration_from_args is not ORIGINAL_CONFIGURATION or base.install_normal_brake_cap is not ORIGINAL_INSTALLER:
        raise ValueError("unexpected shared expert providers")
    session = dict(policy=None, states=None, output=None,
                   counts=dict(driving_ack_commands=0, normal_policy_commands=0, low_speed_steering_interventions=0))

    def installer(planner, cap, throttle_cap=None):
        if cap != 0. or throttle_cap != .20:
            raise ValueError("fixed v2 normal pedal caps required")
        raw_step, raw_proposal = planner.run_step, None

        def observe_normal(*positional, **kwargs):
            nonlocal raw_proposal
            control = raw_step(*positional, **kwargs)
            raw_proposal = base.control_dict(control)
            return control

        planner.run_step = observe_normal
        metadata = ORIGINAL_INSTALLER(planner, cap, throttle_cap)
        normal_step, policy = planner.run_step, CombinedControl()
        session["policy"] = policy

        def step(*positional, **kwargs):
            control = normal_step(*positional, **kwargs)
            velocity = planner._vehicle.get_velocity()
            snapshot = planner._vehicle.get_world().get_snapshot()
            return policy.apply(control, speed=math.hypot(float(velocity.x), float(velocity.y)),
                timestamp=float(snapshot.timestamp.elapsed_seconds), frame=snapshot.frame,
                normal_proposal=raw_proposal)

        planner.run_step = step
        metadata.update(launch_throttle=.13, launch_handoff_mps=.5, maximum_launch_seconds=8.,
            post_handoff_throttle_ramp_per_second=.05, post_handoff_cap=.20,
            normal_brake_cap=0., desired_deceleration_mps2=.5, low_speed_zero_steering_mps=.75,
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
                record = dict(schema="carla.low_speed_start_ack.v2", command_reason=reason, source_frame=frame,
                    normal_policy=normal, final_basic_agent_proposal=base.control_dict(control),
                    ack_receipt=receipt, learned_control=False, training_data_approved=False)
                with (session["output"] / "expert_launch_control.jsonl").open("a") as stream:
                    stream.write(json.dumps(record, allow_nan=False) + "\n")
                session["counts"]["driving_ack_commands"] += int(receipt["server_accepted"])
                session["counts"]["normal_policy_commands"] += int(normal is not None)
                session["counts"]["low_speed_steering_interventions"] += int(
                    normal is not None and normal["low_speed_steering_intervention"])
            return receipt

    def collect(*positional, **kwargs):
        _, _, _, partial, states, _, manifest = positional
        session.update(states=states, output=partial)
        manifest.update(training_data_approved=False, vehicle_control_approved=False,
                        control_source="BasicAgent_with_fixed_native_launch_zero_normal_brake_low_speed_steer")
        manifest.setdefault("capture_contract", {})["low_speed_start_research"] = dict(
            schema="carla.low_speed_start_capture.v2", profile=asdict(configuration(args)),
            learned_control=False, warmup_is_not_a_launch_demonstration=True, restart_demonstration=False,
            combined_protocol_not_controlled_ab=True, causal_resolution_claimed=False,
            original_goal_stop_quality_unchanged=True, full_future_geometry_approval="NOT_ASSESSED_BY_COLLECTOR",
            training_data_approved=False)
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
