#!/usr/bin/env python3
"""HH_261001 - Non-training brake calibration with a time-based normal-throttle cap, not a smooth-launch claim."""

from contextlib import contextmanager
import math

from scripts.e2e import calibrate_town12_j15824_brake_pulses_20261001 as inherited


scout, collector, lift = inherited.scout, inherited.collector, inherited.lift
ORIGINAL_PROTOCOL, ORIGINAL_PULSE_POLICY = inherited.protocol, inherited.PulsePolicy
POLICY_ID = "town12_fresh_actor_tiny_brake_pulses_timed_entry_v2"


def protocol():
    value = ORIGINAL_PROTOCOL()
    value.pop("launch_handoff_mps")
    value.pop("post_handoff_ramp_per_second")
    value.pop("launch_throttle")
    return {**value, "policy_id": POLICY_ID, "launch_initial_throttle_cap": .13,
        "launch_throttle_cap_ramp_per_second": .01, "launch_cap_maximum": .20,
        "launch_cap_formula": "min(0.20, 0.13 + 0.01 * actual_elapsed_simulation_seconds)",
        "actual_normal_throttle": "min(original normal PID proposal, timed cap)",
        "speed_latched_launch_handoff": False, "maximum_launch_seconds": 20.,
        "v1_inner_eight_second_handoff_timeout_removed": True,
        "launch_stage": "timed_cap_launch_and_stabilize", "smoother_launch_claimed": False,
        "revision": "Only launch cap/timing and its explicit labels change; all pulse/geometry/ownership/ACK gates remain original.",
        "reason": "V1 held 0.13 for 8 s without reaching 0.5 m/s; max0.444892 m/s and a measured +3.206322 m/s2 transient."}


class TimedThrottleCap:
    """HH_261001 - Use real elapsed native time; never force throttle above the original PID proposal."""
    def __init__(self):
        self.started_at = self.previous_time = self.last = None

    def apply(self, control, *, speed, timestamp, frame, normal_proposal):
        scout.require(math.isfinite(speed) and speed >= 0. and math.isfinite(timestamp),
                      "finite measured launch speed and native time required")
        scout.require(self.previous_time is None or timestamp >= self.previous_time, "native launch clock moved backwards")
        scout.require(float(control.brake) == 0., "normal launch brake must already be zero")
        if self.started_at is None:
            self.started_at = timestamp
        elapsed = timestamp - self.started_at
        scout.require(elapsed < 20., "bounded 20-second timed launch expired")
        cap = min(.20, .13 + .01 * elapsed)
        before = collector.control_dict(control)
        control.throttle = min(float(control.throttle), cap)
        self.previous_time = timestamp
        self.last = {"schema": "town12.nontraining_timed_launch.v2", "source_frame": int(frame),
            "source_timestamp": timestamp, "measured_planar_speed_mps": speed,
            "launch_started_at": self.started_at, "elapsed_simulation_seconds": elapsed,
            "normal_throttle_cap": cap, "raw_normal_planner_proposal": normal_proposal,
            "normal_control_before_timed_cap": before, "normal_control_after_timed_cap": collector.control_dict(control),
            "speed_handoff_latched": False, "smoother_launch_claimed": False,
            "training_data_admitted": False, "learned_control": False}
        return control


class TimedPulsePolicy(ORIGINAL_PULSE_POLICY):
    """HH_261001 - Keep the inherited measured entry-band/pulse logic and distinguish the changed launch stage."""
    def apply(self, *args, **kwargs):
        control = super().apply(*args, **kwargs)
        if self.last["stage"] == "gentle_launch_and_stabilize":
            self.last["stage"] = "timed_cap_launch_and_stabilize"
        return control


@contextmanager
def configured_worker():
    old = inherited.LaunchControl, inherited.PulsePolicy, inherited.protocol
    try:
        inherited.LaunchControl, inherited.PulsePolicy, inherited.protocol = TimedThrottleCap, TimedPulsePolicy, protocol
        yield
    finally:
        inherited.LaunchControl, inherited.PulsePolicy, inherited.protocol = old


def main(argv=None):
    with configured_worker():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
