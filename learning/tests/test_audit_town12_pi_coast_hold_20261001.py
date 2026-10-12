"""HH_261001 - Verify the independent offline equations and strict raw guard checks without contacting CARLA."""

from types import SimpleNamespace

import pytest

from scripts.e2e import audit_town12_pi_coast_hold_20261001 as audit
from scripts.e2e import calibrate_town12_j15824_pi_coast_hold_20261001 as worker


def control():
    return SimpleNamespace(throttle=.2, brake=0., steer=.05, gear=1,
        reverse=False, hand_brake=False, manual_gear_shift=False)


def test_independent_reference_matches_entire_causal_hold_sequence():
    actual, reference = worker.StartStopPolicy(), audit.IndependentReference()
    speeds = [0.] + [1.5] * 10 + [.04] * 41
    for index, speed in enumerate(speeds):
        raw = worker.collector.control_dict(control())
        expected = reference.decision(100 + index, index * .05, speed, raw)
        actual.apply(control(), speed=speed, timestamp=index * .05, frame=100 + index)
        assert audit.same(expected, actual.last)
        if expected["stage"] == audit.HOLD:
            reference.observation(expected["stage"], .01)
            actual.observe(speed=.01, frame=101 + index)
    assert reference.hold_observed == actual.hold_observations == 40
    assert reference.integral == actual.pi.integral


def test_independent_timeout_reproduces_fixed_twenty_seconds():
    reference = audit.IndependentReference()
    for index in range(400):
        reference.decision(index, index * .05, 1.449, worker.collector.control_dict(control()))
    with pytest.raises(ValueError, match="20-second PI entry stabilization timed out"):
        reference.decision(400, 20., 1.449, worker.collector.control_dict(control()))
    assert reference.entry is None


def test_float32_actuator_tolerance_never_masks_integral_or_speed_changes():
    assert audit.same({"throttle": .2}, {"throttle": .20000000298})
    assert not audit.same({"integral_after": .2}, {"integral_after": .20000000298})
    assert not audit.same({"raw_actor_planar_speed_mps": .05}, {"raw_actor_planar_speed_mps": .0500001})
    assert not audit.same({"value": float("nan")}, {"value": float("nan")})


def test_independent_guards_enforce_geometry_and_free_space():
    row = {"speed_mps": 1.5, "front_bumper_stop_distance_m": 60.,
        "route_projection": {"progress_m": 10.}, "signal_state": "Red",
        "body_corners": [{"lane": [588, 0, 1], "lane_margin_m": .1}] * 8,
        "free_disk_bytes": 5 * 1024 ** 3}
    assert not audit.guard_errors(row)
    assert audit.guard_errors({**row, "free_disk_bytes": row["free_disk_bytes"] - 1}) == ["disk_floor"]
    assert "instantaneous_lane_inset" in audit.guard_errors({**row, "body_corners": [
        {"lane": [588, 0, 1], "lane_margin_m": .099}] * 8})


def test_hold_observation_is_never_inferred_from_a_command():
    reference = audit.IndependentReference()
    reference.hold_commands = 1
    with pytest.raises(ValueError, match="hold speed"):
        reference.observation(audit.HOLD, .051)
    assert reference.hold_observed == 0
    reference.observation(audit.HOLD, .01)
    with pytest.raises(ValueError, match="duplicate or missing"):
        reference.observation(audit.HOLD, .01)
