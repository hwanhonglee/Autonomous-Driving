"""HH_261001 - Distinguish preview clipping from actual throttle change."""

from scripts.e2e.audit_ah_speed_cap_trace_20261001 import longitudinal


def test_preview_clip_can_leave_saturated_throttle_identical():
    low = longitudinal(2.54, 1.78, 2.5)
    high = longitudinal(2.54, 1.78, 30/3.6)
    assert low["desired_speed_mps"] < high["desired_speed_mps"]
    assert low["throttle"] == high["throttle"] == .5


def test_later_preview_clip_reduces_throttle_without_inventing_brake():
    low = longitudinal(2.57, 1.915, 2.5)
    high = longitudinal(2.57, 1.915, 30/3.6)
    assert high["throttle"] > low["throttle"]
    assert low["brake"] == high["brake"] == 0.


def test_nonbinding_cap_cannot_change_zero_preview_stop():
    low = longitudinal(0., .01, 2.5)
    high = longitudinal(0., .01, 30/3.6)
    assert low == high
    assert low["brake"] == .3 and low["throttle"] == 0.
