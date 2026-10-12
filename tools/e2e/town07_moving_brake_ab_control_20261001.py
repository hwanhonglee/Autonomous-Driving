"""HH_261001 - Isolated normal-moving-brake intervention; no gain or feedforward changes."""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import contextmanager
import math


POLICY_ID = "town07_w1_normal_moving_brake_ab_v1"
ARMS = ("A_original", "B_suppress_normal_moving_brake")
STOP_SPEED_MPS = 0.05


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _number(value, name, lower, upper):
    _require(not isinstance(value, bool) and isinstance(value, (int, float))
             and math.isfinite(value) and lower <= value <= upper,
             "invalid original normal tracker " + name)
    return value


def normal_tracker_control(original, arm):
    """Return a copy; B changes only brake for the original tracker's positive moving target."""
    _require(arm in ARMS, "fixed moving-brake A/B arm required")
    _require(isinstance(original, Mapping), "original normal tracker control mapping required")
    desired = _number(original.get("desired_speed_mps"), "desired speed", 0., 2.5)
    _number(original.get("brake"), "brake", 0., 1.)
    result = dict(original)
    if arm == ARMS[1] and desired > STOP_SPEED_MPS:
        result["brake"] = 0.
    return result


def intervention_audit(original, proposed, arm):
    """Record the unmodified proposal separately from CARLA actuator keyword arguments."""
    expected = normal_tracker_control(original, arm)
    _require(isinstance(proposed, Mapping) and dict(proposed) == expected,
             "moving-brake intervention altered another tracker field")
    changed = expected["brake"] != original["brake"]
    return dict(policy_id=POLICY_ID, arm=arm,
                desired_speed_mps=original["desired_speed_mps"],
                stop_speed_threshold_mps=STOP_SPEED_MPS,
                moving_target=original["desired_speed_mps"] > STOP_SPEED_MPS,
                original_normal_tracker_control=dict(original),
                original_normal_brake=original["brake"],
                proposed_normal_brake=expected["brake"],
                normal_brake_suppression_proposed=changed,
                feedforward_added=False, integral_added=False,
                control_cadence_changed=False, stop_branch_changed=False,
                emergency_authority_changed=False)


@contextmanager
def tracker_scope(research, arm):
    """Hook only this process's tracker; preserve original gates, deadline and emergency return.

    The caller must bind/pin W1 identity and fixed collector settings separately.
    Enter this scope before the existing sampled-input archive so the observer
    receives the audited final decision. This scope never starts a simulator,
    loads weights, writes files, or changes the research module on disk.
    """
    _require(arm in ARMS, "fixed moving-brake A/B arm required")
    original_control = research.trajectory_control
    original_step = research.SimulationResearchPolicy.step
    _require(callable(original_control) and callable(original_step),
             "original research tracker and policy step required")
    _require(not getattr(original_control, "_moving_brake_ab_hook", False)
             and not getattr(original_step, "_moving_brake_ab_hook", False),
             "nested moving-brake intervention forbidden")
    last_audit = None
    active_step = False

    def control(selected, current_speed_mps, config):
        nonlocal last_audit
        original = original_control(selected, current_speed_mps, config)
        proposed = normal_tracker_control(original, arm)
        last_audit = intervention_audit(original, proposed, arm)
        return proposed

    def step(policy, *args, **kwargs):
        nonlocal last_audit, active_step
        _require(not active_step, "reentrant moving-brake policy step forbidden")
        active_step, last_audit = True, None
        try:
            decision = original_step(policy, *args, **kwargs)
            _require(isinstance(decision, Mapping), "original research decision mapping required")
            result = dict(decision)
            normal_returned = (decision.get("control_source") == "learned_trajectory_tracker"
                               and decision.get("learned_control_proposed") is True
                               and decision.get("emergency_reason") is None)
            if normal_returned:
                _require(last_audit is not None,
                         "normal research decision bypassed the original tracker")
                expected = normal_tracker_control(last_audit["original_normal_tracker_control"], arm)
                _require(decision.get("control") == expected,
                         "returned research control differs from the scoped tracker")
            audit = dict(last_audit) if last_audit is not None else dict(
                policy_id=POLICY_ID, arm=arm,
                normal_brake_suppression_proposed=False,
                original_normal_tracker_control=None)
            audit.update(normal_tracker_called=last_audit is not None,
                         normal_control_returned=normal_returned,
                         normal_brake_suppression_returned=(normal_returned
                             and audit["normal_brake_suppression_proposed"]),
                         actuator_acknowledged=False,
                         actual_control_source_requires_original_collector_receipt=True)
            result["moving_brake_ab"] = audit
            return result
        finally:
            active_step, last_audit = False, None

    control._moving_brake_ab_hook = step._moving_brake_ab_hook = True
    research.trajectory_control = control
    research.SimulationResearchPolicy.step = step
    try:
        yield
    finally:
        research.trajectory_control = original_control
        research.SimulationResearchPolicy.step = original_step
