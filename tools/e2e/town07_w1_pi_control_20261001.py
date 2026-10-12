"""HH_261001 - Scope a controller-only W1 PI comparison without changing frozen model code."""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import asdict
import weakref

from portable_e2e.longitudinal_pi_research_20261001 import (
    LongitudinalPIConfig, LongitudinalPIResearch, _number, _require)


POLICY_ID = "town07_w1_proportional_vs_pi_only_20261001_v1"
ARMS = ("A_original", "B_pi_only")
CADENCE_NS = 100_000_000
CADENCE_TOLERANCE_NS = 1_000
CONFIG = LongitudinalPIConfig()


def _validate_original(original, controller):
    _require(isinstance(original, Mapping), "original normal tracker mapping required")
    _number(original.get("desired_speed_mps"), "original desired speed", 0., 2.5)
    for key, upper in (("throttle", .5), ("brake", 1.), ("steer", 1.)):
        _number(original.get(key), "original " + key, -1. if key == "steer" else 0., upper)
    _require(controller.speed_cap_mps == 2.5 and controller.throttle_gain == .7
             and controller.brake_gain == .7 and controller.maximum_throttle == .5
             and controller.speed_preview_s == .3,
             "fixed original W1 proportional tracker settings differ")


@contextmanager
def tracker_scope(research, arm):
    """HH_261001 - Replace only normal throttle/brake inside the existing deadline.

    Original inference, raw argmax, candidate gates, selected path, pure-pursuit
    steering, model stop branch and full-brake emergency return stay in place.
    State is private to each policy object and resets after any rejected normal
    proposal. The caller separately freezes W1 weights, collector and owner.
    No acknowledgement, feature approval, installation or physical run occurs.
    """
    _require(arm in ARMS, "fixed PI comparison arm required")
    CONFIG.validate()
    original_control = research.trajectory_control
    original_step = research.SimulationResearchPolicy.step
    _require(callable(original_control) and callable(original_step)
             and callable(getattr(research, "_json_finite", None)),
             "original W1 research tracker and policy required")
    _require(not getattr(original_control, "_pi_research_hook", False)
             and not getattr(original_step, "_pi_research_hook", False)
             and not getattr(original_control, "_moving_brake_ab_hook", False)
             and not getattr(original_step, "_moving_brake_ab_hook", False),
             "nested controller intervention forbidden")
    controllers, stamps = weakref.WeakKeyDictionary(), weakref.WeakKeyDictionary()
    active = None

    def reset(policy):
        if policy in controllers:
            controllers[policy].reset()
        stamps.pop(policy, None)

    def control(selected, current_speed_mps, controller):
        _require(active is not None, "PI tracker must be called within one original policy step")
        policy, now_ns = active["policy"], active["now_ns"]
        _require(active["audit"] is None, "multiple tracker calls within one camera inference forbidden")
        original = original_control(selected, current_speed_mps, controller)
        _validate_original(original, controller)
        _require(type(now_ns) is int and now_ns >= 0, "PI policy integer timestamp required")
        previous = stamps.get(policy)
        if previous is not None and abs(now_ns - previous - CADENCE_NS) > CADENCE_TOLERANCE_NS:
            reset(policy)
            raise ValueError("PI camera-policy cadence differs from fixed 100 ms")
        pi = controllers.setdefault(policy, LongitudinalPIResearch(CONFIG))
        proposed = dict(original)
        audit = dict(policy_id=POLICY_ID, arm=arm, gains=asdict(CONFIG),
                     source_checkpoint_sha256=getattr(policy.model, "_research_source_checkpoint_sha256", None),
                     original_normal_tracker_control=dict(original), camera_policy_timestamp_ns=now_ns,
                     previous_camera_policy_timestamp_ns=previous,
                     camera_policy_timestep_s=.1, held_control_native_timestep_s=.05,
                     calibrated_feedforward_available=False, feedforward_added=False,
                     integral_added=arm == ARMS[1], throttle_floor_added=False,
                     selected_speed_or_path_changed=False, steering_changed=False,
                     stop_branch_changed=False, emergency_authority_changed=False,
                     control_cadence_changed=False, actuator_acknowledged=False)
        if arm == ARMS[1]:
            request, pi_audit = pi.update(original["desired_speed_mps"], current_speed_mps)
            if request is not None:
                proposed.update(request)
            audit.update(pi_audit)
        else:
            pi.reset()
            audit.update(integral_before=0., integral_after=0.,
                         stop_branch_preserved=original["desired_speed_mps"] <= .05,
                         integral_reset_reason="original_proportional_arm")
        stamps[policy] = now_ns
        audit["proposed_normal_tracker_control"] = dict(proposed)
        active["audit"] = audit
        return proposed

    def step(policy, *args, **kwargs):
        nonlocal active
        _require(active is None, "reentrant PI policy step forbidden")
        active = dict(policy=policy, now_ns=kwargs.get("now_ns"), audit=None)
        try:
            decision = original_step(policy, *args, **kwargs)
            _require(isinstance(decision, Mapping), "original research decision mapping required")
            result = dict(decision)
            normal = (decision.get("control_source") == "learned_trajectory_tracker"
                      and decision.get("learned_control_proposed") is True
                      and decision.get("emergency_reason") is None)
            audit = active["audit"]
            if normal:
                _require(audit is not None, "normal W1 decision bypassed audited original tracker")
                _require(decision.get("control") == research._json_finite(audit["proposed_normal_tracker_control"]),
                         "returned W1 control differs from audited tracker")
            else:
                reset(policy)
            if audit is None:
                audit = dict(policy_id=POLICY_ID, arm=arm, gains=asdict(CONFIG),
                             original_normal_tracker_control=None, proposed_normal_tracker_control=None,
                             feedforward_added=False, throttle_floor_added=False,
                             actuator_acknowledged=False)
            audit.update(normal_tracker_called=active["audit"] is not None,
                         normal_control_returned=normal,
                         controller_intervention_returned=normal and arm == ARMS[1],
                         integral_state_reset_after_rejection=not normal,
                         committed_integral_after=controllers[policy].integral_error if policy in controllers else 0.,
                         actual_control_source_requires_original_collector_receipt=True)
            result["longitudinal_pi_research"] = audit
            return result
        except BaseException:
            reset(policy)
            raise
        finally:
            active = None

    control._pi_research_hook = step._pi_research_hook = True
    research.trajectory_control, research.SimulationResearchPolicy.step = control, step
    try:
        yield
    finally:
        research.trajectory_control, research.SimulationResearchPolicy.step = original_control, original_step
        controllers.clear()
        stamps.clear()
