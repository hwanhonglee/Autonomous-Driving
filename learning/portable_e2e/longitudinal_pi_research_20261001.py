"""HH_261001 - Bound an experimental PI tracker without a sustaining-throttle floor."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _number(value, name, lower, upper):
    _require(not isinstance(value, bool) and isinstance(value, (int, float))
             and math.isfinite(value) and lower <= value <= upper,
             "invalid PI " + name)
    return float(value)


@dataclass(frozen=True)
class LongitudinalPIConfig:
    """HH_261001 - Freeze conservative gains; no uncalibrated feedforward is enabled."""

    proportional_gain: float = .7
    integral_gain: float = .2
    timestep_s: float = .1
    maximum_throttle: float = .5
    maximum_brake: float = 1.
    maximum_integral_error: float = 1.
    maximum_desired_speed_mps: float = 2.5
    stop_speed_threshold_mps: float = .05

    def validate(self):
        for name, value in asdict(self).items():
            _number(value, name, 0., 10.)
            _require(value > 0., "positive PI " + name + " required")
        _require(self.proportional_gain <= 1. and self.integral_gain <= .2
                 and self.timestep_s == .1 and self.maximum_throttle <= .5
                 and self.maximum_brake <= 1. and self.maximum_integral_error <= 1.
                 and self.maximum_desired_speed_mps <= 2.5
                 and self.stop_speed_threshold_mps == .05,
                 "PI research bounds differ")


class LongitudinalPIResearch:
    """HH_261001 - Integrate once per camera policy call, not per native ACK tick.

    The owner must reset this state at handoff, any rejected proposal, a sensor
    cadence discontinuity, or a stop. This class never owns a simulator or reads
    a route, expert action, future label, model candidate, or actor physics.
    """

    def __init__(self, config=None):
        self.config = config or LongitudinalPIConfig()
        self.config.validate()
        self.integral_error = 0.

    def reset(self):
        self.integral_error = 0.

    def update(self, desired_speed_mps, current_speed_mps):
        cfg = self.config
        desired = _number(desired_speed_mps, "desired speed", 0., cfg.maximum_desired_speed_mps)
        observed = _number(current_speed_mps, "observed speed", -.1, 100.)
        before = self.integral_error
        if desired <= cfg.stop_speed_threshold_mps:
            self.reset()
            return None, dict(stop_branch_preserved=True, integral_before=before,
                              integral_after=0., integral_reset_reason="model_stop_target",
                              feedforward=0., throttle_floor_added=False)
        # HH_261001 - Use only the unchanged causal model target and observed current velocity.
        error = desired - max(0., observed)
        unbounded_integral = before + error * cfg.timestep_s
        candidate = max(-cfg.maximum_integral_error,
                        min(cfg.maximum_integral_error, unbounded_integral))
        unsaturated = cfg.proportional_gain * error + cfg.integral_gain * candidate
        # HH_261001 - Conditional integration prevents further windup into actuator saturation.
        held_for_saturation = ((unsaturated > cfg.maximum_throttle and error > 0.)
                               or (unsaturated < -cfg.maximum_brake and error < 0.))
        if held_for_saturation:
            candidate = before
        self.integral_error = candidate
        signed_request = cfg.proportional_gain * error + cfg.integral_gain * candidate
        throttle = min(cfg.maximum_throttle, max(0., signed_request))
        brake = min(cfg.maximum_brake, max(0., -signed_request))
        return dict(throttle=throttle, brake=brake), dict(
            desired_speed_mps=desired, observed_speed_mps=observed,
            error_mps=error, timestep_s=cfg.timestep_s,
            integral_before=before, integral_unbounded=unbounded_integral,
            integral_after=candidate, integral_bound_applied=abs(unbounded_integral) > cfg.maximum_integral_error,
            integral_held_for_saturation=held_for_saturation,
            signed_request=signed_request, signed_unsaturated_candidate=unsaturated,
            stop_branch_preserved=False, integral_reset_reason=None,
            feedforward=0., throttle_floor_added=False,
            throttle_saturated=signed_request > cfg.maximum_throttle,
            brake_saturated=signed_request < -cfg.maximum_brake)
