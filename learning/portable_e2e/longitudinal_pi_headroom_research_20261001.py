"""HH_261001 - Reserve actuator headroom below the unchanged 2.5 m/s model safety envelope."""

from __future__ import annotations

from .longitudinal_pi_research_20261001 import (
    LongitudinalPIResearch, _number, _require)


SCHEMA = "portable_e2e.longitudinal_pi_actuator_headroom.20261001.v1"
CONTROLLER_TARGET_CAP_MPS = 2.40


class LongitudinalPIHeadroomResearch(LongitudinalPIResearch):
    """HH_261001 - Change only the controller's cruise target, not NN trajectories or measured speed.

    This is a disclosed calibration experiment, not exact tracking of the
    original 9 km/h target or evidence of model-only driving performance. The
    fixed 0.1 m/s headroom does not guarantee absence of physical overshoot.
    The original selected trajectory, decoder limit and actual-speed gate must
    remain outside this controller and unchanged.
    """

    def __init__(self, config=None):
        super().__init__(config)
        _require(self.config.maximum_desired_speed_mps == 2.5,
                 "headroom intervention requires the unchanged 2.5 m/s research cap")

    def update(self, desired_speed_mps, current_speed_mps):
        # HH_261001 - Validate the original target before applying the explicitly declared controller cap.
        original = _number(desired_speed_mps, "desired speed", 0.,
                           self.config.maximum_desired_speed_mps)
        governed = min(original, CONTROLLER_TARGET_CAP_MPS)
        # HH_261001 - Delegate exactly once: held native ACK ticks must never reintegrate this PI state.
        request, audit = super().update(governed, current_speed_mps)
        audit.update(schema=SCHEMA,
            original_model_target_mps=original,
            governed_controller_target_mps=governed,
            controller_target_cap_mps=CONTROLLER_TARGET_CAP_MPS,
            controller_target_governed=governed != original,
            nominal_unchanged_model_speed_cap_mps=2.5,
            controller_headroom_mps=2.5-CONTROLLER_TARGET_CAP_MPS,
            explicit_controller_calibration_intervention=True,
            raw_NN_decoded_selected_trajectory_changed=False,
            observed_speed_clipped=False,
            unchanged_model_stop_branch_preserved=audit["stop_branch_preserved"],
            original_9_kmh_target_tracking_claimed=False,
            model_only_performance_claimed=False,
            one_fresh_camera_update_one_PI_integration=True)
        return request, audit
