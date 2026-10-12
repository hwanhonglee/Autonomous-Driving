"""HH_261001 - Add a disclosed PI target-headroom calibration to the frozen owned-camera V2 policy."""

from __future__ import annotations

from .longitudinal_pi_headroom_research_20261001 import LongitudinalPIHeadroomResearch
from .temporal_owned_camera_history_v2_20261001 import OwnedCameraTemporalSimulationResearchPolicyV2


class HeadroomOwnedCameraTemporalSimulationResearchPolicyV2(OwnedCameraTemporalSimulationResearchPolicyV2):
    """HH_261001 - Inherit original inputs, NN, decoder, gates, deadline and reset authority unchanged."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.pi is None:
            raise ValueError("explicit longitudinal PI required for the headroom experiment")
        # HH_261001 - Install one private PI instance before any prefill or actuation; no second update is introduced.
        self.pi = LongitudinalPIHeadroomResearch(self.pi.config)
