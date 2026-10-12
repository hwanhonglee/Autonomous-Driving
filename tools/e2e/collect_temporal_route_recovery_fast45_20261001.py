"""HH_261001 - Activate the new learned route-recovery model with identical safety and actual post-ACK recording."""

from __future__ import annotations

from contextlib import contextmanager

import torch

from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3, MODEL_ID
from portable_e2e.temporal_single_copy_history_research_20261001 import SingleCopyOwnedHeadroomPolicyV2
from . import collect_temporal_v2_headroom_fast45_research_20261001 as previous
from .defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


frozen = previous.frozen
validate_capture = previous.validate_capture
_active = False


class RecoverySingleCopyPolicyV3(SingleCopyOwnedHeadroomPolicyV2):
    """HH_261001 - Disclose the changed learned architecture; original raw decoder, gate, PI and deadlines remain."""

    def step(self, *args, **kwargs):
        result = super().step(*args, **kwargs)
        result.update(model_architecture_id=MODEL_ID, learned_route_recovery_branch=True,
                      runtime_route_as_output_replacement=False, expert_fallback_after_handoff=False,
                      raw_neural_curvature_math_unchanged_from_V2=False,
                      decoder_gate_tracker_math_unchanged_except_disclosed_headroom=True)
        return result


@contextmanager
def activated(model, args):
    """HH_261001 - The explicit new owner loads V3, never relabels it as the old CPU-finite V2 class."""
    global _active
    if _active:
        raise RuntimeError("recovery collector activation cannot nest")
    if type(model) is not RouteRecoveryTemporalModelV3 or model.model_id != MODEL_ID:
        raise ValueError("explicit loaded route-recovery V3 architecture required")
    if any(module.training for module in model.modules()) or not all(
            value.device.type == "cpu" and value.dtype == torch.float32 for value in model.state_dict().values()):
        raise ValueError("declared recovery runtime requires eval-mode float32 CPU weights")
    binding = getattr(model, "_research_recovery_owner_binding", {})
    if getattr(model, "_research_recovery_simulator_control_authorized", False) is not True \
            or binding.get("purpose") != "owned_carla_research_only" \
            or binding.get("simulation_only") is not True \
            or binding.get("owner_plan_sha256") != getattr(args,"research_source_sha256",None) \
            or not isinstance(binding.get("owner_plan_sha256"),str) \
            or len(binding["owner_plan_sha256"]) != 64:
        raise ValueError("numeric-only weights cannot actuate; exact private research owner declaration required")
    previous.validate_capture(args, frozen.collector.load_route(args.route_file))
    headroom = previous.headroom
    original_validate = frozen.validate_capture
    original_policy = headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
    _active = True
    frozen.validate_capture = previous.validate_capture
    headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2 = RecoverySingleCopyPolicyV3
    try:
        with headroom.activated(model, args):
            # HH_261001 - Journal resides with the exact episode; inherited ACK timing is captured before flush.
            partial = args.output.with_name(args.output.name+".partial")
            with deferred_recording(frozen.collector, partial):
                yield
    finally:
        frozen.validate_capture = original_validate
        headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2 = original_policy
        _active = False
