"""HH_261001 - Retain both complete CPU input validations while scanning finite values without Torch intermediates."""

from __future__ import annotations

import copy
import math
from typing import Mapping

import numpy as np
import torch
from torch import Tensor

from .contract import ContractError
from .temporal_cached_encoder_v2 import CachedTemporalModelV2
from .temporal_model_v2 import HISTORY_MODES


SCHEMA = "portable_e2e.temporal_cpu_float32_finite_scan.20261001.v1"


def cpu_float32_all_finite(value):
    """HH_261001 - Inspect the same CPU float32 storage, including every masked value; never memoize mutable tensors."""
    if not isinstance(value, Tensor) or value.device.type != "cpu" or value.dtype != torch.float32:
        raise ContractError("finite-scan revision supports the declared float32 CPU ABI only")
    # HH_261001 - NumPy honors strides/storage offset and scans all elements; no pixels or weights are transformed.
    return bool(np.isfinite(value.detach().numpy()).all())


class CPUFiniteCachedTemporalModelV2(CachedTemporalModelV2):
    """HH_261001 - Preserve original neural math/cache guards and both validations; replace only the finite predicate."""

    def _validate_inputs(self, inputs: Mapping[str, Tensor], history_mode: str) -> int:
        # HH_261001 - All validation statements below match the frozen model method except the finite predicate.
        if not isinstance(history_mode, str) or history_mode not in HISTORY_MODES:
            raise ContractError(f"history_mode must be one of {sorted(HISTORY_MODES)}")
        images = inputs["images"]
        if not isinstance(images, Tensor) or images.ndim != 6 or images.shape[0] < 1:
            raise ContractError("images must have [B,5,6,3,180,320] with B >= 1")
        batch = images.shape[0]
        cfg = self.config
        shapes = {
            "images": (batch, 5, 6, 3, 180, 320), "calibration": (batch, 6, 16),
            "ego_history": (batch, 10, 13), "ego_history_mask": (batch, 10),
            "route_xy": (batch, 128, 2), "route_mask": (batch, 128),
            "time_valid": (batch, 5), "relative_pose": (batch, 5, 3),
            "time_offsets_s": (batch, 5),
        }
        masks = frozenset(("ego_history_mask", "route_mask", "time_valid"))
        reference = self.queries
        for name, value in inputs.items():
            if not isinstance(value, Tensor) or tuple(value.shape) != shapes[name]:
                raise ContractError(f"{name} must have shape {shapes[name]}")
            if value.device != reference.device:
                raise ContractError(f"{name} must be on the model device")
            if name in masks:
                if value.dtype != torch.bool:
                    raise ContractError(f"{name} must use boolean validity, not numeric weights")
            else:
                if not value.is_floating_point() or value.dtype != reference.dtype:
                    raise ContractError(f"{name} must have the model floating dtype")
                if not cpu_float32_all_finite(value):
                    raise ContractError(f"{name} must contain only finite values, including masked slots")
        if not bool(inputs["time_valid"][:, -1].all().item()):
            raise ContractError("current camera time must be valid")
        if not bool(inputs["ego_history_mask"][:, -1].all().item()):
            raise ContractError("current ego state must be valid")
        route_mask = inputs["route_mask"]
        if not bool(route_mask[:, :2].all().item()) or bool((route_mask[:, 1:] & ~route_mask[:, :-1]).any().item()):
            raise ContractError("dense route validity must be a prefix containing at least two points")
        offsets = inputs["time_offsets_s"]
        if bool((offsets > 0.0).any().item()):
            raise ContractError("future camera times are forbidden")
        if not bool((offsets[:, 1:] > offsets[:, :-1]).all().item()):
            raise ContractError("camera times must increase strictly to the current frame")
        expected = offsets.new_tensor(cfg.time_offsets_s).expand_as(offsets)
        if not bool(torch.isclose(offsets, expected, atol=3.0e-6, rtol=0.0).all().item()):
            raise ContractError("camera time offsets do not match the pinned five causal slots")
        if not bool((inputs["relative_pose"][:, -1].abs() <= 1.0e-6).all().item()):
            raise ContractError("current relative pose must be zero in the current vehicle frame")
        valid_yaw = inputs["relative_pose"][..., 2][inputs["time_valid"]]
        if bool((valid_yaw.abs() > math.pi + 1.0e-6).any().item()):
            raise ContractError("valid relative yaw must be radians wrapped to [-pi, pi]")
        return batch


def upgrade_cpu_finite_model(model):
    """HH_261001 - Copy only an exact frozen loaded CPU export into the separately versioned equivalent network."""
    if type(model) is not CachedTemporalModelV2 or any(module.training for module in model.modules()):
        raise ContractError("finite revision requires the original eval-mode cached temporal model")
    if not all(value.device.type == "cpu" and value.dtype == torch.float32 for value in model.state_dict().values()):
        raise ContractError("finite revision requires the exact float32 CPU checkpoint ABI")
    fields = ("_research_temporal_checkpoint_sha256", "_research_temporal_export_npz_sha256",
              "_research_temporal_metadata_sha256", "_research_temporal_export_metadata")
    if not all(hasattr(model, name) for name in fields):
        raise ContractError("finite revision requires the original source-pinned loaded export")
    # HH_261001 - Construction cannot perturb the caller's random stream or change exported tensor contents.
    with torch.random.fork_rng(devices=[]):
        result = CPUFiniteCachedTemporalModelV2(model.config).eval()
    result.load_state_dict(model.state_dict(), strict=True)
    result.requires_grad_(False)
    for name in fields:
        setattr(result, name, copy.deepcopy(getattr(model, name)))
    result._research_cpu_finite_validation_revision = SCHEMA
    return result
