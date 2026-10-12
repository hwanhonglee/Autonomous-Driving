"""HH_261001 - Replace only fixed 33-to-257 decoder interpolation with deterministic dense GEMM.

Frozen decoder geometry, retiming, physical gates, labels and observed initial
speed remain unchanged. The eight subdivision weights are exactly representable
binary fractions. Floating accumulation can differ from native interpolation;
this revision claims tested numerical equivalence, not bitwise equivalence.
"""

from __future__ import annotations

from contextlib import contextmanager
import threading

import torch
from torch import Tensor
from torch.nn import functional as F

from .contract import ContractError
from .path_time_decoder_v2 import PathTimeDecoderV2


KERNEL_ID = "fixed33_to257_align_corners_dense_gemm.20261001.v1"
_LOCK = threading.RLock()
_ACTIVE_OWNER = None


def interpolation_matrix(*, dtype=torch.float32, device=None):
    """HH_261001 - Construct the unchanged align-corners linear map without CUDA scatter operations."""
    if dtype not in (torch.float32, torch.float64):
        raise ContractError("deterministic decoder interpolation needs float32 or float64")
    matrix = torch.zeros(33, 257, dtype=dtype, device="cpu")
    for column in range(257):
        lower = column // 8
        upper = min(lower+1, 32)
        fraction = (column % 8) / 8.
        matrix[lower, column] = 1-fraction
        matrix[upper, column] += fraction
    return matrix if device is None else matrix.to(device=device)


def fixed_linear_interpolation(value: Tensor, matrix: Tensor | None = None) -> Tensor:
    """HH_261001 - Use GEMM backward rather than CUDA upsample's nondeterministic atomic reduction."""
    if (not isinstance(value, Tensor) or value.ndim != 3 or value.shape[0] < 1
            or value.shape[1] < 1 or value.shape[-1] != 33
            or value.dtype not in (torch.float32, torch.float64)):
        raise ContractError("fixed interpolation requires nonempty [N,C,33] float tensors")
    if not bool(torch.isfinite(value).all()):
        raise ContractError("fixed interpolation input must be finite")
    if matrix is None:
        matrix = interpolation_matrix(dtype=value.dtype, device=value.device)
    if (matrix.shape != (33, 257) or matrix.dtype != value.dtype
            or matrix.device != value.device or matrix.requires_grad):
        raise ContractError("fixed interpolation matrix ABI differs")
    return (value.reshape(-1, 33) @ matrix).reshape(value.shape[0], value.shape[1], 257)


@contextmanager
def fixed_decoder_interpolation_scope(matrix):
    """HH_261001 - Scope the fixed-kernel dispatch to exactly one serial decoder forward call.

    Every unrelated interpolation is delegated unchanged. Calls with the exact
    decoder signature from another thread, nested scopes, or third-party
    changes to the dispatch are rejected instead of silently sharing ownership.
    """
    global _ACTIVE_OWNER
    with _LOCK:
        if _ACTIVE_OWNER is not None:
            raise ContractError("deterministic decoder interpolation scope cannot nest")
        owner = threading.get_ident()
        original = F.interpolate
        _ACTIVE_OWNER = owner
        receipt = dict(kernel_id=KERNEL_ID, fixed_calls=0, unrelated_calls_delegated=0)

        def dispatch(input, size=None, scale_factor=None, mode="nearest", align_corners=None,
                     recompute_scale_factor=None, **kwargs):
            fixed = (isinstance(input, Tensor) and input.ndim == 3 and input.shape[1:] == (1, 33)
                     and size == 257 and scale_factor is None and mode == "linear"
                     and align_corners is True and recompute_scale_factor is None and not kwargs)
            if fixed:
                if threading.get_ident() != owner:
                    raise ContractError("fixed decoder interpolation used by a non-owner thread")
                receipt["fixed_calls"] += 1
                return fixed_linear_interpolation(input, matrix)
            receipt["unrelated_calls_delegated"] += 1
            return original(input, size=size, scale_factor=scale_factor, mode=mode,
                align_corners=align_corners, recompute_scale_factor=recompute_scale_factor, **kwargs)

        F.interpolate = dispatch
        try:
            yield receipt
        finally:
            changed = F.interpolate is not dispatch
            F.interpolate = original
            _ACTIVE_OWNER = None
            if changed:
                raise ContractError("interpolation dispatch changed during owned scope")


class DeterministicPathTimeDecoderV2(PathTimeDecoderV2):
    """HH_261001 - Reuse every frozen decoder operation except the two fixed spatial interpolations."""

    def __init__(self, config=None):
        super().__init__(config)
        self.register_buffer("fixed_interpolation_matrix", interpolation_matrix(), persistent=False)
        self.last_interpolation_receipt = None

    def forward(self, curvature_knots, preferred_speed_knots, current_speed,
                intent, stop_distance_m, supported_distance_m):
        matrix = self.fixed_interpolation_matrix.to(device=curvature_knots.device, dtype=curvature_knots.dtype)
        with fixed_decoder_interpolation_scope(matrix) as receipt:
            output = super().forward(curvature_knots, preferred_speed_knots, current_speed,
                                     intent, stop_distance_m, supported_distance_m)
        if receipt["fixed_calls"] != 2:
            raise ContractError("frozen decoder no longer uses exactly two declared fixed interpolations")
        self.last_interpolation_receipt = dict(receipt)
        return output
