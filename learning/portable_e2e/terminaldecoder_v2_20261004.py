"""HH_261004 - Prepare a semantic terminal mesh without changing physical gates.

The frozen V1 envelope, time retiming, witnesses, checks and ordinary decoder
remain authoritative.  Only grid construction changes: exact 0/first100ms/D
and original 2m raw-field breakpoints partition the authentic support.  Each
interval is subdivided uniformly, not combined with a nearly overlapping grid.
No near-cell merge, distance adjustment, future extrapolation or actuation is
permitted.  Truly unrepresentable positive cells still reject explicitly.
"""

from __future__ import annotations

import math

import torch

from .path_time_decoder_terminal_v1_20261002 import PathTimeDecoderTerminalV1, _require


SCHEMA = "portable_e2e.causal_terminal_semantic_mesh.prepared.20261004.v2"
FROZEN_TERMINAL_V1_SHA256 = "df9beb1d7f71c6757907f7b6e7792ba3d9b7e2679d75c165b952cf6f6895943a"


class PathTimeDecoderTerminalV2(PathTimeDecoderTerminalV1):
    """HH_261004 - Override only the grid; every old numerical/physical audit remains."""

    def _grid(self, distance, boundary):
        _require(distance.ndim == boundary.ndim == 0 and distance.dtype == boundary.dtype == torch.float64
                 and distance.device == boundary.device and bool(torch.isfinite(distance))
                 and bool(torch.isfinite(boundary)) and bool(distance > 0)
                 and bool((boundary >= 0) & (boundary <= distance)),
                 "SEMANTIC_GRID_INPUT", "positive finite float64 actual D and in-support exact boundary required")
        # HH_261004 - All semantic breakpoints are retained exactly; only literally identical anchors are shared.
        raw_knots = torch.arange(1, 33, dtype=distance.dtype, device=distance.device) * 2.
        anchors = torch.sort(torch.cat((distance.new_zeros(1), boundary.reshape(1), distance.reshape(1),
                                        raw_knots[(raw_knots > 0) & (raw_knots < distance)]))).values
        distinct = torch.cat((torch.ones(1, dtype=torch.bool, device=anchors.device), anchors[1:] > anchors[:-1]))
        anchors = anchors[distinct]
        maximum_cell = min(self.config.spatial_step_m, float(distance) / 256.)
        _require(math.isfinite(maximum_cell) and maximum_cell > 0,
                 "UNREPRESENTABLE_SEMANTIC_GRID", "actual positive support cannot be subdivided representably")
        chunks = [anchors[:1]]
        for first, last in zip(anchors[:-1], anchors[1:]):
            interval = last - first
            count = math.ceil(float(interval) / maximum_cell)
            _require(count >= 1, "UNREPRESENTABLE_SEMANTIC_GRID", "actual semantic interval must remain positive")
            # HH_261004 - Generate interiors only inside their interval; exact anchors are never reconstructed or moved.
            fractions = torch.arange(1, count, dtype=distance.dtype, device=distance.device) / count
            chunks.extend((first + interval * fractions, last.reshape(1)))
        grid = torch.cat(chunks)
        _require(bool((grid[1:] > grid[:-1]).all()), "UNREPRESENTABLE_SEMANTIC_GRID",
                 "every genuine positive cell must be distinct; no near-cell deletion or silent repair")
        _require(bool(grid[0] == 0) and bool(grid[-1] == distance) and bool((grid == boundary).any()),
                 "SEMANTIC_GRID_BOUNDARY", "exact zero, first100ms and authentic endpoint must survive")
        _require(bool((grid[1:] - grid[:-1] <= self.config.spatial_step_m + self.config.tolerance).all()),
                 "SEMANTIC_GRID_SAMPLING", "semantic cells retain the unchanged maximum0.25m sampling bound")
        return grid


def prepared_contract():
    """HH_261004 - A new grid is numerical preparation, not learned or actual driving performance."""
    return dict(schema=SCHEMA, prepared_cpu_only=True, prepared_not_deployed=True,
        frozen_terminal_v1_sha256=FROZEN_TERMINAL_V1_SHA256,
        overridden_methods=["_grid"], ordinary_outputs_errors_exact_delegate=True,
        anchors="exact0, exactfirst100ms arc, original2m raw-field knots within unchangedD, exactD",
        subdivision="each semantic interval: ceil(length/min(.25,D/256)) equal cells",
        near_cell_merge=False, authentic_distance_changed=False, missing_future_extrapolated=False,
        all_original_envelope_time_physical_audits_inherited=True, maximum_spatial_cell_m=.25,
        model_or_weights_changed=False, learned_STOP_claim=False, vehicle_control_approved=False,
        main_functions_approved=0, subfunctions_approved=0)
