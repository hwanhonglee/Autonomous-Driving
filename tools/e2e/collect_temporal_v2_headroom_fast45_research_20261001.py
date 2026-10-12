"""HH_261001 - Declare a separate 45-second headroom/CPU-finite simulation collector, retaining original control authority."""

from __future__ import annotations

from contextlib import contextmanager

import torch

from portable_e2e.temporal_cpu_finite_validation_v2_20261001 import (
    CPUFiniteCachedTemporalModelV2, upgrade_cpu_finite_model)
from . import collect_temporal_v2_headroom_research_20261001 as headroom


frozen = headroom.owned.frozen
_active = False


def validate_capture(args, route):
    """HH_261001 - The new owner prospectively selects 45 seconds; no old 20-second contract is altered."""
    frozen._require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 45 and args.target_speed_kmh == 9
        and args.weather == "ClearNoon" and args.seed == 0 and args.spawn_z_offset_m == .5
        and args.research_speed_cap_mps == 2.5 and args.research_max_cte_m == 1.
        and route["town"] == "Town07" and route["scenario"] == "straight"
        and isinstance(args.research_source_sha256, str) and len(args.research_source_sha256) == 64
        and all(x in "0123456789abcdef" for x in args.research_source_sha256)
        and args.research_longitudinal_pi is True,
        "fixed isolated 45-second headroom/CPU-finite Town07 precursor contract differs")


@contextmanager
def activated(model, args):
    """HH_261001 - Scope a new validation contract around the existing exact observed-session activation."""
    global _active
    if _active:
        raise RuntimeError("45-second headroom/CPU-finite activation cannot nest")
    if not isinstance(model, CPUFiniteCachedTemporalModelV2):
        raise ValueError("declared CPU-finite model revision required")
    _active = True
    previous = frozen.validate_capture
    frozen.validate_capture = validate_capture
    try:
        with headroom.activated(model, args):
            yield
    finally:
        frozen.validate_capture = previous
        _active = False


def main(argv=None):
    args = frozen.parse_args(argv)
    validate_capture(args, frozen.collector.load_route(args.route_file))
    torch.set_num_threads(1)
    model = upgrade_cpu_finite_model(frozen.load_npz(args.research_weights, args.research_metadata,
        weights_sha256=args.research_weights_sha256, metadata_sha256=args.research_metadata_sha256))
    with activated(model, args):
        print(frozen.collector.run(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
