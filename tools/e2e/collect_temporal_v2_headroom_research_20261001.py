"""HH_261001 - Activate a separately declared 2.40 m/s PI headroom experiment without editing frozen capture code."""

from __future__ import annotations

from contextlib import contextmanager

import torch

from portable_e2e.temporal_headroom_camera_research_v2_20261001 import (
    HeadroomOwnedCameraTemporalSimulationResearchPolicyV2)
from portable_e2e.temporal_simulation_research_v2_20261001 import load_npz
from . import collect_temporal_v2_owned_camera_research_20261001 as owned


_active = False


@contextmanager
def activated(model, args):
    """HH_261001 - Scope only the owned activation's policy binding and restore it on every exit."""
    global _active
    if _active:
        raise RuntimeError("headroom activation cannot nest")
    _active = True
    saved_policy = owned.OwnedCameraTemporalSimulationResearchPolicyV2
    owned.OwnedCameraTemporalSimulationResearchPolicyV2 = HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
    try:
        with owned.activated(model, args):
            yield
    finally:
        owned.OwnedCameraTemporalSimulationResearchPolicyV2 = saved_policy
        _active = False


def run(model, args):
    with activated(model, args):
        return owned.frozen.collector.run(args)


def main(argv=None):
    args = owned.frozen.parse_args(argv)
    owned.frozen.validate_capture(args, owned.frozen.collector.load_route(args.route_file))
    if not args.research_longitudinal_pi:
        raise ValueError("explicit longitudinal PI required for the headroom experiment")
    torch.set_num_threads(1)
    model = load_npz(args.research_weights, args.research_metadata,
        weights_sha256=args.research_weights_sha256, metadata_sha256=args.research_metadata_sha256)
    print(run(model, args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
