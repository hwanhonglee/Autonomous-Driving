"""HH_261001 - Activate new immutable-byte camera ownership without editing the frozen V2 collector."""

from __future__ import annotations

from contextlib import contextmanager
import json
import time

import torch

from portable_e2e.temporal_owned_camera_history_v2_20261001 import (
    SCHEMA, OwnedCameraTemporalSimulationResearchPolicyV2, own_observed_camera_bytes)
from portable_e2e.temporal_simulation_research_v2_20261001 import load_npz
from . import collect_temporal_v2_simulation_research_20261001 as frozen


class OwnedCameraTemporalLiveResearchSessionV2(frozen.TemporalLiveResearchSessionV2):
    """HH_261001 - Own immutable copies of each actual exact-frame six-camera source at observation delivery."""

    def observe_bundle(self, bundle, frame):
        super().observe_bundle(bundle, frame)
        started = time.perf_counter_ns()
        observed = self.observed_frames[-1]
        owned = own_observed_camera_bytes(observed)
        self.observed_frames[-1] = owned
        finished = time.perf_counter_ns()
        with (self.output / "owned_camera_source.jsonl").open("a") as stream:
            stream.write(json.dumps(dict(schema=SCHEMA, native_frame=frame,
                source_image_timestamp_simulation_ns=observed.timestamp_ns,
                source_ownership_started_monotonic_ns=started, source_ownership_finished_monotonic_ns=finished,
                exact_original_source_bytes_owned=6*360*640*4,
                immutable_bytes_backing=True, source_pixel_values_changed=False,
                ownership_seconds=(finished-started)/1.e9,
                ownership_within_observed_bundle_delivery_to_ACK_measurement=True,
                source_SHA_recorded_by_policy=True, no_source_camera_SHA_omitted=True,
                exact_normalized_tensor_archive_retained=False), allow_nan=False) + "\n")


_active = False


@contextmanager
def activated(model, args):
    """HH_261001 - Replace only this worker's session/policy class bindings; restore all hooks on any exit."""
    global _active
    if _active:
        raise RuntimeError("owned camera activation cannot nest")
    _active = True
    saved_session = frozen.TemporalLiveResearchSessionV2
    saved_policy = frozen.TemporalSimulationResearchPolicyV2
    frozen.TemporalLiveResearchSessionV2 = OwnedCameraTemporalLiveResearchSessionV2
    frozen.TemporalSimulationResearchPolicyV2 = OwnedCameraTemporalSimulationResearchPolicyV2
    try:
        with frozen.activated(model, args):
            yield
    finally:
        frozen.TemporalLiveResearchSessionV2, frozen.TemporalSimulationResearchPolicyV2 = saved_session, saved_policy
        _active = False


def run(model, args):
    with activated(model, args):
        return frozen.collector.run(args)


def main(argv=None):
    args = frozen.parse_args(argv)
    frozen.validate_capture(args, frozen.collector.load_route(args.route_file))
    torch.set_num_threads(1)
    model = load_npz(args.research_weights, args.research_metadata,
        weights_sha256=args.research_weights_sha256, metadata_sha256=args.research_metadata_sha256)
    print(run(model, args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
