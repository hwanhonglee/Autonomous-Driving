"""HH_261001 - Profile full-size synthetic BGRA computational cost without physical control or driving claims."""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import torch

from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.simulation_research import ControllerConfig
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e.temporal_simulation_research_v2_20261001 import (
    ObservedTemporalFrameV2, TemporalSimulationResearchPolicyV2, load_npz)


ROOT = Path(__file__).resolve().parents[2]


def summary(values):
    return dict(count=len(values), mean=float(np.mean(values)), p95=float(np.percentile(values, 95)),
                maximum=max(values)) if values else dict(count=0, mean=None, p95=None, maximum=None)


def synthetic_frame(index, *, identity, pixels):
    """HH_261001 - Synthetic timestamps/poses are benchmark fixtures, never admitted as observed live inputs."""
    stamp = 1_000_000_000 + index * 100_000_000
    calibration = (.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.)
    state = (1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.)
    inputs = RuntimeInputs(tuple(RuntimeCameraFrame(stamp, image.copy(), "bgra8") for image in pixels),
        (calibration,) * 6, (state,) * 10, (True,) * 10, ((0., 0.), (100., 0.)))
    return ObservedTemporalFrameV2(stamp, inputs, (index * .1, 0., 0.),
        (0., 0., 0., 1.), stamp, **identity)


def untrained_fixture_model():
    """HH_261001 - Straight/speed head fixture permits complete decoder/gate/PI timing, not trained performance."""
    torch.manual_seed(123)
    model = CachedTemporalModelV2(TemporalModelConfigV2()).eval()
    with torch.no_grad():
        model.curvature_head.weight.zero_()
        model.curvature_head.bias.zero_()
        model.speed_head.weight.zero_()
        model.speed_head.bias.fill_(math.log(math.expm1(2.)))
    digest = hashlib.sha256()
    for key, value in model.state_dict().items():
        digest.update(key.encode())
        digest.update(value.detach().cpu().numpy().tobytes())
    model._research_temporal_checkpoint_sha256 = digest.hexdigest()
    model._research_temporal_export_npz_sha256 = hashlib.sha256(b"no-export-untrained-latency-fixture").hexdigest()
    model._research_temporal_metadata_sha256 = hashlib.sha256(b"no-sidecar-untrained-latency-fixture").hexdigest()
    return model


def profile(model, trials, longitudinal_pi, *, policy_class=TemporalSimulationResearchPolicyV2, own_frame=lambda frame: frame):
    """HH_261001 - Enforce the original deadline; resets and rejected controls remain recorded."""
    identity = dict(episode_id="SYNTHETIC_BGRA_LATENCY_ONLY_NOT_DRIVING", split="train",
        rig_id="synthetic-six-camera-latency-fixture",
        source_sha256=hashlib.sha256(b"synthetic-bgra-fixture-not-CARLA-source").hexdigest())
    policy = policy_class(model, ControllerConfig(2.7, .6),
        **identity, deadline_s=.1, longitudinal_pi=longitudinal_pi)
    rng = np.random.default_rng(123)
    pixels = tuple(rng.integers(0, 256, (360, 640, 4), dtype=np.uint8) for _ in range(6))
    frames = deque(maxlen=10)
    rows, prefills = [], []
    next_index = 0
    for trial in range(trials):
        if policy._prefill_count < 2:
            # HH_261001 - Cold prefills are disclosed and measured, never waived as live control time.
            for _ in range(10):
                frames.append(own_frame(synthetic_frame(next_index, identity=identity, pixels=pixels)))
                next_index += 1
            for _ in range(2):
                current = frames[-1]
                prefills.append(policy.prefill(current, past_frames=tuple(frames)[:-1],
                    now_ns=current.timestamp_ns, ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns))
                frames.append(own_frame(synthetic_frame(next_index, identity=identity, pixels=pixels)))
                next_index += 1
        current = frames[-1]
        started = time.perf_counter_ns()
        decision = policy.step(current, past_frames=tuple(frames)[:-1], now_ns=current.timestamp_ns,
                               ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)
        finished = time.perf_counter_ns()
        rows.append(dict(trial=trial, full_call_seconds=(finished-started)/1.e9,
            policy_total_seconds=decision["total_seconds"], raw_nn_seconds=decision["raw_nn_seconds"],
            cache=decision.get("cache"), decoder_reached=decision["decoded_output"] is not None,
            geometry_gate=decision["candidate_gates"], PI_reached="longitudinal_pi_research" in decision,
            learned_proposal_accepted_for_fixture=decision["learned_control_proposed"],
            emergency_reason=decision["emergency_reason"]))
        frames.append(own_frame(synthetic_frame(next_index, identity=identity, pixels=pixels)))
        next_index += 1
    return dict(schema="portable_e2e.full_bgra_computational_latency_fixture.20261001.v1",
        simulation_or_vehicle_run=False, measured_camera_callback=False,
        synthetic_inputs=True, synthetic_current_and_history_poses=True,
        actual_driving_performance_measured=False, feature_approved=False, ten_hz_end_to_end_proven=False,
        original_hard_policy_deadline_seconds=.1, deadline_relaxed=False,
        source_checkpoint_sha256=model._research_temporal_checkpoint_sha256,
        model_config=model.config.to_dict(), CPU_threads=torch.get_num_threads(), torch_version=torch.__version__,
        source_camera_shape=[360, 640, 4], retained_source_frames=10, camera_count=6,
        retained_source_bytes=10*6*360*640*4, longitudinal_pi=longitudinal_pi,
        policy_runtime_source_sha256=hashlib.sha256((ROOT / "learning/portable_e2e/temporal_simulation_research_v2_20261001.py").read_bytes()).hexdigest(),
        full_call_seconds=summary([r["full_call_seconds"] for r in rows]),
        policy_total_seconds=summary([r["policy_total_seconds"] for r in rows]),
        raw_nn_seconds=summary([r["raw_nn_seconds"] for r in rows if r["raw_nn_seconds"] is not None]),
        fixture_accepted_count=sum(r["learned_proposal_accepted_for_fixture"] for r in rows),
        deadline_rejected_count=sum(r["emergency_reason"] == "temporal inference/control deadline exceeded" for r in rows),
        cold_prefills=prefills, rows=rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--longitudinal-pi", action="store_true")
    parser.add_argument("--untrained-latency-only", action="store_true")
    parser.add_argument("--immutable-owned-camera-history", action="store_true")
    parser.add_argument("--weights", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--weights-sha256")
    parser.add_argument("--metadata-sha256")
    args = parser.parse_args(argv)
    if not 1 <= args.trials <= 100:
        parser.error("bounded fixture requires 1..100 trials")
    loaded = (args.weights, args.metadata, args.weights_sha256, args.metadata_sha256)
    if args.untrained_latency_only and any(loaded) or not args.untrained_latency_only and not all(loaded):
        parser.error("choose explicit untrained latency fixture OR all four exact trained export inputs")
    torch.set_num_threads(1)
    model = untrained_fixture_model() if args.untrained_latency_only else load_npz(args.weights, args.metadata,
        weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256)
    options = {}
    if args.immutable_owned_camera_history:
        from portable_e2e.temporal_owned_camera_history_v2_20261001 import (
            OwnedCameraTemporalSimulationResearchPolicyV2, own_observed_camera_bytes)
        options.update(policy_class=OwnedCameraTemporalSimulationResearchPolicyV2, own_frame=own_observed_camera_bytes)
    result = profile(model, args.trials, args.longitudinal_pi, **options)
    result["untrained_fixture_weights"] = args.untrained_latency_only
    result["immutable_owned_camera_history"] = args.immutable_owned_camera_history
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
