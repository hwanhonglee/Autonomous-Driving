"""HH_261001 - Seal every original-versus-NumPy finite-scan timing before a fixed 20+20 CPU-only comparison."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import torch

from portable_e2e.temporal_cpu_finite_validation_v2_20261001 import upgrade_cpu_finite_model
from portable_e2e.temporal_headroom_camera_research_v2_20261001 import HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
from portable_e2e.temporal_owned_camera_history_v2_20261001 import own_observed_camera_bytes
from . import profile_temporal_v2_full_bgra_20261001 as benchmark
from . import research_temporal_v2_owned_camera_town07_20261001 as frozen_owner


ROOT = frozen_owner.ROOT
OUTPUT = ROOT/"storage/experiments/validation/2026-10-01/temporal_v2_cpu_finite_latency_sealed_v1"


class MeasuredHeadroomPolicy(HeadroomOwnedCameraTemporalSimulationResearchPolicyV2):
    """HH_261001 - Retain the complete actual fixture decision after the inherited deadline clock finishes."""
    records = []
    def step(self, *args, **kwargs):
        result = super().step(*args, **kwargs)
        self.records.append(result)
        return result


def main():
    frozen_owner.require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh sealed benchmark output required")
    frozen_owner.disk_check()
    pins = frozen_owner.runtime_sources()
    sources = frozen_owner.imported_source_closure((Path(__file__).resolve(),))
    pins.update({str(path.relative_to(ROOT)):frozen_owner.digest(path) for path in sorted(sources)})
    declared = dict(schema="portable_e2e.temporal_cpu_finite_guard_sealed_latency.20261001.v1",
        declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=pins, weights_sha256=frozen_owner.WEIGHTS_SHA, metadata_sha256=frozen_owner.METADATA_SHA,
        source_checkpoint_sha256=frozen_owner.CHECKPOINT_SHA,
        arms=["owned_headroom_original_finite", "owned_headroom_numpy_finite"], trials_per_arm=20,
        same_full_BGRA_retained_source_bytes=10*6*360*640*4,
        CPU_threads=1, hard_policy_deadline_s=.1, policy_deadline_relaxed=False,
        controller_target_cap_mps=2.4, explicit_controller_calibration_intervention=True,
        original_speed_envelope_mps=2.5, NN_decoder_selected_math_unchanged=True,
        full_fixture_decisions_preserved=True, all_resets_and_prefills_preserved=True,
        simulation_or_vehicle_run=False, actual_driving_performance_measured=False,
        sensor_callback_entry_measured=False, observed_bundle_to_ACK_measured=False,
        ten_hz_end_to_end_proven=False, feature_approved=False, outcome_selected_retry=False)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    frozen_owner.write_new(OUTPUT/"declaration.json", declared)
    declaration_sha = frozen_owner.digest(OUTPUT/"declaration.json")
    for name, sha in pins.items():
        target = OUTPUT/"source"/name; target.parent.mkdir(parents=True, exist_ok=True)
        with (ROOT/name).open("rb") as source, target.open("xb") as destination:
            shutil.copyfileobj(source, destination)
        frozen_owner.require(frozen_owner.digest(target)==sha, "source changed before benchmark")
    torch.set_num_threads(1)
    model = frozen_owner.load_npz(frozen_owner.WEIGHTS, frozen_owner.METADATA,
        weights_sha256=frozen_owner.WEIGHTS_SHA, metadata_sha256=frozen_owner.METADATA_SHA)
    results = []
    for arm in declared["arms"]:
        current = model if arm == declared["arms"][0] else upgrade_cpu_finite_model(model)
        MeasuredHeadroomPolicy.records = []
        result = benchmark.profile(current, 20, True, policy_class=MeasuredHeadroomPolicy,
                                   own_frame=own_observed_camera_bytes)
        result.update(arm=arm, declaration_sha256=declaration_sha,
            full_fixture_decisions=MeasuredHeadroomPolicy.records,
            actual_observed_camera_or_vehicle_run=False,
            finite_guard_revision="originalTorch" if current is model else "CPUfloat32NumPy",
            raw_NN_decoder_selected_math_unchanged=True)
        path = OUTPUT/(arm+".json"); frozen_owner.write_new(path, result)
        results.append(dict(arm=arm, fixture_accepted_count=result["fixture_accepted_count"],
            deadline_rejected_count=result["deadline_rejected_count"],
            full_call_seconds=result["full_call_seconds"], policy_total_seconds=result["policy_total_seconds"],
            raw_nn_seconds=result["raw_nn_seconds"], receipt_sha256=frozen_owner.digest(path)))
    frozen_owner.require(all(frozen_owner.digest(ROOT/name)==sha==frozen_owner.digest(OUTPUT/"source"/name)
        for name, sha in pins.items()), "source/archive changed during benchmark")
    frozen_owner.require(frozen_owner.digest(OUTPUT/"declaration.json")==declaration_sha, "declaration changed")
    report = dict(schema=declared["schema"]+".summary", declaration_sha256=declaration_sha,
        results=results, source_and_archives_unchanged=True, physical_driving_measured=False,
        full_callback_to_ACK_measured=False, ten_hz_end_to_end_proven=False, feature_approved=False)
    frozen_owner.write_new(OUTPUT/"summary.json", report)
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
