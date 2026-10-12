"""HH_261001 - Prospectively seal both computational latency fixtures under identical final-3000 weights."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import torch

from portable_e2e.temporal_owned_camera_history_v2_20261001 import (
    OwnedCameraTemporalSimulationResearchPolicyV2, own_observed_camera_bytes)
from portable_e2e.temporal_simulation_research_v2_20261001 import load_npz
from . import profile_temporal_v2_full_bgra_20261001 as benchmark


ROOT = benchmark.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/temporal_v2_full_bgra_latency_sealed_v1"
EXPORT = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_low_speed_joint_v2/exports/step_003000"
WEIGHTS_SHA = "768c10aefef21b46587d34da79896dcf1d6acf0a7373eab27045d6c4e2460c90"
METADATA_SHA = "eb74e50c5150b374aaa21bced41fd703507ed16c9fa863fd4a06e9a5896590b7"


def digest(path):
    path = Path(path)
    if not path.is_file() or path.is_symlink():
        raise ValueError("regular non-symlink source required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def main():
    if OUTPUT.exists():
        raise ValueError("fresh create-only profiling output required")
    weights, metadata = EXPORT / "model_state.npz", EXPORT / "metadata.json"
    if digest(weights) != WEIGHTS_SHA or digest(metadata) != METADATA_SHA:
        raise ValueError("fixed final-3000 export changed")
    meta = json.loads(metadata.read_text())
    names = set(meta["source_sha256"]) | {
        str(Path(__file__).relative_to(ROOT)), str(Path(benchmark.__file__).relative_to(ROOT)),
        "learning/portable_e2e/temporal_owned_camera_history_v2_20261001.py",
        "learning/portable_e2e/temporal_simulation_research_v2_20261001.py",
        "tools/canonical_workspace.py", "tools/workspace_python.py",
    }
    pins = {name: digest(ROOT / name) for name in sorted(names)}
    if any(pins[name] != sha for name, sha in meta["source_sha256"].items()):
        raise ValueError("immutable training source pin changed")
    OUTPUT.mkdir(parents=True)
    plan = dict(comment="HH_261001 - Both ordered arms use the same synthetic inputs, weights and original deadline.",
        schema="portable_e2e.final3000_sealed_bgra_latency_comparison.20261001.v1",
        declared_at_utc=datetime.now(timezone.utc).isoformat(),
        arms=["generic_original", "immutable_owned_history"], trials_per_arm=20,
        weights_sha256=WEIGHTS_SHA, metadata_sha256=METADATA_SHA,
        source_checkpoint_sha256=meta["checkpoint_sha256"], source_sha256=pins,
        original_hard_policy_deadline_seconds=.1, CPU_threads=1,
        longitudinal_pi=True, simulation_or_vehicle_run=False, physical_driving_measured=False,
        ten_hz_end_to_end_proven=False, outcome_selected_retry=False,
        previous_exploratory_stdout_profile_had_preexecution_source_snapshot=False)
    write_new(OUTPUT / "declaration.json", plan)
    for name, sha in pins.items():
        target = OUTPUT / "source" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        with (ROOT / name).open("rb") as source, target.open("xb") as destination:
            shutil.copyfileobj(source, destination)
        if digest(target) != sha:
            raise ValueError("source changed during archive")
    torch.set_num_threads(1)
    model = load_npz(weights, metadata, weights_sha256=WEIGHTS_SHA, metadata_sha256=METADATA_SHA)
    results = []
    for arm in plan["arms"]:
        kwargs = dict(policy_class=OwnedCameraTemporalSimulationResearchPolicyV2,
            own_frame=own_observed_camera_bytes) if arm == "immutable_owned_history" else {}
        result = benchmark.profile(model, 20, True, **kwargs)
        result.update(arm=arm, declaration_sha256=digest(OUTPUT / "declaration.json"),
            generic_and_owned_share_same_prospectively_sealed_profiler=True,
            observed_camera_data=False, vehicle_control_executed=False)
        write_new(OUTPUT / (arm + ".json"), result)
        results.append(dict(arm=arm, fixture_accepted_count=result["fixture_accepted_count"],
            deadline_rejected_count=result["deadline_rejected_count"],
            policy_total_seconds=result["policy_total_seconds"],
            full_call_seconds=result["full_call_seconds"],
            receipt_sha256=digest(OUTPUT / (arm + ".json"))))
    if any(digest(ROOT / name) != sha or digest(OUTPUT / "source" / name) != sha for name, sha in pins.items()):
        raise ValueError("source changed during fixed computational benchmarks")
    summary = dict(schema=plan["schema"] + ".summary", declaration_sha256=digest(OUTPUT / "declaration.json"),
        results=results, source_and_archives_unchanged=True,
        physical_driving_measured=False, ten_hz_end_to_end_proven=False,
        independent_generalization_approved=False, full_feature_approval=False)
    write_new(OUTPUT / "summary.json", summary)
    print(json.dumps(summary, sort_keys=True, indent=2))


if __name__ == "__main__":
    main()
