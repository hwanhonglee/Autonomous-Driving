"""HH_261002 - Own an exact final6000 joint-versus-V3 signed45 research comparison, not promotion."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path
import re

from portable_e2e.temporal_route_joint_export_research_20261002 import (
    EXPORT_SCHEMA, OWNER_SCHEMA, PARENT_CHECKPOINT_SHA, PARENT_DECLARATION_SHA,
    PARENT_METADATA_SHA, PARENT_NPZ_SHA, load_route_joint_research_npz, verify_scope,
)
from . import research_temporal_signed_recovery45_v3_town07_20261002 as corrected
from . import research_temporal_signed_recovery45_town07_20261002 as signed
from . import collect_temporal_route_joint_signed45_20261002 as live

ROOT, base, parent = corrected.ROOT, corrected.base, corrected.parent
recovery = signed.previous
RUN = ROOT / "storage/experiments/training/2026-10-02/temporal_route_joint_v1"
WEIGHTS, METADATA = RUN / "exports/step_006000/model_state.npz", RUN / "exports/step_006000/metadata.json"
WARM_PARENT = ROOT / "storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000"
BACKBONE_PARENT = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000"
MODULE = "scripts.e2e.research_temporal_route_joint_signed45_town07_20261002"
SCHEMA = "portable_e2e.actual_route_joint_final6000_signed45_three_arms.20261002.v1"
CORRECTED_OWNER_SHA = "b2f126b8f8d4c515805dcca10288a079cf509987f051a4f88446fc9027b948db"
BACKBONE_NPZ_SHA = "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735"
BACKBONE_METADATA_SHA = "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"
require, digest, read_json = base.require, base.digest, base.read_json
_signed_fixed, _signed_diagnostic, _signed_summary = corrected.fixed_contract, signed.diagnostic_report, signed.batch_summary
_original_worker = base.worker
_active, _exports, _owner_plan_sha = False, None, None


def batch_paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "fresh safe joint signed45 version required")
    output = ROOT / ("storage/experiments/validation/2026-10-02/temporal_route_joint_signed45_town07_" + run_id)
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    return signed.capture_argv(run_id, longitudinal_pi, declaration_sha)


def fixed_contract(run_id, longitudinal_pi):
    require(_exports is not None, "externally pinned completed final6000 joint export required")
    result = _signed_fixed(run_id, longitudinal_pi)
    # HH_261002 - Delete inherited old-learning assertions; the54 learned/47 bytefixed partition is explicit instead.
    for name in ("current_fixed_final3000_V3_weights_unchanged", "original_backbone_weights_byte_unchanged",
                 "parent_checkpoint_updates", "parent_raw_updates", "parent_mixed_updates", "backbone_optimizer_updates",
                 "route_recovery_branch_optimizer_updates", "no_new_training_or_checkpoint_selection"):
        result.pop(name, None)
    result.update(schema=SCHEMA, experiment="Fixed final6000 joint candidate under the same corrected signed45 V3 baseline setup",
        source_checkpoint_sha256=_exports["checkpoint_sha256"], optimizer_updates=6000, known_total_optimizer_updates=12200,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=6200,
        checkpoint_selection="prospectively fixed final6000 only; no observed-best or HOLDOUT selection",
        current_fixed_final6000_joint_weights_unchanged=True, original_backbone_weights_byte_unchanged=False,
        no_training_during_physical_study=True, no_checkpoint_selection=True,
        joint6000_training_completed_before_physical_study=True,
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True, semantic_outputs_preserved=False, stop_intent_trained=False,
        training_schema="portable_e2e.temporal_route_joint_training.20261002.v1", new_numeric_export_schema=EXPORT_SCHEMA,
        actual_same_profiles_and_corrected_manual_first_gear_teacher_as_V3_baseline=True,
        actual_same_2p40_PI_45seconds_and_all_original_safety_thresholds=True,
        no_output_path_speed_or_intent_replacement=True, full_MAIN9_completion_claimed=False,
        parent_numeric_export_sha256=dict(weights=PARENT_NPZ_SHA, metadata=PARENT_METADATA_SHA),
        backbone_parent_numeric_export_sha256=dict(weights=BACKBONE_NPZ_SHA, metadata=BACKBONE_METADATA_SHA),
        new_fixed_endpoint_export_sha256=dict(weights=_exports["weights_sha256"], metadata=_exports["metadata_sha256"]),
        actual_training_declaration_sha256=_exports["declaration_sha256"],
        endpoint_screen_result_is_offline_not_physical_approval=True)
    # HH_261002 - The actual child imports this NEW module and must receive the same four externally pinned identities.
    command = result["worker_command"]
    require(command[command.index("-m") + 1] == MODULE, "joint private-worker module differs")
    require(not any(value.startswith("--expected-") for value in command), "unknown inherited worker asset overrides")
    command.extend(value for name in ("weights", "metadata", "checkpoint", "declaration")
        for value in ("--expected-" + name + "-sha256", _exports[name + "_sha256"]))
    return result


def runtime_sources():
    """HH_261002 - Explicit NEW6000 closure replaces only the old3000 metadata guard; inherited byte guards stay exact."""
    require(_exports is not None, "fixed6000 source scope not bound")
    live.verify_frozen()
    require(digest(Path(corrected.__file__)) == CORRECTED_OWNER_SHA, "corrected baseline owner changed")
    imported = base.imported_source_closure((Path(__file__).resolve(), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"))
    imported.update((ROOT / "tools/e2e/run_carla_map.sh", ROOT / "tools/e2e/env.sh", base.DEFAULT_CONTRACT_PATH, base.INHERITED_FREEZE))
    description = read_json(METADATA)
    require(digest(METADATA) == _exports["metadata_sha256"] and description["checkpoint_sha256"] == _exports["checkpoint_sha256"]
        and description["declaration_sha256"] == _exports["declaration_sha256"], "exact final6000 metadata changed")
    verify_scope(description)
    require(len(description["source_sha256"]) == 26, "actual final6000 complete26 training-source pins required")
    for name, sha in description["source_sha256"].items():
        source = ROOT / name
        require(not Path(name).is_absolute() and ".." not in Path(name).parts and source.resolve().is_relative_to(ROOT)
            and digest(source) == sha, "fixed joint training-source dependency changed: " + name)
        imported.add(source)
    pins = {str(path.relative_to(ROOT)): digest(path) for path in sorted(imported)}
    require(all(pins.get(name) == sha for name, sha in parent.FROZEN_REVISIONS.items()), "inherited reviewed runtime revision changed")
    require(digest(base.INHERITED_FREEZE) == base.INHERITED_FREEZE_SHA, "inherited source freeze changed")
    inherited = read_json(base.INHERITED_FREEZE)["source_sha256"]
    require(all(name not in inherited or inherited[name] == sha for name, sha in pins.items()), "inherited frozen dependency changed")
    return pins


def load_model(weights, metadata, *, weights_sha256, metadata_sha256):
    require(_exports is not None and Path(weights) == WEIGHTS and Path(metadata) == METADATA
        and weights_sha256 == _exports["weights_sha256"] and metadata_sha256 == _exports["metadata_sha256"],
        "exact externally pinned joint final6000 assets required")
    actuation = _owner_plan_sha is not None
    binding = dict(schema=OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=MODULE,
        purpose="owned_carla_research_only" if actuation else "offline_cpu_numeric_validation_only",
        weights_sha256=weights_sha256, metadata_sha256=metadata_sha256,
        checkpoint_sha256=_exports["checkpoint_sha256"], declaration_sha256=_exports["declaration_sha256"],
        fixed_endpoint_updates=6000, simulator_control_authorized=actuation, simulation_only=True,
        owner_plan_sha256=_owner_plan_sha, vehicle_control_approved=False, model_promoted=False)
    return load_route_joint_research_npz(weights, metadata, weights_sha256=weights_sha256, metadata_sha256=metadata_sha256,
        parent_weights=WARM_PARENT / "model_state.npz", parent_metadata=WARM_PARENT / "metadata.json",
        backbone_parent_weights=BACKBONE_PARENT / "model_state.npz", backbone_parent_metadata=BACKBONE_PARENT / "metadata.json",
        research_owner_binding=binding, source_root=ROOT)


def private_worker(run_id, expected):
    global _owner_plan_sha
    require(_active and _exports is not None and _owner_plan_sha is None, "exclusive NEW joint private worker required")
    plan = base.verify_plan(run_id, expected)
    base.assert_owned_worker(plan, expected)
    _owner_plan_sha = expected
    try:
        return _original_worker(run_id, expected)
    finally:
        _owner_plan_sha = None


def diagnostic_report(output, plan, owner):
    result = _signed_diagnostic(output, plan, owner)
    result.update(schema=SCHEMA + ".diagnostic_report", original_backbone_weights_byte_unchanged=False,
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, semantic_outputs_preserved=False,
        source_checkpoint_sha256=plan["source_checkpoint_sha256"], optimizer_updates=6000,
        actual_same_V3_baseline_physical_setup=True, full_MAIN9_completion_claimed=False)
    return result


def batch_summary(run_id, expected):
    result = _signed_summary(run_id, expected)
    result.update(schema=SCHEMA + ".batch_summary", optimizer_updates=6000,
        actual_same_V3_baseline_physical_setup=True, full_MAIN9_completion_claimed=False)
    return result


def verify_exports(pins):
    require(isinstance(pins, dict) and set(pins) == {"weights_sha256", "metadata_sha256", "checkpoint_sha256", "declaration_sha256"}
        and all(isinstance(v, str) and re.fullmatch(r"[0-9a-f]{64}", v) for v in pins.values()), "four external final6000 identities required")
    require(digest(WEIGHTS) == pins["weights_sha256"] and digest(METADATA) == pins["metadata_sha256"], "externally pinned actual joint bytes differ")
    description = read_json(METADATA); verify_scope(description)
    require(description["npz_sha256"] == pins["weights_sha256"] and description["checkpoint_sha256"] == pins["checkpoint_sha256"]
        and description["declaration_sha256"] == pins["declaration_sha256"], "actual joint endpoint lineage differs")
    for path, sha in ((WARM_PARENT / "model_state.npz", PARENT_NPZ_SHA), (WARM_PARENT / "metadata.json", PARENT_METADATA_SHA),
                      (BACKBONE_PARENT / "model_state.npz", BACKBONE_NPZ_SHA), (BACKBONE_PARENT / "metadata.json", BACKBONE_METADATA_SHA)):
        require(digest(path) == sha, "one of both exact parent asset pairs changed")
    warm = read_json(WARM_PARENT / "metadata.json")
    require(warm["checkpoint_sha256"] == PARENT_CHECKPOINT_SHA and warm["declaration_sha256"] == PARENT_DECLARATION_SHA,
        "actual V3 warm-parent lineage changed")
    return description


@contextmanager
def activated(pins):
    """HH_261002 - Fully restore version bindings; no old file, weight, capture, plan, or mutable outcome is edited."""
    global _active, _exports
    require(not _active and not parent._active and not signed._active and not recovery._active and not corrected._active,
        "joint owner must not overlap an active baseline owner scope")
    description = verify_exports(pins)
    saved_assets = base.ASSETS
    assets = {path: sha for path, sha in saved_assets.items() if path not in (base.WEIGHTS, base.METADATA)}
    assets.update({WEIGHTS: pins["weights_sha256"], METADATA: pins["metadata_sha256"],
        WARM_PARENT / "model_state.npz": PARENT_NPZ_SHA, WARM_PARENT / "metadata.json": PARENT_METADATA_SHA,
        BACKBONE_PARENT / "model_state.npz": BACKBONE_NPZ_SHA, BACKBONE_PARENT / "metadata.json": BACKBONE_METADATA_SHA,
        signed.GEOMETRY: signed.GEOMETRY_SHA, live.manual.v2.CENTER_MANIFEST: live.manual.v2.CENTER_MANIFEST_SHA})
    changes = {
        base: dict(WEIGHTS=WEIGHTS, METADATA=METADATA, WEIGHTS_SHA=pins["weights_sha256"], METADATA_SHA=pins["metadata_sha256"],
            CHECKPOINT_SHA=pins["checkpoint_sha256"], ASSETS=assets, worker=private_worker),
        parent: dict(batch_paths=batch_paths, capture_argv=capture_argv, fixed_contract=fixed_contract, runtime_sources=runtime_sources,
            load_model=load_model, live=live, MODULE=MODULE, SCHEMA=SCHEMA, diagnostic_report=diagnostic_report,
            CHECKPOINT_SHA=pins["checkpoint_sha256"], ASSETS=assets, batch_summary=batch_summary),
        signed: dict(batch_paths=batch_paths, live=live, MODULE=MODULE, SCHEMA=SCHEMA),
        recovery: dict(_exports=dict(pins), PARENT=WARM_PARENT),
    }
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    _active = True; _exports = dict(pins)
    try:
        for module, values in changes.items():
            for name, value in values.items(): setattr(module, name, value)
        runtime_sources()
        yield
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items(): setattr(module, name, value)
        _exports = None; _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remainder = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
        for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return parent.main(remainder)


if __name__ == "__main__":
    raise SystemExit(main())
