"""HH_261002 - Predeclare control then intrinsic final6000 Town03 research; keep all failed attempts and original gates."""
from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

from portable_e2e import temporal_intrinsic_curve_export_research_20261002 as loader
from . import research_temporal_route_joint_town03_right_v3_20261002 as parent
from . import collect_temporal_intrinsic_town03_right_20261002 as live

base, ROOT = parent.base, parent.ROOT
old = parent.previous.previous
numeric = parent.numeric
require, digest, read_json, write_new = base.require, base.digest, base.read_json, base.write_new
MODULE = "scripts.e2e.research_temporal_intrinsic_town03_right_20261002"
SCHEMA = live.OWNER_SCHEMA
PARENT_OWNER_SHA = "0a545281db1c54ec844347f95d3a7444a1ec4bd305eaacb132886f47ff655623"
CURRENT_PARENT = ROOT/"storage/experiments/training/2026-10-02/temporal_route_joint_v1/exports/step_006000"
ARMS = ("control", "intrinsic_curve")
_scope, _active, _batch_sha = None, False, None
_base_environment = old.environment
_evidence_assets = old.threads.evidence_assets
_parent_fixed = parent.fixed_contract


def paths(run_id):
    require(run_id == "v1" and _scope is not None, "fresh fixed intrinsic pair v1/arm scope required")
    root = ROOT/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1"
    output = root/_scope.arm
    return output, root/(_scope.arm+"_plan.json")


def batch_path():
    return ROOT/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1/batch_plan.json"


def authority(path, expected_sha):
    path = loader.canonical(path)
    require(path.is_relative_to(ROOT/"storage/experiments/plans/2026-10-02")
        or path.is_relative_to(ROOT/"storage/experiments/training/2026-10-02"), "completed pair authority must stay in scoped local evidence")
    value = loader.read_json(path, expected_sha)
    require(value.get("schema") == loader.AUTHORITY_SCHEMA and value.get("mode") == "RESEARCH_ONLY"
        and value.get("arm_order") == list(ARMS) and value.get("both_fixed_endpoints_completed") is True
        and value.get("parent") == loader.PARENT and value.get("parent_history_sha256") == loader.PARENT_HISTORY_SHA
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "external BOTH fixed endpoints/current parent research authority required")
    run = loader.canonical(value["pair_run"])
    require(run.is_relative_to(ROOT/"storage/experiments/training/2026-10-02"), "new pair mirror must remain local and separate from old captures")
    require(set(value.get("endpoints", {})) == set(ARMS), "both arms required before any physical study")
    for arm in ARMS:
        pins = value["endpoints"][arm]
        require(set(pins) == {"weights_sha256", "metadata_sha256", "checkpoint_sha256", "declaration_sha256"}
            and all(loader.is_sha(pin) for pin in pins.values()), "four external identities required for each fixed endpoint")
        description = loader.read_json(run/arm/"exports/step_006000/metadata.json", pins["metadata_sha256"])
        loader.verify_scope(description)
        require(description["arm"] == arm and description["source_sha256"] == value["source_sha256"]
            and description["npz_sha256"] == pins["weights_sha256"] and description["checkpoint_sha256"] == pins["checkpoint_sha256"]
            and description["declaration_sha256"] == pins["declaration_sha256"], "actual fixed arm metadata differs")
    require(len(value["source_sha256"]) == 29, "all29 unchanged fit-source pins required")
    return path, value, run


def verify_exports(pins):
    require(_scope is not None and pins == _scope.pins, "selected fixed arm identities differ from completed pair authority")
    _, value, run = authority(_scope.authority_path, _scope.authority_sha)
    require(value == _scope.authority and run == _scope.run, "completed pair authority changed")
    for arm in ARMS:
        endpoint = value["endpoints"][arm]; folder = run/arm/"exports/step_006000"
        require(digest(folder/"model_state.npz") == endpoint["weights_sha256"], "one of BOTH fixed numeric endpoints changed")
    for name, pin in value["source_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and str(relative) == name
            and digest(ROOT/relative) == pin, "actual29 training-source identity differs")
    require(digest(CURRENT_PARENT/"model_state.npz") == loader.PARENT["weights_sha256"]
        and digest(CURRENT_PARENT/"metadata.json") == loader.PARENT["metadata_sha256"], "actual current6000 parent changed")
    return loader.read_json(_scope.metadata, pins["metadata_sha256"])


def runtime_sources():
    require(_scope is not None, "actual completed intrinsic scope required")
    live.verify_frozen(); verify_exports(_scope.pins)
    imported = base.imported_source_closure((Path(__file__).resolve(), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py"))
    imported.update((ROOT/"tools/e2e/run_carla_map.sh", ROOT/"tools/e2e/env.sh", base.DEFAULT_CONTRACT_PATH, base.INHERITED_FREEZE))
    imported.update(ROOT/name for name in _scope.authority["source_sha256"])
    result = {str(path.relative_to(ROOT)): digest(path) for path in sorted(imported)}
    require(all(result.get(name) == pin for name, pin in _scope.authority["source_sha256"].items()), "all29 sources must be archived")
    require(all(result.get(name) == pin for name, pin in numeric.parent.FROZEN_REVISIONS.items()), "original runtime revision changed")
    require(digest(base.INHERITED_FREEZE) == base.INHERITED_FREEZE_SHA, "original source freeze changed")
    inherited = read_json(base.INHERITED_FREEZE)["source_sha256"]
    require(all(name not in inherited or inherited[name] == pin for name, pin in result.items()), "original frozen dependency changed")
    return result


def evidence_assets(pins):
    require(_scope is not None and pins == _scope.pins, "new intrinsic endpoint must be selected explicitly")
    # HH_261002 - Retain the original benchmark AS THE CURRENT-PARENT benchmark; never claim it evaluated new weights.
    result = _evidence_assets(loader.PARENT)
    value, run = _scope.authority, _scope.run
    result.update({_scope.authority_path: _scope.authority_sha, run/"declaration.json": value["pair_declaration_sha256"],
        run/"summary.json": value["pair_summary_sha256"], CURRENT_PARENT/"model_state.npz": loader.PARENT["weights_sha256"],
        CURRENT_PARENT/"metadata.json": loader.PARENT["metadata_sha256"]})
    for arm in ARMS:
        p = value["endpoints"][arm]; export = run/arm/"exports/step_006000"
        result.update({export/"model_state.npz": p["weights_sha256"], export/"metadata.json": p["metadata_sha256"],
            run/arm/"declaration.json": p["declaration_sha256"], run/arm/"checkpoints/step_006000.pt": p["checkpoint_sha256"]})
    require(all(digest(path) == pin for path, pin in result.items()), "complete paired/source benchmark asset identity differs")
    return result


def load_model(weights, metadata, *, weights_sha256, metadata_sha256):
    require(_scope is not None and Path(weights) == _scope.weights and Path(metadata) == _scope.metadata
        and weights_sha256 == _scope.pins["weights_sha256"] and metadata_sha256 == _scope.pins["metadata_sha256"],
        "exact predeclared intrinsic pair endpoint required")
    old.threads.configure_threads(); plan_sha = _scope.numeric._owner_plan_sha
    binding = dict(schema=loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=MODULE, arm=_scope.arm,
        purpose="owned_carla_research_only" if plan_sha is not None else "offline_cpu_numeric_validation_only",
        completed_pair_authority_sha256=_scope.authority_sha, fixed_endpoint_updates=6000,
        simulator_control_authorized=plan_sha is not None, owner_plan_sha256=plan_sha,
        simulation_only=True, model_promoted=False, vehicle_control_approved=False, **_scope.pins)
    model = loader.load_intrinsic_curve_research_npz(weights, metadata, weights_sha256=weights_sha256, metadata_sha256=metadata_sha256,
        current_parent_weights=CURRENT_PARENT/"model_state.npz", current_parent_metadata=CURRENT_PARENT/"metadata.json",
        warm_parent_weights=numeric.WARM_PARENT/"model_state.npz", warm_parent_metadata=numeric.WARM_PARENT/"metadata.json",
        backbone_parent_weights=numeric.BACKBONE_PARENT/"model_state.npz", backbone_parent_metadata=numeric.BACKBONE_PARENT/"metadata.json",
        completed_pair_authority=_scope.authority_path, completed_pair_authority_sha256=_scope.authority_sha,
        research_owner_binding=binding, source_root=ROOT)
    model._research_cpu_thread_numerical_revision = live.THREAD_SCHEMA
    model._research_speed_preview_calibration = live.PREVIEW_SCHEMA
    old.threads.live.thread_readback()
    return model


def fixed_contract(run_id, longitudinal_pi):
    result = _parent_fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Fixed control then intrinsic geometry pair on the original Town03 known-DEV right turn",
        arm=_scope.arm, paired_arm_order=list(ARMS), both_pair_endpoints_completed_before_declaration=True,
        completed_pair_authority=str(_scope.authority_path), completed_pair_authority_sha256=_scope.authority_sha,
        optimizer_updates=6000, additional_optimizer_updates=6000, known_total_optimizer_updates=18200,
        parent_optimizer_updates=6000, parent_known_total_optimizer_updates=12200,
        training_schema=loader.TRAINING_SCHEMA, new_numeric_export_schema=loader.EXPORT_SCHEMA,
        actual_current_parent_four_sha256=dict(loader.PARENT), parent_actual6000_history_sha256=loader.PARENT_HISTORY_SHA,
        parent_numeric_export_sha256=dict(weights=loader.PARENT["weights_sha256"], metadata=loader.PARENT["metadata_sha256"]),
        warm_ancestor_numeric_export_sha256=dict(weights=numeric.PARENT_NPZ_SHA, metadata=numeric.PARENT_METADATA_SHA),
        training_source_count=29, original_export_false_release_flags_preserved=True,
        current_fixed_final6000_joint_weights_unchanged=False, current_fixed_intrinsic_pair_endpoint_unchanged=True,
        new_inference_loader_is_separate_research_authority=True, model_or_head_capability_claimed=False,
        original_thread_benchmark_applies_to_parent_not_new_pair=True,
        all_copied_mutable_capture_providers_current_before_startup=True,
        actual_original_source_JPEG_enqueue_and_ACK_recorder_identity_verified=True,
        post_ACK_recording_excluded_from_bundle_delivery_to_ACK_metric=True,
        sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
        all_prior_failed_Town03_attempts_preserved=True, no_automatic_promotion=True)
    result["event"]["schema"] = live.SCHEMA+".event"
    result["worker_command"].extend(["--arm", _scope.arm, "--completed-pair-authority", str(_scope.authority_path),
        "--completed-pair-authority-sha256", _scope.authority_sha])
    return result


def complete_interval(manifest, states, cameras, event):
    # HH_261002 - Source-authenticate the manifest's ORIGINAL typed error; retain complete failure, never turn success.
    live.verify_frozen()
    exact = "BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail"
    capture = next(node for node in ast.parse(Path(live.collector.__file__).read_bytes()).body
        if isinstance(node, ast.FunctionDef) and node.name == "collect_episode")
    require(sum(isinstance(node, ast.Raise) and isinstance(node.exc, ast.Call)
        and isinstance(node.exc.func, ast.Name) and node.exc.func.id == "CollectionError"
        and len(node.exc.args) == 1 and isinstance(node.exc.args[0], ast.Constant) and node.exc.args[0].value == exact
        for node in ast.walk(capture)) == 1,
            "exact frozen collector budget error source required")
    checked = dict(manifest)
    if checked.get("status") == "failed" and checked.get("error") != "CollectionError: "+exact:
        return False
    if checked.get("error") == "CollectionError: "+exact: checked["error"] = exact
    return parent.complete_interval(checked, states, cameras, event)


def environment():
    result = _base_environment()
    if _batch_sha is not None: result["HH_INTRINSIC_PAIR_BATCH_SHA256"] = _batch_sha
    return result


@contextmanager
def activated(path, sha, arm):
    global _scope, _active
    require(not _active and _scope is None and arm in ARMS, "exclusive fixed intrinsic arm scope required")
    require(digest(Path(parent.__file__)) == PARENT_OWNER_SHA, "frozen actual V3 owner changed")
    require(live.previous.numeric is numeric, "unknown frozen Green-fixture numeric owner binding")
    source, value, run = authority(path, sha); pins = value["endpoints"][arm]
    export = run/arm/"exports/step_006000"
    adapter = SimpleNamespace(**vars(numeric))
    adapter.WEIGHTS, adapter.METADATA, adapter.MODULE = export/"model_state.npz", export/"metadata.json", MODULE
    adapter.verify_exports, adapter.runtime_sources, adapter.load_model = verify_exports, runtime_sources, load_model
    adapter._owner_plan_sha, adapter._exports = None, dict(pins)
    _scope = SimpleNamespace(authority_path=source, authority_sha=sha, authority=value, run=run, arm=arm,
        pins=pins, weights=adapter.WEIGHTS, metadata=adapter.METADATA, numeric=adapter)
    targets = (old, parent.previous, parent)
    saved_numeric = {target: target.numeric for target in targets}
    thread_provider = SimpleNamespace(**vars(old.threads)); thread_provider.evidence_assets = evidence_assets
    saved_threads, saved_fixture_numeric = old.threads, live.previous.numeric; _active = True
    try:
        for target in targets: target.numeric = adapter
        # HH_261002 - The frozen Green fixture and private worker must authenticate the SAME scoped owner and endpoint.
        live.previous.numeric = adapter
        old.threads = thread_provider
        with parent.activated(pins):
            changes = dict(paths=paths, fixed_contract=fixed_contract, runtime_sources=runtime_sources, load_npz=load_model,
                live=live, MODULE=MODULE, SCHEMA=SCHEMA, environment=environment)
            saved = {target: {name: getattr(target, name) for name in changes if hasattr(target, name)} for target in (*targets, base)}
            previous_complete = old.complete_interval
            try:
                for target in (*targets, base):
                    for name in saved[target]: setattr(target, name, changes[name])
                old.complete_interval = complete_interval
                yield
            finally:
                old.complete_interval = previous_complete
                for target in reversed((*targets, base)):
                    for name, original in saved[target].items(): setattr(target, name, original)
    finally:
        live.previous.numeric = saved_fixture_numeric
        old.threads = saved_threads
        for target in targets: target.numeric = saved_numeric[target]
        _scope, _active = None, False


def verify_batch(expected):
    require(loader.is_sha(expected) and digest(batch_path()) == expected, "externally pinned complete paired physical declaration required")
    value = read_json(batch_path())
    require(value.get("schema") == SCHEMA+".batch" and value.get("arm_order") == list(ARMS)
        and value.get("completed_pair_authority_sha256") == _scope.authority_sha
        and set(value.get("case_sha256", {})) == set(ARMS), "both arms must be declared before any actual capture")
    for arm, pin in value["case_sha256"].items():
        require(loader.is_sha(pin) and digest(batch_path().parent/(arm+"_plan.json")) == pin, "one prospective arm plan changed")
    return value


def main(argv=None):
    global _batch_sha
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "declare", "preflight", "execute", "worker"), default="contract")
    parser.add_argument("--run-id", choices=("v1",), default="v1")
    parser.add_argument("--arm", choices=ARMS)
    parser.add_argument("--completed-pair-authority", type=Path, required=True)
    parser.add_argument("--completed-pair-authority-sha256", required=True)
    parser.add_argument("--batch-sha256"); parser.add_argument("--plan-sha256")
    parser.add_argument("--longitudinal-pi", action="store_true")
    for name in ("weights", "metadata", "checkpoint", "declaration"): parser.add_argument("--expected-"+name+"-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        require(args.arm is None and not batch_path().exists(), "declare BOTH fixed arms once, before any capture")
        plans = {}
        for arm in ARMS:
            with activated(args.completed_pair_authority, args.completed_pair_authority_sha256, arm):
                plan = base.build_plan("v1", True); base.disk_check(); plans[arm] = (paths("v1")[1], plan)
        for path, plan in plans.values(): path.parent.mkdir(parents=True, exist_ok=True); write_new(path, plan)
        write_new(batch_path(), dict(schema=SCHEMA+".batch", arm_order=list(ARMS),
            completed_pair_authority_sha256=args.completed_pair_authority_sha256,
            case_sha256={arm: digest(path) for arm, (path, _) in plans.items()}, simulator_started=False))
        print(json.dumps(dict(batch_declaration=str(batch_path()), batch_sha256=digest(batch_path()), simulator_started=False)))
        return 0
    require(args.arm in ARMS, "explicit predeclared arm required")
    with activated(args.completed_pair_authority, args.completed_pair_authority_sha256, args.arm):
        supplied = {name+"_sha256": getattr(args, "expected_"+name+"_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
        require(all(pin is None for pin in supplied.values()) or supplied == _scope.pins, "worker/caller endpoint override differs")
        if args.mode == "contract": print(json.dumps(fixed_contract("v1", True), indent=2)); return 0
        _batch_sha = args.batch_sha256 or (os.environ.get("HH_INTRINSIC_PAIR_BATCH_SHA256") if args.mode == "worker" else None)
        try:
            batch = verify_batch(_batch_sha)
            require(args.plan_sha256 == batch["case_sha256"][args.arm], "externally pinned selected case differs from fixed batch")
            if args.arm == "intrinsic_curve" and args.mode in ("execute", "worker"):
                previous_result = read_json(batch_path().parent/"control/owner_result.json")
                require(previous_result.get("declaration_sha256") == batch["case_sha256"]["control"]
                    and previous_result.get("cleanup_errors") == [] and previous_result.get("source_hashes_match") is True
                    and previous_result.get("external_source_hashes_match") is True,
                    "control attempt must finish/clean first regardless of its driving outcome")
            if args.mode == "preflight": print(json.dumps(base.preflight("v1", args.plan_sha256)[1], indent=2)); return 0
            if args.mode == "execute": return base.execute("v1", args.plan_sha256)
            return base.worker("v1", args.plan_sha256)
        finally: _batch_sha = None


if __name__ == "__main__": raise SystemExit(main())
