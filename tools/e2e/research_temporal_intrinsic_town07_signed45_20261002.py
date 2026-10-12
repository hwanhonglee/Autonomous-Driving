"""HH_261002 - Predeclare BOTH fixed pair endpoints times ALL three signed45 profiles; no automatic selection or promotion."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

from . import research_temporal_intrinsic_town03_right_20261002 as paired
from . import research_temporal_route_joint_signed45_town07_20261002 as numeric
from . import research_temporal_route_joint_preview05_signed45_town07_20261002 as readback
from . import collect_temporal_intrinsic_town07_signed45_20261002 as live

ROOT, base, protocol, signed = paired.ROOT, numeric.base, numeric.parent, numeric.signed
loader = paired.loader
require, digest, read_json, write_new = base.require, base.digest, base.read_json, base.write_new
MODULE = "scripts.e2e.research_temporal_intrinsic_town07_signed45_20261002"
SCHEMA = "portable_e2e.owned_intrinsic_pair_town07_signed45.20261002.v1"
ARMS = paired.ARMS
PAIRED_SHA = "3e95420e29ad4bf5ba46b9cd680a7fa81979a94a7dd1309ae3cbbdb60677ed9c"
_fixed, _diagnostic, _summary = numeric.fixed_contract, numeric.diagnostic_report, numeric.batch_summary
_environment = paired.old.threads.environment
_scope, _active, _pair_sha = None, False, None


def pair_root():
    return ROOT/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town07_signed45_v1"


def pair_path(): return pair_root()/"paired_batch_plan.json"


def batch_paths(run_id):
    require(run_id == "v1" and _scope is not None, "fixed prospective paired arm required")
    output = pair_root()/_scope.arm
    return output, pair_root()/(_scope.arm+"_batch_plan.json")


def runtime_sources():
    pins = paired.runtime_sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in pins or pins[name] == sha, "new protect import closure conflicts")
        pins[name] = sha
    return pins


def load_model(weights, metadata, **pins):
    # HH_261002 - Same completed-pair loader/current-parent law; the scoped owner ID is THIS Town07 owner, not Town03.
    return paired.load_model(weights, metadata, **pins)


def fixed_contract(run_id, longitudinal_pi):
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Protect straight/signed recovery after fixed intrinsic geometry continuation",
        arm=_scope.arm, paired_arm_order=list(ARMS), prospective_physical_case_count=6,
        both_pair_endpoints_completed_before_declaration=True, completed_pair_authority=str(_scope.authority_path),
        completed_pair_authority_sha256=_scope.authority_sha, additional_optimizer_updates=6000,
        known_total_optimizer_updates=18200, parent_optimizer_updates=6000, parent_known_total_optimizer_updates=12200,
        training_schema=loader.TRAINING_SCHEMA, new_numeric_export_schema=loader.EXPORT_SCHEMA,
        actual_current_parent_four_sha256=dict(loader.PARENT), parent_actual6000_history_sha256=loader.PARENT_HISTORY_SHA,
        parent_numeric_export_sha256=dict(weights=loader.PARENT["weights_sha256"], metadata=loader.PARENT["metadata_sha256"]),
        training_source_count=29, current_fixed_final6000_joint_weights_unchanged=False,
        current_fixed_intrinsic_pair_endpoint_unchanged=True, original_export_false_release_flags_preserved=True,
        cpu_thread_numerical_preset=paired.old.threads.numerical_preset(),
        causal_selected_speed_preview_calibration=readback.calibration_contract(),
        original_thread_benchmark_applies_to_parent_not_new_pair=True,
        teacher_startup_and_spawn_calibration_unchanged=True, actual_same_original_signed45_profiles=True,
        additional_full_driving_actor_world_XY_speed_rate_guard_mps2=dict(minimum=-6., maximum=3.),
        all_actual_camera_bootstrap_ACK_and_JPEG_module_providers_verified_before_capture=True,
        sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
        full_MAIN9_completion_claimed=False, full_RTE01_approved=False, model_promoted=False, vehicle_control_approved=False)
    result["worker_command"].extend(["--arm", _scope.arm, "--completed-pair-authority", str(_scope.authority_path),
        "--completed-pair-authority-sha256", _scope.authority_sha])
    return result


def disk_check(free=None):
    value = shutil.disk_usage(ROOT).free if free is None else free
    asset_bytes = sum(path.stat().st_size for path in _scope.assets)
    required = base.FLOOR+6*base.ARM_RESERVE+base.SHARED_RESERVE+6*asset_bytes
    require(type(value) is int and value >= required, "full SIX protected episodes/archives reserve unavailable")
    return dict(free_bytes=value, required_bytes=required, prospective_trials=6,
        episode_quota_bytes=base.EPISODE_QUOTA, sampled_exact_input_quota_bytes=base.SAMPLE_QUOTA,
        six_trial_complete_source_asset_archive_reserve_bytes=6*asset_bytes)


def environment():
    value = _environment()
    if _pair_sha is not None: value["HH_INTRINSIC_PROTECT_PAIR_SHA256"] = _pair_sha
    return value


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    try:
        configuration = readback.configuration_report(episode)
        threads = paired.old.threads.readback_report(episode)
        acceleration = readback.proto.acceleration_report(episode)
        manifest = read_json(episode/"actual_recording_binding.json")
        require(manifest["schema"] == live.SCHEMA+".actual_recording_binding"
            and manifest["original_capture_live_module_globals"] is True and manifest["all_actual_providers_current"] is True
            and manifest["copied_mutable_provider_names"] == list(live.NAMES)
            and manifest["enqueue_and_ACK_share_exact_recorder"] is True
            and manifest["actual_recording_bundle_count"] == manifest["actual_after_ACK_receipt_count"] == 450
            and manifest["pending_source_camera_count"] == 0, "complete actual source/ACK/JPEG binding proof required")
        protocol.verify_derived_source_closure(output, plan)
        passed = configuration["passed"] and threads["passed"] and acceleration["passed"]
        result.update(actual_preview_configuration=configuration, actual_thread_settings=threads,
            native_acceleration_diagnostic=acceleration, actual_recording_binding=manifest)
    except Exception as error:
        passed = False; result["intrinsic_protect_diagnostic_error"] = type(error).__name__+": "+str(error)
    result.update(schema=SCHEMA+".diagnostic_report", arm=plan["arm"], known_total_optimizer_updates=18200,
        exposure_to_ACK_wall_latency_proven=False, formal_MAIN9_completion_claimed=False)
    result["learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed")) and passed
    result["joint_motion_body_policy_wall100ms_passed"] = result["learned_driving_screen_passed"] \
        and result.get("predeclared_observed_bundle_to_ACK_wall_latency_screen_passed") is True
    result["status"] = "PASS_BOUNDED_INTRINSIC_PROTECT" if result["joint_motion_body_policy_wall100ms_passed"] else "FAIL_BOUNDED_INTRINSIC_PROTECT"
    return result


@contextmanager
def activated(path, sha, arm):
    global _scope, _active
    require(not _active and _scope is None and paired._scope is None and not paired._active
        and not numeric._active and arm in ARMS, "exclusive new intrinsic protect scope required")
    require(digest(Path(paired.__file__)) == PAIRED_SHA, "frozen completed-pair owner helper changed")
    source, value, run = paired.authority(path, sha); pins = value["endpoints"][arm]
    export = run/arm/"exports/step_006000"
    _scope = SimpleNamespace(authority_path=source, authority_sha=sha, authority=value, run=run,
        arm=arm, pins=pins, weights=export/"model_state.npz", metadata=export/"metadata.json", numeric=numeric)
    old_paired = paired._scope, paired.MODULE
    numeric_values = dict(WEIGHTS=_scope.weights, METADATA=_scope.metadata, MODULE=MODULE, SCHEMA=SCHEMA,
        live=live, verify_exports=paired.verify_exports, runtime_sources=runtime_sources,
        load_model=load_model, fixed_contract=fixed_contract, batch_paths=batch_paths, diagnostic_report=diagnostic_report)
    signed_values = dict(live=live, batch_paths=batch_paths, MODULE=MODULE, SCHEMA=SCHEMA)
    saved_numeric = {k:getattr(numeric,k) for k in numeric_values}
    saved_signed = {k:getattr(signed,k) for k in signed_values}
    old_environment, old_disk = base.environment, protocol.disk_check
    _active = True
    try:
        paired._scope, paired.MODULE = _scope, MODULE
        _scope.assets = paired.evidence_assets(pins)
        base.environment, protocol.disk_check = environment, disk_check
        for name, val in numeric_values.items(): setattr(numeric,name,val)
        for name, val in signed_values.items(): setattr(signed,name,val)
        assets = base.ASSETS
        base.ASSETS = dict(assets)|_scope.assets
        try:
            with numeric.activated(pins): yield
        finally: base.ASSETS = assets
    finally:
        for name,val in saved_signed.items(): setattr(signed,name,val)
        for name,val in saved_numeric.items(): setattr(numeric,name,val)
        base.environment, protocol.disk_check = old_environment, old_disk
        paired._scope, paired.MODULE = old_paired
        _scope, _active = None, False


def verify_pair(expected):
    require(loader.is_sha(expected) and digest(pair_path()) == expected, "external full SIX-case declaration required")
    value = read_json(pair_path())
    require(value.get("schema") == SCHEMA+".paired_batch" and value.get("arm_order") == list(ARMS)
        and value.get("physical_case_count") == 6 and value.get("trial_count_per_arm") == 3
        and value.get("completed_pair_authority_sha256") == _scope.authority_sha
        and value.get("automatic_retry") is False and value.get("outcome_selected_retry") is False
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "both fixed three-profile protect plans required before any capture")
    require(set(value.get("arm_batch_sha256", {})) == set(ARMS), "all SIX prospective cases required")
    for arm,pin in value["arm_batch_sha256"].items():
        batch_path = pair_root()/(arm+"_batch_plan.json")
        require(loader.is_sha(pin) and digest(batch_path) == pin,
                "one complete arm batch declaration changed")
        batch = read_json(batch_path)
        require(batch.get("schema") == SCHEMA+".batch" and batch.get("prospective_trial_count") == 3
            and [case.get("trial_index") for case in batch.get("trials", [])] == [0,1,2],
            "all three ordered cases of BOTH fixed arms required")
        for index,case in enumerate(batch["trials"]):
            path = pair_root()/(arm+"_trial_%02d_plan.json"%index)
            require(case.get("declaration_path") == str(path) and path.resolve() == path and not path.is_symlink()
                and loader.is_sha(case.get("declaration_sha256")) and digest(path) == case["declaration_sha256"],
                "one of SIX case plans changed or escaped its fixed arm path")
    return value


def main(argv=None):
    global _pair_sha
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract","declare","preflight","execute","worker","summary"), default="contract")
    parser.add_argument("--run-id", choices=("v1",), default="v1")
    parser.add_argument("--arm", choices=ARMS)
    parser.add_argument("--trial", type=int, choices=(0,1,2), default=0)
    parser.add_argument("--completed-pair-authority", type=Path, required=True)
    parser.add_argument("--completed-pair-authority-sha256", required=True)
    parser.add_argument("--paired-batch-sha256"); parser.add_argument("--plan-sha256"); parser.add_argument("--trial-plan-sha256")
    for name in ("weights","metadata","checkpoint","declaration"): parser.add_argument("--expected-"+name+"-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        require(args.arm is None and not pair_root().exists(), "all SIX fixed cases must be fresh and predeclared together")
        results = {}
        for arm in ARMS:
            with activated(args.completed_pair_authority,args.completed_pair_authority_sha256,arm):
                results[arm] = protocol.declare_batch("v1")
        write_new(pair_path(), dict(schema=SCHEMA+".paired_batch", arm_order=list(ARMS), physical_case_count=6,
            trial_count_per_arm=3, completed_pair_authority_sha256=args.completed_pair_authority_sha256,
            arm_batch_sha256={arm:r["batch_declaration_sha256"] for arm,r in results.items()}, simulator_started=False,
            automatic_retry=False, outcome_selected_retry=False, model_promoted=False, vehicle_control_approved=False))
        print(dict(paired_batch_path=str(pair_path()), paired_batch_sha256=digest(pair_path()), simulator_started=False)); return 0
    require(args.arm in ARMS, "explicit fixed arm required")
    with activated(args.completed_pair_authority,args.completed_pair_authority_sha256,args.arm):
        supplied = {name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
        require(all(v is None for v in supplied.values()) or supplied == _scope.pins, "caller/worker cannot override selected fixed endpoint")
        if args.mode == "contract": return protocol.main(["--mode","contract","--run-id","v1","--trial",str(args.trial)])
        _pair_sha = args.paired_batch_sha256 or (os.environ.get("HH_INTRINSIC_PROTECT_PAIR_SHA256") if args.mode=="worker" else None)
        try:
            value = verify_pair(_pair_sha); batch_pin = value["arm_batch_sha256"][args.arm]
            require(args.plan_sha256 is None or args.plan_sha256 == batch_pin, "selected arm batch differs")
            batch = protocol.verify_batch("v1", batch_pin)
            if args.mode == "worker":
                require(args.trial_plan_sha256 == batch["trials"][args.trial]["declaration_sha256"], "private case outside six fixed declarations")
            if args.arm == "intrinsic_curve" and args.mode in ("execute","worker"):
                for index in range(3):
                    prior = read_json(pair_root()/"control"/("trial_%02d"%index)/"owner_result.json")
                    control_batch = read_json(pair_root()/"control_batch_plan.json")
                    require(prior["declaration_sha256"] == control_batch["trials"][index]["declaration_sha256"]
                        and prior["cleanup_errors"] == [] and prior["source_hashes_match"] is True
                        and prior["external_source_hashes_match"] is True, "all three control attempts must finish cleanly first, failures retained")
            command=["--mode",args.mode,"--run-id","v1","--trial",str(args.trial),"--plan-sha256",batch_pin]
            if args.trial_plan_sha256: command.extend(["--trial-plan-sha256",args.trial_plan_sha256])
            return protocol.main(command)
        finally: _pair_sha = None


if __name__ == "__main__": raise SystemExit(main())
