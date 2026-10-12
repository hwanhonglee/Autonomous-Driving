"""HH_261005 - Prepare a fresh CPU6/masked-copy known-DEV full209 owner; no deployment, training or TEST approval claim.

Reproduce only after root source review and a fresh declaration:
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.research_temporal_route_joint_town03_straight_full209_v5_20261005 -- \
 --expected-weights-sha256 <GOOD6000 weights> --expected-metadata-sha256 <metadata> \
 --expected-checkpoint-sha256 <checkpoint> --expected-declaration-sha256 <TRAIN declaration> \
 --mode declare --run-id v5 --longitudinal-pi
Use the same command with --mode preflight/execute and --plan-sha256 <new plan>.
"""

import argparse
import copy
from contextlib import contextmanager
from pathlib import Path
from types import FunctionType

import torch

from . import research_temporal_route_joint_town03_straight_full209_v4_20261004 as previous
from . import research_temporal_route_joint_town03_right_20261002 as right
from . import collect_temporal_route_joint_town03_straight_full209_v5_20261005 as live


base, ROOT, numeric = previous.base, previous.ROOT, right.numeric
require, digest = previous.require, previous.digest
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_full209_v5_20261005"
SCHEMA = live.OWNER_SCHEMA
PARENT_SHA256 = "d156ae91a5e9516b1e3a19cf8899290f15e5c6dd419b9f148351ac19604b4403"
EVIDENCE_PINS = {
    "storage/experiments/diagnostics/2026-10-04/full209_masked_copy_JPEG_counterfactual_v1/ROOT_complete_independent_postcheck.json":
        "562c56c742795c0589552c860dca752c132f95ed6b810c1248af7906bac7f796",
    "storage/experiments/diagnostics/2026-10-04/full209_thread468_JPEG_counterfactual_v2/ROOT_complete_independent_postcheck.json":
        "d470f11cff0a6e338db607d2cd668354bc9cb7374ac363f63cad639ca8097fb7",
}
NATIVE_EVIDENCE_PINS = {
    "storage/experiments/plans/2026-10-05/full209_native_quota_v2/declaration.json":
        "27fbb4e77fb08d6d1f75d08d4b20febf5b12dabb996b5060205b99015760e4f3",
    "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/summary.json":
        "f1ceb6c6e673d9c1a1e7f6287ce8407474f918edfb9288819b20e9419f9dd2d3",
    "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/ROOT_complete_independent_postcheck.json":
        "84fc338af45bf1833fb273f9d647ee13988dfce1d56d08572f34fcc757735dd6",
}
_fixed, _sources, _strict_load = previous.fixed_contract, previous.runtime_sources, numeric.load_model
_right_loader = right.load_model
_original_capture = previous.previous.previous.previous._original_capture
_diagnostic = FunctionType(previous.diagnostic_report.__code__,
    {**previous.diagnostic_report.__globals__, "live": live, "SCHEMA": SCHEMA},
    previous.diagnostic_report.__name__, previous.diagnostic_report.__defaults__, previous.diagnostic_report.__closure__)
_diagnostic.__kwdefaults__ = dict(previous.diagnostic_report.__kwdefaults__ or {})
_active = False


def paths(run_id):
    require(run_id == "v5", "fresh Oct5 full209 V5 requires exact v5 identity")
    output = ROOT / "storage/experiments/validation/2026-10-05/temporal_route_joint_town03_known_dev_straight_full209_v5"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    require(_active and run_id == "v5", "fresh capture requires its exclusive V5 owner scope")
    result = _original_capture(run_id, longitudinal_pi, declaration_sha)
    index = result.index("--max-duration-sec") + 1
    require(result[index] == "45", "original fullroute duration seam changed")
    result[index] = "110"
    return result


def evidence():
    # HH_261005 - Pin complete prior numerical proof without regrading prefill rejections or claiming authentic terminal/BGRA parity.
    for name, expected in EVIDENCE_PINS.items():
        require(digest(ROOT / name) == expected, "whole offline masked-copy/thread proof changed")
    masked, thread = [base.read_json(ROOT / name) for name in EVIDENCE_PINS]
    require(masked["status"] == "PASS_COMPLETE_INDEPENDENT_POSTCHECK" and masked["contexts"] == 888
        and masked["all_frozen_originals_unchanged"] is True and masked["original_actual_FAIL_unchanged"] is True,
        "complete original masked-copy proof required")
    require(thread["all_contexts"] == 2664 and thread["all_timed_calls"] == 90
        and thread["original_parent_exit_code"] == 2 and thread["original_prefill_without_decoder_rejections"] == 15
        and thread["terminal_comparisons_unverified"] == 189
        and thread["input_sha_and_cache_work_and_window_unchanged_verified"] is True
        and thread["numeric_tolerance"] == dict(atol=1e-6, rtol=1e-6)
        and [row["threads"] for row in thread["arm_evidence"]] == [4, 6, 8]
        and all(row["contexts"] == 888 and row["all_original_802_fresh_ordinary_screens_passed"] is True
            and set(row["maximum_raw_errors"]) == {"curvature_knots", "preferred_speed_knots", "intent_logits",
                "stop_distance_raw", "acceleration_prediction"}
            and all(value == 0 for value in row["maximum_raw_errors"].values()) for row in thread["arm_evidence"]),
        "complete strict raw numerical proof and retained original failures required")
    return dict(EVIDENCE_PINS)


def fixed_contract(run_id, longitudinal_pi):
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Fresh known-DEV full209 V5 explicit masked-copy GOOD6000 and fixed CPU6 research",
        frozen_V4_preserved=True, cpu_thread_numerical_preset=dict(schema=live.THREAD_SCHEMA,
            intra_op_threads=6, inter_op_threads=1, original_strict_loader_constructor_global_preserved=True,
            fixed_after_strict_original_load_and_all101_state_copy=True,
            benchmark_OPENBLAS_NUM_THREADS="1", actual_OPENBLAS_NUM_THREADS="6",
            BLAS_thread_environment_intervention_explicitly_declared=True,
            per_call_threadpool_retuning=False, cross_thread_bitwise_equivalence_claimed=False,
            lossy_JPEG_raw5_atol_rtol_1e_6_verified=True, original_prefill_rejections15_retained=True,
            original_terminal189_comparisons_unverified=True, actual_sensor_latency_gain_proven=False),
        CPU_thread_environment=dict(OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS="6"),
        actual_masked_copy_candidate=dict(source=str(live.CANDIDATE_SOURCE.relative_to(ROOT)),
            source_sha256=live.CANDIDATE_SHA256, exact_type="AllValidMaskedCopyRouteRecoveryTemporalModelV3",
            original_factory_constructor_global_preserved=True, all101_state_tensors_and_owner_metadata_copied=True,
            trusted_readonly_all_valid_cached_path_only=True, all_original_finite_hash_cache_and_head_hook_guards_retained=True,
            no_unpinned_upgrader_or_model_class_alias=True, original_weights_unchanged=True),
        complete_offline_evidence_sha256=evidence(),
        model_controller_sensor_timing_and_start_offset_unchanged=False,
        actual_runtime_activation_not_offline_parity_is_required=True, actual_closed_loop_gain_not_yet_proven=True)
    for name, sha in NATIVE_EVIDENCE_PINS.items():
        require(digest(ROOT / name) == sha, "complete native quota comparator evidence changed")
    native = base.read_json(ROOT / "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/summary.json")
    require(native["status"] == "COMPLETED_ALL60_EQUAL" and native["no_membership_cache_or_guard_omission"] is True
        and native["timings"]["old"]["count"] == native["timings"]["native"]["count"] == 30
        and native["failed_tail_cropped"] is False, "all fixed native quota calls and original failures required")
    result["complete_tree_quota_counter_intervention"].update(
        implementation="explicit pinned native fd-relative full entry census, no incremental cache",
        source="tools/e2e/full_tree_quota_native_v1_20261005.py", C_source="tools/e2e/full_tree_quota_native_v1_20261005.c",
        helper_sha256=live.NATIVE_HELPER_SHA256, C_source_sha256=live.NATIVE_C_SHA256,
        library=str(live.NATIVE_LIBRARY), library_sha256=live.NATIVE_LIBRARY_SHA256,
        same_original_walk_before_exact_bundle=True, no_follow_pinned_library_descriptor_required=True,
        complete_entry_directory_regular_special_counts_and_clock_proof_required=True,
        comparator_evidence_sha256=dict(NATIVE_EVIDENCE_PINS), warmed_final_tree_is_not_live_latency=True)
    result["synchronous_parallel_JPEG_after_ACK"]["controller_NN_weights_math_and_CPU_threads_unchanged"] = False
    result["synchronous_parallel_JPEG_after_ACK"]["controller_NN_weights_and_JPEG_source_guards_unchanged"] = True
    result["prospective_visual_publication"]["gallery"] = "docs/assets/validation/2026-10-05/portable_e2e_learning_cycle_v1/01_town03_full_route209_v5"
    require(result["policy_deadline_seconds"] == .1 and result["actual_callback_timing"]["callback_to_ACK_maximum_s"] == .1
        and result["model_speed_cap_mps"] == 2.5 and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["controller_target_cap_mps"] == 2.4 and result["speed_preview_s"] == .5
        and result["body_clearance_screen"]["lane_inset_m"] == .1 and result["total_capture_seconds"] == 110
        and result["original_catalog_goal_tolerance_m"] == 2.5
        and result["worker_command"][result["worker_command"].index("-m") + 1] == MODULE,
        "fresh V5 changed original physical/latency/goal bounds or actual worker identity")
    return result


def runtime_sources():
    require(digest(Path(previous.__file__)) == PARENT_SHA256, "frozen full209 V4 owner changed")
    result = _sources()
    live.verify_frozen()
    seeds = (Path(__file__).resolve(), Path(live.__file__).resolve(), live.CANDIDATE_SOURCE, live.CANDIDATE_TEST,
        ROOT / "learning/tests/test_town03_full209_collector_v5_20261005.py",
        ROOT / "learning/tests/test_town03_full209_owner_v5_20261005.py", Path(live.native_quota.__file__).resolve(),
        live.native_quota.SOURCE, ROOT / "learning/tests/test_full_tree_quota_native_v1_20261005.py",
        ROOT / "tools/e2e/benchmark_full209_native_quota_v2_20261005.py",
        ROOT / "tools/e2e/check_native_quota_benchmark_v2_20261005.py",
        ROOT / "learning/tests/test_benchmark_full209_native_quota_v2_20261005.py",
        ROOT / "learning/tests/test_benchmark_full209_native_quota_v1_20261005.py")
    require(all(path.resolve() == path and path.is_file() and not path.is_symlink() for path in seeds), "fresh V5 source/test closure not prepared")
    for path in base.imported_source_closure(seeds):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "fresh V5 source closure conflict")
        result[name] = sha
    for name, sha in (evidence() | NATIVE_EVIDENCE_PINS).items():
        require(digest(ROOT / name) == sha, "complete native quota evidence changed before source closure")
        require(name not in result or result[name] == sha, "whole offline evidence closure conflict")
        result[name] = sha
    # HH_261005 - Preserve every workspace source in the sealed13-item comparator closure; GCC is provenance only, never an actual runtime compiler.
    native_plan = base.read_json(ROOT / "storage/experiments/plans/2026-10-05/full209_native_quota_v2/declaration.json")
    require(len(native_plan["sources"]) == 13, "complete original native comparator source denominator required")
    for name, sha in native_plan["sources"].items():
        path = Path(name)
        if not path.is_relative_to(ROOT):
            require(name == "/usr/bin/x86_64-linux-gnu-gcc-11"
                and sha == "821af3c74506283c179ca413bb33e6b528805a4dd8a5c09df125e5ad560a9e89",
                "unexpected external native compiler provenance")
            continue
        require(path.is_absolute() and path.resolve() == path and path.is_relative_to(ROOT)
            and path.is_file() and not path.is_symlink() and digest(path) == sha,
            "original sealed native comparator source changed")
        relative = str(path.relative_to(ROOT))
        require(relative not in result or result[relative] == sha, "original native source archive pin conflicts")
        result[relative] = sha
    postcheck = base.read_json(ROOT / "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/ROOT_complete_independent_postcheck.json")
    require(result["tools/e2e/check_native_quota_benchmark_v2_20261005.py"] == postcheck["checker_sha256"],
        "native whole-counterfactual checker source seal changed")
    return result


def load_model(weights, metadata, **pins):
    """HH_261005 - Keep the genuine strict original constructor, then construct the explicit candidate and copy every state/owner field."""
    live.verify_frozen()
    constructor_globals = numeric.load_route_joint_research_npz.__globals__
    require(constructor_globals["RouteRecoveryTemporalModelV3"] is live.RouteRecoveryTemporalModelV3,
        "original numeric factory constructor global changed")
    if torch.get_num_interop_threads() != 1:
        torch.set_num_interop_threads(1)
    torch.set_num_threads(4)
    control = _strict_load(weights, metadata, **pins)
    require(type(control) is live.RouteRecoveryTemporalModelV3 and len(control.state_dict()) == 101
        and torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1,
        "strict original GOOD6000 factory or explicit baseline changed")
    model = live.AllValidMaskedCopyRouteRecoveryTemporalModelV3(control.config, control.recovery_config)
    model.load_state_dict({name: value.detach().clone() for name, value in control.state_dict().items()}, strict=True)
    model.requires_grad_(False).eval()
    for name in ("_research_temporal_checkpoint_sha256", "_research_temporal_export_npz_sha256", "_research_temporal_metadata_sha256",
                 "_research_temporal_export_metadata", "_research_recovery_owner_binding", "_research_recovery_numeric_loader_schema",
                 "_research_recovery_simulator_control_authorized"):
        setattr(model, name, copy.deepcopy(getattr(control, name)))
    require(all(torch.equal(value, model.state_dict()[name]) for name, value in control.state_dict().items())
        and constructor_globals["RouteRecoveryTemporalModelV3"] is live.RouteRecoveryTemporalModelV3,
        "all101 original weights and genuine constructor global must remain equal")
    torch.set_num_threads(6)
    model._research_cpu_thread_numerical_revision = live.THREAD_SCHEMA
    model._research_speed_preview_calibration = live.PREVIEW_SCHEMA
    model._research_masked_copy_actual_binding = dict(source_sha256=live.CANDIDATE_SHA256,
        original_constructor_global_preserved=True, all101_original_state_tensors_equal=True,
        cpu_intraop_threads=6, cpu_interop_threads=1)
    live.validate_candidate(model)
    return model


def environment():
    result = right._original_environment()
    result.update(OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS="6")
    return result


def readback_report(episode):
    records, timing = [right.jsonl(episode / name) for name in ("cpu_thread_readback.jsonl", "wall_timing.jsonl")]
    expected = [(row["command_frame"], row["command_reason"]) for row in timing
        if row.get("command_reason") in ("initial_drive_control", "next_driving_control") and row.get("command_frame") is not None]
    actual = [(row["command_frame"], row["command_reason"]) for row in records]
    passed = bool(expected) and actual == expected and len(actual) == len(set(actual)) and all(
        row.get("schema") == live.THREAD_SCHEMA and row.get("intra_op_threads") == 6
        and row.get("inter_op_threads") == 1 and row.get("before_original_propose") is True for row in records)
    return dict(status="PASS_COMPLETE_V5_CPU6_READBACK" if passed else "FAIL_V5_CPU6_READBACK", passed=passed,
        normal_proposal_denominator=len(expected), readback_count=len(actual),
        observed_settings_before_prefill_handoff_and_hold=True,
        source_sha256={str(episode / name): digest(episode / name) for name in ("cpu_thread_readback.jsonl", "wall_timing.jsonl")})


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    inherited = result.get("learned_driving_screen_passed") is True
    result.update(schema=SCHEMA + ".diagnostic_report", fresh_owner_revision="v5", original_V4_preserved=True,
        actual_masked_copy_CPU6_declared=True, actual_closed_loop_gain_claimed=False,
        inherited_V4_full209_screen_passed_before_V5_conjunction=inherited,
        actual_V5_native_quota_audit_passed=False, actual_V5_CPU6_readback_passed=False,
        actual_V5_candidate_and_native_capture_binding_passed=False)
    try:
        # HH_261005 - Explicitly consume actual V5 proofs here; a frozen inner diagnostic's V4 delegate cannot certify new helpers.
        episode = output / "episode" if (output / "episode").is_dir() else output / "episode.partial"
        names = ("quota_walk_timing_receipts.json", "camera_frames.jsonl", "wall_timing.jsonl",
                 "cpu_thread_readback.jsonl", "manifest.json")
        raw = result["raw_source_sha256"]
        require(all(str(episode / name) in raw and digest(episode / name) == raw[str(episode / name)] for name in names),
            "all original V5 audit input seals required before reading")
        cameras, timings = [right.jsonl(episode / name) for name in ("camera_frames.jsonl", "wall_timing.jsonl")]
        quota = live.audit_quota_walks(base.read_json(episode / "quota_walk_timing_receipts.json"), cameras, timings)
        result["actual_V5_native_quota_audit"] = quota
        require(quota["passed"] is True, "every actual V5 native quota row must independently pass")
        result["actual_V5_native_quota_audit_passed"] = True
        thread = readback_report(episode)
        result["actual_V5_CPU6_readback"] = thread
        require(thread["passed"] is True, "complete actual V5 CPU6 proposal readback required")
        result["actual_V5_CPU6_readback_passed"] = True
        capture = base.read_json(episode / "manifest.json")["capture_contract"]
        candidate, native = capture["actual_masked_copy_CPU6"], capture["actual_native_full_tree_quota"]
        require(candidate["candidate_type"] == live.AllValidMaskedCopyRouteRecoveryTemporalModelV3.__module__
                + "." + live.AllValidMaskedCopyRouteRecoveryTemporalModelV3.__name__
            and candidate["source_sha256"] == live.CANDIDATE_SHA256 and candidate["thread_schema"] == live.THREAD_SCHEMA
            and candidate["intra_op_threads"] == 6 and candidate["inter_op_threads"] == 1
            and all(candidate.get(name) is True for name in ("strict_original_constructor_global_preserved",
                "all101_original_state_tensors_equal", "original_finite_hash_cache_decoder_and_permanent_head_guards_retained",
                "complete_tree_quota_semantics_unchanged", "original100ms_thresholds_unchanged"))
            and candidate["model_promoted"] is False and candidate["actual_sensor_latency_gain_proven"] is False,
            "actual exact-class/state-copy/CPU6 capture binding required, not a declared unused helper")
        require(native["schema"] == live.native_quota.SCHEMA and native["helper_sha256"] == live.NATIVE_HELPER_SHA256
            and native["C_source_sha256"] == live.NATIVE_C_SHA256 and native["library"] == str(live.NATIVE_LIBRARY)
            and native["library_sha256"] == live.NATIVE_LIBRARY_SHA256 and native["incremental_cache_used"] is False
            and all(native.get(name) is True for name in ("every_entry_every_call", "original_quota_free_floor_reserves_unchanged",
                "original_walk_before_exact_camera_bundle_unchanged", "unrelated_paths_original_delegate")),
            "actual sealed native engine and unchanged full census lifecycle binding required")
        right.verify_closure(output, plan)
        require(all(digest(episode / name) == raw[str(episode / name)] for name in names), "V5 audit input changed during independent checks")
        live.verify_native_artifacts()
        result["actual_V5_candidate_and_native_capture_binding_passed"] = True
    except Exception as error:
        result["actual_V5_binding_or_native_diagnostic_error"] = type(error).__name__ + ": " + str(error)
    passed = inherited and all(result[name] is True for name in (
        "actual_V5_native_quota_audit_passed", "actual_V5_CPU6_readback_passed", "actual_V5_candidate_and_native_capture_binding_passed"))
    result.update(learned_driving_screen_passed=passed, full209_research_motion_body_goal_callback100ms_passed=passed,
        status="PASS_KNOWN_DEV_FULL209_RESEARCH" if passed else "FAIL_KNOWN_DEV_FULL209_RESEARCH")
    return result


@contextmanager
def activated(pins):
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_SHA256 and right.load_model is _right_loader,
        "fresh V5 cannot nest or replace an unknown original model factory")
    changes = {previous: dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, diagnostic_report=diagnostic_report, live=live, MODULE=MODULE, SCHEMA=SCHEMA),
        right: dict(load_model=load_model, environment=environment), right.threads: dict(readback_report=readback_report)}
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    _active = True
    try:
        for module, values in changes.items():
            for name, value in values.items():
                setattr(module, name, value)
        with previous.activated(pins):
            # HH_261005 - Archive the exact private native artifact and reserve its bytes through the unchanged original disk guard.
            old_assets, old_benchmark = base.ASSETS, right._benchmark_assets
            live.verify_native_artifacts()
            base.ASSETS = dict(old_assets) | {live.NATIVE_LIBRARY: live.NATIVE_LIBRARY_SHA256}
            right._benchmark_assets = dict(old_benchmark) | {live.NATIVE_LIBRARY: live.NATIVE_LIBRARY_SHA256}
            try:
                yield
            finally:
                base.ASSETS, right._benchmark_assets = old_assets, old_benchmark
                live.verify_native_artifacts()
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items():
                setattr(module, name, value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
