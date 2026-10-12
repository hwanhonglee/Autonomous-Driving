"""HH_261004 - Own one new known-DEV callback45 attempt, keeping all prior evidence and original safety gates."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import stat

from . import collect_temporal_route_joint_town03_straight_callback45_v3_20261004 as live
from . import research_temporal_route_joint_town03_straight_v3_20261004 as previous


base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
require, digest, read_json = previous.require, previous.digest, previous.read_json
ROUTE, XODR, UMAP = previous.ROUTE, previous.XODR, previous.UMAP
XODR_SHA, UMAP_SHA = previous.XODR_SHA, previous.UMAP_SHA
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_callback45_v3_20261004"
SCHEMA = live.OWNER_SCHEMA
RUN_ID = "callback45_v3"
PARENT_OWNER_SHA = "26e5740c8e6194a8a1872573d33c1f657954556aec3362c74cf36cc50881bd0c"
V3_OUTPUT = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3"
V3_EVIDENCE_SHA = {
    "declaration.json": "4c189966e4666d99aa8d81c6470f19170e18433e39b67a0435f5433a613a2a6b",
    "diagnostic_report.json": "3cdf704449bb0f1fc14ed8a07e1a7cb50fab2b39a53e4d6fa27a55354ee073a0",
    "owner_result.json": "d3cbe0251639c210785e6628cf8483a69a807c9f0b208f492796b3e22e1d5677",
    "episode.partial/manifest.json": "c53231a5e1f791f41c819808ea411e33a945c229db09121bb50dd4f752703dfe",
}
TEST_SOURCE = ROOT / "learning/tests/test_temporal_route_joint_town03_straight_callback45_v3_20261004.py"
CALLBACK_JSON_MAX_BYTES = 16 * 1024**2
CALLBACK_V1_OUTPUT = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v1"
CALLBACK_V1_EVIDENCE_SHA = {
    "declaration.json": "c8ea57d1eb9471611b2b9d3902cbfb7a9ae6c3ede71d30d4e7e3b567e505be71",
    "diagnostic_report.json": "26f072b2a69884ba5c5f275ab7243786bd8eb3ac586f63b6950a3ba9cae8a638",
    "owner_result.json": "3c312a3b5daab501b0d1b0c5321a3e05a71091d91dec58e2ef03e725de6f8830",
    "episode.partial/manifest.json": "67821aebaf04d7427ffe1db2ebe47c2fd8769c6c634a1b8603346e5b7799fb57",
    "episode.partial/callback_timing_receipts.json": "8d84a4b39a4fe1f3a06ee7870530abcc534c8cb50cae654f8ff3aa01a3c0b28b",
}
CALLBACK_V1_SOURCE_SHA = {
    "tools/e2e/collect_temporal_route_joint_town03_straight_callback45_v1_20261004.py": "5e01c6aa1bdc23079128d8674c1d794ea179e1fe64b4ecffffc12b9f61e58e39",
    "tools/e2e/research_temporal_route_joint_town03_straight_callback45_v1_20261004.py": "6633369a2a3017ef893c39bdd6f1ed7496b16a841fbb7eaf65f893f247633926",
    "learning/tests/test_temporal_route_joint_town03_straight_callback45_v1_20261004.py": "cb7b577034c32a03f3a5f8fddf680fa369c4074147f23c36582db527a699bbb7",
}
CALLBACK_V2_OUTPUT = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v2"
CALLBACK_V2_EVIDENCE_SHA = {
    "declaration.json": "5794c055f01fc342fd049b1e20c4b8026d2c3d0d6e1f88a521fa30196a4043fb",
    "diagnostic_report.json": "3f8561aca288960113a64f3429bc3381aace87a7bef68fdc850fabb5848c384b",
    "owner_result.json": "798612158de7f90b4b111a3d933a237cd6020f4d030df150cd153c8e1fff151f",
    "episode.partial/manifest.json": "0be981c4a40964fac04020e2b2972d5a6ba4fedc58c1541dea92f52fa25e2ca1",
    "episode.partial/callback_timing_receipts.json": "ea38029aef488350a67f9cbc5487af9a4d772cf126a83147be00403a6ee16fa2",
}
CALLBACK_V2_SOURCE_SHA = {
    "tools/e2e/collect_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "5c645accae929efd2e004a5e2cc417e9eb5d8be59b8f6743ab4d9c8b92b9df07",
    "tools/e2e/research_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "5981f7f2a65523887dae1a708d0562cfbe2d464ba8afc2c92f9b601cd767d88c",
    "learning/tests/test_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "3d358ef3b40a4f3c4cd887f6c237bcd8984a06c0461afd763749222d655c44c6",
}
_parent_fixed, _parent_sources, _parent_diagnostic = previous.fixed_contract, previous.runtime_sources, previous.diagnostic_report
_active = False


def read_callback_receipts(path, expected_sha256, *, schema=None):
    """HH_261004 - Read only the callback ABI under a16MiB/count/schema/SHA/no-symlink bound; generic JSON stays unchanged."""
    path = Path(path).absolute()
    expected_schema = live.SCHEMA + ".callback_receipts" if schema is None else schema
    require(path.name == "callback_timing_receipts.json" and path.resolve() == path
        and isinstance(expected_sha256, str) and re.fullmatch(r"[0-9a-f]{64}", expected_sha256),
        "canonical callback receipt path and authenticated SHA required")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode) and 0 < before.st_size <= CALLBACK_JSON_MAX_BYTES,
                "callback-only bounded regular JSON required")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            payload = stream.read(CALLBACK_JSON_MAX_BYTES + 1)
        after = os.fstat(descriptor)
        current = os.stat(path, follow_symlinks=False)
        require(len(payload) == before.st_size <= CALLBACK_JSON_MAX_BYTES
            and (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)
                == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
            and (current.st_dev, current.st_ino, current.st_size, current.st_mtime_ns, current.st_ctime_ns)
                == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
            and path.resolve() == path and hashlib.sha256(payload).hexdigest() == expected_sha256,
            "callback receipt path/bytes changed or SHA differs")
    finally:
        os.close(descriptor)
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate callback JSON key")
            result[key] = value
        return result
    value = json.loads(payload, object_pairs_hook=unique_object,
        parse_constant=lambda token: require(False, "nonfinite callback JSON number: " + token))
    require(isinstance(value, dict) and value.get("schema") == expected_schema
        and value.get("clock") == "time.perf_counter_ns_monotonic"
        and type(value.get("callback_count")) is int
        and 0 <= value["callback_count"] <= 12000,
        "exact callback ABI schema/clock/count required")
    for name, maximum in (("callbacks", 12000), ("camera_bindings", 6), ("bundles", 450),
                          ("policy_stage_marks", 1350), ("errors", 12000)):
        require(isinstance(value.get(name), list) and len(value[name]) <= maximum,
                "bounded callback ABI array required: " + name)
    if "native_stage_marks" in value:
        require(isinstance(value["native_stage_marks"], list) and len(value["native_stage_marks"]) <= 8100
            and all(isinstance(row, dict) for row in value["native_stage_marks"]),
            "bounded native-stage scalar ABI array required")
    require(len(value["callbacks"]) == value["callback_count"] and len(value["camera_bindings"]) == 6
        and len(value["policy_stage_marks"]) % 3 == 0
        and all(isinstance(row, dict) for name in
                ("callbacks", "camera_bindings", "bundles", "policy_stage_marks") for row in value[name])
        and all(isinstance(row.get("cameras"), list) and len(row["cameras"]) == 6
                and all(isinstance(camera, dict) for camera in row["cameras"]) for row in value["bundles"]),
        "callback receipt count/row/six-camera ABI differs")
    return value


def _verify_callback_v1_failure():
    """HH_261004 - Preserve the actual failed callback v1 and its frozen sources before every fresh revision."""
    require(all(digest(CALLBACK_V1_OUTPUT / name) == sha for name, sha in CALLBACK_V1_EVIDENCE_SHA.items())
        and all(digest(ROOT / name) == sha for name, sha in CALLBACK_V1_SOURCE_SHA.items()),
        "archived callback45_v1 failed evidence or source changed")
    result = read_json(CALLBACK_V1_OUTPUT / "diagnostic_report.json")
    require(result.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and result.get("actual_native_state_count") == 900 and result.get("actual_camera_bundle_count") == 450
        and result.get("actual_ACK_receipt_count") == 904 and result.get("full_RTE01_approved") is False
        and result.get("diagnostic_error") == "RuntimeError: accepted fresh proposal lacks exact selected-knot preview witness"
        and result.get("callback_timing_diagnostic_error") == "RuntimeError: bounded JSON required",
        "archived callback45_v1 failure denominator or original errors differ")
    return result


def _verify_callback_v2_failure():
    """HH_261004 - Keep the complete v2 FAIL, not regrade it after correcting the identity-join assumption."""
    require(all(digest(CALLBACK_V2_OUTPUT / name) == sha for name, sha in CALLBACK_V2_EVIDENCE_SHA.items())
        and all(digest(ROOT / name) == sha for name, sha in CALLBACK_V2_SOURCE_SHA.items()),
        "archived callback45_v2 failed evidence or source changed")
    result = read_json(CALLBACK_V2_OUTPUT / "diagnostic_report.json")
    require(result.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and result.get("actual_native_state_count") == 900 and result.get("actual_camera_bundle_count") == 450
        and result.get("actual_ACK_receipt_count") == 904 and result.get("full_RTE01_approved") is False
        and result.get("inherited_learned_driving_screen_passed") is True
        and result.get("straight_motion_body_policy_wall100ms_passed") is True
        and result.get("callback_timing_diagnostic_error") ==
            "CollectionError: duplicate, incomplete or malformed actual callback receipt",
        "archived callback45_v2 failure denominator or original errors differ")
    return result


def paths(run_id):
    require(run_id == RUN_ID, "new callback45 owner requires exact fresh callback45_v3")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v3"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    """HH_261004 - Reuse the original fixed45 CLI through fresh scoped paths, bypassing frozen revision-ID wrappers."""
    require(_active and run_id == RUN_ID, "fresh callback45 CLI requires its private owner scope")
    return previous.previous._parent_capture(run_id, longitudinal_pi, declaration_sha)


def _verify_v3_evidence():
    """HH_261004 - Keep the actual successful bounded v3 and its original failed-budget manifest immutable."""
    require(V3_OUTPUT.is_dir() and not V3_OUTPUT.is_symlink()
        and all(digest(V3_OUTPUT / name) == sha for name, sha in V3_EVIDENCE_SHA.items()),
        "archived v3 bounded precursor evidence changed")
    plan = read_json(V3_OUTPUT / "declaration.json")
    result = read_json(V3_OUTPUT / "diagnostic_report.json")
    require(plan["run_id"] == "v3" and plan["town"] == "Town03" and plan["scenario"] == "straight"
        and plan["original_navigation_sha256"] == live.ROUTE_SHA
        and all(digest(ROOT / path) == sha for path, sha in plan["source_sha256"].items())
        and all(digest(path) == sha for path, sha in plan["external_source_sha256"].items())
        and result["status"] == "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and result["actual_native_state_count"] == 900 and result["actual_camera_bundle_count"] == 450
        and result["actual_ACK_receipt_count"] == 904 and result["full_RTE01_approved"] is False,
        "archived v3 closure or original limited denominator differs")
    return result


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Add prospective callback-to-plan/ACK measurement gates; never relax original motion or safety."""
    _verify_v3_evidence()
    _verify_callback_v1_failure()
    _verify_callback_v2_failure()
    result = _parent_fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, output=str(paths(run_id)[0]), declaration_path=str(paths(run_id)[1]),
        experiment="One new fixed45 known-DEV straight with actual first camera callbacks and decoded-plan timing",
        archived_v3_reference=dict(path=str(V3_OUTPUT), sha256=dict(V3_EVIDENCE_SHA),
            status="PASS_KNOWN_DEV_STRAIGHT_PRECURSOR", overwritten=False, official_function_approval=False),
        failed_callback45_v1_reference=dict(path=str(CALLBACK_V1_OUTPUT), sha256=dict(CALLBACK_V1_EVIDENCE_SHA),
            source_sha256=dict(CALLBACK_V1_SOURCE_SHA), status="FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR", overwritten=False),
        failed_callback45_v2_reference=dict(path=str(CALLBACK_V2_OUTPUT), sha256=dict(CALLBACK_V2_EVIDENCE_SHA),
            source_sha256=dict(CALLBACK_V2_SOURCE_SHA), status="FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR", overwritten=False),
        actual_callback_timing=dict(schema=live.SCHEMA + ".callback_receipts", attempts=1,
            repeated_role_frame_or_simstamp_callbacks_preserved=True,
            repeated_native_frame_or_simstamp_alone_is_image_identity=False,
            selected_source_join_fields=["role", "native_frame", "image_object_id",
                "source_image_timestamp_simulation_ns"],
            selected_source_exactly_one_original_callback_required=True,
            ambiguous_same_role_native_frame_object_receipts_denied=True,
            source_actor_and_BGRA_SHA_required=True,
            causal_history_joins_consumed_bundle_identity_and_SHA_only=True,
            earliest_or_latest_duplicate_receipt_selected_by_latency=False,
            repeated_callbacks_with_different_pixels_permitted_only_as_distinct_unmodified_sources=True,
            native_dispatch_timing=dict(stages=list(live.NATIVE_STAGES), native_ticks_required=900,
                native_camera_ticks_required=450, native_scalar_marks_required=7200,
                first_tick_frame_unknown_until_actual_world_tick_returns=True,
                physics_control_RPC_or_source_order_changed=False, scalar_clocks_only_no_waits=True,
                external_CPU_load_or_hardware_exposure_causation_claimed=False),
            callback_receipt_JSON_maximum_bytes=CALLBACK_JSON_MAX_BYTES,
            callback_receipt_JSON_SHA_and_schema_and_array_bounds_required=True,
            generic_JSON_reader_limit_changed=False, original_same_recorder_bundle_provider_replaced=False,
            timed_policy_globals_bound_after_complete_preview_PI_activation=True,
            latest_camera_bundles_required=450, latest_camera_callback_receipts_required=2700,
            all_extra_bootstrap_or_unconsumed_callbacks_retained=True,
            source_BGRA_SHA_format="SHA256(ASCII bgra8 prefix + exact original BGRA bytes)",
            original_queue_put_argument_and_return_and_exception_preserved=True,
            additional_queue_or_source_image_buffer=False, input_control_or_inference_order_changed=False,
            actual_base_policy_observation_statement_count=3, source_math_inverse_AST_required=True,
            first_callback_to_plan_maximum_seconds=.1, first_callback_to_ACK_maximum_seconds=.1,
            callback_to_policy_source_SHA_required=True, every_causal_history_SHA_required=True,
            fresh_and_held_and_teacher_and_prefill_separate=True,
            simulated_camera_cadence_hz=10, hardware_exposure_wall_clock_observed=False,
            ten_hz_wall_rate_proven=False, wall_interval_reported_without_cadence_certification=True,
            callback_SHA_cost_after_original_queue_put=True, callback_receipt_file_IO_during_capture=False,
            cleanup_only_receipt_drain_maximum_seconds=5.0,
            model_structure_changed=False, model_weights_changed=False, controller_math_changed=False,
            decoder_gate_or_tracker_math_changed=False, original_deadline_relaxed=False),
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    require(result["maximum_native_state_count"] == 900 and result["maximum_camera_bundle_count"] == 450
        and result["policy_deadline_seconds"] == .1 and result["body_clearance_screen"]["lane_inset_m"] == .1
        and result["speed_preview_s"] == .5 and result["controller_target_cap_mps"] == 2.4
        and result["worker_command"][result["worker_command"].index("-m") + 1] == MODULE,
        "original fixed45 driving gates or actual fresh worker changed")
    return result


def runtime_sources():
    """HH_261004 - Seal all three new files with the complete inherited source closure and unchanged prior evidence."""
    require(digest(Path(previous.__file__)) == PARENT_OWNER_SHA, "frozen v3 owner changed")
    _verify_v3_evidence()
    _verify_callback_v1_failure()
    _verify_callback_v2_failure()
    result = _parent_sources()
    live.verify_frozen()
    files = tuple(base.imported_source_closure((Path(__file__).resolve(), Path(live.__file__).resolve()))) + (TEST_SOURCE,)
    for path in files:
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "callback45 source closure conflict")
        result[name] = sha
    for name, sha in (CALLBACK_V1_SOURCE_SHA | CALLBACK_V2_SOURCE_SHA).items():
        require(name not in result or result[name] == sha, "frozen callback v1 source closure conflict")
        result[name] = sha
    return result


def _jsonl(path):
    require(path.is_file() and not path.is_symlink(), "actual journal required: " + str(path))
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def diagnostic_report(output, plan, owner):
    """HH_261004 - Keep the full inherited outcome and require genuine callback/SHA/plan/ACK evidence additionally."""
    result = _parent_diagnostic(output, plan, owner)
    # HH_261004 - Preserve the old measured bundle-to-ACK outcome separately from the additional callback screen.
    result["inherited_straight_motion_body_policy_wall100ms_passed"] = bool(
        result.get("straight_motion_body_policy_wall100ms_passed"))
    result["inherited_learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed"))
    result.update(schema=SCHEMA + ".diagnostic_report", archived_v3_evidence_preserved=False,
        archived_callback45_v1_failure_preserved=False,
        archived_callback45_v2_failure_preserved=False,
        native_pre_delivery_stage_closure_passed=False,
        callback_receipt_identity_and_SHA_passed=False, callback_plan_ACK_chain_passed=False,
        fresh_callback_to_plan_wall100ms_passed=False, fresh_callback_to_ACK_wall100ms_passed=False,
        actual_six_camera_callback_observed=False, hardware_exposure_to_ACK_proven=False,
        ten_hz_wall_rate_proven=False)
    try:
        _verify_v3_evidence()
        _verify_callback_v1_failure()
        _verify_callback_v2_failure()
        result["archived_v3_evidence_preserved"] = True
        result["archived_callback45_v1_failure_preserved"] = True
        result["archived_callback45_v2_failure_preserved"] = True
        episode = output / "episode" if (output / "episode").is_dir() else output / "episode.partial"
        callback_path = episode / "callback_timing_receipts.json"
        expected_sha = result.get("raw_source_sha256", {}).get(str(callback_path))
        value = read_callback_receipts(callback_path, expected_sha)
        cameras, commands, timings = (_jsonl(episode / name) for name in
            ("camera_frames.jsonl", "learned_control.jsonl", "wall_timing.jsonl"))
        counts = read_json(episode / "learned_summary.json")["counts"]
        audit = live.audit_receipts(value, cameras, commands, timings, counts)
        result["actual_callback_timing"] = audit
        # HH_261004 - A later native-stage closure failure must not hide already authenticated callback timing results.
        for key in ("callback_receipt_identity_and_SHA_passed", "callback_plan_ACK_chain_passed",
                    "fresh_callback_to_plan_wall100ms_passed", "fresh_callback_to_ACK_wall100ms_passed",
                    "actual_six_camera_callback_observed"):
            result[key] = audit[key]
        states = _jsonl(episode / "states.jsonl")
        native = live.audit_native_markers(value, states, cameras, timings, audit)
        result["native_pre_delivery_timing"] = native
        result["native_pre_delivery_stage_closure_passed"] = native["native_pre_delivery_stage_closure_passed"]
        result["callback_receipt_JSON_readback"] = dict(path=str(callback_path), sha256=expected_sha,
            actual_bytes=callback_path.stat().st_size, maximum_bytes=CALLBACK_JSON_MAX_BYTES,
            strict_schema_and_array_bounds_passed=True, generic_JSON_reader_limit_changed=False)
        require(all(digest(path) == sha for path, sha in result.get("raw_source_sha256", {}).items()),
                "original callback timing evidence changed during audit")
    except Exception as error:
        result["callback_timing_diagnostic_error"] = type(error).__name__ + ": " + str(error)
    passed = all(result.get(key) is True for key in ("archived_v3_evidence_preserved", "archived_callback45_v1_failure_preserved",
        "archived_callback45_v2_failure_preserved",
        "native_pre_delivery_stage_closure_passed",
        "callback_receipt_identity_and_SHA_passed", "callback_plan_ACK_chain_passed",
        "fresh_callback_to_plan_wall100ms_passed", "fresh_callback_to_ACK_wall100ms_passed"))
    result["callback_straight_motion_body_policy_wall100ms_passed"] = bool(
        result.get("straight_motion_body_policy_wall100ms_passed")) and passed
    result["learned_driving_screen_passed"] = result["inherited_learned_driving_screen_passed"] and passed
    result["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if result["learned_driving_screen_passed"] \
        else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    result.update(full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Propagate only the new private owner delegates through the unchanged v3 lock/PID/fixture lifecycle."""
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_OWNER_SHA,
            "new callback45 owner cannot nest or use a changed v3 owner")
    _verify_v3_evidence()
    _verify_callback_v1_failure()
    _verify_callback_v2_failure()
    values = dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, diagnostic_report=diagnostic_report, live=live, MODULE=MODULE, SCHEMA=SCHEMA)
    saved = {name: getattr(previous, name) for name in values}
    _active = True
    try:
        for name, value in values.items():
            setattr(previous, name, value)
        with previous.activated(pins):
            yield
    finally:
        for name, value in reversed(tuple(saved.items())):
            setattr(previous, name, value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
            for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
