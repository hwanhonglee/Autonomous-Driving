"""HH_261005 - Verify fresh V5 ownership, exact source/model factory and complete CPU6 readback, never actual driving."""

from contextlib import contextmanager
import copy
import json
from unittest import mock

import pytest
import torch

from scripts.e2e import research_temporal_route_joint_town03_straight_full209_v5_20261005 as m
from test_temporal_route_joint_town03_straight_v3_20261004 import pins
from test_town03_full209_collector_v5_20261005 import candidates


@pytest.fixture(scope="module", autouse=True)
def one_synthetic_thread():
    original = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(original)


@contextmanager
def scope():
    with mock.patch.object(m.numeric, "verify_exports"), mock.patch.object(m.right.threads, "evidence_assets", return_value={}):
        with m.activated(pins()):
            yield


def test_fresh_contract_preserves_all_original_driving_storage_and_failed_interval_guards():
    before = (m.previous.paths, m.previous.live, m.right.load_model, m.right.environment, m.right.threads.readback_report)
    with scope():
        plan, argv = m.fixed_contract("v5", True), m.capture_argv("v5", True, "a" * 64)
        assert m.base.load_npz is m.load_model and m.base.environment is m.environment
        assert m.base.live is m.live and m.base.paths("v5") == m.paths("v5")
        assert m.base.ASSETS[m.live.NATIVE_LIBRARY] == m.live.NATIVE_LIBRARY_SHA256
        with mock.patch.object(m, "_sources", return_value={}):
            closure = m.runtime_sources()
    assert before == (m.previous.paths, m.previous.live, m.right.load_model, m.right.environment, m.right.threads.readback_report)
    assert not m._active and not m.previous._active
    assert plan == json.loads(json.dumps(plan, allow_nan=False))
    assert plan["schema"] == m.SCHEMA and "/2026-10-05/" in plan["output"]
    assert plan["run_id"] == "v5" and plan["worker_command"][plan["worker_command"].index("-m") + 1] == m.MODULE
    assert argv[argv.index("--max-duration-sec") + 1] == "110" and argv[argv.index("--port") + 1] == "2440"
    assert plan["model_speed_cap_mps"] == 2.5 and plan["native_actor_planar_speed_maximum_mps"] == 2.7
    assert plan["controller_target_cap_mps"] == 2.4 and plan["speed_preview_s"] == .5
    assert plan["body_clearance_screen"]["lane_inset_m"] == .1 and plan["original_catalog_goal_tolerance_m"] == 2.5
    assert plan["policy_deadline_seconds"] == plan["actual_callback_timing"]["callback_to_plan_maximum_s"] == .1
    assert plan["actual_callback_timing"]["callback_to_ACK_maximum_s"] == .1
    assert plan["cpu_thread_numerical_preset"]["intra_op_threads"] == 6
    assert plan["cpu_thread_numerical_preset"]["inter_op_threads"] == 1
    assert plan["cpu_thread_numerical_preset"]["benchmark_OPENBLAS_NUM_THREADS"] == "1"
    assert plan["cpu_thread_numerical_preset"]["actual_OPENBLAS_NUM_THREADS"] == "6"
    assert plan["cpu_thread_numerical_preset"]["BLAS_thread_environment_intervention_explicitly_declared"] is True
    assert plan["actual_masked_copy_candidate"]["no_unpinned_upgrader_or_model_class_alias"] is True
    assert plan["complete_tree_quota_counter_intervention"]["no_JPEG_payload_read"] is True
    assert plan["complete_tree_quota_counter_intervention"]["every_camera_walk_and_failed_attempt_retained"] is True
    assert plan["complete_tree_quota_counter_intervention"]["same_original_walk_before_exact_bundle"] is True
    assert plan["complete_tree_quota_counter_intervention"]["library_sha256"] == m.live.NATIVE_LIBRARY_SHA256
    assert plan["synchronous_parallel_JPEG_after_ACK"]["schema"] == m.live.jpeg.SCHEMA
    assert plan["two_phase_signal_restoration"]["expected_original_signal_count"] == 38
    assert plan["attempts"] == 1 and plan["automatic_retry"] is False
    assert plan["all_actual_rows_until_original_catalog_goal_or_original_safety_abort_required"] is True
    assert plan["prospective_visual_publication"]["complete_actual_interval_including_failures_required"] is True
    assert plan["official_completed_main_domains"] == plan["official_completed_subfunctions"] == 0
    assert not plan["model_promoted"] and not plan["independent_validation"]
    for path in (m.live.CANDIDATE_SOURCE, m.live.CANDIDATE_TEST):
        assert closure[str(path.relative_to(m.ROOT))] == m.digest(path)
    assert all(closure[name] == expected for name, expected in m.EVIDENCE_PINS.items())
    assert all(closure[name] == expected for name, expected in m.NATIVE_EVIDENCE_PINS.items())
    assert closure["tools/e2e/full_tree_quota_native_v1_20261005.c"] == m.live.NATIVE_C_SHA256
    assert closure["tools/e2e/full_tree_quota_native_v1_20261005.py"] == m.live.NATIVE_HELPER_SHA256


@pytest.mark.parametrize("run_id", ["v1", "v4", "v6", "../v5", None])
def test_wrong_revision_or_standalone_capture_rejects(run_id):
    with pytest.raises(RuntimeError): m.paths(run_id)


def test_nested_owner_and_inner_exception_restore_factory_identity():
    before = (m.right.load_model, m.right.environment, m.base.live, m.previous.live)
    marker = RuntimeError("synthetic V5 owner failure")
    with pytest.raises(RuntimeError) as caught:
        with scope():
            with pytest.raises(RuntimeError):
                with m.activated(pins()): pass
            raise marker
    assert caught.value is marker and before == (m.right.load_model, m.right.environment, m.base.live, m.previous.live)


def test_strict_original_factory_then_all101_clone_and_one_CPU6_target_preserve_constructor():
    control, _ = candidates()
    for name, value in dict(_research_temporal_checkpoint_sha256="c" * 64,
        _research_temporal_export_npz_sha256="w" * 64, _research_temporal_metadata_sha256="m" * 64,
        _research_temporal_export_metadata={"synthetic": True}, _research_recovery_numeric_loader_schema="synthetic").items():
        setattr(control, name, value)
    state, calls = dict(threads=1, interop=24), []
    constructor = m.numeric.load_route_joint_research_npz.__globals__["RouteRecoveryTemporalModelV3"]
    def set_threads(value):
        calls.append(("intraop", value)); state["threads"] = value
    def set_interop(value):
        calls.append(("interop", value)); state["interop"] = value
    def strict_load(weights, metadata, **kwargs):
        calls.append(("original_strict_loader", state["threads"]))
        assert state == dict(threads=4, interop=1)
        assert m.numeric.load_route_joint_research_npz.__globals__["RouteRecoveryTemporalModelV3"] is constructor
        return control
    with mock.patch.object(m, "_strict_load", strict_load), \
         mock.patch.object(torch, "set_num_threads", side_effect=set_threads), mock.patch.object(torch, "set_num_interop_threads", side_effect=set_interop), \
         mock.patch.object(torch, "get_num_threads", side_effect=lambda: state["threads"]), \
         mock.patch.object(torch, "get_num_interop_threads", side_effect=lambda: state["interop"]):
        candidate = m.load_model("synthetic.npz", "synthetic.json", weights_sha256="a" * 64, metadata_sha256="b" * 64)
    assert calls == [("interop", 1), ("intraop", 4), ("original_strict_loader", 4), ("intraop", 6)]
    assert type(candidate) is m.live.AllValidMaskedCopyRouteRecoveryTemporalModelV3
    assert all(torch.equal(value, candidate.state_dict()[name]) for name, value in control.state_dict().items())
    assert candidate._research_recovery_owner_binding == control._research_recovery_owner_binding
    assert candidate._research_recovery_owner_binding is not control._research_recovery_owner_binding
    assert candidate._research_temporal_export_metadata == control._research_temporal_export_metadata
    assert m.numeric.load_route_joint_research_npz.__globals__["RouteRecoveryTemporalModelV3"] is constructor is m.live.RouteRecoveryTemporalModelV3


@pytest.mark.parametrize("change", ["none", "CPU4", "interop", "missing", "duplicate", "tail"])
def test_complete_actual_readback_denominator_no_favorable_tail_drop(tmp_path, change):
    timing = [dict(command_frame=100 + index, command_reason="next_driving_control") for index in range(3)]
    rows = [dict(schema=m.live.THREAD_SCHEMA, **row, intra_op_threads=6, inter_op_threads=1, before_original_propose=True) for row in timing]
    if change == "CPU4": rows[-1]["intra_op_threads"] = 4
    elif change == "interop": rows[-1]["inter_op_threads"] = 2
    elif change == "missing": rows.pop()
    elif change == "duplicate": rows.append(copy.deepcopy(rows[-1]))
    elif change == "tail": rows[-1]["command_frame"] = 101
    for name, values in (("cpu_thread_readback.jsonl", rows), ("wall_timing.jsonl", timing)):
        (tmp_path / name).write_text("".join(json.dumps(row) + "\n" for row in values))
    result = m.readback_report(tmp_path)
    assert result["passed"] is (change == "none") and result["normal_proposal_denominator"] == 3


@pytest.mark.parametrize("passed", [False, True])
def test_V5_diagnostic_never_relabels_inherited_failure(passed):
    inherited = dict(learned_driving_screen_passed=passed, full209_research_motion_body_goal_callback100ms_passed=passed,
        status="PASS_KNOWN_DEV_FULL209_RESEARCH" if passed else "FAIL_KNOWN_DEV_FULL209_RESEARCH",
        retained_failure="synthetic original fail" if not passed else None)
    with mock.patch.object(m, "_diagnostic", return_value=copy.deepcopy(inherited)):
        result = m.diagnostic_report(None, None, None)
    assert result["learned_driving_screen_passed"] is False and result["status"] == "FAIL_KNOWN_DEV_FULL209_RESEARCH"
    assert result["inherited_V4_full209_screen_passed_before_V5_conjunction"] is passed
    assert result["retained_failure"] == inherited["retained_failure"]
    assert result["fresh_owner_revision"] == "v5" and result["actual_closed_loop_gain_claimed"] is False


def native_diagnostic_fixture(tmp_path):
    # HH_261005 - Tiny scalar files are synthetic verifier inputs, never actual cameras, BGRA, model outputs or approval evidence.
    episode = tmp_path / "episode.partial"
    episode.mkdir()
    live = m.live
    timing = [dict(command_frame=100, command_reason="next_driving_control",
        observed_bundle=dict(native_frame=100, exact_bundle_delivery_monotonic_ns=30))]
    row = dict(sequence=1, status="PASS", native_frame=100, timestamp_simulation_ns=5_000_000_000,
        capture_phase="driving", complete_walk=True, source_payload_read=False, cached_or_incremental_byte_count_used=False,
        entries=3, regular_files=2, directories_including_root=2, special_nodes_ignored_like_original=0,
        regular_file_bytes=12, returned_regular_file_bytes=12, walk_started_monotonic_ns=10, walk_finished_monotonic_ns=20,
        native_census_schema=live.native_quota.SCHEMA, native_census_source_sha256=live.NATIVE_C_SHA256,
        native_census_helper_sha256=live.NATIVE_HELPER_SHA256, native_census_library_sha256=live.NATIVE_LIBRARY_SHA256,
        native_census_library_path=str(live.NATIVE_LIBRARY), native_library_admitted_by_pinned_no_follow_descriptor=True,
        directory_generation_postcheck_required=True)
    quota = dict(schema=live.SCHEMA + ".quota_walk_receipts", clock="time.perf_counter_ns_monotonic", maximum_walk_count=1100,
        maximum_count_exceeded=False, every_attempt_including_failures_retained=True, complete_walk_every_call=True,
        incremental_cache_used=False, original_quota_free_floor_and_reserves_unchanged=True,
        actual_sensor_latency_gain_proven_by_walk_timing=False, actual_walk_count=1, rows=[row])
    candidate = dict(candidate_type=live.AllValidMaskedCopyRouteRecoveryTemporalModelV3.__module__ + "."
        + live.AllValidMaskedCopyRouteRecoveryTemporalModelV3.__name__, source_sha256=live.CANDIDATE_SHA256,
        thread_schema=live.THREAD_SCHEMA, intra_op_threads=6, inter_op_threads=1,
        strict_original_constructor_global_preserved=True, all101_original_state_tensors_equal=True,
        original_finite_hash_cache_decoder_and_permanent_head_guards_retained=True,
        complete_tree_quota_semantics_unchanged=True, original100ms_thresholds_unchanged=True,
        model_promoted=False, actual_sensor_latency_gain_proven=False)
    native = dict(schema=live.native_quota.SCHEMA, helper_sha256=live.NATIVE_HELPER_SHA256,
        C_source_sha256=live.NATIVE_C_SHA256, library=str(live.NATIVE_LIBRARY), library_sha256=live.NATIVE_LIBRARY_SHA256,
        incremental_cache_used=False, every_entry_every_call=True, original_quota_free_floor_reserves_unchanged=True,
        original_walk_before_exact_camera_bundle_unchanged=True, unrelated_paths_original_delegate=True)
    objects = {"quota_walk_timing_receipts.json": quota,
        "manifest.json": dict(capture_contract=dict(actual_masked_copy_CPU6=candidate, actual_native_full_tree_quota=native))}
    lines = {"camera_frames.jsonl": [dict(frame=100, timestamp=5., capture_phase="driving")],
        "wall_timing.jsonl": timing, "cpu_thread_readback.jsonl": [dict(schema=live.THREAD_SCHEMA, command_frame=100,
            command_reason="next_driving_control", intra_op_threads=6, inter_op_threads=1, before_original_propose=True)]}
    for name, value in objects.items(): (episode / name).write_text(json.dumps(value))
    for name, values in lines.items(): (episode / name).write_text("".join(json.dumps(row) + "\n" for row in values))
    return episode, objects, lines


@pytest.mark.parametrize("failure", ["none", "inherited", "quota_library", "quota_count", "quota_clock", "thread_CPU4",
    "candidate_type", "native_library", "closure", "raw_before", "raw_after"])
def test_actual_V5_diagnostic_explicitly_consumes_all_native_rows_readbacks_and_capture_binding(tmp_path, failure):
    episode, objects, lines = native_diagnostic_fixture(tmp_path)
    if failure == "quota_library": objects["quota_walk_timing_receipts.json"]["rows"][0]["native_census_library_sha256"] = "0" * 64
    elif failure == "quota_count": objects["quota_walk_timing_receipts.json"]["rows"][0]["entries"] = 2
    elif failure == "quota_clock": objects["quota_walk_timing_receipts.json"]["rows"][0]["walk_finished_monotonic_ns"] = 31
    elif failure == "thread_CPU4": lines["cpu_thread_readback.jsonl"][0]["intra_op_threads"] = 4
    elif failure == "candidate_type": objects["manifest.json"]["capture_contract"]["actual_masked_copy_CPU6"]["candidate_type"] = "unpinned.alias"
    elif failure == "native_library": objects["manifest.json"]["capture_contract"]["actual_native_full_tree_quota"]["library_sha256"] = "0" * 64
    for name, value in objects.items(): (episode / name).write_text(json.dumps(value))
    for name, values in lines.items(): (episode / name).write_text("".join(json.dumps(row) + "\n" for row in values))
    raw = {str(path): m.digest(path) for path in episode.iterdir()}
    if failure == "raw_before": raw[str(episode / "manifest.json")] = "0" * 64
    inherited = dict(learned_driving_screen_passed=failure != "inherited", raw_source_sha256=raw,
        retained_failure="synthetic original fail" if failure == "inherited" else None)
    def verify_closure(output, plan):
        if failure == "closure": raise RuntimeError("synthetic closure failure")
        if failure == "raw_after": (episode / "manifest.json").write_text("changed after native audit")
    with mock.patch.object(m, "_diagnostic", return_value=copy.deepcopy(inherited)), \
         mock.patch.object(m.right, "verify_closure", side_effect=verify_closure), \
         mock.patch.object(m.live, "audit_quota_walks", wraps=m.live.audit_quota_walks) as actual_native_audit:
        result = m.diagnostic_report(tmp_path, {}, {})
    assert actual_native_audit.call_count == (0 if failure == "raw_before" else 1)
    assert result["learned_driving_screen_passed"] is (failure == "none")
    assert result["full209_research_motion_body_goal_callback100ms_passed"] is (failure == "none")
    assert result["status"] == ("PASS_KNOWN_DEV_FULL209_RESEARCH" if failure == "none" else "FAIL_KNOWN_DEV_FULL209_RESEARCH")
    if failure in ("none", "inherited"):
        assert all(result[name] for name in ("actual_V5_native_quota_audit_passed", "actual_V5_CPU6_readback_passed",
            "actual_V5_candidate_and_native_capture_binding_passed"))
    else: assert result["actual_V5_binding_or_native_diagnostic_error"]
    if failure == "inherited": assert result["retained_failure"] == "synthetic original fail"
