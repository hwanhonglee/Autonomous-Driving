"""HH_261005 - Synthetic candidate admission and genuine inherited factory/recorder checks, without actual weights or CARLA."""

import inspect
import os
from types import SimpleNamespace
from unittest import mock

import pytest
import torch

from scripts.e2e import collect_temporal_route_joint_town03_straight_full209_v5_20261005 as m
from test_temporal_all_valid_masked_copy_research_v1_20261004 import models
from test_temporal_route_joint_town03_straight_v3_20261004 import fake_model_and_patches, pins


@pytest.fixture(scope="module", autouse=True)
def one_synthetic_thread():
    original = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(original)


def candidates():
    old, model = models()
    for value in (old, model):
        value.requires_grad_(False).eval()
        value._research_recovery_owner_binding = dict(purpose="owned_carla_research_only", simulation_only=True,
            owner_plan_sha256="a" * 64)
        value._research_recovery_simulator_control_authorized = True
        value._research_speed_preview_calibration = m.PREVIEW_SCHEMA
        value._research_cpu_thread_numerical_revision = m.THREAD_SCHEMA
        value._research_masked_copy_actual_binding = dict(source_sha256=m.CANDIDATE_SHA256,
            original_constructor_global_preserved=True, all101_original_state_tensors_equal=True,
            cpu_intraop_threads=6, cpu_interop_threads=1)
    return old, model


def test_frozen_parent_candidate_and_original_constructor_global_identity():
    m.verify_frozen()
    assert m.OWNER_SCHEMA.endswith("20261005.v5") and m.SCHEMA == m.previous.SCHEMA
    assert m.recovery.RouteRecoveryTemporalModelV3 is m.RouteRecoveryTemporalModelV3
    activation = m.candidate_activation_factory()
    function = activation.__wrapped__
    assert function.__globals__["RouteRecoveryTemporalModelV3"] is m.RouteRecoveryTemporalModelV3
    assert function.__globals__["AllValidMaskedCopyRouteRecoveryTemporalModelV3"] is m.AllValidMaskedCopyRouteRecoveryTemporalModelV3


@pytest.mark.parametrize("change", ["baseline", "subclass", "receipt", "zero_branch", "hook", "active_cache"])
def test_unknown_candidate_or_relaxed_guard_is_rejected(change):
    old, model = candidates()
    if change == "baseline": model = old
    elif change == "subclass":
        class Unpinned(m.AllValidMaskedCopyRouteRecoveryTemporalModelV3):
            pass
        model = Unpinned(model.config)
    elif change == "receipt": model._research_masked_copy_actual_binding["all101_original_state_tensors_equal"] = False
    elif change == "zero_branch": model.route_recovery_branch[-1].weight.data.zero_()
    elif change == "hook": model.curvature_head._forward_hooks.clear()
    else: model._recovery_active = True
    with mock.patch.object(torch, "get_num_threads", return_value=6), mock.patch.object(torch, "get_num_interop_threads", return_value=1):
        with pytest.raises(RuntimeError): m.validate_candidate(model)


def test_original_constructor_alias_is_rejected_not_blindly_admitted():
    with mock.patch.object(m.recovery, "RouteRecoveryTemporalModelV3", m.AllValidMaskedCopyRouteRecoveryTemporalModelV3):
        with pytest.raises(RuntimeError, match="constructor global was replaced"):
            m.verify_frozen()


@pytest.mark.parametrize("threads,interop", [(4, 1), (8, 1), (6, 2)])
def test_exact_CPU6_and_interop1_required(threads, interop):
    with mock.patch.object(torch, "get_num_threads", return_value=threads), mock.patch.object(torch, "get_num_interop_threads", return_value=interop):
        with pytest.raises(RuntimeError): m.thread_readback()


@pytest.mark.parametrize("failure", [False, True])
def test_real_inherited_V5_factory_recorder_fixture_and_candidate_admission_restore(tmp_path, failure):
    from scripts.e2e import research_temporal_route_joint_town03_straight_full209_v5_20261005 as owner
    original = m.previous.previous.previous
    _, model = candidates()
    _, patches = fake_model_and_patches()
    before = (m.recovery.activated, m.recovery.RouteRecoveryTemporalModelV3, m.threads.THREAD_SCHEMA,
        m.threads.thread_readback, original.collector.collect_episode, original.collector._save_jpeg,
        original.collector.AcknowledgedControlTransport, original.fixture.GreenSignalFixture, original.fixture.OWNER_SCHEMA)
    original_quota_counter = original.quota_counter.scandir_directory_bytes
    marker, recorder = RuntimeError("synthetic V5 factory failure before capture"), None
    def enter():
        nonlocal recorder
        # HH_261005 - Omit only numeric asset metadata/BasicAgent imports, never original constructor types or the actual collector chain.
        with mock.patch.object(owner.numeric, "verify_exports"), mock.patch.object(owner.right.threads, "evidence_assets", return_value={}), \
             patches[0], patches[5], mock.patch.object(torch, "get_num_threads", return_value=6), \
             mock.patch.object(torch, "get_num_interop_threads", return_value=1), owner.activated(pins()):
            args = m.frozen.parse_args(owner.capture_argv("v5", True, "a" * 64))
            args.output = tmp_path / "episode"
            with m.activated(model, args):
                recorder = m.jpeg.assert_runtime_recorder(original.collector)
                assert type(recorder) is m.jpeg.ParallelExactJPEGAfterACK
                assert original.fixture.GreenSignalFixture is m.signals.TwoPhaseGreenSignalFixture
                assert original.fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"] == m.OWNER_SCHEMA
                assert original.fixture.GreenSignalFixture.__bases__[0].__init__.__globals__["OWNER_SCHEMA"] == m.OWNER_SCHEMA
                assert m.recovery.RouteRecoveryTemporalModelV3 is m.RouteRecoveryTemporalModelV3
                assert m.recovery._active and m.thread_readback()["intra_op_threads"] == 6
                counter = inspect.getclosurevars(original.quota_counter.scandir_directory_bytes).nonlocals
                assert type(counter["engine"]) is m.native_quota.NativeFullCensus
                assert counter["engine"].library_sha256 == m.NATIVE_LIBRARY_SHA256
                function = inspect.getclosurevars(original.full_capture_factory()).nonlocals["function"]
                for name in ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
                             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"):
                    assert function.__globals__[name] is getattr(original.collector, name)
                assert function.__globals__["_save_jpeg"].__self__ is recorder
                assert function.__globals__["full_goal_control_reason"] is original.full_goal_control_reason
                if failure: raise marker
    if failure:
        with pytest.raises(RuntimeError) as caught: enter()
        assert caught.value is marker
    else: enter()
    assert recorder is not None and recorder.closed
    assert before == (m.recovery.activated, m.recovery.RouteRecoveryTemporalModelV3, m.threads.THREAD_SCHEMA,
        m.threads.thread_readback, original.collector.collect_episode, original.collector._save_jpeg,
        original.collector.AcknowledgedControlTransport, original.fixture.GreenSignalFixture, original.fixture.OWNER_SCHEMA)
    assert not m._active and not m.recovery._active and not m.previous._active
    assert original.quota_counter.scandir_directory_bytes is original_quota_counter
    assert not (tmp_path / "episode").exists() and not (tmp_path / "episode.partial").exists()


def test_actual_native_owned_tree_counts_every_entry_and_foreign_path_delegates(tmp_path):
    root = tmp_path / "episode.partial"
    root.mkdir()
    (root / "nested").mkdir()
    (root / "nested/data").write_bytes(b"synthetic")
    (root / ".hidden").write_bytes(b"abc")
    engine = m.native_quota.NativeFullCensus(m.NATIVE_LIBRARY,
        expected_source_sha256=m.NATIVE_C_SHA256, expected_library_sha256=m.NATIVE_LIBRARY_SHA256)
    foreign = mock.Mock(return_value=17)
    counter, row = m.native_counter(engine, root, foreign), {}
    assert counter(root, receipt=row) == 12
    assert (row["entries"], row["directories_including_root"], row["regular_files"]) == (3, 2, 2)
    assert row["regular_file_bytes"] == 12 and row["cached_or_incremental_byte_count_used"] is False
    assert row["native_census_helper_sha256"] == m.NATIVE_HELPER_SHA256
    assert row["native_census_library_sha256"] == m.NATIVE_LIBRARY_SHA256
    assert row["directory_generation_postcheck_required"] is True
    assert counter(tmp_path, receipt=row) == 17
    foreign.assert_called_once_with(tmp_path, receipt=row)
    os.symlink(root / ".hidden", root / "denied")
    failed_row = {}
    with pytest.raises(OSError): counter(root, receipt=failed_row)
    assert failed_row["native_census_library_sha256"] == m.NATIVE_LIBRARY_SHA256
    assert "complete_walk" not in failed_row


@pytest.mark.parametrize("change", ["none", "library", "source", "helper", "entries", "generation", "clock", "failed"])
def test_native_audit_adds_pins_without_removing_original_counts_clock_or_failed_tail(change):
    row = dict(sequence=1, status="PASS", native_frame=100, timestamp_simulation_ns=5_000_000_000,
        capture_phase="driving", complete_walk=True, source_payload_read=False, cached_or_incremental_byte_count_used=False,
        entries=3, regular_files=2, directories_including_root=2, special_nodes_ignored_like_original=0,
        regular_file_bytes=12, returned_regular_file_bytes=12, walk_started_monotonic_ns=10, walk_finished_monotonic_ns=20,
        native_census_schema=m.native_quota.SCHEMA, native_census_source_sha256=m.NATIVE_C_SHA256,
        native_census_helper_sha256=m.NATIVE_HELPER_SHA256, native_census_library_sha256=m.NATIVE_LIBRARY_SHA256,
        native_census_library_path=str(m.NATIVE_LIBRARY), native_library_admitted_by_pinned_no_follow_descriptor=True,
        directory_generation_postcheck_required=True)
    value = dict(schema=m.SCHEMA + ".quota_walk_receipts", clock="time.perf_counter_ns_monotonic", maximum_walk_count=1100,
        maximum_count_exceeded=False, every_attempt_including_failures_retained=True, complete_walk_every_call=True,
        incremental_cache_used=False, original_quota_free_floor_and_reserves_unchanged=True,
        actual_sensor_latency_gain_proven_by_walk_timing=False, actual_walk_count=1, rows=[row])
    if change in ("library", "source", "helper"): row["native_census_" + change + "_sha256"] = "0" * 64
    elif change == "entries": row["entries"] = 2
    elif change == "generation": row["directory_generation_postcheck_required"] = False
    elif change == "clock": row["walk_finished_monotonic_ns"] = 31
    elif change == "failed": row["status"] = "FAIL"
    cameras = [dict(frame=100, timestamp=5., capture_phase="driving")]
    timing = [dict(command_frame=100, observed_bundle=dict(native_frame=100, exact_bundle_delivery_monotonic_ns=30))]
    if change != "none":
        with pytest.raises(RuntimeError): m.audit_quota_walks(value, cameras, timing)
    else:
        result = m.audit_quota_walks(value, cameras, timing)
        assert result["passed"] and result["native_helper_C_and_library_pins_verified"]
        assert result["original_quota_free_floor_and_reserves_unchanged"] is True
