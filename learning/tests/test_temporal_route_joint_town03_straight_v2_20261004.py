"""HH_261004 - Prove the fresh Town03 v2 recording/corridor revision on CPU without CARLA or GPU."""

from contextlib import contextmanager
import importlib.util
import inspect
import json
from pathlib import Path
import sys
from types import CodeType, SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_straight_v2_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_v2_20261004 as owner
from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


ROOT = Path(__file__).resolve().parents[2]
FIXTURE_SPEC = importlib.util.spec_from_file_location("hh_town03_v2_runtime_fixture",
    ROOT / "learning/tests/test_temporal_route_joint_town03_right_v3_20261002.py")
fixture = importlib.util.module_from_spec(FIXTURE_SPEC)
FIXTURE_SPEC.loader.exec_module(fixture)


def pins():
    return {name + "_sha256": str(index) * 64
            for index, name in enumerate(("weights", "metadata", "checkpoint", "declaration"), 1)}


def good6000_pins():
    return dict(
        weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
        metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
        checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
        declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")


@contextmanager
def owner_scope(values=None):
    with mock.patch.object(owner.numeric, "verify_exports"), \
         mock.patch.object(owner.previous.runtime_owner.previous.previous.threads,
                           "evidence_assets", return_value={}):
        with owner.activated(pins() if values is None else values):
            yield


def arguments():
    with owner_scope():
        return live.frozen.parse_args(owner.base.capture_argv("v2", True, "a" * 64))


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def test_v2_contract_changes_only_recording_evidence_and_authenticated_start_corridor():
    with owner_scope():
        contract = owner.fixed_contract("v2", True)
    assert contract["schema"] == live.OWNER_SCHEMA
    assert contract["output"].endswith("town03_known_dev_straight_v2")
    assert contract["actual_deferred_JPEG_after_ACK_required"] is True
    assert contract["actual_post_ACK_recording"]["required_camera_bundles"] == 450
    assert contract["actual_post_ACK_recording"]["required_JPEG_count"] == 2700
    assert contract["actual_post_ACK_recording"]["required_complete_ACK_receipts"] == 904
    corridor = contract["authenticated_start_corridor"]
    assert corridor["upstream_lane"] == [67, 0, 2]
    assert corridor["initial_route_lane"] == [1426, 0, 2]
    assert corridor["enabled_only_when_body_arc_lower_bound_negative"] is True
    assert corridor["original_navigation_bytes_unchanged"] is True
    assert corridor["recorded_actor_bounding_box_radius_m"] == 2.884351637197339
    assert corridor["lane_inset_m"] == 0.1 and corridor["topology_seam_maximum_m"] == 0.005
    assert corridor["gate_relaxed"] is False and corridor["failed_prefix_cropped"] is False
    assert contract["failed_v1_archived_reference"]["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    assert contract["failed_v1_archived_reference"]["overwritten"] is False
    assert contract["intervention"]["model_weights_changed"] is False
    assert contract["intervention"]["controller_changed"] is False
    assert contract["intervention"]["latency_gate_changed"] is False
    assert contract["total_capture_seconds"] == 45
    assert contract["maximum_native_state_count"] == 900
    assert contract["maximum_camera_bundle_count"] == 450
    assert contract["model_speed_cap_mps"] == 2.5
    assert contract["native_actor_planar_speed_maximum_mps"] == 2.7
    assert contract["policy_deadline_seconds"] == 0.1
    assert contract["controller_target_cap_mps"] == 2.4 and contract["speed_preview_s"] == 0.5
    assert contract["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.0
    assert contract["precursor_motion_screen"]["cte_p95_maximum_m"] == 0.35
    assert contract["precursor_motion_screen"]["cte_absolute_maximum_m"] == 0.75
    assert contract["body_clearance_screen"]["lane_inset_m"] == 0.1
    assert contract["worker_command"][contract["worker_command"].index("-m") + 1] == owner.MODULE
    assert not contract["full_RTE01_approved"] and not contract["model_promoted"]


def test_runtime_AST_adds_only_the_sixth_copied_JPEG_provider():
    live.verify_frozen()
    capture = live.runtime_factory()
    function = inspect.getclosurevars(capture).nonlocals["function"]
    nested = [item for item in function.__code__.co_consts if isinstance(item, CodeType)]
    assert any("_save_jpeg" in item.co_names for item in nested)
    assert callable(live.capture_factory())


def test_full_compiled_straight_capture_late_binds_same_post_ACK_recorder(tmp_path, monkeypatch):
    """HH_261004 - Reproduce the formerly stale copied-global seam through the real compiled collector AST."""
    old_activate = live.frozen.activated
    written = []

    def source_save(image, output, quality):
        assert image.frame >= 100 and quality == 95
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb") as stream:
            stream.write(b"exact synthetic image source")
        written.append((image.frame, str(output)))

    @contextmanager
    def activated(model, args):
        with old_activate(model, args), deferred_recording(live.collector, args.output):
            yield

    def namespace(*args, **kwargs):
        if type(kwargs.get("timestamp")) in (int, float) and "frame" not in kwargs:
            kwargs["frame"] = int(round(kwargs["timestamp"] / 0.05))
        return SimpleNamespace(*args, **kwargs)

    before = (live.collector._save_jpeg, live.previous.previous.run_event_capture,
              live.previous._original_factory)
    monkeypatch.setattr(fixture, "live", live)
    monkeypatch.setattr(fixture, "SimpleNamespace", namespace)
    monkeypatch.setattr(live.collector, "_save_jpeg", source_save)
    monkeypatch.setattr(live.frozen, "activated", activated)
    monkeypatch.setattr(live.previous, "_original_factory", live.straight.cloned_collect_episode)
    monkeypatch.setattr(live.previous.previous, "run_event_capture", live.checked_straight_capture)
    calls, states, manifest, error, runtime_proof = fixture.run_compiled_capture(tmp_path)
    partial = tmp_path / "episode.partial"
    rows = [json.loads(line) for line in (partial / "recording_after_ack.jsonl").read_text().splitlines()]
    proof = json.loads((partial / "actual_recording_binding.json").read_text())
    assert error.rstrip().endswith("RuntimeError: intentional next-tick failure after real first-fresh proposal")
    assert len(states) == 25 and runtime_proof["observed_bundle_calls"] == 13
    assert len(rows) == proof["actual_recording_bundle_count"] == proof["actual_after_ACK_receipt_count"] == 13
    assert len(written) == 78 and proof["pending_source_camera_count"] == 0
    assert proof["all_copied_mutable_providers_current"] is True
    assert proof["enqueue_and_ACK_share_exact_recorder"] is True
    assert proof["controller_or_policy_math_changed"] is False
    assert all(row["enqueued_camera_count"] == row["written_camera_count"] == 6
        and row["acknowledged_before_recording"] is True and row["failure"] is None
        and row["all_queued_images_written"] is True for row in rows)
    assert [row["frame"] for row in rows] == list(range(102, 127, 2))
    assert ("ACK", "exception_cleanup_abort") in calls and manifest["cleanup"]["completed"]
    assert before[0] is not live.collector._save_jpeg
    assert before[1] is not live.previous.previous.run_event_capture
    assert before[2] is not live.previous._original_factory


def test_missing_deferred_recorder_fails_before_original_capture(tmp_path):
    with pytest.raises(Exception, match="exact-JPEG enqueue"):
        live.checked_straight_capture(lambda: None, None, None, None, tmp_path)
    assert not (tmp_path / "actual_recording_binding.json").exists()


@pytest.mark.parametrize("failure", [False, True])
def test_full_v2_activation_installs_and_restores_recording_and_geometry_scopes(failure):
    straight = live.straight
    right = live.previous.previous
    recovery = right.signed1.previous
    fast45 = recovery.previous
    args = arguments()
    basic = SimpleNamespace(BasicAgent=type("FakeBasicAgent", (), {}))
    fake = SimpleNamespace(model_id=recovery.MODEL_ID, modules=lambda: [], state_dict=lambda: {},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only", simulation_only=True,
                                               owner_plan_sha256="a" * 64))
    before = (live.collector._save_jpeg, live.collector.AcknowledgedControlTransport,
        live.collector.collect_episode, owner.previous.previous.geometry, owner.previous.previous.body_report)
    with mock.patch.object(right.previous.previous.previous, "validate_model_binding"), \
         mock.patch.object(recovery, "RouteRecoveryTemporalModelV3", SimpleNamespace), \
         mock.patch.object(fast45, "CPUFiniteCachedTemporalModelV2", SimpleNamespace), \
         mock.patch.object(right.previous.previous.torch, "get_num_threads", return_value=4), \
         mock.patch.object(right.previous.previous.torch, "get_num_interop_threads", return_value=1), \
         mock.patch.dict(sys.modules, {"agents": SimpleNamespace(),
             "agents.navigation": SimpleNamespace(basic_agent=basic),
             "agents.navigation.basic_agent": basic}):
        def enter():
            with owner_scope():
                assert owner.previous.previous.geometry is owner.start_geometry
                assert owner.previous.previous.body_report is owner.body_report
                with live.activated(fake, args):
                    assert getattr(live.collector, "_exact_jpeg_after_ACK_active") is True
                    assert live._recorder() is live.collector._save_jpeg.__self__
                    if failure:
                        raise RuntimeError("scope failure")
        if failure:
            with pytest.raises(RuntimeError, match="scope failure"):
                enter()
        else:
            enter()
    assert before == (live.collector._save_jpeg, live.collector.AcknowledgedControlTransport,
        live.collector.collect_episode, owner.previous.previous.geometry, owner.previous.previous.body_report)
    assert live._active is False and straight._active is False and owner._active is False


def diagnostic_fixture(tmp_path):
    episode = tmp_path / "episode.partial"
    proof = dict(schema=live.SCHEMA + ".actual_recording_binding",
        copied_provider_names=["_blank_brake_control", "checked_agent_initialization_observation",
            "apply_spawn_z_offset", "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"],
        all_copied_mutable_providers_current=True, enqueue_and_ACK_share_exact_recorder=True,
        controller_or_policy_math_changed=False, actual_recording_bundle_count=450,
        actual_after_ACK_receipt_count=450, pending_source_camera_count=0)
    episode.mkdir(parents=True)
    (episode / "actual_recording_binding.json").write_text(json.dumps(proof))
    write_rows(episode / "recording_after_ack.jsonl", [dict(frame=2 * index) for index in range(450)])
    write_rows(episode / "camera_frames.jsonl", [dict(frame=2 * index) for index in range(450)])
    write_rows(episode / "control_receipts.jsonl", [dict(sequence=index + 1) for index in range(904)])
    write_rows(episode / "wall_timing.jsonl", [dict(ACK_sequence=index + 1) for index in range(904)])
    derived = tmp_path / "derived_body"
    derived.mkdir()
    (derived / "start_corridor_topology.json").write_text(json.dumps(dict(
        schema=owner.start_geometry.SCHEMA,
        authorized_start_prefix_lane=list(owner.start_geometry.PREFIX_LANE),
        initial_route_lane=list(owner.start_geometry.INITIAL_ROUTE_LANE),
        route_input_modified=False, global_driving_lane_union_used=False)))
    return proof


@pytest.mark.parametrize("bad", [None, "bundle", "ack", "parent"])
def test_diagnostic_requires_complete_new_binding_in_addition_to_parent_gates(tmp_path, monkeypatch, bad):
    proof = diagnostic_fixture(tmp_path)
    if bad == "bundle":
        proof["actual_recording_bundle_count"] = 449
        (tmp_path / "episode.partial/actual_recording_binding.json").write_text(json.dumps(proof))
    elif bad == "ack":
        rows = [dict(sequence=index + 1) for index in range(903)]
        write_rows(tmp_path / "episode.partial/control_receipts.jsonl", rows)
    parent_pass = bad != "parent"
    inherited = dict(recording=dict(passed=True, JPEG_count=2700), raw_source_sha256={},
        learned_driving_screen_passed=parent_pass,
        straight_motion_body_policy_wall100ms_passed=parent_pass)
    monkeypatch.setattr(owner, "_parent_diagnostic", lambda *args: dict(inherited))
    monkeypatch.setattr(owner.previous.previous, "verify_closure", lambda *args: None)
    result = owner.diagnostic_report(tmp_path, {}, {})
    expected = bad is None
    assert result["actual_after_ACK_recording_binding_passed"] is (bad not in ("bundle", "ack"))
    assert result["learned_driving_screen_passed"] is expected
    assert result["straight_motion_body_policy_wall100ms_passed"] is expected
    assert result["status"] == ("PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if expected
                                else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR")
    assert not result["full_RTE01_approved"] and not result["model_promoted"]


def test_runtime_source_closure_includes_new_owner_collector_and_geometry():
    with owner_scope(good6000_pins()):
        sources = owner.runtime_sources()
    for module in (live, owner, owner.start_geometry):
        path = Path(module.__file__).resolve()
        assert sources[str(path.relative_to(owner.ROOT))] == owner.digest(path)


def test_real_plan_build_uses_current_assets_without_CUDA_or_creating_the_fresh_output():
    output, declaration = owner.paths("v2")
    assert not output.exists() and not declaration.exists()
    with owner.activated(good6000_pins()):
        plan = owner.base.build_plan("v2", True)
    assert plan["schema"] == live.OWNER_SCHEMA and plan["run_id"] == "v2"
    assert plan["source_sha256"]["tools/e2e/collect_temporal_route_joint_town03_straight_v2_20261004.py"] \
        == owner.digest(Path(live.__file__))
    assert plan["source_sha256"]["tools/e2e/research_temporal_route_joint_town03_straight_v2_20261004.py"] \
        == owner.digest(Path(owner.__file__))
    assert plan["actual_deferred_JPEG_after_ACK_required"] is True
    assert plan["authenticated_start_corridor"]["upstream_lane"] == [67, 0, 2]
    assert not output.exists() and not declaration.exists()


def test_paths_are_fresh_create_only_and_other_versions_fail():
    output, declaration = owner.paths("v2")
    assert output.name.endswith("known_dev_straight_v2")
    assert declaration.name.endswith("known_dev_straight_v2_plan.json")
    assert output != owner.V1_OUTPUT and not output.exists()
    for value in ("v1", "v3", "../v2", True):
        with pytest.raises(Exception):
            owner.paths(value)
