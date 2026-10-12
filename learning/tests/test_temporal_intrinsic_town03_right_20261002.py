"""HH_261002 - Fake full compiled capture and paired declaration only; no CARLA/model/actual fit payloads."""
from contextlib import contextmanager, ExitStack
import importlib.util
import inspect
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from types import CodeType, FunctionType
from unittest import mock

import pytest
from scripts.e2e import collect_temporal_intrinsic_town03_right_20261002 as live
from scripts.e2e import research_temporal_intrinsic_town03_right_20261002 as owner
from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hh_frozen_real_capture_fixture", ROOT/"learning/tests/test_temporal_route_joint_town03_right_v3_20261002.py")
fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)


def test_new_runtime_ast_preserves_event_and_adds_actual_jpeg_provider():
    live.verify_frozen(); capture = live.runtime_factory()
    function = inspect.getclosurevars(capture).nonlocals["function"]
    nested = [code for code in function.__code__.co_consts if isinstance(code, CodeType)]
    assert any("_save_jpeg" in code.co_names for code in nested)
    assert callable(live.capture_factory()) and live.RightTurnObserver is live.parent.RightTurnObserver


def test_REAL_full_compiled_capture_records_every_source_after_same_actual_ACK(tmp_path, monkeypatch):
    old_activate = live.frozen.activated; written = []
    def source_save(image, output, quality):
        assert image.frame >= 100
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb") as stream: stream.write(b"exact synthetic image source")
        written.append((image.frame, str(output)))
    @contextmanager
    def activated(model, args):
        with old_activate(model, args), deferred_recording(live.collector, args.output):
            yield
    def namespace(*args, **kwargs):
        # HH_261002 - The original fixture omitted source image.frame; provide its exact synthetic snapshot identity.
        if type(kwargs.get("timestamp")) in (int, float) and "frame" not in kwargs:
            kwargs["frame"] = int(round(kwargs["timestamp"]/.05))
        return SimpleNamespace(*args, **kwargs)
    before = (live.collector._save_jpeg, live.previous.previous.run_event_capture)
    monkeypatch.setattr(fixture, "live", live)
    monkeypatch.setattr(fixture, "SimpleNamespace", namespace)
    monkeypatch.setattr(live.collector, "_save_jpeg", source_save)
    monkeypatch.setattr(live.frozen, "activated", activated)
    monkeypatch.setattr(live.previous.previous, "run_event_capture", live.checked_event_capture)
    calls, states, manifest, error, proof = fixture.run_compiled_capture(tmp_path)
    assert error.rstrip().endswith("RuntimeError: intentional next-tick failure after real first-fresh proposal")
    partial = tmp_path/"episode.partial"
    recordings = [json.loads(line) for line in (partial/"recording_after_ack.jsonl").read_text().splitlines()]
    binding = json.loads((partial/"actual_recording_binding.json").read_text())
    assert len(states) == 25 and len(recordings) == binding["actual_recording_bundle_count"] == 13
    assert len(written) == 78 and binding["actual_after_ACK_receipt_count"] == 13
    assert binding["pending_source_camera_count"] == 0 and binding["all_copied_mutable_providers_current"] is True
    assert binding["enqueue_and_ACK_share_exact_recorder"] is True and binding["exposure_to_ACK_latency_measured"] is False
    assert all(row["enqueued_camera_count"] == row["written_camera_count"] == 6
        and row["all_queued_images_written"] is True and row["acknowledged_before_recording"] is True
        and type(row["ACK_receipt_sequence"]) is int and row["failure"] is None for row in recordings)
    assert [r["frame"] for r in recordings] == list(range(102, 127, 2))
    assert ("ACK", "exception_cleanup_abort") in calls and manifest["cleanup"]["completed"]
    assert proof["initial_drive_prefill_count"] == 0 and proof["first_fresh_driving_prefill_count"] == 1


def test_missing_deferred_recorder_fails_before_capture(tmp_path):
    with pytest.raises(Exception, match="JPEG enqueue"):
        live.checked_event_capture(lambda: None, None, None, None, tmp_path)


@pytest.mark.parametrize("name", ["_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
    "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"])
def test_any_stale_copied_provider_fails_before_original_capture(tmp_path, monkeypatch, name):
    class ACK:
        def send(self, *a, **k): return dict(sequence=1, server_accepted=True)
    monkeypatch.setattr(live.collector, "AcknowledgedControlTransport", ACK)
    monkeypatch.setattr(live.collector, "_save_jpeg", lambda *a: None)
    with deferred_recording(live.collector, tmp_path):
        providers = {key:getattr(live.collector, key) for key in ("_blank_brake_control", "checked_agent_initialization_observation",
            "apply_spawn_z_offset", "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")}
        providers[name] = object()
        function = FunctionType((lambda: None).__code__, providers)
        with pytest.raises(Exception, match="stale"): live.checked_event_capture(function, None, None, None, tmp_path)
        assert not (tmp_path/"actual_recording_binding.json").exists()


def test_different_enqueue_and_ACK_recorder_rejected(tmp_path, monkeypatch):
    class ACK:
        def send(self, *a, **k): return dict(sequence=1, server_accepted=True)
    monkeypatch.setattr(live.collector, "AcknowledgedControlTransport", ACK)
    monkeypatch.setattr(live.collector, "_save_jpeg", lambda *a: None)
    with deferred_recording(live.collector, tmp_path/"a"):
        active = live.collector.AcknowledgedControlTransport
        save = live.collector._save_jpeg
        with deferred_recording(SimpleNamespace(_save_jpeg=lambda *a:None, AcknowledgedControlTransport=ACK), tmp_path/"b") as other:
            monkeypatch.setattr(live.collector, "_save_jpeg", other.enqueue)
            with pytest.raises(Exception, match="same bounded recorder"): live._recorder()
            monkeypatch.setattr(live.collector, "_save_jpeg", save)
        assert live._recorder() is save.__self__ and live.collector.AcknowledgedControlTransport is active


@pytest.mark.parametrize("error, expected", [
    ("CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail", True),
    ("BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail", False),
    ("RuntimeError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail", False),
    ("CollectionError: different failure", False)])
def test_complete110_budget_uses_exact_observed_error_not_turn_success(error, expected):
    states = [dict(frame=100+i, capture_phase="stationary_warmup" if i<24 else "driving") for i in range(2200)]
    cameras = [dict(frame=s["frame"]) for s in states[::2]]
    manifest = dict(status="failed", error=error, capture_contract=dict(capture_phase_schedule=dict(stationary_tail=dict(scheduled_ticks=0))))
    event = dict(completed=False)
    assert owner.complete_interval(manifest, states, cameras, event) == expected
    assert not event["completed"]
    assert not owner.complete_interval(manifest, states[:-1], cameras, event)


@contextmanager
def fake_scope(monkeypatch, tmp_path, arm="control"):
    data = dict(endpoints={name:{key+"_sha256": str(i)*64 for i,key in enumerate(("weights","metadata","checkpoint","declaration"),1)}
        for name in owner.ARMS}, source_sha256={"code":"a"*64})
    path = tmp_path/"authority.json"; run = tmp_path/"pair_run"
    monkeypatch.setattr(owner, "authority", lambda *a:(path, data, run))
    monkeypatch.setattr(owner, "verify_exports", lambda *a:{})
    monkeypatch.setattr(owner, "evidence_assets", lambda *a:{})
    with owner.activated(path, "a"*64, arm): yield


def test_real_owned_plan_argv_knownDEV_preflight_and_all_aliases_restore(tmp_path, monkeypatch):
    before = owner.base.live, owner.base.MODULE, owner.old.numeric, owner.old.threads, owner.parent.numeric, owner.old.complete_interval
    monkeypatch.setattr(owner, "ROOT", tmp_path)
    with fake_scope(monkeypatch, tmp_path):
        config = owner.fixed_contract("v1", True)
        assert config["arm"] == "control" and config["paired_arm_order"] == ["control", "intrinsic_curve"]
        assert config["optimizer_updates"] == config["additional_optimizer_updates"] == 6000
        assert config["known_total_optimizer_updates"] == 18200 and config["parent_known_total_optimizer_updates"] == 12200
        assert config["town"] == "Town03" and config["dataset_split"] == "val" and config["training_source_count"] == 29
        assert config["model_speed_cap_mps"] == 2.5 and config["policy_deadline_seconds"] == .1
        assert config["total_capture_seconds"] == 110 and config["full_original_navigation_points"] == 213
        assert config["speed_preview_s"] == .5 and config["controller_target_cap_mps"] == 2.4
        assert config["observed_wall_latency_screen"]["exposure_to_ACK_wall_latency_proven"] is False
        args = live.frozen.parse_args(owner.base.capture_argv("v1", True, "a"*64))
        live.frozen.validate_capture(args, live.collector.load_route(args.route_file))
        assert str(owner._scope.weights) == str(args.research_weights) and args.seed == live.SEED
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(owner.base, "digest", return_value="a"*64))
            stack.enter_context(mock.patch.object(owner.base, "runtime_sources", return_value={"code":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base, "external_sources", return_value={"api":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base, "ASSETS", {p:"a"*64 for p in owner.base.ASSETS}))
            plan = owner.base.build_plan("v1", True)
            stack.enter_context(mock.patch.object(owner.base, "verify_plan", return_value=plan))
            stack.enter_context(mock.patch.object(owner.base.lifecycle, "_assert_no_other_carla"))
            stack.enter_context(mock.patch.object(owner.base, "ports_free"))
            stack.enter_context(mock.patch.object(owner.base, "disk_check", return_value={}))
            imported = SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.old.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
            stack.enter_context(mock.patch.object(owner.base.subprocess, "run", return_value=imported))
            stack.enter_context(mock.patch.object(owner.base, "load_npz", return_value=SimpleNamespace(_research_temporal_checkpoint_sha256="3"*64)))
            assert owner.base.preflight("v1", "a"*64)[1]["status"] == "PREFLIGHT_PASS_NO_SIMULATOR"
    assert before == (owner.base.live, owner.base.MODULE, owner.old.numeric, owner.old.threads, owner.parent.numeric, owner.old.complete_interval)


def test_numeric_model_binding_rejects_unsealed_or_wrong_arms(monkeypatch):
    monkeypatch.setattr(live, "verify_scope", lambda d:None)
    binding = dict(schema=live.NUMERIC_OWNER_SCHEMA, arm="control", fixed_endpoint_updates=6000,
        purpose="owned_carla_research_only", simulator_control_authorized=True, simulation_only=True,
        owner_plan_sha256="a"*64, completed_pair_authority_sha256="b"*64, vehicle_control_approved=False, model_promoted=False)
    model = SimpleNamespace(_research_temporal_export_metadata=dict(arm="control"), _research_recovery_owner_binding=binding,
        _research_recovery_numeric_loader_schema=live.LOADER_SCHEMA, _research_recovery_simulator_control_authorized=True,
        _research_intrinsic_completed_pair_authority_sha256="b"*64)
    live.validate_model_binding(model, SimpleNamespace(research_source_sha256="a"*64))
    binding["arm"] = "best"
    with pytest.raises(Exception): live.validate_model_binding(model, SimpleNamespace(research_source_sha256="a"*64))


def test_owner_every_alias_restores_after_exception(tmp_path, monkeypatch):
    before = owner.base.live, owner.base.MODULE, owner.old.numeric, owner.old.threads, owner.parent.numeric, owner.old.complete_interval, live.previous.numeric
    with pytest.raises(RuntimeError, match="sentinel"):
        with fake_scope(monkeypatch, tmp_path, "intrinsic_curve"):
            assert owner._scope.arm == "intrinsic_curve"
            raise RuntimeError("sentinel")
    assert before == (owner.base.live, owner.base.MODULE, owner.old.numeric, owner.old.threads, owner.parent.numeric, owner.old.complete_interval, live.previous.numeric)
    assert owner._scope is None and not owner._active


@pytest.mark.parametrize("fault", [None, "missing_exports", "missing_private_plan", "wrong_town", "changed_declaration"])
def test_REAL_private_worker_to_full_live_activation_Green_ownership(tmp_path, monkeypatch, fault):
    # HH_261002 - Exercise the original private worker and original Green ownership check through the real nested capture scopes.
    old = live.previous.previous; recovery = old.signed1.previous; fast45 = recovery.previous
    original_numeric = live.previous.numeric
    basic = SimpleNamespace(BasicAgent=type("FakeBasicAgent", (), {}))
    monkeypatch.setattr(live, "verify_scope", lambda description: None)
    with fake_scope(monkeypatch, tmp_path):
        adapter = owner._scope.numeric
        assert live.previous.numeric is owner.old.numeric is owner.parent.numeric is adapter
        assert adapter._exports == owner._scope.pins and adapter._owner_plan_sha is None
        args = live.frozen.parse_args(owner.base.capture_argv("v1", True, "a"*64))
        args.output = tmp_path/"capture/episode"
        declaration = args.output.parent/"declaration.json"
        declaration.parent.mkdir()
        plan = dict(schema=live.OWNER_SCHEMA, town="Town07" if fault == "wrong_town" else "Town03", original_navigation_sha256=live.ROUTE_SHA)
        owner.write_new(declaration, plan); pin = owner.digest(declaration)
        args.research_source_sha256 = pin
        model = SimpleNamespace(model_id=recovery.MODEL_ID, modules=lambda: [], state_dict=lambda: {},
            _research_speed_preview_calibration=live.PREVIEW_SCHEMA, _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
            _research_recovery_simulator_control_authorized=True, _research_recovery_numeric_loader_schema=live.LOADER_SCHEMA,
            _research_temporal_export_metadata=dict(arm="control"), _research_intrinsic_completed_pair_authority_sha256="b"*64,
            _research_recovery_owner_binding=dict(schema=live.NUMERIC_OWNER_SCHEMA, arm="control", fixed_endpoint_updates=6000,
                purpose="owned_carla_research_only", simulation_only=True, simulator_control_authorized=True,
                owner_plan_sha256=pin, completed_pair_authority_sha256="b"*64, vehicle_control_approved=False, model_promoted=False))
        def actual_worker(run_id, expected):
            assert adapter._owner_plan_sha == expected == pin
            with live.activated(model, args):
                assert live.previous.OWNER_SCHEMA == live.OWNER_SCHEMA
                assert live.previous.numeric is adapter
                if fault == "missing_exports": adapter._exports = None
                if fault == "missing_private_plan": adapter._owner_plan_sha = None
                if fault == "changed_declaration": declaration.write_text(json.dumps(dict(plan, mutation=True)))
                live.previous.assert_owned_capture(args)
                return 0
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(owner.base, "verify_plan", return_value=plan))
            guard = stack.enter_context(mock.patch.object(owner.base, "assert_owned_worker"))
            stack.enter_context(mock.patch.object(owner.old, "_original_worker", side_effect=actual_worker))
            stack.enter_context(mock.patch.object(recovery, "RouteRecoveryTemporalModelV3", SimpleNamespace))
            stack.enter_context(mock.patch.object(fast45, "CPUFiniteCachedTemporalModelV2", SimpleNamespace))
            stack.enter_context(mock.patch.object(old.previous.previous.torch, "get_num_threads", return_value=4))
            stack.enter_context(mock.patch.object(old.previous.previous.torch, "get_num_interop_threads", return_value=1))
            stack.enter_context(mock.patch.dict(sys.modules, {"agents": SimpleNamespace(), "agents.navigation": SimpleNamespace(basic_agent=basic),
                "agents.navigation.basic_agent": basic}))
            if fault is None:
                assert owner.base.worker("v1", pin) == 0
                assert guard.call_count == 2
                assert all(call.args == (plan, pin) for call in guard.call_args_list)
            else:
                with pytest.raises(Exception): owner.base.worker("v1", pin)
                assert guard.call_count == 1
        assert adapter._owner_plan_sha is None and not live._active and not live.parent._active
    assert live.previous.numeric is original_numeric


def test_unknown_fixture_numeric_alias_rejected_before_any_scope(tmp_path, monkeypatch):
    monkeypatch.setattr(live.previous, "numeric", SimpleNamespace())
    with pytest.raises(Exception, match="unknown frozen Green-fixture"):
        with owner.activated(tmp_path/"authority.json", "a"*64, "control"): pytest.fail("unknown scope entered")
    assert owner._scope is None and not owner._active


def test_BOTH_case_plans_exist_before_batch_receipt_or_any_execution(tmp_path, monkeypatch):
    monkeypatch.setattr(owner, "ROOT", tmp_path)
    seen = []
    @contextmanager
    def scope(path, pin, arm):
        assert not owner.batch_path().exists()
        owner._scope = SimpleNamespace(arm=arm)
        try: yield
        finally: owner._scope = None
    def build(run_id, pi):
        assert pi is True and not owner.batch_path().exists()
        seen.append(owner._scope.arm)
        return dict(schema=owner.SCHEMA, arm=owner._scope.arm, simulator_started=False)
    monkeypatch.setattr(owner, "activated", scope)
    monkeypatch.setattr(owner.base, "build_plan", build)
    monkeypatch.setattr(owner.base, "disk_check", lambda: {})
    assert owner.main(["--mode", "declare", "--completed-pair-authority", str(tmp_path/"auth.json"),
        "--completed-pair-authority-sha256", "a"*64]) == 0
    batch = json.loads(owner.batch_path().read_text())
    assert seen == ["control", "intrinsic_curve"] and batch["arm_order"] == seen
    for arm, pin in batch["case_sha256"].items():
        assert owner.digest(owner.batch_path().parent/(arm+"_plan.json")) == pin
    assert batch["simulator_started"] is False
