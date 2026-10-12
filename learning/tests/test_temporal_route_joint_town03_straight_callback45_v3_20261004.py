"""HH_261004 - Verify callback identity, unchanged actual policy math, owner admission and full timing denominators on CPU."""

import ast
import copy
from contextlib import contextmanager
from dataclasses import replace
import hashlib
import inspect
import json
import os
from pathlib import Path
import socket
import sys
import threading
from queue import SimpleQueue
from types import SimpleNamespace
from unittest import mock

import pytest
import torch

from scripts.e2e import collect_temporal_route_joint_town03_straight_callback45_v3_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_callback45_v3_20261004 as owner
from scripts.e2e import collect_temporal_route_joint_preview05_signed45_20261002 as preview
from scripts.e2e.collect_temporal_route_recovery_fast45_20261001 import RecoverySingleCopyPolicyV3
from portable_e2e.temporal_owned_camera_history_v2_20261001 import own_observed_camera_bytes
from portable_e2e.simulation_research import ControllerConfig
from test_temporal_route_joint_town03_straight_v3_20261004 import (
    FakeWorld, fake_model_and_patches, entered, good6000_pins, pins)
from test_temporal_owned_camera_history_v2_20261001 import frame as observed_frame, IDENTITY


@contextmanager
def owner_scope(values=None):
    threads = owner.previous.previous.previous.runtime_owner.previous.previous.threads
    with mock.patch.object(owner.numeric, "verify_exports"), \
         mock.patch.object(threads, "evidence_assets", return_value={}):
        with owner.activated(pins() if values is None else values):
            yield


def arguments(tmp_path=None):
    with owner_scope():
        args = live.frozen.parse_args(owner.base.capture_argv(owner.RUN_ID, True, "a" * 64))
    if tmp_path is not None:
        args.output = tmp_path / "episode"
    return args


def image(frame=1, stamp=.1):
    return SimpleNamespace(frame=frame, timestamp=stamp, width=640, height=360,
                           raw_data=b"\x03" * (360 * 640 * 4))


def test_actual_callback_first_instruction_is_the_monotonic_clock():
    tree = ast.parse(Path(live.__file__).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CallbackLedger")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "callback")
    callback = next(n for n in method.body if isinstance(n, ast.FunctionDef) and n.name == "observed")
    assert ast.unparse(callback.body[0]) == "entered = self.clock()"
    assert inspect.signature(live.CallbackLedger).parameters["clock"].default is live.time.perf_counter_ns


@pytest.mark.parametrize("failure", [False, True])
def test_actual_callback_preserves_one_original_image_put_return_or_exception(tmp_path, failure):
    seen, failure_object, sentinel = [], RuntimeError("original queue failure"), object()
    clock = iter((100, 101, 102, 103)).__next__
    ledger = live.CallbackLedger(object(), tmp_path, clock)
    source = image()
    def put(value):
        seen.append(value)
        if failure:
            raise failure_object
        return sentinel
    callback = ledger.callback(live.collector.MODEL_CAMERA_ORDER[0], 17, put)
    if failure:
        with pytest.raises(RuntimeError) as caught:
            callback(source)
        assert caught.value is failure_object
    else:
        assert callback(source) is sentinel
    assert seen == [source] and seen[0] is source
    row = ledger.callbacks[0]
    assert row["callback_enter_monotonic_ns"] == 100
    assert row["original_queue_put_calls"] == 1 and row["image_object_id"] == id(source)
    assert row["source_BGRA_encoding_prefixed_sha256"] == hashlib.sha256(b"bgra8" + source.raw_data).hexdigest()
    assert row["source_image_strong_reference_retained"] is False
    assert row["receipt_error"] is None and ledger.inflight == 0
    assert row["original_queue_put_returned"] is (not failure)


def test_diagnostic_metadata_failure_never_replaces_original_queue_return(tmp_path):
    ledger, sentinel = live.CallbackLedger(object(), tmp_path), object()
    source = image(); source.raw_data = b"not a full camera"
    callback = ledger.callback(live.collector.MODEL_CAMERA_ORDER[0], 17, lambda value: sentinel)
    assert callback(source) is sentinel
    assert "BGRA byte count" in ledger.callbacks[0]["receipt_error"]
    assert ledger.callbacks[0]["original_queue_put_calls"] == 1 and ledger.inflight == 0


def test_callback_registration_refuses_duplicate_or_unknown_role_without_replacing_queue(tmp_path):
    ledger = live.CallbackLedger(object(), tmp_path)
    role = live.collector.MODEL_CAMERA_ORDER[0]
    ledger.callback(role, 17, lambda source: None)
    for name, actor in ((role, 18), ("not a camera", 19), (live.collector.MODEL_CAMERA_ORDER[1], True)):
        with pytest.raises(Exception):
            ledger.callback(name, actor, lambda source: None)
    assert len(ledger.bindings) == 1 and ledger.callbacks == []


def test_receipt_close_waits_only_after_capture_without_holding_original_queue(tmp_path):
    ledger = live.CallbackLedger(object(), tmp_path)
    ledger.capture_started = True
    queued, release = threading.Event(), threading.Event()
    class DelayedImage:
        frame, timestamp, width, height = 1, .1, 640, 360
        @property
        def raw_data(self):
            assert release.wait(2.0)
            return b"\x01" * (360 * 640 * 4)
    callback = ledger.callback(live.collector.MODEL_CAMERA_ORDER[0], 17, lambda source: queued.set())
    thread = threading.Thread(target=callback, args=(DelayedImage(),))
    thread.start()
    assert queued.wait(1.0)  # HH_261004 - Main/control can consume the original queue before receipt SHA finishes.
    assert ledger.inflight == 1
    timer = threading.Timer(.02, release.set)
    timer.start()
    value = ledger.finish()
    thread.join(1.0); timer.join(1.0)
    assert not thread.is_alive() and value["inflight_callbacks"] == 0
    assert value["callback_count"] == 1 and value["callbacks"][0]["receipt_error"] is None
    assert value["physics_or_control_wait_added"] is False
    assert json.loads((tmp_path / "callback_timing_receipts.json").read_text()) == value


def test_no_capture_activation_does_not_create_an_episode_or_receipt(tmp_path):
    path = tmp_path / "absent.partial"
    ledger = live.CallbackLedger(object(), path)
    assert ledger.finish() is None and not path.exists()


def test_receipt_write_accepts_original_finalized_episode_and_never_recreates_partial(tmp_path):
    episode = tmp_path / "episode"; episode.mkdir()
    partial = tmp_path / "episode.partial"
    ledger = live.CallbackLedger(object(), partial); ledger.capture_started = True
    value = ledger.finish()
    assert (episode / "callback_timing_receipts.json").is_file()
    assert not partial.exists() and value["receipt_close_after_original_capture_cleanup"] is True
    with pytest.raises(FileExistsError):
        ledger.finish()


def test_actual_camera_listen_AST_changes_one_expression_and_refuses_other_source():
    tree = ast.parse(Path(live.collector.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
    changed = live.instrument_callback_AST(original)
    expressions = [ast.unparse(n) for n in ast.walk(changed) if isinstance(n, ast.Call)]
    assert expressions.count("sensor.listen(timed_camera_callback(spec.name, sensor.id, queues[spec.name].put))") == 1
    assert expressions.count("collision.listen(event_recorder.on_collision)") == 1
    assert expressions.count("lane_invasion.listen(event_recorder.on_lane_invasion)") == 1
    with pytest.raises(Exception, match="exact original"):
        live.instrument_callback_AST(ast.parse("def collect_episode(): pass").body[0])


def test_compiled_actual_policy_contains_only_three_instrumented_original_statements():
    function = live.timed_policy_step()
    assert function.__globals__["policy_marker"] is live.policy_marker
    assert function.__globals__["trajectory_control"] is live.policy_runtime.trajectory_control
    assert function.__globals__["validate_and_select_trajectory"] is live.policy_runtime.validate_and_select_trajectory
    assert "policy_marker" in function.__code__.co_names
    assert function.__code__.co_argcount == live._original_step.__code__.co_argcount


@pytest.mark.parametrize("timed", [False, True])
@pytest.mark.parametrize("failure", [False, True])
def test_actual_bundle_assignment_observer_preserves_same_provider_arguments_and_originals(timed, failure):
    """HH_261004 - Exercise each inverse-proven original assignment without replacing its bundle-provider object."""
    original = next(n for n in ast.parse(Path(live.collector.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
    changed = live.instrument_callback_AST(original)
    calls = [n for n in ast.walk(changed) if isinstance(n, ast.Assign)
             and len(n.targets) == 1 and ast.unparse(n.targets[0]) == "bundle"
             and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == "exact_camera_bundle"]
    assert len(calls) == 2
    assignment = next(n for n in calls if bool(n.value.keywords) is timed)
    seen, observed, bundle, error = [], [], {role: image() for role in live.collector.MODEL_CAMERA_ORDER}, RuntimeError("original provider")
    queues, timing = object(), object()
    def provider(*args, **kwargs):
        seen.append((args, kwargs))
        if failure:
            raise error
        return bundle
    namespace = dict(exact_camera_bundle=provider, queues=queues, frame=19,
        args=SimpleNamespace(sensor_timeout_sec=3.0), tick_timing=timing,
        callback_bundle_selected=lambda returned, frame: observed.append((returned, frame)))
    module = ast.Module(body=[copy.deepcopy(assignment), ast.parse("callback_bundle_selected(bundle, frame)").body[0]], type_ignores=[])
    code = compile(ast.fix_missing_locations(module), "CPU_ONLY_actual_bundle_assignment", "exec")
    if failure:
        with pytest.raises(RuntimeError) as caught:
            exec(code, namespace)
        assert caught.value is error and observed == []
    else:
        exec(code, namespace)
        assert observed == [(bundle, 19)] and observed[0][0] is bundle
        assert all(observed[0][0][role] is bundle[role] for role in bundle)
    assert seen == [((queues, 19, 3.0), dict(timing=timing) if timed else {})]
    assert namespace["exact_camera_bundle"] is provider


def _real_good6000_policy():
    """HH_261004 - Load the actual sealed GOOD6000 data-only model on CPU; no simulator authorization is requested."""
    with owner.activated(good6000_pins()):
        model = owner.numeric.load_model(owner.numeric.WEIGHTS, owner.numeric.METADATA,
            weights_sha256=good6000_pins()["weights_sha256"], metadata_sha256=good6000_pins()["metadata_sha256"])
    assert model._research_recovery_simulator_control_authorized is False
    return model


@pytest.mark.parametrize("cause", ["normal", "deadline", "external_emergency", "short_navigation"])
def test_real_good6000_raw_decoded_gate_selected_PI_and_control_are_bitwise_unchanged(monkeypatch, tmp_path, cause):
    """HH_261004 - Compare actual GOOD6000 numerical execution, not a fake forward or substituted decoder."""
    saved_threads = torch.get_num_threads(); torch.set_num_threads(4)
    try:
        model = _real_good6000_policy()
        second_model = copy.deepcopy(model)
        identity = dict(IDENTITY)
        config = replace(ControllerConfig(2.7, .6), speed_preview_s=.5)
        before = RecoverySingleCopyPolicyV3(model, config, **identity, longitudinal_pi=True)
        after = RecoverySingleCopyPolicyV3(second_model, config, **identity, longitudinal_pi=True)
        sources = [own_observed_camera_bytes(observed_frame(i)) for i in range(11)]
        monkeypatch.setattr(live.policy_runtime.time, "perf_counter", lambda: 1.0)
        for policy in (before, after):
            for index in (8, 9):
                current = sources[index]
                policy.prefill(current, past_frames=tuple(sources[max(0, index - 9):index]),
                    now_ns=current.timestamp_ns, ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)
        current = sources[10]
        if cause == "short_navigation":
            current = replace(current, inputs=replace(current.inputs, route_xy_base_m=((0., 0.), (1., 0.))))
        kwargs = dict(past_frames=tuple(sources[1:10]), now_ns=current.timestamp_ns,
                      ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)
        if cause == "external_emergency":
            kwargs["emergency_reason"] = "synthetic CPU test emergency"
        if cause == "deadline":
            # HH_261004 - Replay the original policy clock calls, including the unchanged owned-input timing wrapper.
            def expired_clock():
                counter = expired_clock.counter
                expired_clock.counter += 1
                return 1.0 if counter < 5 else 1.2
            expired_clock.counter = 0
            monkeypatch.setattr(live.policy_runtime.time, "perf_counter", expired_clock)
        # HH_261004 - Enter the genuine inherited preview/PI/recorder scope before comparing the original and timed step.
        args = arguments(tmp_path)
        _, patches = fake_model_and_patches()
        second_model._research_speed_preview_calibration = live.PREVIEW_SCHEMA
        second_model._research_cpu_thread_numerical_revision = live.THREAD_SCHEMA
        second_model._research_recovery_simulator_control_authorized = True
        second_model._research_recovery_owner_binding = dict(purpose="owned_carla_research_only",
            simulation_only=True, owner_plan_sha256="a" * 64)
        with owner_scope(), patches[0], patches[3], patches[4], patches[5], live.activated(second_model, args):
            assert live.policy_runtime.trajectory_control is preview.trajectory_control
            assert live.policy_runtime.TemporalSimulationResearchPolicyV2.step.__globals__["trajectory_control"] \
                is preview.trajectory_control
            first_session, second_session = (SimpleNamespace(_preview_tracker_receipt=None) for _ in range(2))
            with mock.patch.object(preview, "_proposal_session", first_session), \
                 mock.patch.object(live.policy_runtime.TemporalSimulationResearchPolicyV2, "step", live._original_step):
                first = before.step(current, **kwargs)
            if cause == "deadline":
                expired_clock.counter = 0
            with mock.patch.object(preview, "_proposal_session", second_session):
                second = after.step(current, **kwargs)
            assert first_session._preview_tracker_receipt == second_session._preview_tracker_receipt
            ledger = live._ledger
            if cause == "normal":
                receipt = second_session._preview_tracker_receipt
                assert receipt is not None and receipt["schema"] == live.PREVIEW_SCHEMA
                assert receipt["configured_speed_preview_s"] == .5
                assert receipt["proposed_preview_index"] == 4
                assert receipt["original_preview_index"] == 2
                assert receipt["effective_selected_target_mps"] == second["control"]["desired_speed_mps"]
                assert second["longitudinal_pi_research"]["controller_target_cap_mps"] == 2.4
        # HH_261004 - Timing markers are separate receipts; all actual numerical/model/control results remain exact.
        for key in ("raw_output", "decoded_output", "candidate_gates", "selected", "control",
                    "longitudinal_pi_research", "emergency_reason", "learned_control_proposed", "raw_nn_seconds",
                    "total_seconds", "cache"):
            assert first.get(key) == second.get(key), key
        if cause == "normal":
            assert first["learned_control_proposed"] is True, first["emergency_reason"]
            assert [r["stage"] for r in ledger.plans] == list(live.STAGES)
        else:
            assert first["learned_control_proposed"] is False
            assert first["control"]["brake"] == 1.0
        assert live.policy_runtime.TemporalSimulationResearchPolicyV2.step is live._original_step
    finally:
        torch.set_num_threads(saved_threads)


def _actual_owner_guard(tmp_path, schema):
    """HH_261004 - Produce real current process generations and an actual local socket for the unmocked guard."""
    run = tmp_path / "guard"; run.mkdir()
    declaration = run / "declaration.json"
    declaration.write_text(json.dumps(dict(schema=schema, town="Town03", scenario="straight", run_id=owner.RUN_ID,
        original_navigation_sha256=live.ROUTE_SHA, output=str(run)), sort_keys=True) + "\n")
    expected = owner.digest(declaration)
    current, parent = owner.base.process_record(os.getpid()), owner.base.process_record(os.getppid())
    (run / "owner_started.json").write_text(json.dumps(dict(declaration_sha256=expected,
        owner=dict(pid=parent["pid"], start_ticks=parent["start_ticks"]),
        server=dict(pid=current["pid"], start_ticks=current["start_ticks"]),
        socket_owner=dict(pid=current["pid"], start_ticks=current["start_ticks"], pgid=current["pid"]))) + "\n")
    return SimpleNamespace(output=run / "episode", research_source_sha256=expected), expected


@pytest.mark.parametrize("failure", [False, True])
def test_complete_actual_collector_activation_and_factory_restores_aliases(tmp_path, failure):
    args = arguments(tmp_path)
    model, patches = fake_model_and_patches()
    before = (live.previous.SCHEMA, live.previous.OWNER_SCHEMA, live.straight.cloned_collect_episode,
              live.policy_runtime.TemporalSimulationResearchPolicyV2.step, live.collector.exact_camera_bundle)
    def run():
        with owner_scope(), entered(patches), live.activated(model, args):
            assert live.previous._actual_fixture_owner_schema() == live.OWNER_SCHEMA
            assert live.straight.cloned_collect_episode is live.callback_capture_factory
            assert live.policy_runtime.TemporalSimulationResearchPolicyV2.step is not live._original_step
            assert live.policy_runtime.TemporalSimulationResearchPolicyV2.step.__globals__["trajectory_control"] \
                is preview.trajectory_control
            bundle = live.collector.exact_camera_bundle
            recorder = live.previous.previous._recorder()
            # HH_261004 - Construct the real copied factory, without starting CARLA or invoking a fake capture.
            factory = live.callback_capture_factory()
            assert callable(factory) and live._ledger.capture_started is False
            assert live.collector.exact_camera_bundle is bundle and live.previous.previous._recorder() is recorder
            if failure:
                raise RuntimeError("actual scope test failure")
    if failure:
        with pytest.raises(RuntimeError, match="actual scope test failure"):
            run()
    else:
        run()
    assert before == (live.previous.SCHEMA, live.previous.OWNER_SCHEMA, live.straight.cloned_collect_episode,
                      live.policy_runtime.TemporalSimulationResearchPolicyV2.step, live.collector.exact_camera_bundle)
    assert live._active is False and live._ledger is None and not (tmp_path / "episode.partial").exists()


def test_actual_compiled_camera_registration_uses_real_owned_callback_helper(tmp_path):
    """HH_261004 - Execute the exact instrumented listen expression with the actual compiled provider namespace."""
    args, (model, patches) = arguments(tmp_path), fake_model_and_patches()
    with owner_scope(), entered(patches), live.activated(model, args):
        factory = live.callback_capture_factory()
        function = inspect.getclosurevars(factory).nonlocals["function"]
        original = next(n for n in ast.parse(Path(live.collector.__file__).read_bytes()).body
                        if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
        changed = live.instrument_callback_AST(original)
        listen = next(n for n in ast.walk(changed) if isinstance(n, ast.Expr)
                      and ast.unparse(n).startswith("sensor.listen(timed_camera_callback("))
        code = compile(ast.fix_missing_locations(ast.Module(body=[listen], type_ignores=[])),
                       "CPU_ONLY_actual_original_listen_expression", "exec")
        queues = {role: SimpleQueue() for role in live.collector.MODEL_CAMERA_ORDER}
        for i, role in enumerate(live.collector.MODEL_CAMERA_ORDER):
            sensor = SimpleNamespace(id=17 + i)
            sensor.listen = lambda callback, actor=sensor: setattr(actor, "actual_callback", callback)
            namespace = dict(function.__globals__, spec=SimpleNamespace(name=role), sensor=sensor, queues=queues)
            exec(code, namespace)
            source = image()
            assert sensor.actual_callback(source) is None
            assert queues[role].get_nowait() is source
        assert len(live._ledger.callbacks) == 6 and len(live._ledger.bindings) == 6
        assert all(r["source_byte_count"] == 360 * 640 * 4 for r in live._ledger.callbacks)
        assert all(r["original_queue_put_calls"] == 1 for r in live._ledger.callbacks)


def test_capture_start_marker_denies_foreign_path_repeat_or_unowned_scope(tmp_path):
    ledger = live.CallbackLedger(object(), tmp_path / "episode.partial")
    with mock.patch.object(live, "_active", True), mock.patch.object(live, "_ledger", ledger):
        with pytest.raises(Exception, match="output-scoped"):
            live.callback_capture_started(tmp_path / "different.partial")
        assert ledger.capture_started is False
        live.callback_capture_started(ledger.output)
        assert ledger.capture_started is True
        with pytest.raises(Exception, match="output-scoped"):
            live.callback_capture_started(ledger.output)
    with pytest.raises(Exception, match="output-scoped"):
        live.callback_capture_started(ledger.output)


def test_actual_fixture_guard_install_and_restore_uses_new_schema_and_real_PID_socket_evidence(tmp_path):
    args, (model, patches) = arguments(tmp_path), fake_model_and_patches()
    with owner_scope(), entered(patches), live.activated(model, args):
        fixture_args, expected = _actual_owner_guard(tmp_path, live.OWNER_SCHEMA)
        sock = socket.socket(); sock.bind(("127.0.0.1", 0)); sock.listen(1)
        saved = live.fixture.numeric._owner_plan_sha
        live.fixture.numeric._owner_plan_sha = expected
        try:
            with mock.patch.object(owner.base, "PORT", sock.getsockname()[1]):
                live.fixture.assert_owned_capture(fixture_args)
                world, manifest = FakeWorld(), {}
                original = world.light.state
                fixture = live.fixture.GreenSignalFixture(world, fixture_args, manifest, SimpleNamespace(name="Green"))
                fixture.install()
                assert fixture.record["schema"] == live.SCHEMA + ".owned_green_signal_fixture"
                assert world.light.state.name == "Green"
                fixture.restore()
                assert fixture.restored and world.light.state is original and world.light.is_frozen() is False
        finally:
            live.fixture.numeric._owner_plan_sha = saved
            sock.close()


@pytest.mark.parametrize("fault", ["SHA", "schema", "PID"])
def test_actual_fixture_denies_wrong_owner_before_any_mutation(tmp_path, fault):
    args, (model, patches) = arguments(tmp_path), fake_model_and_patches()
    with owner_scope(), entered(patches), live.activated(model, args):
        schema = owner.previous.V2_OWNER_SCHEMA if fault == "schema" else live.OWNER_SCHEMA
        fixture_args, expected = _actual_owner_guard(tmp_path, schema)
        if fault == "SHA":
            fixture_args.research_source_sha256 = "f" * 64
            expected = fixture_args.research_source_sha256
        elif fault == "PID":
            path = Path(fixture_args.output).parent / "owner_started.json"
            value = json.loads(path.read_text()); current = owner.base.process_record(os.getpid())
            value["owner"] = dict(pid=current["pid"], start_ticks=current["start_ticks"])
            path.write_text(json.dumps(value) + "\n")
        saved = live.fixture.numeric._owner_plan_sha; live.fixture.numeric._owner_plan_sha = expected
        world, manifest = FakeWorld(), {}
        try:
            with pytest.raises(Exception):
                live.fixture.GreenSignalFixture(world, fixture_args, manifest, SimpleNamespace(name="Green"))
        finally:
            live.fixture.numeric._owner_plan_sha = saved
        assert world.frame == 100 and world.light.state.name == "Red" and manifest == {}


def test_bad_factory_policy_or_parent_source_is_rejected_before_activation(tmp_path, monkeypatch):
    args = arguments(tmp_path); model, patches = fake_model_and_patches()
    with owner_scope(), entered(patches):
        with mock.patch.object(live.straight, "cloned_collect_episode", lambda: None):
            with pytest.raises(Exception, match="unknown factory/policy"):
                with live.activated(model, args):
                    pytest.fail("foreign factory admitted")
        with mock.patch.object(live.policy_runtime.TemporalSimulationResearchPolicyV2, "step", lambda self: None):
            with pytest.raises(Exception, match="unknown factory/policy"):
                with live.activated(model, args):
                    pytest.fail("foreign policy admitted")
        with mock.patch.object(live, "PARENT_SHA", "0" * 64):
            with pytest.raises(Exception, match="v3 collector changed"):
                with live.activated(model, args):
                    pytest.fail("wrong source SHA admitted")
    assert not live._active and live._ledger is None and not (tmp_path / "episode.partial").exists()


def synthetic_complete_receipts():
    """HH_261004 - Synthetic CPU protocol fixtures are not CARLA evidence or feature approval."""
    roles = tuple(live.collector.MODEL_CAMERA_ORDER)
    sha = hashlib.sha256(b"bgra8" + b"\x03" * (360 * 640 * 4)).hexdigest()
    callbacks, cameras, bundles, marks, commands, timing = [], [], [], [], [], []
    # HH_261004 - Include all six bootstrap callbacks, not just the later selected camera frames.
    for frame in range(451):
        stamp = frame * 100_000_000
        sources = []
        for offset, role in enumerate(roles):
            entered_ns = 1_000_000_000 + stamp + offset * 100_000
            row = dict(role=role, actor_id=17 + offset, image_object_id=frame * 10 + offset,
                receipt_sequence=len(callbacks) + 1, native_frame=frame,
                source_image_timestamp_simulation_ns=stamp, callback_enter_monotonic_ns=entered_ns,
                queue_put_started_monotonic_ns=entered_ns + 1, queue_put_finished_monotonic_ns=entered_ns + 2,
                source_SHA_finished_monotonic_ns=entered_ns + 3,
                source_BGRA_encoding_prefixed_sha256=sha, source_byte_count=360 * 640 * 4,
                original_queue_put_calls=1, original_queue_put_returned=True,
                original_queue_put_error=None, receipt_error=None)
            callbacks.append(row)
            sources.append({k: row[k] for k in ("role", "native_frame", "image_object_id", "source_image_timestamp_simulation_ns")})
        if frame:
            cameras.append(dict(frame=frame, timestamp=stamp / 1.e9, camera_order=list(roles),
                source_timestamps={role: stamp / 1.e9 for role in roles}))
            bundles.append(dict(native_frame=frame, cameras=sources,
                bundle_binding_finished_monotonic_ns=1_000_000_000 + stamp + 1_000_000))
    for index in range(1, 905):
        timing.append(dict(ACK_sequence=index))
    for frame in (10, 11, 12):
        stamp, start, sequence = frame * 100_000_000, 1_000_000_000 + frame * 100_000_000, frame
        for offset, stage in enumerate(live.STAGES):
            marks.append(dict(policy_object_id=123, frame_timestamp_simulation_ns=stamp,
                stage=stage, stage_finished_monotonic_ns=start + (10 + offset * 10) * 1_000_000,
                original_policy_math_changed=False))
        commands.append(dict(inference_frame=frame, command_frame=frame, ack_receipt_sequence=sequence,
            learned_control_proposed=True, held_between_camera_frames=False,
            owned_camera_history=dict(source_camera_frames=[dict(timestamp_ns=(frame - i) * 100_000_000,
                camera_sha256=[sha] * 6) for i in (8, 6, 4, 2, 0)])))
        timing[sequence - 1] = dict(ACK_sequence=sequence, inference_frame=frame, command_frame=frame,
            learned_control_proposed=True, held_between_camera_frames=False, ACK_server_accepted=True,
            control_source="learned_trajectory_tracker", proposal_started_monotonic_ns=start + 5_000_000,
            proposal_finished_monotonic_ns=start + 35_000_000, ACK_started_monotonic_ns=start + 40_000_000,
            ACK_finished_monotonic_ns=start + 50_000_000,
            observed_bundle=dict(exact_bundle_delivery_monotonic_ns=start + 1_000_000))
    value = dict(schema=live.SCHEMA + ".callback_receipts", clock="time.perf_counter_ns_monotonic",
        hardware_exposure_clock_observed=False, hardware_exposure_to_ACK_proven=False,
        ten_hz_wall_rate_proven=False, inflight_callbacks=0, errors=[], scalar_receipt_limit_exceeded=False,
        all_callbacks_including_bootstrap_or_unconsumed_preserved=True,
        source_math_inverse_AST_verified=True, callback_count=len(callbacks), callbacks=callbacks,
        camera_bindings=[dict(role=role, actor_id=17 + i) for i, role in enumerate(roles)],
        bundles=bundles, policy_stage_marks=marks)
    return value, cameras, commands, timing, dict(inference_count=3)


def test_complete_synthetic_join_preserves_all_callbacks_history_SHAs_and_measured_wall_distinction():
    result = live.audit_receipts(*synthetic_complete_receipts())
    assert result["callback_count"] == 2706 and result["selected_callback_count"] == 2700
    assert result["extra_unselected_callback_count"] == 6
    assert result["callback_to_temporal_input_SHA_comparison_count"] == 3 * 5 * 6
    assert result["fresh_NN_plan_count"] == 3 and len(result["rows"]) == 3
    assert result["metrics"]["callback_first_to_plan_seconds"]["maximum"] == .03
    assert result["metrics"]["callback_first_to_ACK_seconds"]["maximum"] == .05
    assert result["observed_decoded_plan_wall_rate_hz"] == pytest.approx(10.)
    assert result["hardware_exposure_to_ACK_proven"] is False and result["ten_hz_wall_rate_proven"] is False
    assert result["full_RTE01_approved"] is False and result["training_data_approved"] is False


@pytest.mark.parametrize("fault", ["missing_callback", "duplicate_callback", "wrong_object", "wrong_SHA",
    "wrong_past_SHA", "wrong_stamp", "wrong_actor", "held", "missing_stage", "duplicate_stage",
    "time_reversal", "fake_exposure", "fake_rate", "inflight", "missing_ACK", "unaccepted_ACK"])
def test_complete_synthetic_join_denies_every_identity_clock_and_denominator_fault(fault):
    value, cameras, commands, timing, counts = synthetic_complete_receipts()
    if fault == "missing_callback": value["callbacks"].pop(60); value["callback_count"] -= 1
    elif fault == "duplicate_callback":
        value["callbacks"][61]["native_frame"] = value["callbacks"][55]["native_frame"]
        value["callbacks"][61]["image_object_id"] = value["callbacks"][55]["image_object_id"]
        value["callbacks"][61]["source_image_timestamp_simulation_ns"] = value["callbacks"][55]["source_image_timestamp_simulation_ns"]
    elif fault == "wrong_object": value["bundles"][9]["cameras"][0]["image_object_id"] += 1
    elif fault == "wrong_SHA": value["callbacks"][60]["source_BGRA_encoding_prefixed_sha256"] = "a" * 64
    elif fault == "wrong_past_SHA": commands[0]["owned_camera_history"]["source_camera_frames"][0]["camera_sha256"][0] = "a" * 64
    elif fault == "wrong_stamp": value["callbacks"][60]["source_image_timestamp_simulation_ns"] += 1
    elif fault == "wrong_actor": value["callbacks"][60]["actor_id"] += 99
    elif fault == "held": commands[0]["held_between_camera_frames"] = True
    elif fault == "missing_stage": value["policy_stage_marks"].pop(2)
    elif fault == "duplicate_stage": value["policy_stage_marks"].append(dict(value["policy_stage_marks"][0]))
    elif fault == "time_reversal": value["policy_stage_marks"][0]["stage_finished_monotonic_ns"] = 1
    elif fault == "fake_exposure": value["hardware_exposure_clock_observed"] = True
    elif fault == "fake_rate": value["ten_hz_wall_rate_proven"] = True
    elif fault == "inflight": value["inflight_callbacks"] = 1
    elif fault == "missing_ACK": timing.pop()
    elif fault == "unaccepted_ACK": timing[9]["ACK_server_accepted"] = False
    with pytest.raises(Exception):
        live.audit_receipts(value, cameras, commands, timing, counts)


def test_callback_gate_failure_is_not_hidden_by_inherited_bundle_to_ACK_pass():
    value, cameras, commands, timing, counts = synthetic_complete_receipts()
    # HH_261004 - Keep every source and fresh row; make one real-clock protocol interval exceed the new fixed threshold.
    timing[9]["ACK_finished_monotonic_ns"] += 100_000_000
    result = live.audit_receipts(value, cameras, commands, timing, counts)
    assert result["callback_plan_ACK_chain_passed"] is True
    assert result["fresh_callback_to_plan_wall100ms_passed"] is True
    assert result["fresh_callback_to_ACK_wall100ms_passed"] is False
    assert len(result["rows"]) == 3 and result["metrics"]["callback_first_to_ACK_seconds"]["maximum"] == .15


def test_create_only_fixed_plan_and_source_closure_contains_all_three_new_files_without_output():
    output, declaration = owner.paths(owner.RUN_ID)
    assert not output.exists() and not declaration.exists()
    with owner.activated(good6000_pins()):
        plan = owner.base.build_plan(owner.RUN_ID, True)
    assert plan["schema"] == live.OWNER_SCHEMA and plan["run_id"] == owner.RUN_ID
    assert plan["worker_command"][plan["worker_command"].index("-m") + 1] == owner.MODULE
    for module in (live, owner):
        path = Path(module.__file__).resolve()
        assert plan["source_sha256"][str(path.relative_to(owner.ROOT))] == owner.digest(path)
    assert plan["source_sha256"][str(owner.TEST_SOURCE.relative_to(owner.ROOT))] == owner.digest(owner.TEST_SOURCE)
    assert plan["archived_v3_reference"]["overwritten"] is False
    assert plan["failed_v1_archived_reference"]["overwritten"] is False
    assert plan["failed_v2_archived_reference"]["overwritten"] is False
    assert plan["failed_callback45_v1_reference"]["overwritten"] is False
    assert plan["actual_callback_timing"]["callback_receipt_JSON_maximum_bytes"] == 16 * 1024**2
    assert plan["actual_callback_timing"]["generic_JSON_reader_limit_changed"] is False
    assert plan["actual_callback_timing"]["original_same_recorder_bundle_provider_replaced"] is False
    assert plan["actual_callback_timing"]["first_callback_to_plan_maximum_seconds"] == .1
    assert plan["actual_callback_timing"]["first_callback_to_ACK_maximum_seconds"] == .1
    assert plan["maximum_native_state_count"] == 900 and plan["maximum_camera_bundle_count"] == 450
    assert plan["body_clearance_screen"]["lane_inset_m"] == .1
    assert plan["speed_preview_s"] == .5 and plan["controller_target_cap_mps"] == 2.4
    assert not output.exists() and not declaration.exists()


def test_missing_callback_receipts_keep_original_screen_separate_and_fail_new_combined_screen(tmp_path, monkeypatch):
    original = dict(learned_driving_screen_passed=True, straight_motion_body_policy_wall100ms_passed=True)
    monkeypatch.setattr(owner, "_parent_diagnostic", lambda *args: dict(original))
    result = owner.diagnostic_report(tmp_path, {}, {})
    assert result["straight_motion_body_policy_wall100ms_passed"] is True
    assert result["inherited_straight_motion_body_policy_wall100ms_passed"] is True
    assert result["inherited_learned_driving_screen_passed"] is True
    assert result["callback_straight_motion_body_policy_wall100ms_passed"] is False
    assert result["learned_driving_screen_passed"] is False and result["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    assert result["actual_six_camera_callback_observed"] is False
    assert result["hardware_exposure_to_ACK_proven"] is False and result["ten_hz_wall_rate_proven"] is False
    assert result["full_RTE01_approved"] is False and result["full_MAIN9_completion_claimed"] is False


def test_callback_only_reader_accepts_actual_frozen_v1_all5692244_bytes_without_regrading_or_modifying_it():
    """HH_261004 - Read actual frozen callback bytes on CPU only; the historical FAIL and generic JSON limit stay intact."""
    path = owner.CALLBACK_V1_OUTPUT / "episode.partial/callback_timing_receipts.json"
    expected = owner.CALLBACK_V1_EVIDENCE_SHA["episode.partial/callback_timing_receipts.json"]
    value = owner.read_callback_receipts(path, expected,
        schema=live.SCHEMA.replace("callback45_v3", "callback45_v1") + ".callback_receipts")
    assert path.stat().st_size == 5_692_244 and value["callback_count"] == 5405
    assert len(value["bundles"]) == 450 and len(value["policy_stage_marks"]) == 415 * 3
    with pytest.raises(RuntimeError, match="bounded JSON"):
        owner.read_json(path)
    assert owner.digest(path) == expected
    assert owner._verify_callback_v1_failure()["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"


@pytest.mark.parametrize("fault", ["none", "SHA", "schema", "callback_count", "boolean_count", "too_many_callbacks",
    "too_many_bindings", "too_many_bundles", "too_many_stages", "partial_stages", "bad_six_camera_bundle",
    "nonfinite", "duplicate_key", "oversize", "symlink_file", "symlink_parent"])
def test_callback_reader_enforces_only_its_explicit_bounded_ABI_and_authenticated_bytes(tmp_path, fault):
    value, *_ = synthetic_complete_receipts()
    path = tmp_path / "callback_timing_receipts.json"
    if fault == "schema": value["schema"] = "foreign.callback_receipts"
    elif fault == "callback_count": value["callback_count"] -= 1
    elif fault == "boolean_count": value["callback_count"] = True
    elif fault == "too_many_callbacks": value["callbacks"] *= 5; value["callback_count"] = len(value["callbacks"])
    elif fault == "too_many_bindings": value["camera_bindings"].append(dict(value["camera_bindings"][0]))
    elif fault == "too_many_bundles": value["bundles"].append(dict(value["bundles"][0]))
    elif fault == "too_many_stages": value["policy_stage_marks"] *= 151
    elif fault == "partial_stages": value["policy_stage_marks"].pop()
    elif fault == "bad_six_camera_bundle": value["bundles"][0]["cameras"].pop()
    payload = json.dumps(value).encode()
    if fault == "nonfinite": payload = payload.replace(b'"inflight_callbacks": 0', b'"inflight_callbacks": NaN')
    elif fault == "duplicate_key": payload = payload[:-1] + b', "schema": "duplicate"}'
    elif fault == "oversize": payload = b" " * (owner.CALLBACK_JSON_MAX_BYTES + 1)
    path.write_bytes(payload)
    expected = hashlib.sha256(payload).hexdigest()
    if fault == "SHA": expected = "0" * 64
    elif fault == "symlink_file":
        original = tmp_path / "original.json"; path.rename(original); path.symlink_to(original)
    elif fault == "symlink_parent":
        alias = tmp_path / "alias"; alias.symlink_to(tmp_path, target_is_directory=True); path = alias / path.name
    if fault == "none":
        assert owner.read_callback_receipts(path, expected) == value
    else:
        with pytest.raises((RuntimeError, OSError, ValueError)):
            owner.read_callback_receipts(path, expected)


def test_archived_callback_v1_sources_and_full_failure_are_in_new_plan_closure():
    with owner.activated(good6000_pins()):
        plan = owner.base.build_plan(owner.RUN_ID, True)
    assert plan["failed_callback45_v1_reference"]["sha256"] == owner.CALLBACK_V1_EVIDENCE_SHA
    assert plan["failed_callback45_v1_reference"]["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    for name, sha in owner.CALLBACK_V1_SOURCE_SHA.items():
        assert plan["source_sha256"][name] == sha == owner.digest(owner.ROOT / name)


@pytest.mark.parametrize("pixels", ["same", "different"])
@pytest.mark.parametrize("clock_offset", [-500_000_000, 500_000_000])
@pytest.mark.parametrize("native_frame", [10, 999])
def test_repeated_callbacks_preserve_distinct_sources_without_selecting_a_favorable_clock(pixels, clock_offset, native_frame):
    """HH_261004 - Same frame/stamp is not source identity; late/early unconsumed receipts never replace consumed objects."""
    value, cameras, commands, timing, counts = synthetic_complete_receipts()
    repeated = dict(value["callbacks"][60])
    repeated["image_object_id"] += 100_000
    repeated["native_frame"] = native_frame
    repeated["receipt_sequence"] = len(value["callbacks"]) + 1
    for key in ("callback_enter_monotonic_ns", "queue_put_started_monotonic_ns",
                "queue_put_finished_monotonic_ns", "source_SHA_finished_monotonic_ns"):
        repeated[key] += clock_offset
    if pixels == "different":
        repeated["source_BGRA_encoding_prefixed_sha256"] = "a" * 64
    value["callbacks"].append(repeated); value["callback_count"] += 1
    result = live.audit_receipts(value, cameras, commands, timing, counts)
    assert result["callback_count"] == 2707 and result["extra_unselected_callback_count"] == 7
    assert result["metrics"]["callback_first_to_plan_seconds"]["maximum"] == .03
    assert result["metrics"]["callback_first_to_ACK_seconds"]["maximum"] == .05
    assert result["rows"][0]["selected_source_callback_receipt_sequences"][0] == 61
    assert result["rows"][0]["selected_source_image_object_ids"][0] == value["callbacks"][60]["image_object_id"]
    assert result["latest_or_earliest_repeated_callback_chosen_by_latency"] is False
    census = result["repeated_callback_receipt_census"]
    assert census["same_role_simstamp_extra_callback_count"] == 1
    assert census["same_role_native_frame_extra_callback_count"] == int(native_frame == 10)
    assert census["repeated_frame_groups_with_different_SHA_count"] == int(native_frame == 10 and pixels == "different")


@pytest.mark.parametrize("selected", [False, True])
def test_same_role_frame_object_receipt_ambiguity_is_denied_even_for_unconsumed_images(selected):
    value, cameras, commands, timing, counts = synthetic_complete_receipts()
    repeated = dict(value["callbacks"][60 if selected else 0])
    repeated["receipt_sequence"] = len(value["callbacks"]) + 1
    value["callbacks"].append(repeated); value["callback_count"] += 1
    with pytest.raises(Exception, match="ambiguous image-object reuse"):
        live.audit_receipts(value, cameras, commands, timing, counts)


@pytest.mark.parametrize("fault", ["actor", "stamp", "SHA", "binding_object", "history_stamp", "history_SHA"])
def test_forged_past_source_cannot_join_an_unconsumed_repeated_receipt(fault):
    value, cameras, commands, timing, counts = synthetic_complete_receipts()
    original = value["callbacks"][12]  # HH_261004 - First fresh decision genuinely consumes frame2 as its oldest history input.
    repeated = dict(original, image_object_id=original["image_object_id"] + 100_000,
        receipt_sequence=len(value["callbacks"]) + 1)
    value["callbacks"].append(repeated); value["callback_count"] += 1
    if fault == "actor": original["actor_id"] += 1
    elif fault == "stamp": original["source_image_timestamp_simulation_ns"] += 1
    elif fault == "SHA": original["source_BGRA_encoding_prefixed_sha256"] = "a" * 64
    elif fault == "binding_object": value["bundles"][1]["cameras"][0]["image_object_id"] += 1
    elif fault == "history_stamp": commands[0]["owned_camera_history"]["source_camera_frames"][0]["timestamp_ns"] = 0
    elif fault == "history_SHA": commands[0]["owned_camera_history"]["source_camera_frames"][0]["camera_sha256"][0] = "a" * 64
    with pytest.raises(Exception):
        live.audit_receipts(value, cameras, commands, timing, counts)


def actual_v2_readonly_inputs():
    """HH_261004 - Adapt only an in-memory schema label for an independent CPU join, never rewrite or regrade v2 files."""
    episode = owner.CALLBACK_V2_OUTPUT / "episode.partial"
    value = owner.read_callback_receipts(episode / "callback_timing_receipts.json",
        owner.CALLBACK_V2_EVIDENCE_SHA["episode.partial/callback_timing_receipts.json"],
        schema=live.SCHEMA.replace("callback45_v3", "callback45_v2") + ".callback_receipts")
    value["schema"] = live.SCHEMA + ".callback_receipts"
    cameras, commands, timing = (owner._jsonl(episode / name) for name in
        ("camera_frames.jsonl", "learned_control.jsonl", "wall_timing.jsonl"))
    return value, cameras, commands, timing, owner.read_json(episode / "learned_summary.json")["counts"]


def test_actual_v2_complete_source_object_join_and_all_failed_latency_tails_are_retained():
    result = live.audit_receipts(*actual_v2_readonly_inputs())
    assert result["callback_count"] == 5405 and result["selected_callback_count"] == 2700
    assert result["extra_unselected_callback_count"] == 2705 and result["fresh_NN_plan_count"] == 415
    assert result["callback_to_temporal_input_SHA_comparison_count"] == 12_450
    assert result["exact_consumed_image_object_join_passed"] is True
    assert result["consumed_history_bundle_identity_and_SHA_passed"] is True
    census = result["repeated_callback_receipt_census"]
    assert census["same_role_native_frame_group_count"] == 2231
    assert census["same_role_native_frame_extra_callback_count"] == 2231
    assert census["same_role_simstamp_extra_callback_count"] == 2620
    assert census["repeated_frame_groups_with_different_SHA_count"] == 2231
    assert census["repeated_frame_groups_with_identical_SHA_only_count"] == 0
    assert census["maximum_callbacks_per_role_native_frame"] == 2
    assert result["metrics"]["callback_first_to_plan_seconds"]["maximum"] == .109828826
    assert result["metrics"]["callback_first_to_ACK_seconds"]["maximum"] == .111276636
    assert result["fresh_callback_to_plan_wall100ms_passed"] is False
    assert result["fresh_callback_to_ACK_wall100ms_passed"] is False
    assert len(result["rows"]) == 415
    assert result["hardware_exposure_to_ACK_proven"] is False and result["ten_hz_wall_rate_proven"] is False
    assert owner._verify_callback_v2_failure()["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    assert owner.digest(owner.CALLBACK_V2_OUTPUT / "episode.partial/callback_timing_receipts.json") \
        == owner.CALLBACK_V2_EVIDENCE_SHA["episode.partial/callback_timing_receipts.json"]


def test_new_plan_preserves_all_v2_failed_sources_and_prospective_identity_join():
    with owner.activated(good6000_pins()):
        plan = owner.base.build_plan(owner.RUN_ID, True)
    assert plan["failed_callback45_v2_reference"]["sha256"] == owner.CALLBACK_V2_EVIDENCE_SHA
    assert plan["failed_callback45_v2_reference"]["overwritten"] is False
    for name, sha in owner.CALLBACK_V2_SOURCE_SHA.items():
        assert plan["source_sha256"][name] == sha == owner.digest(owner.ROOT / name)
    contract = plan["actual_callback_timing"]
    assert contract["selected_source_exactly_one_original_callback_required"] is True
    assert contract["causal_history_joins_consumed_bundle_identity_and_SHA_only"] is True
    assert contract["earliest_or_latest_duplicate_receipt_selected_by_latency"] is False
    assert contract["first_callback_to_plan_maximum_seconds"] == .1
    assert contract["first_callback_to_ACK_maximum_seconds"] == .1


def synthetic_complete_native_marks():
    """HH_261004 - Synthetic scalar protocol only, not a simulator tick, camera capture or driving capability."""
    value, cameras, commands, timings, counts = synthetic_complete_receipts()
    states, marks = [], []
    for index in range(900):
        frame, phase = index + 1, "stationary_warmup" if index < 2 else "driving"
        states.append(dict(frame=frame, capture_phase=phase))
        start = 1_000_000_000 + frame * 100_000_000 - 1_000_000
        for offset, stage in enumerate(live.NATIVE_STAGES[:7]):
            marks.append(dict(tick_index=index, capture_phase=phase,
                native_frame=None if stage == "world_tick_started" else frame, stage=stage,
                stage_finished_monotonic_ns=start + offset * 100_000,
                agent_run_step_invoked=phase == "driving" if stage in
                    ("agent_branch_started", "agent_branch_finished") else None,
                original_native_order_or_math_changed=False))
        if frame <= 450:
            for stage, offset in (("exact_bundle_started", 700_000), ("exact_bundle_finished", 2_000_000)):
                marks.append(dict(tick_index=index, capture_phase=phase, native_frame=frame, stage=stage,
                    stage_finished_monotonic_ns=start + offset, agent_run_step_invoked=None,
                    original_native_order_or_math_changed=False))
    value["native_stage_marks"] = marks
    return value, states, cameras, commands, timings, counts


def test_native_actual_collector_AST_adds_only_exact_scalar_boundaries_and_uses_no_invented_frame():
    original = next(n for n in ast.parse(Path(live.collector.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
    changed = live.instrument_callback_AST(original)
    calls = [node for node in ast.walk(changed) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "native_timing_marker"]
    assert len(calls) == 11  # HH_261004 - Seven native boundaries and two markers in each mutually exclusive bundle branch.
    stages = [node.args[3].value for node in calls]
    assert stages.count("world_tick_started") == 1
    assert stages.count("world_tick_returned") == 1
    assert stages.count("snapshot_bound") == 1
    assert stages.count("actor_state_ready") == 1
    assert stages.count("agent_branch_started") == 1 and stages.count("agent_branch_finished") == 1
    assert stages.count("native_state_finished") == 1
    assert stages.count("exact_bundle_started") == 2 and stages.count("exact_bundle_finished") == 2
    beginning = next(node for node in calls if node.args[3].value == "world_tick_started")
    assert isinstance(beginning.args[2], ast.Constant) and beginning.args[2].value is None
    assert "wait" not in live.native_timing_marker.__code__.co_names


def test_native_scalar_marker_uses_actual_clock_and_original_branch_values(tmp_path):
    ledger = live.CallbackLedger(object(), tmp_path, iter((100, 110)).__next__)
    ledger.native_mark(0, "stationary_warmup", None, "world_tick_started")
    ledger.native_mark(0, "stationary_warmup", 19, "agent_branch_finished", False)
    assert ledger.native_marks[0]["native_frame"] is None
    assert ledger.native_marks[1]["native_frame"] == 19
    assert ledger.native_marks[1]["agent_run_step_invoked"] is False
    assert [row["stage_finished_monotonic_ns"] for row in ledger.native_marks] == [100, 110]
    assert all(row["original_native_order_or_math_changed"] is False for row in ledger.native_marks)


def test_complete_native_stage_protocol_preserves_all900_states_and7200_marks_and_reports_signed_clock_relationships():
    value, states, cameras, commands, timings, counts = synthetic_complete_native_marks()
    callback = live.audit_receipts(value, cameras, commands, timings, counts)
    result = live.audit_native_markers(value, states, cameras, timings, callback)
    assert result["native_pre_delivery_stage_closure_passed"] is True
    assert result["native_tick_count"] == 900 and result["native_camera_tick_count"] == 450
    assert result["native_scalar_mark_count"] == 7200
    assert len(result["full_native_rows"]) == 900 and len(result["all_fresh_critical_path_rows"]) == 3
    assert result["native_frame_invented_before_tick_return"] is False
    assert result["CPU_scheduling_or_external_load_cause_proven"] is False
    assert result["hardware_exposure_to_ACK_proven"] is False


@pytest.mark.parametrize("fault", ["missing", "duplicate", "frame", "phase", "invented_start_frame",
    "reversed_clock", "missing_state", "agent_branch", "bundle_delivery", "too_many_reader_marks"])
def test_native_stage_closure_denies_incomplete_wrong_frame_phase_clock_and_branch_receipts(tmp_path, fault):
    value, states, cameras, commands, timings, counts = synthetic_complete_native_marks()
    callback = live.audit_receipts(value, cameras, commands, timings, counts)
    marks = value["native_stage_marks"]
    if fault == "missing": marks.pop()
    elif fault == "duplicate": marks[1] = dict(marks[0])
    elif fault == "frame": marks[1]["native_frame"] += 1
    elif fault == "phase": marks[1]["capture_phase"] = "driving"
    elif fault == "invented_start_frame": marks[0]["native_frame"] = 1
    elif fault == "reversed_clock": marks[1]["stage_finished_monotonic_ns"] = 1
    elif fault == "missing_state": states.pop()
    elif fault == "agent_branch": marks[5]["agent_run_step_invoked"] = True
    elif fault == "bundle_delivery": timings[9]["observed_bundle"]["exact_bundle_delivery_monotonic_ns"] = 1
    elif fault == "too_many_reader_marks":
        marks += [dict(marks[0])] * 901
        path = tmp_path / "callback_timing_receipts.json"; path.write_text(json.dumps(value))
        with pytest.raises(RuntimeError, match="bounded native-stage"):
            owner.read_callback_receipts(path, owner.digest(path))
        return
    with pytest.raises(Exception):
        live.audit_native_markers(value, states, cameras, timings, callback)


def test_native_closure_failure_does_not_hide_authenticated_callback_results_or_pass_the_combined_gate(tmp_path, monkeypatch):
    """HH_261004 - Keep stage-specific successes/failures separate in synthetic diagnostics, never claim actual driving."""
    value, cameras, commands, timings, counts = synthetic_complete_receipts()
    episode = tmp_path / "episode.partial"; episode.mkdir()
    callback = episode / "callback_timing_receipts.json"; callback.write_text(json.dumps(value))
    for name, rows in (("camera_frames.jsonl", cameras), ("learned_control.jsonl", commands),
                       ("wall_timing.jsonl", timings), ("states.jsonl", [])):
        (episode / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
    (episode / "learned_summary.json").write_text(json.dumps(dict(counts=counts)))
    original = dict(learned_driving_screen_passed=True, straight_motion_body_policy_wall100ms_passed=True,
        raw_source_sha256={str(callback): owner.digest(callback)})
    monkeypatch.setattr(owner, "_parent_diagnostic", lambda *args: dict(original))
    result = owner.diagnostic_report(tmp_path, {}, {})
    assert result["callback_receipt_identity_and_SHA_passed"] is True
    assert result["callback_plan_ACK_chain_passed"] is True
    assert result["fresh_callback_to_plan_wall100ms_passed"] is True
    assert result["native_pre_delivery_stage_closure_passed"] is False
    assert "all900 native ticks" in result["callback_timing_diagnostic_error"]
    assert result["callback_straight_motion_body_policy_wall100ms_passed"] is False
    assert result["learned_driving_screen_passed"] is False
    assert result["status"] == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    assert result["full_RTE01_approved"] is False
    assert len(result["actual_callback_timing"]["rows"]) == 3
