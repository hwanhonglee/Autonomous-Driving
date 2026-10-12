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

from scripts.e2e import collect_temporal_route_joint_town03_straight_callback45_v1_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_callback45_v1_20261004 as owner
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
        config = ControllerConfig(2.7, .6)
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
        first = before.step(current, **kwargs)
        if cause == "deadline":
            expired_clock.counter = 0
        ledger = live.CallbackLedger(second_model, tmp_path)
        with mock.patch.object(live, "_active", True), mock.patch.object(live, "_ledger", ledger), \
             mock.patch.object(live.policy_runtime.TemporalSimulationResearchPolicyV2, "step", live.timed_policy_step()):
            second = after.step(current, **kwargs)
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
            # HH_261004 - Construct the real copied factory, without starting CARLA or invoking a fake capture.
            factory = live.callback_capture_factory()
            assert callable(factory) and live._ledger.capture_started is False
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
            cameras.append(dict(frame=frame, timestamp=stamp / 1.e9))
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
