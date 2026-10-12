"""HH_261004 - Prove timing-only parity with real decoders and genuine witness evaluation.

All poses below are explicitly synthetic CPU fixtures on the known-DEV catalog.
No fixture is an actual drive or a substitute for closed-loop qualification.
Witness and decoder implementations are never replaced with success stubs.
"""

import ast
import copy
import gc
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.path_time_decoder_v2 import GO, PathTimeDecoderV2
from portable_e2e import temporal_navigation_terminal_policy_v2_20261004 as original
from portable_e2e import temporal_navigation_terminal_policy_v3_timing_20261004 as timing


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("frozen_cpu_geometry_for_stage_timing",
    ROOT / "tools/e2e/benchmark_navigation_terminal_retiming_v1_20261004.py")
geometry_helpers = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry_helpers)


def fixture(support=10., *, stamp=1_000_000_000, speed=2.4, dtype=torch.float32):
    # HH_261004 - Share only synthetic inputs; each real bridge constructs/evaluates its own witness.
    payload = geometry_helpers.read_canonical_regular(ROOT / geometry_helpers.ROUTE_REL)
    geometry = geometry_helpers.route_geometry(json.loads(payload))
    case = dict(source_index=0, actual_state_sha256="a"*64, actual_state=dict(z=.1), stamp_ns=stamp,
        original_camera_sha256=[str(index)*64 for index in range(1, 7)], record=dict(inference_frame=1,
        owned_camera_history=dict(normalized_input_tensor_sha256=dict(ego_history="b"*64))))
    _, _, observation_payload, _, _ = geometry_helpers.counterfactual_payloads(case, support, geometry)
    observation = json.loads(observation_payload)
    frame = SimpleNamespace(timestamp_ns=stamp, pose_timestamp_ns=stamp,
        position_map_m=tuple(observation["position_map_m"]), orientation_xyzw=tuple(observation["orientation_xyzw"]),
        episode_id="SYNTHETIC_CPU_TIMING_NOT_ACTUAL_CAPTURE", inputs=SimpleNamespace(
        route_xy_base_m=tuple(tuple(point) for point in observation["provided_route_xy_base_m"]),
        camera_frames=[SimpleNamespace(timestamp_ns=stamp) for _ in range(6)]))
    window = SimpleNamespace(frames=(None, None, None, None,
        SimpleNamespace(timestamp_ns=stamp, camera_sha256=tuple(case["original_camera_sha256"]))))
    def binding(frame, tensors, window):
        return (dict(synthetic_CPU_fixture=True, actual_observed_native_pose=True, counterfactual_not_observed=False,
            position_map_m=frame.position_map_m, orientation_xyzw=frame.orientation_xyzw, timestamp_ns=frame.pose_timestamp_ns),
            dict(synthetic_CPU_fixture=True, actual_observed_native_motion=True, timestamp_ns=frame.timestamp_ns))
    def policy():
        return SimpleNamespace(decoder=PathTimeDecoderV2(), _last_owned_input_audit=dict(frame_timestamp_ns=stamp,
            every_source_camera_SHA_preserved=True, source_pixels_changed=False,
            normalized_input_tensor_sha256=dict(ego_history="b"*64)))
    old = original.ActualNavigationTerminalDecoderBridge(policy(), payload, binding)
    new = timing.TimedNavigationTerminalDecoderBridge(policy(), payload, binding)
    for bridge in (old, new):
        bridge.bind_current(frame, {}, window, now_ns=stamp, ego_stamp_ns=stamp, route_stamp_ns=stamp)
    args = [torch.linspace(-.01, .01, 33, dtype=dtype).reshape(1, 33),
        torch.linspace(2.2, 2.4, 33, dtype=dtype).reshape(1, 33), torch.tensor([speed], dtype=dtype),
        torch.tensor([GO]), torch.zeros(1, dtype=dtype), torch.tensor([support], dtype=dtype)]
    return old, new, args


def exact_outputs(left, right):
    assert set(left) == set(right)
    for key in left:
        assert left[key].dtype == right[key].dtype and left[key].shape == right[key].shape
        assert torch.equal(left[key], right[key]), key


def ordered_receipt(bridge, expected, status="COMPLETED"):
    receipt = bridge.last_timing
    assert receipt["schema"] == timing.SCHEMA and receipt["status"] == status
    assert tuple(row["stage"] for row in receipt["stages"]) == expected
    assert len(receipt["stages"]) <= len(timing.TERMINAL_STAGES)
    previous = receipt["call_started_monotonic_ns"]
    for row in receipt["stages"]:
        assert type(row["start_monotonic_ns"]) is type(row["finish_monotonic_ns"]) is int
        assert row["start_monotonic_ns"] == previous
        assert row["finish_monotonic_ns"] >= previous
        assert row["elapsed_ns"] == row["finish_monotonic_ns"] - row["start_monotonic_ns"]
        previous = row["finish_monotonic_ns"]
    assert previous == receipt["call_finished_monotonic_ns"]
    assert sum(row["elapsed_ns"] for row in receipt["stages"]) == receipt["total_observed_ns"]
    assert receipt["original_policy_deadline_seconds"] == .1
    assert receipt["original_policy_deadline_unchanged"] is True
    assert receipt["numerical_outputs_changed"] is receipt["weights_changed"] is False
    assert receipt["learned_STOP_claim"] is receipt["vehicle_control_approved"] is False
    assert receipt["main_functions_approved"] == receipt["subfunctions_approved"] == 0
    assert receipt["source_rehashed_during_call"] is False
    json.dumps(receipt, allow_nan=False)


def test_frozen_source_pin_and_removable_AST_are_exact():
    assert timing.FROZEN_BRIDGE_SHA256 == "e4c0118de9d9e420890d1c0f11df4fe469cf9cdb7e969f1cd96d2448798c40ab"
    assert ast.dump(timing._restore(timing._TIMED_CALL_AST)) == ast.dump(timing._ORIGINAL_CALL_AST)
    assert timing.TimedNavigationTerminalDecoderBridge.__call__ is timing._COMPILED_CALL


@pytest.mark.parametrize("support", [17.9, 10., 5., 2.5])
@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_real_terminal_decoding_and_genuine_witness_are_bitwise_identical(support, dtype):
    old, new, args = fixture(support, dtype=dtype)
    exact_outputs(old(*args), new(*args))
    assert old.last_receipt == new.last_receipt
    assert old.previous_witness == new.previous_witness
    assert new.previous_witness.terminal_permission is True
    assert new.last_receipt["witness_decision"]["status"] == "VERIFIED"
    ordered_receipt(new, timing.TERMINAL_STAGES)


@pytest.mark.parametrize("support", [18., 32., 64.])
def test_real_ordinary_decoder_is_exact_and_skips_all_proof_stages(support):
    old, new, args = fixture(support)
    def forbidden_binding(*args):
        pytest.fail("ordinary calls must not invoke terminal source binding")
    old.source_binding = new.source_binding = forbidden_binding
    exact_outputs(old(*args), new(*args))
    assert old.last_receipt == new.last_receipt
    assert old.previous_witness is new.previous_witness is None
    ordered_receipt(new, timing.ORDINARY_STAGES)


def test_per_call_does_not_hash_or_read_frozen_code_sources(monkeypatch):
    old, new, args = fixture()
    monkeypatch.setattr(timing, "_source_tree", lambda: pytest.fail("per-call source compilation denied"))
    monkeypatch.setattr(Path, "read_bytes", lambda path: pytest.fail("per-call source/file read denied"))
    exact_outputs(old(*args), new(*args))
    ordered_receipt(new, timing.TERMINAL_STAGES)


def test_sequential_real_witnesses_retain_cached_mission_and_exact_original_sequence():
    old, new, args = fixture(17.9)
    mission = None
    for sequence, support in enumerate((17.9, 10., 5., 2.5)):
        inputs, _, args = fixture(support, stamp=1_000_000_000+sequence*100_000_000)
        frame, tensors, window, now, ego, route = inputs.current
        for bridge in (old, new):
            bridge.policy._last_owned_input_audit = copy.deepcopy(inputs.policy._last_owned_input_audit)
            bridge.bind_current(frame, tensors, window, now_ns=now, ego_stamp_ns=ego, route_stamp_ns=route)
        exact_outputs(old(*args), new(*args))
        assert old.last_receipt == new.last_receipt and old.previous_witness == new.previous_witness
        assert new.previous_witness.sequence_index == sequence
        if mission is None: mission = new.mission_payload
        assert old.mission_payload == new.mission_payload == mission
        ordered_receipt(new, timing.TERMINAL_STAGES)


@pytest.mark.parametrize("fault,stages", [
    ("unbound", ("entry",)),
    ("bad_support", ("entry",)),
    ("owned_source", ("entry",)),
    ("provenance", ("entry", "source_binding")),
    ("route", timing.TERMINAL_STAGES[:4]),
    ("stale", timing.TERMINAL_STAGES[:7]),
    ("speed", timing.TERMINAL_STAGES[:9]),
    ("ordinary_speed", timing.ORDINARY_STAGES),
])
def test_original_exceptions_keep_identity_message_and_failed_stage_tails(fault, stages):
    old, new, args = fixture(18. if fault == "ordinary_speed" else 10.)
    for bridge in (old, new):
        if fault == "unbound": bridge.current = None
        elif fault == "owned_source": bridge.policy._last_owned_input_audit["frame_timestamp_ns"] += 1
        elif fault == "provenance": bridge.source_binding = lambda *args: (dict(actual_observed_native_pose=False), {})
        elif fault == "route": bridge.current[0].inputs.route_xy_base_m = ((0., 0.), (64., 0.))
        elif fault == "stale":
            frame, tensors, window, now, ego, route = bridge.current
            bridge.bind_current(frame, tensors, window, now_ns=now+150_000_001, ego_stamp_ns=ego, route_stamp_ns=route)
    if fault == "bad_support": args[-1] = torch.tensor([float("nan")])
    elif fault in ("speed", "ordinary_speed"): args[2] = torch.tensor([2.6])
    with pytest.raises(ContractError) as original_error:
        old(*args)
    callbacks = tuple(gc.callbacks)
    with pytest.raises(type(original_error.value)) as timed_error:
        new(*args)
    assert str(timed_error.value) == str(original_error.value)
    assert tuple(gc.callbacks) == callbacks
    assert new.last_timing["exception_type"] == type(original_error.value).__name__
    ordered_receipt(new, stages, "FAILED")
    assert new._hh_timing_active is False


def test_noncontract_exception_is_not_swallowed_or_replaced():
    old, new, args = fixture()
    marker = RuntimeError("same original source-binding exception object")
    def explode(*args):
        raise marker
    old.source_binding = new.source_binding = explode
    for bridge in (old, new):
        with pytest.raises(RuntimeError) as error:
            bridge(*args)
        assert error.value is marker
    ordered_receipt(new, timing.TERMINAL_STAGES[:2], "FAILED")


@pytest.mark.parametrize("binding", [(), (object(),), object(), (SimpleNamespace(),)*6])
def test_malformed_binding_cannot_replace_original_exception_while_formatting_timing(binding):
    old, new, args = fixture()
    old.current = new.current = binding
    with pytest.raises(Exception) as original_error:
        old(*args)
    with pytest.raises(type(original_error.value)) as timed_error:
        new(*args)
    assert str(original_error.value) == str(timed_error.value)
    ordered_receipt(new, ("entry",), "FAILED")


def test_nested_timing_attempt_cannot_destroy_outer_scalar_receipt():
    _, new, args = fixture()
    binding = new.source_binding
    def nested(*arguments):
        with pytest.raises(ContractError, match="serial timing bridge"):
            new(*args)
        return binding(*arguments)
    new.source_binding = nested
    new(*args)
    ordered_receipt(new, timing.TERMINAL_STAGES)


def test_duplicate_witness_failure_is_retained_and_bind_resets_only_timing():
    old, new, args = fixture()
    old(*args); new(*args)
    witness = new.previous_witness
    with pytest.raises(ContractError, match="NONMONOTONIC_OBSERVATION"):
        new(*args)
    ordered_receipt(new, timing.TERMINAL_STAGES[:7], "FAILED")
    frame, tensors, window, now, ego, route = new.current
    new.bind_current(frame, tensors, window, now_ns=now, ego_stamp_ns=ego, route_stamp_ns=route)
    assert new.last_timing is None and new.previous_witness is witness


def test_scoped_gc_observation_preserves_other_callbacks_and_never_changes_GC(monkeypatch):
    # HH_261004 - Notify the observer directly; this synthetic callback test never forces collection.
    callbacks = tuple(gc.callbacks)
    enabled, thresholds = gc.isenabled(), gc.get_threshold()
    monkeypatch.setattr(gc, "collect", lambda *args: pytest.fail("forced GC forbidden"))
    monkeypatch.setattr(gc, "disable", lambda *args: pytest.fail("GC disable forbidden"))
    monkeypatch.setattr(gc, "set_threshold", lambda *args: pytest.fail("GC threshold change forbidden"))
    observer = timing._ScopedGCObservation()
    for _ in range(timing.MAX_GC_EVENTS+3):
        observer.observe("start", dict(generation=0))
        observer.observe("stop", dict(generation=0, collected=1, uncollectable=0))
    value = observer.finish()
    assert len(value["events"]) == timing.MAX_GC_EVENTS and value["overflow_start_count"] == 3
    assert value["actual_start_count"] == value["actual_stop_count"] == timing.MAX_GC_EVENTS+3
    assert value["unmatched_start_count"] == 0
    assert all(row["finish_monotonic_ns"] >= row["start_monotonic_ns"] for row in value["events"])
    assert tuple(gc.callbacks) == callbacks and gc.isenabled() == enabled and gc.get_threshold() == thresholds


def test_gc_observer_does_not_attribute_other_thread_events():
    observer = timing._ScopedGCObservation()
    observer.thread_id = -1
    observer.observe("start", dict(generation=0))
    observer.observe("stop", dict(generation=0, collected=1, uncollectable=0))
    assert observer.finish()["actual_start_count"] == 0


def test_factory_inherits_original_build_and_step_and_does_not_catch_unhandled_exception():
    old, _, args = fixture(18.)
    payload = geometry_helpers.read_canonical_regular(ROOT / geometry_helpers.ROUTE_REL)
    marker = RuntimeError("outer timing attachment cannot swallow original step exception")
    class Base:
        def __init__(self):
            self.identity = ("synthetic", "val")
            self.decoder = PathTimeDecoderV2()
        def build_inputs(self, *args, **kwargs):
            raise marker
        def step(self, *args, **kwargs):
            raise marker
    original_class = original.navigation_terminal_policy_class(Base, payload, old.source_binding)
    timed_class = timing.navigation_terminal_policy_class(Base, payload, old.source_binding)
    assert timed_class.build_inputs.__code__ is original_class.build_inputs.__code__
    assert timed_class.__mro__[1].step.__code__ is original_class.step.__code__
    policy = timed_class()
    assert isinstance(policy.decoder, timing.TimedNavigationTerminalDecoderBridge)
    assert isinstance(policy.decoder.ordinary_decoder, PathTimeDecoderV2)
    with pytest.raises(RuntimeError) as error:
        policy.step()
    assert error.value is marker


def test_factory_attaches_research_timing_to_original_return_without_control_changes():
    old, _, args = fixture(18.)
    frame, tensors, window, now, ego, route = old.current
    payload = geometry_helpers.read_canonical_regular(ROOT / geometry_helpers.ROUTE_REL)
    class Base:
        def __init__(self):
            self.identity = ("synthetic", "val")
            self.decoder = PathTimeDecoderV2()
            self.deadline_s = .1
        def step(self):
            return dict(original_return=True, original_deadline=self.deadline_s, decoded=self.decoder(*args))
    cls = timing.navigation_terminal_policy_class(Base, payload, old.source_binding)
    policy = cls()
    policy.decoder.bind_current(frame, tensors, window, now_ns=now, ego_stamp_ns=ego, route_stamp_ns=route)
    result = policy.step()
    exact_outputs(result["decoded"], old(*args))
    assert result["original_return"] is True and result["original_deadline"] == .1
    assert result["navigation_terminal_research"] == old.last_receipt
    assert result["navigation_terminal_timing_research"] is policy.decoder.last_timing
    assert result["navigation_terminal_timing_attachment_outside_original_policy_timer"] is True
