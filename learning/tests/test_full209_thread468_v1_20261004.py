"""HH_261004 - Synthetic CPU proof of thread ordering, full numerical tolerance and bounded failure evidence, never actual NN timing."""

import copy
import json
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import benchmark_full209_thread468_v1_20261004 as b


@pytest.fixture(scope="module", autouse=True)
def synthetic_one_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def zeros_raw():
    return {name: torch.zeros(shape, dtype=torch.float32) for name, shape in b.RAW_SHAPES.items()}


def test_complete_preserved_denominators_original_tolerance_and_no_adoption_claim():
    value = b.contract()
    assert value["thread_arms"] == [4, 6, 8] and value["separate_fresh_serial_process_per_arm"]
    assert (value["original_prefills"], value["original_fresh_NN"], value["all_original_contexts"]) == (23, 865, 888)
    assert (value["original_V2_fresh_ordinary"], value["original_V2_fresh_terminal"]) == (802, 63)
    assert value["selected_context_indices"][0] == 0 and value["selected_context_indices"][-1] == 887
    assert value["timed_calls_per_arm"] == 30 and value["first_selected_context_cold_cache"]
    assert (value["preload_intraop_threads"], value["preload_interop_threads"], value["postloader_required_intraop_threads"]) == (4, 1, 4)
    assert value["strict_GOOD6000_loader_configures_threads"] is False
    assert value["raw_absolute_tolerance"] == value["raw_relative_tolerance"] == 1e-6
    assert value["ordinary_decoder_XY_pointwise_Euclidean_limit_m"] == .05
    assert value["ordinary_decoder_XY_chord_speed_absolute_limit_mps"] == .1
    assert value["maximum_archive_bytes_per_arm"] == 12 * 1024**2 and value["maximum_total_archive_bytes"] == 48 * 1024**2
    for name in ("per_call_threadpool_retuning", "cross_thread_bitwise_equality_claimed", "intermediate_or_failure_tail_cropping",
                 "complete_safety_engine_and_control_parity_verified", "actual_sensor_latency_gain_proven", "model_promoted"):
        assert value[name] is False


@pytest.mark.parametrize("target", [4, 6, 8])
def test_thread_arm_changes_once_only_after_loader_retains_explicit_CPU4_state(target):
    calls = []
    class Runtime:
        threads = 4
        def get_num_threads(self):
            return self.threads
        def get_num_interop_threads(self):
            return 1
        def set_num_threads(self, threads):
            calls.append(threads)
            self.threads = threads
    runtime = Runtime()
    b.set_threads_after_load(runtime, target)
    assert calls == [target] and runtime.threads == target


def test_explicit_V1_runtime_initialization_precedes_loader_and_sets_interop_only_once():
    # HH_261004 - Model-free ordering proof: inherited defaults are not mistaken for numeric-loader configuration.
    calls = []
    class Runtime:
        threads, interop = 24, 24
        def get_num_threads(self):
            return self.threads
        def get_num_interop_threads(self):
            return self.interop
        def set_num_threads(self, value):
            calls.append(("intraop", value))
            self.threads = value
        def set_num_interop_threads(self, value):
            calls.append(("interop", value))
            self.interop = value
    runtime = Runtime()
    b.initialize_worker_runtime(runtime)
    assert (runtime.threads, runtime.interop) == (4, 1)
    calls.append(("strict_loader_retains_baseline", 4))
    b.set_threads_after_load(runtime, 6)
    assert calls == [("intraop", 4), ("interop", 1), ("strict_loader_retains_baseline", 4), ("intraop", 6)]
    assert (runtime.threads, runtime.interop) == (6, 1)


@pytest.mark.parametrize("bad", [True, False, 2, 5, 10, None])
def test_no_unplanned_thread_arm(bad):
    with pytest.raises(RuntimeError):
        b.set_threads_after_load(None, bad)


@pytest.mark.parametrize("initial,interop", [(1, 1), (6, 1), (4, 2)])
def test_preloader_or_wrong_interop_thread_configuration_is_rejected(initial, interop):
    runtime = SimpleNamespace(get_num_threads=lambda: initial, get_num_interop_threads=lambda: interop,
        set_num_threads=lambda threads: pytest.fail("must reject before setting"))
    with pytest.raises(RuntimeError):
        b.set_threads_after_load(runtime, 8)


def test_all_five_heads_accept_original_tolerance_without_bit_equality_claim():
    first = b.raw_values(zeros_raw())
    current = copy.deepcopy(first)
    current["curvature_knots"][0][0] = .5e-6
    result = b.raw_comparison(first, current)
    assert set(result) == set(b.RAW_SHAPES)
    assert all(row["allclose"] for row in result.values())
    assert result["curvature_knots"]["maximum_absolute_error"] == pytest.approx(.5e-6)
    assert result["curvature_knots"]["bitwise_equal_diagnostic"] is False
    current["curvature_knots"][0][0] = 1.5e-6
    assert b.raw_comparison(first, current)["curvature_knots"]["allclose"] is False


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_each_raw_head_rejects_nonfinite_values(bad):
    raw = zeros_raw()
    raw["acceleration_prediction"][0, 0] = bad
    with pytest.raises(RuntimeError):
        b.raw_values(raw)
    archived = b.raw_values(zeros_raw())
    archived["acceleration_prediction"][0][0] = bad
    with pytest.raises(RuntimeError):
        b.raw_comparison(b.raw_values(zeros_raw()), archived)


def test_missing_or_wrong_raw_shape_and_dtype_never_pass():
    raw = zeros_raw()
    raw.pop("stop_distance_raw")
    with pytest.raises(RuntimeError):
        b.raw_values(raw)
    raw = zeros_raw()
    raw["intent_logits"] = torch.zeros(1, 4)
    with pytest.raises(RuntimeError):
        b.raw_values(raw)
    raw = zeros_raw()
    raw["intent_logits"] = raw["intent_logits"].double()
    with pytest.raises(RuntimeError):
        b.raw_values(raw)


def synthetic_geometry(support=64.):
    tensors = dict(ego_history=torch.zeros(1, 10, 13), ego_history_mask=torch.ones(1, 10, dtype=torch.bool),
        route_xy=torch.zeros(1, 128, 2), route_mask=torch.ones(1, 128, dtype=torch.bool))
    tensors["ego_history"][0, -1, 1] = .2
    tensors["route_xy"][0, :, 0] = torch.arange(128) * (support / 127.)
    raw = zeros_raw()
    raw["preferred_speed_knots"].fill_(.8)
    return raw, tensors


def test_pure_ordinary_GO_decoder_uses_real_nonimage_helpers_and_keeps_all64_points():
    raw, tensors = synthetic_geometry()
    before = b.previous.tensor_hashes(tensors)
    result = b.ordinary_decoder_screen(raw, tensors)
    assert result["status"] == "ORDINARY_GO_COUNTERFACTUAL_DECODED" and result["ordinary_decoder_executed"]
    assert torch.tensor(result["xy"]).shape == (1, 1, 64, 2)
    assert torch.tensor(result["chord_speed"]).shape == (1, 1, 64)
    assert b.previous.tensor_hashes(tensors) == before
    assert result["full_safety_engine_parity_verified"] is False and result["original_BGRA_proof_fabricated"] is False


def test_short_support_remains_terminal_unverified_without_fabricated_witness():
    raw, tensors = synthetic_geometry(17.)
    result = b.ordinary_decoder_screen(raw, tensors)
    assert result["status"] == "TERMINAL_PROOF_UNVERIFIED" and result["ordinary_decoder_executed"] is False
    assert "xy" not in result and "chord_speed" not in result
    assert b.decoder_comparison(result, result)["ordinary_numeric_screen_passed"] is False


def test_original_decoder_rejection_is_retained_without_clipping_or_skipping_raw_context():
    raw, tensors = synthetic_geometry()
    tensors["ego_history"][0, -1, 1] = 2.6
    result = b.ordinary_decoder_screen(raw, tensors)
    assert result["status"] == "ORDINARY_DECODER_REJECTED"
    assert result["exception_type"] == "ContractError" and "clipping" in result["exception"]
    assert b.decoder_comparison(result, result)["ordinary_numeric_screen_passed"] is False


def decoded(x=0., y=0., speed=0.):
    return dict(status="ORDINARY_GO_COUNTERFACTUAL_DECODED", current_base_link_planar_speed_mps=.2,
        causal_navigation_support_m=64., xy=[[[[x, y] for _ in range(64)]]], chord_speed=[[[speed for _ in range(64)]]])


def test_XY_engine_screen_uses_pointwise_Euclidean_error_and_unchanged_speed_threshold():
    assert b.decoder_comparison(decoded(), decoded(.04, 0., .0999))["ordinary_numeric_screen_passed"]
    diagonal = b.decoder_comparison(decoded(), decoded(.04, .04, 0.))
    assert diagonal["maximum_pointwise_XY_error_m"] > .05 and not diagonal["ordinary_numeric_screen_passed"]
    assert not b.decoder_comparison(decoded(), decoded(0., 0., .1001))["ordinary_numeric_screen_passed"]


def original_provenance_fixture():
    events, learned = [], []
    for index in range(888):
        frame, stamp = 1024 + 2 * index, 10_000_000_000 + index * 100_000_000
        event = dict(native_frame=frame, anchor_timestamp_ns=stamp,
            kind="original_prefill" if index < 23 else "original_fresh_NN")
        events.append(event)
        if index >= 23:
            terminal = index >= 825
            support = 17. if terminal else 64.
            receipt = dict(frame_timestamp_ns=stamp, inherited_tensor_support_m=support,
                branch="authenticated_navigation_terminal_V2" if terminal else "ordinary_exact_delegate")
            if terminal:
                receipt["witness_decision"] = dict(status="VERIFIED", synthetic_fixture=True)
            learned.append(dict(held_between_camera_frames=False, inference_frame=frame,
                causal_navigation_support_m=support, navigation_terminal_research=receipt))
    return events, learned


def test_V2_original_census_and_witness_identity_separate_prefill_from_actual_decoder():
    result = b.original_decoder_provenance(*original_provenance_fixture())
    assert sum(row["original_decoder_role"] == "PREFILL_NO_ACTUAL_DECODER" for row in result) == 23
    assert sum(row["original_decoder_role"] == "ACTUAL_ORDINARY" for row in result) == 802
    assert sum(row["original_decoder_role"] == "ACTUAL_TERMINAL" for row in result) == 63
    assert result[-1]["original_terminal_witness_logged"] and result[-1]["replay_terminal_proof_verified"] is False


@pytest.mark.parametrize("mutation", ["unsupported_branch", "support_changed", "wrong_frame", "unverified_witness", "V4_census"])
def test_original_provenance_cannot_be_relabelled_or_borrow_V4_counts(mutation):
    events, learned = original_provenance_fixture()
    receipt = learned[-1]["navigation_terminal_research"]
    if mutation == "unsupported_branch":
        receipt["branch"] = "illustrative_terminal"
    elif mutation == "support_changed":
        learned[-1]["causal_navigation_support_m"] = 18.
    elif mutation == "wrong_frame":
        receipt["frame_timestamp_ns"] += 1
    elif mutation == "unverified_witness":
        receipt["witness_decision"]["status"] = "NOT_VERIFIED"
    else:
        learned[-1]["causal_navigation_support_m"] = receipt["inherited_tensor_support_m"] = 64.
        receipt["branch"] = "ordinary_exact_delegate"
    with pytest.raises(RuntimeError):
        b.original_decoder_provenance(events, learned)


def test_selected_cold_and_failed_timing_attempts_retained_with_exact_api(monkeypatch, tmp_path):
    timings, observed = [], []
    error = ValueError("synthetic original cached rejection")
    class Model:
        def forward_cached(self, tensors, *, window):
            observed.append((tensors, window))
            raise error
    clock = iter((100, 100100))
    monkeypatch.setattr(b.time, "perf_counter_ns", lambda: next(clock))
    with pytest.raises(ValueError) as caught:
        b.timed_forward(Model(), 2, 3, 8, 0, timings, tmp_path)
    assert caught.value is error and observed == [(2, 3)] and timings[0]["elapsed_ns"] == 100000
    assert timings[0]["original_first_cold_context"] and timings[0]["status"] == "REJECTED"
    assert json.loads((tmp_path / "all30_timing_rows.jsonl").read_text()) == timings[0]


def test_intermediate_context_executes_once_without_thread_retune_or_timing(monkeypatch, tmp_path):
    observed = []
    model = SimpleNamespace(forward_cached=lambda tensors, window: observed.append((tensors, window)) or "raw")
    monkeypatch.setattr(b.time, "perf_counter_ns", lambda: pytest.fail("unselected original intermediate must not be timed"))
    assert b.timed_forward(model, 2, 3, 6, 1, [], tmp_path) == "raw" and observed == [(2, 3)]


def test_parent_small_archive_limit_is_separate_from_per_arm_limit(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    monkeypatch.setattr(b, "MAX_ARM_BYTES", 64 * 1024 + 1)
    monkeypatch.setattr(b, "MAX_TOTAL_BYTES", 64 * 1024 + 1000)
    b.bounded_append(tmp_path, "comparison.jsonl", {"all_contexts_retained": True})
    with pytest.raises(RuntimeError):
        b.bounded_append(tmp_path, "comparison.jsonl", {"too_big": "a" * 1100})
