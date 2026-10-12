"""HH_261004 - Synthetic CPU review of complete causal scheduling and failure retention; no actual NN replay or timing claim."""

import copy
import json
from types import SimpleNamespace

import pytest

from scripts.e2e import benchmark_full209_masked_copy_v1_20261004 as b


@pytest.mark.parametrize("count", [30, 31, 450, 865, 888, 901])
def test_fixed_thirty_pairs_include_first_cold_and_last_context(count):
    indices = b.selected_indices(count)
    assert indices == [(count - 1) * index // 29 for index in range(30)]
    assert len(set(indices)) == 30 and indices[0] == 0 and indices[-1] == count - 1


@pytest.mark.parametrize("count", [True, False, None, 0, 29, 888.])
def test_no_incomplete_or_noninteger_timing_denominator(count):
    with pytest.raises(RuntimeError):
        b.selected_indices(count)


def synthetic_catalog():
    """HH_261004 - Full synthetic cardinalities are scheduling fixtures, never loaded real camera/state evidence."""
    cameras = [dict(frame=1000 + 2 * index, timestamp=10. + index / 10.) for index in range(901)]
    names = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask",
             "time_valid", "relative_pose", "time_offsets_s")
    prefills, fresh = [], []
    for index in range(12, 900):
        audit = dict(frame_timestamp_ns=round(cameras[index]["timestamp"] * 1e9),
            source_camera_frames=[dict(timestamp_ns=round(cameras[index + offset]["timestamp"] * 1e9),
                camera_sha256=["b" * 64] * 6) for offset in (-8, -6, -4, -2, 0)],
            normalized_input_tensor_sha256={name: "a" * 64 for name in names})
        if index < 35:
            prefills.append(dict(native_frame=cameras[index]["frame"], owned_camera_history=audit))
        else:
            fresh.append(dict(inference_frame=cameras[index]["frame"], command_frame=cameras[index]["frame"],
                held_between_camera_frames=False, raw_output={"synthetic": [1.]}, owned_camera_history=audit))
    held = [dict(row, command_frame=row["command_frame"] + 1, held_between_camera_frames=True) for row in fresh]
    return cameras, prefills, [dict(control_source="expert_startup")] + fresh + held


def test_all888_contexts_keep23_prefills865_fresh_and_ignore_held_commands():
    rows = b.event_catalog(*synthetic_catalog())
    assert len(rows) == 888
    assert [row["camera_ordinal"] for row in rows] == list(range(12, 900))
    assert sum(row["kind"] == "original_prefill" for row in rows) == 23
    assert sum(row["kind"] == "original_fresh_NN" for row in rows) == 865
    assert rows[0]["camera_ordinal"] == 12 and rows[-1]["camera_ordinal"] == 899
    assert all(len(row["original_BGRA_source_camera_frames"]) == 5 for row in rows)


@pytest.mark.parametrize("mutation", ["missing_prefill", "missing_fresh", "duplicate_fresh", "future_slot",
    "missing_slot", "wrong_anchor", "wrong_command_frame", "missing_raw", "missing_hash", "unsupported_new_NN"])
def test_context_catalog_rejects_missing_or_fabricated_original_contexts(mutation):
    cameras, prefills, learned = synthetic_catalog()
    first = learned[1]
    if mutation == "missing_prefill":
        prefills.pop()
    elif mutation == "missing_fresh":
        learned.pop(1)
    elif mutation == "duplicate_fresh":
        learned[2] = copy.deepcopy(first)
    elif mutation == "future_slot":
        first["owned_camera_history"]["source_camera_frames"][0]["timestamp_ns"] += 1_000_000_000
    elif mutation == "missing_slot":
        first["owned_camera_history"]["source_camera_frames"][0] = None
    elif mutation == "wrong_anchor":
        first["owned_camera_history"]["frame_timestamp_ns"] += 1
    elif mutation == "wrong_command_frame":
        first["command_frame"] += 1
    elif mutation == "missing_raw":
        first.pop("raw_output")
    elif mutation == "missing_hash":
        first["owned_camera_history"]["normalized_input_tensor_sha256"].pop("ego_history")
    elif mutation == "unsupported_new_NN":
        first["inference_frame"] = first["command_frame"] = cameras[900]["frame"]
    with pytest.raises(RuntimeError):
        b.event_catalog(cameras, prefills, learned)


def test_contract_separates_actual_denominators_lossy_inputs_and_full_timer_scope():
    value = b.contract()
    assert (value["all_original_files"], value["all_camera_bundles"], value["all_original_JPEGs"]) == (7366, 901, 5406)
    assert (value["original_prefills"], value["original_fresh_NN"], value["complete_recorded_contexts"]) == (23, 865, 888)
    assert value["unexecuted_warmup_ordinals"] == list(range(12))
    assert value["final_camera_without_original_NN_ordinal"] == 900
    assert value["model_context_camera_ordinals"] == list(range(12, 900))
    assert value["first_selected_pair_is_original_cold_prefill"] is True
    assert value["no_warmup_or_failed_tail_cropping"] is True
    assert value["no_rejected_trial_dropping_or_retry"] is True
    assert value["reconstructed_lossy_JPEG"] is True
    assert (value["CPU_threads"], value["interop_threads"]) == (4, 1)
    for name in ("original_lossless_BGRA_equality_claimed", "original_actual_raw_NN_equality_claimed",
                 "decoder_or_control_executed", "actual_sensor_latency_gain_proven", "ten_hz_pipeline_proven",
                 "actual_runtime_activated", "model_promoted", "training_data_approved"):
        assert value[name] is False
    assert value["main_functions_approved"] == value["subfunctions_approved"] == 0


def test_timer_includes_exact_cached_forward_and_keeps_cold_call(monkeypatch):
    observed, journal, timings = [], [], []
    inputs, window, result = object(), object(), object()
    class Model:
        def forward_cached(self, supplied, *, window):
            observed.append((supplied, window))
            return result
    clock = iter((100, 9100))
    monkeypatch.setattr(b.time, "perf_counter_ns", lambda: next(clock))
    monkeypatch.setattr(b, "append_journal", lambda name, row: journal.append((name, row)))
    returned = b.call(Model(), inputs, window, arm="B", context_index=0, timed_pair=0, timings=timings)
    assert returned is result and observed == [(inputs, window)]
    assert len(timings) == 1 and timings[0]["elapsed_ns"] == 9000
    assert timings[0]["original_first_cold_context"] is True
    assert timings[0]["status"] == "COMPLETED" and journal == [("all_timing_rows.jsonl", timings[0])]


def test_failed_timed_forward_retains_original_exception_attempt_and_duration(monkeypatch):
    error, journal, timings = ValueError("synthetic original rejection"), [], []
    class Model:
        def forward_cached(self, supplied, *, window):
            raise error
    clock = iter((100, 300100))
    monkeypatch.setattr(b.time, "perf_counter_ns", lambda: next(clock))
    monkeypatch.setattr(b, "append_journal", lambda name, row: journal.append((name, row)))
    with pytest.raises(ValueError) as caught:
        b.call(Model(), None, None, arm="A", context_index=887, timed_pair=29, timings=timings)
    assert caught.value is error
    assert timings[0]["status"] == "REJECTED" and timings[0]["elapsed_ns"] == 300000
    assert timings[0]["context_index"] == 887 and timings[0]["pair"] == 29
    assert journal == [("all_timing_rows.jsonl", timings[0])]


def test_untimed_intervening_context_is_executed_once_without_hidden_timer(monkeypatch):
    calls = []
    class Model:
        def forward_cached(self, supplied, *, window):
            calls.append((supplied, window))
            return {"synthetic": 1}
    monkeypatch.setattr(b.time, "perf_counter_ns", lambda: pytest.fail("not a selected timed context"))
    assert b.call(Model(), 2, 3, arm="A", context_index=1, timed_pair=None, timings=[]) == {"synthetic": 1}
    assert calls == [(2, 3)]


def test_failed_prefix_is_retained_with_complete_prospective_denominators(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    error = RuntimeError("synthetic parity mismatch after return")
    parity, timings = [{"context_index": 0}], [{"status": "REJECTED", "pair": 0}]
    with pytest.raises(RuntimeError) as caught:
        with b.retain_attempt(parity, timings):
            raise error
    value = json.loads((tmp_path / "FAILED_attempt.json").read_text())
    assert caught.value is error and value["planned_contexts"] == 888 and value["planned_timed_calls"] == 60
    assert value["completed_parity_rows"] == parity and value["attempted_timing_rows"] == timings
    assert value["original_actual_FAIL_changed"] is False and value["model_promoted"] is False


@pytest.mark.parametrize("content", ['{"x":1,"x":2}\n', '{"x":NaN}\n', '{"x":Infinity}\n'])
def test_raw_record_reader_rejects_ambiguous_or_nonfinite_input(tmp_path, content):
    path = tmp_path / "synthetic.jsonl"
    path.write_text(content)
    with pytest.raises(RuntimeError):
        b.jsonl(path)


def synthetic_raw():
    import torch
    return {name: torch.tensor([[1., -0., 2.]], dtype=torch.float32) for name in
        ("curvature_knots", "preferred_speed_knots", "intent_logits", "stop_distance_raw", "acceleration_prediction")}


def test_tiny_synthetic_raw_outputs_require_exact_bytes_and_complete_all_five_fields():
    first = synthetic_raw()
    second = {name: value.clone() for name, value in first.items()}
    assert b.compare_raw(first, second) == b.tensor_hashes(first)
    second["intent_logits"][0, 1] = 0.
    with pytest.raises(RuntimeError):
        b.compare_raw(first, second)
    second = {name: value.clone() for name, value in first.items()}
    second.pop("stop_distance_raw")
    with pytest.raises(RuntimeError):
        b.compare_raw(first, second)


def test_tiny_synthetic_cache_comparison_retains_original_work_identity_and_feature_bytes():
    import torch
    from portable_e2e.temporal_cached_encoder_v2 import CameraFrameIdentityV2, TemporalCacheStatsV2
    frame = CameraFrameIdentityV2(100, ("a" * 64,) * 6)
    entry = SimpleNamespace(pixel_sha256="b" * 64, features=torch.tensor([[1., 2.]]))
    model = SimpleNamespace(cache_stats=TemporalCacheStatsV2(1, 6, 4, 1, 9, False, None),
        image_encoder=SimpleNamespace(entries={frame: entry}))
    before = b.cache_signature(model)
    assert before["work"]["cnn_images"] == 6 and before["entries"][0]["timestamp_ns"] == 100
    entry.features[0, 0] = 2.
    assert b.cache_signature(model) != before
