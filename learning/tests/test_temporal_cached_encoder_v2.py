"""HH_261001 - Verify research cache parity and causal reuse, without runtime approval."""

from __future__ import annotations

from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
from portable_e2e.temporal_cached_encoder_v2 import (
    CACHED_OUTPUT_ATOL,
    CACHED_OUTPUT_RTOL,
    CameraFrameIdentityV2,
    CachedTemporalModelV2,
    FROZEN_MODEL_SOURCE_SHA256,
    TemporalCacheWindowV2,
)
from portable_e2e.temporal_model_v2 import (
    NOMINAL_TIME_OFFSETS_S,
    TemporalModelConfigV2,
    TemporalModelV2,
    parameter_count,
)


@pytest.fixture(scope="module", autouse=True)
def _one_cpu_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


@pytest.fixture
def cached():
    # HH_261001 - Focused tests keep real input/token shapes with cheaper internal widths.
    torch.manual_seed(91)
    return CachedTemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()


def _sha(text):
    return hashlib.sha256(text.encode("ascii")).hexdigest()


def _window(step=0, *, missing=()):
    """HH_261001 - Real-shape causal 10 Hz data, with reproducible actual pixel payloads."""
    anchor = 2_000_000_000 + step * 100_000_000
    timestamps = tuple(anchor + offset for offset in (-800_000_000, -600_000_000, -400_000_000, -200_000_000, 0))
    groups = []
    frames = []
    for slot, timestamp in enumerate(timestamps):
        generator = torch.Generator().manual_seed(timestamp // 100_000_000 + 711)
        # HH_261001 - Values resemble normalized RGB; no future image or label is supplied.
        groups.append(torch.rand(6, 3, 180, 320, generator=generator) * 4.0 - 2.0)
        frames.append(None if slot in missing else CameraFrameIdentityV2(
            timestamp, tuple(_sha(f"recorded-camera:{timestamp}:{camera}") for camera in range(6))))
    calibration = torch.zeros(1, 6, 16)
    calibration[..., :4] = torch.tensor((0.7, 1.2, 0.5, 0.5))
    calibration[..., (4, 9, 14)] = 1.0
    pose = torch.zeros(1, 5, 3)
    pose[..., 0] = torch.tensor((-1.0, -0.75, -0.5, -0.25, 0.0))
    pose[..., 2] = torch.tensor((0.04, 0.03, 0.02, 0.01, 0.0))
    route = torch.zeros(1, 128, 2)
    route[..., 0] = torch.arange(128)
    valid = torch.ones(1, 5, dtype=torch.bool)
    for slot in missing:
        valid[:, slot] = False
    inputs = {
        "images": torch.stack(groups)[None],
        "calibration": calibration,
        "ego_history": torch.zeros(1, 10, 13),
        "ego_history_mask": torch.ones(1, 10, dtype=torch.bool),
        "route_xy": route,
        "route_mask": torch.ones(1, 128, dtype=torch.bool),
        "time_valid": valid,
        "relative_pose": pose,
        "time_offsets_s": torch.tensor(NOMINAL_TIME_OFFSETS_S)[None],
    }
    window = TemporalCacheWindowV2("episode-one", "train", "rig-one", _sha("validated-source"),
                                  anchor, tuple(frames))
    return inputs, window


def _reference(cached):
    result = TemporalModelV2(cached.config).to(cached.queries.dtype).eval()
    result.load_state_dict(cached.state_dict(), strict=True)
    return result


def _assert_equivalent(actual, expected):
    assert actual.keys() == expected.keys()
    maximum = 0.0
    for name in actual:
        assert torch.allclose(actual[name], expected[name], atol=CACHED_OUTPUT_ATOL, rtol=CACHED_OUTPUT_RTOL), name
        maximum = max(maximum, float((actual[name] - expected[name]).abs().max()))
    return maximum


def _cached_call(cached, inputs, window, **kwargs):
    with torch.no_grad():
        return cached.forward_cached(inputs, window=window, **kwargs)


def _compare(cached, inputs, window, **kwargs):
    reference = _reference(cached)
    with torch.no_grad():
        expected = reference(**inputs, **kwargs)
        actual = cached.forward_cached(inputs, window=window, **kwargs)
    return _assert_equivalent(actual, expected)


def test_frozen_source_and_checkpoint_abi_and_seeded_weights_are_identical():
    source = Path(__file__).resolve().parents[1] / "portable_e2e/temporal_model_v2.py"
    assert hashlib.sha256(source.read_bytes()).hexdigest() == FROZEN_MODEL_SOURCE_SHA256
    config = TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)
    torch.manual_seed(182)
    frozen = TemporalModelV2(config)
    torch.manual_seed(182)
    cached = CachedTemporalModelV2(config)
    assert cached.config == frozen.config
    assert parameter_count(cached) == parameter_count(frozen)
    assert list(cached.state_dict()) == list(frozen.state_dict())
    assert all(torch.equal(value, frozen.state_dict()[name]) for name, value in cached.state_dict().items())
    assert not frozen.load_state_dict(cached.state_dict(), strict=True).missing_keys
    assert not cached.load_state_dict(frozen.state_dict(), strict=True).unexpected_keys
    inputs, window = _window()
    cached.eval()
    _cached_call(cached, inputs, window)
    assert list(cached.state_dict()) == list(frozen.state_dict())
    assert all(torch.equal(value, frozen.state_dict()[name]) for name, value in cached.state_dict().items())


def test_ten_hz_windows_retain_both_parity_chains_and_encode_six_warm_views(cached):
    reference = _reference(cached)
    calls = []
    hook = cached.image_encoder.features.register_forward_pre_hook(lambda module, args: calls.append(args[0].shape[0]))
    try:
        for step in range(8):
            inputs, window = _window(step)
            with torch.no_grad():
                _assert_equivalent(cached.forward_cached(inputs, window=window), reference(**inputs))
            assert cached.cache_stats.cnn_images == (30 if step < 2 else 6)
            assert cached.cache_stats.reused_frame_groups == (0 if step < 2 else 4)
            assert cached.cache_stats.retained_frame_groups <= 9
        assert calls == [30, 30, 6, 6, 6, 6, 6, 6]
        assert cached.cache_stats.retained_frame_groups == 9
    finally:
        hook.remove()


def test_repeated_anchor_with_fresh_equal_tensors_needs_no_cnn(cached):
    inputs, window = _window()
    first = _cached_call(cached, inputs, window)
    fresh = {name: value.clone() for name, value in inputs.items()}
    second = _cached_call(cached, fresh, window)
    _assert_equivalent(first, second)
    assert cached.cache_stats.cnn_images == 0
    assert cached.cache_stats.cnn_calls == 0
    assert cached.cache_stats.reused_frame_groups == 5


def test_same_identity_changed_normalized_pixels_recompute_that_frame(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    inputs["images"][0, 2, 1, 0, 20, 30] += 1.0
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 6
    assert cached.cache_stats.reused_frame_groups == 4


def test_changed_camera_file_hash_recomputes_even_for_equal_pixels(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    frames = list(window.frames)
    hashes = list(frames[1].camera_sha256)
    hashes[0] = _sha("another-recorded-camera-file")
    frames[1] = replace(frames[1], camera_sha256=tuple(hashes))
    _compare(cached, inputs, replace(window, frames=tuple(frames)))
    assert cached.cache_stats.cnn_images == 6
    assert cached.cache_stats.retained_frame_groups == 5


def test_missing_slots_are_not_cached_or_poisoned_and_later_valid_slots_encode(cached):
    inputs, window = _window(missing=(0, 2))
    _compare(cached, inputs, window)
    assert cached.cache_stats.retained_frame_groups == 3
    inputs["images"][:, (0, 2)] = 1234.0
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 0
    valid_inputs, valid_window = _window()
    _compare(cached, valid_inputs, valid_window)
    assert cached.cache_stats.cnn_images == 12
    assert cached.cache_stats.reused_frame_groups == 3


def test_current_only_never_uses_or_populates_past_cache_slots(cached):
    inputs, window = _window()
    _compare(cached, inputs, window, history_mode="current_only")
    assert cached.cache_stats.retained_frame_groups == 1
    inputs["images"][:, :-1] = -999.0
    _compare(cached, inputs, window, history_mode="current_only")
    assert cached.cache_stats.cnn_images == 0
    temporal, _ = _window()
    _compare(cached, temporal, window)
    assert cached.cache_stats.cnn_images == 24


@pytest.mark.parametrize("name", ("route_xy", "ego_history", "relative_pose"))
def test_conditioning_is_recomputed_with_equal_cached_images(cached, name):
    inputs, window = _window()
    first = _cached_call(cached, inputs, window)
    if name == "relative_pose":
        inputs[name][:, 0, 0] += 10.0
    elif name == "route_xy":
        inputs[name][..., 1] += 10.0
    else:
        inputs[name][:, -1, 0] += 10.0
    _compare(cached, inputs, window)
    second = _cached_call(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 0
    assert max(float((first[key] - second[key]).abs().max()) for key in first) > 1.0e-6


@pytest.mark.parametrize("change", (
    {"episode_id": "episode-two"}, {"split": "val"}, {"rig_id": "rig-two"},
    {"source_sha256": _sha("another-validated-source")},
))
def test_episode_split_rig_source_changes_reset_to_full_recompute(cached, change):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    _compare(cached, inputs, replace(window, **change))
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "episode_split_rig_or_source_changed"


def test_changed_actual_calibration_resets_even_if_rig_id_is_reused(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    inputs["calibration"][0, 0, 0] += 0.1
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "episode_split_rig_or_source_changed"


def test_clock_reversal_discards_later_features(cached):
    inputs, window = _window(2)
    _cached_call(cached, inputs, window)
    earlier_inputs, earlier_window = _window(0)
    _compare(cached, earlier_inputs, earlier_window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "clock_reversal"
    assert all(frame.timestamp_ns <= earlier_window.anchor_timestamp_ns for frame in cached.image_encoder.entries)


def test_checkpoint_load_always_resets_even_when_tensor_values_match(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    cached.load_state_dict(cached.state_dict(), strict=True)
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "checkpoint_loaded"


@pytest.mark.parametrize("kind", ("ordinary", "data", "buffer", "topology"))
def test_model_mutation_including_silent_data_edit_resets_cache(cached, kind):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    with torch.no_grad():
        if kind == "ordinary":
            cached.image_encoder.projection.bias.add_(0.01)
        elif kind == "data":
            weight = cached.image_encoder.features[0].weight
            before_version = weight._version
            weight.data.add_(0.001)
            assert weight._version == before_version
        elif kind == "buffer":
            cached.image_grid_xy.add_(0.001)
        else:
            cached.image_encoder.features[2] = torch.nn.SiLU().eval()
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "model_state_changed"


def test_dtype_conversion_discards_unregistered_cache_tensors(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    cached.double()
    converted = {name: value.double() if value.is_floating_point() else value for name, value in inputs.items()}
    _compare(cached, converted, window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "device_or_dtype_changed"


def test_encoder_behavior_setting_change_invalidates_cached_tokens(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    cached.image_encoder.features[1].eps = 1.0
    with torch.no_grad():
        expected = cached(**inputs)
        actual = cached.forward_cached(inputs, window=window)
    _assert_equivalent(actual, expected)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "model_state_changed"


def test_training_and_gradient_enabled_cached_calls_are_rejected(cached):
    inputs, window = _window()
    with pytest.raises(ContractError, match="eval mode and torch.no_grad"):
        cached.forward_cached(inputs, window=window)
    _cached_call(cached, inputs, window)
    cached.train()
    assert not cached.image_encoder.entries
    with torch.no_grad(), pytest.raises(ContractError, match="eval mode"):
        cached.forward_cached(inputs, window=window)
    cached.eval()
    cached.image_encoder.features.train()
    assert not cached.training
    with torch.no_grad(), pytest.raises(ContractError, match="eval mode"):
        cached.forward_cached(inputs, window=window)


def test_normal_forward_remains_full_and_differentiable_after_cached_use(cached):
    inputs, window = _window()
    _cached_call(cached, inputs, window)
    calls = []
    hook = cached.image_encoder.features.register_forward_pre_hook(lambda module, args: calls.append(args[0].shape[0]))
    try:
        output = cached(**inputs)
        sum(value.sum() for value in output.values()).backward()
        assert calls == [30]
        assert cached.image_encoder.features[0].weight.grad.abs().sum() > 0
        assert cached.image_encoder.request is None
    finally:
        hook.remove()


def test_autocast_is_rejected_without_gpu_calls(cached, monkeypatch):
    inputs, window = _window()
    monkeypatch.setattr(torch, "is_autocast_enabled", lambda: True)
    with torch.no_grad(), pytest.raises(ContractError, match="autocast"):
        cached.forward_cached(inputs, window=window)


def test_multi_stream_batch_is_rejected(cached):
    inputs, window = _window()
    doubled = {name: value.repeat((2,) + (1,) * (value.ndim - 1)) for name, value in inputs.items()}
    with torch.no_grad(), pytest.raises(ContractError, match="batch size one"):
        cached.forward_cached(doubled, window=window)


@pytest.mark.parametrize("kind", ("future", "timestamp", "missing", "camera_hash", "anchor", "source"))
def test_invalid_or_future_provenance_is_rejected_before_cnn(cached, kind):
    inputs, window = _window()
    frames = list(window.frames)
    if kind == "future":
        frames[0] = replace(frames[0], timestamp_ns=window.anchor_timestamp_ns + 1)
    elif kind == "timestamp":
        frames[1] = replace(frames[1], timestamp_ns=frames[1].timestamp_ns + 1_000_000)
    elif kind == "missing":
        inputs["time_valid"][:, 0] = False
    elif kind == "camera_hash":
        frames[2] = replace(frames[2], camera_sha256=("not-a-hash",) * 6)
    elif kind == "anchor":
        window = replace(window, anchor_timestamp_ns=True)
    else:
        window = replace(window, source_sha256="dataset-global-index-12")
    with torch.no_grad(), pytest.raises(ContractError):
        cached.forward_cached(inputs, window=replace(window, frames=tuple(frames)))
    assert not cached.image_encoder.entries
    assert cached.image_encoder.request is None


def test_dataset_sub_ms_jitter_does_not_widen_frozen_model_tolerance(cached):
    inputs, window = _window()
    # HH_261001 - 400 us passes the dataset's 1 ms match gate but fails the frozen 3 us model ABI.
    inputs["time_offsets_s"][0, 0] += 0.0004
    frames = list(window.frames)
    frames[0] = replace(frames[0], timestamp_ns=frames[0].timestamp_ns + 400_000)
    with torch.no_grad(), pytest.raises(ContractError, match="pinned five causal slots"):
        cached.forward_cached(inputs, window=replace(window, frames=tuple(frames)))
    assert not cached.image_encoder.entries


def test_failed_forward_clears_partial_cache_and_restores_encoder(cached):
    inputs, window = _window()

    def fail(module, args, output):
        raise RuntimeError("injected downstream research failure")

    hook = cached.intent_head.register_forward_hook(fail)
    try:
        with torch.no_grad(), pytest.raises(RuntimeError, match="injected downstream"):
            cached.forward_cached(inputs, window=window)
        assert cached.image_encoder.request is None
        assert not cached.image_encoder.entries
    finally:
        hook.remove()
    _compare(cached, inputs, window)
    assert cached.cache_stats.cnn_images == 30
    assert cached.cache_stats.reset_reason == "failed_forward"


def test_source_pin_rejects_an_unrecognized_frozen_implementation(monkeypatch):
    import portable_e2e.temporal_cached_encoder_v2 as module

    monkeypatch.setattr(module, "FROZEN_MODEL_SOURCE_SHA256", "0" * 64)
    with pytest.raises(ContractError, match="pinned frozen"):
        module.CachedTemporalModelV2()


def test_cpu_one_thread_real_shape_warm_cache_benchmark():
    """HH_261001 - Measure raw synthetic inference only; timing is diagnostic, not an SLA."""
    torch.manual_seed(381)
    cached = CachedTemporalModelV2().eval()
    reference = _reference(cached)
    full_times, cached_times, differences, image_counts = [], [], [], []
    with torch.no_grad():
        # HH_261001 - Prime both .2-second history chains of a .1-second incoming stream.
        for step in (0, 1):
            inputs, window = _window(step)
            _assert_equivalent(cached.forward_cached(inputs, window=window), reference(**inputs))
            assert cached.cache_stats.cnn_images == 30
        for step in range(2, 8):
            inputs, window = _window(step)
            started = time.perf_counter()
            expected = reference(**inputs)
            full_times.append(time.perf_counter() - started)
            started = time.perf_counter()
            actual = cached.forward_cached(inputs, window=window)
            cached_times.append(time.perf_counter() - started)
            differences.append(_assert_equivalent(actual, expected))
            image_counts.append(cached.cache_stats.cnn_images)
    assert image_counts == [6] * 6
    result = {
        "status": "RESEARCH_RAW_SYNTHETIC_CACHE_BENCHMARK_NOT_RUNTIME_APPROVAL",
        "torch_version": torch.__version__, "device": "cpu", "cpu_threads": torch.get_num_threads(),
        "input_shape": [1, 5, 6, 3, 180, 320], "incoming_period_s": 0.1, "slot_period_s": 0.2,
        "retained_frame_groups": cached.cache_stats.retained_frame_groups,
        "cold_cnn_images_per_window": 30, "warm_cnn_images_per_window": image_counts,
        "full_forward_mean_ms": 1000 * sum(full_times) / len(full_times),
        "cached_forward_mean_ms": 1000 * sum(cached_times) / len(cached_times),
        "full_forward_individual_ms": [1000 * value for value in full_times],
        "cached_forward_individual_ms": [1000 * value for value in cached_times],
        "maximum_output_absolute_difference": max(differences),
        "equivalence_atol": CACHED_OUTPUT_ATOL, "equivalence_rtol": CACHED_OUTPUT_RTOL,
        "includes_payload_and_model_state_hashing": True, "includes_jpeg_decode_or_sensor_io": False,
        "includes_decoder_control_ack_or_autoware": False, "ten_hz_end_to_end_proven": False,
    }
    print("TEMPORAL_CACHE_CPU_BENCHMARK " + json.dumps(result, sort_keys=True))
