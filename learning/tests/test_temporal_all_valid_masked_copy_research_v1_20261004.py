"""HH_261004 - Untimed synthetic CPU proof of guarded allocation reuse, not sensor/vehicle timing."""

import ast
import copy
from dataclasses import replace
import hashlib
from pathlib import Path
import types

import pytest
import torch
from torch import nn
from torch.nn.modules import module as torch_module

from portable_e2e.contract import ContractError
from portable_e2e.temporal_cached_encoder_v2 import (
    CachedTemporalModelV2, CameraFrameIdentityV2, TemporalCacheWindowV2, _CachedSpatialImageEncoderV2)
from portable_e2e import temporal_cpu_finite_validation_v2_20261001 as finite
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, TemporalModelV2
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from portable_e2e import temporal_all_valid_masked_copy_research_v1_20261004 as proposed


@pytest.fixture(scope="module", autouse=True)
def one_thread():
    # HH_261004 - A deterministic small untimed CPU fixture is not an actual performance benchmark.
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


PAIRS = (
    (CachedTemporalModelV2, proposed.AllValidMaskedCopyCachedTemporalModelV2),
    (finite.CPUFiniteCachedTemporalModelV2, proposed.AllValidMaskedCopyCPUFiniteCachedTemporalModelV2),
    (RouteRecoveryTemporalModelV3, proposed.AllValidMaskedCopyRouteRecoveryTemporalModelV3),
)


def models(pair=PAIRS[2]):
    config = TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)
    with torch.random.fork_rng(devices=[]):
        torch.manual_seed(4104)
        old = pair[0](config).eval()
        if isinstance(old, RouteRecoveryTemporalModelV3):
            # HH_261004 - Exercise a real nonzero learned route branch in synthetic fixtures, not just zero initialization.
            old.route_recovery_branch[-1].weight.data.fill_(.0002)
            old.route_recovery_branch[-1].bias.data.fill_(.03)
        new = pair[1](config).eval()
    # HH_261004 - Give both synthetic arms the same checkpoint-loaded reset event, not mismatched construction history.
    state = old.state_dict()
    old.load_state_dict(state, strict=True)
    new.load_state_dict(state, strict=True)
    return old, new


def example(step=0):
    anchor = 2_000_000_000 + step * 100_000_000
    stamps = [anchor + offset for offset in (-800_000_000, -600_000_000, -400_000_000, -200_000_000, 0)]
    frames, images = [], []
    for stamp in stamps:
        generator = torch.Generator().manual_seed(stamp // 100_000_000 + 4104)
        images.append(torch.rand(6, 3, 180, 320, generator=generator) * 2. - 1.)
        frames.append(CameraFrameIdentityV2(stamp, tuple(hashlib.sha256(f"synthetic:{stamp}:{camera}".encode()).hexdigest()
                                                          for camera in range(6))))
    inputs = dict(images=torch.stack(images)[None], calibration=torch.zeros(1, 6, 16),
        ego_history=torch.zeros(1, 10, 13), ego_history_mask=torch.ones(1, 10, dtype=torch.bool),
        route_xy=torch.zeros(1, 128, 2), route_mask=torch.ones(1, 128, dtype=torch.bool),
        time_valid=torch.ones(1, 5, dtype=torch.bool), relative_pose=torch.zeros(1, 5, 3),
        time_offsets_s=torch.tensor([[-.8, -.6, -.4, -.2, 0.]]))
    inputs["route_xy"][0, :, 0] = torch.arange(128) * .5
    inputs["route_xy"][0, :, 1] = .15 + .01 * inputs["route_xy"][0, :, 0]
    inputs["relative_pose"][0, :-1, 0] = inputs["time_offsets_s"][0, :-1]
    window = TemporalCacheWindowV2("all-valid-synthetic", "train", "synthetic-rig", "a" * 64, anchor, tuple(frames))
    return inputs, window


def raw_bytes(value):
    return value.detach().contiguous().numpy().tobytes()


def assert_exact(old, new):
    assert set(old) == set(new)
    assert all(raw_bytes(old[key]) == raw_bytes(new[key]) for key in old)


def observed_selection(monkeypatch):
    seen = []
    def observe(model, images, validity):
        selected = proposed.select_masked_images(model, images, validity)
        seen.append((selected is images, selected.data_ptr() == images.data_ptr()))
        return selected
    monkeypatch.setitem(proposed.AllValidMaskedCopyTemporalModelV2.forward.__globals__, "select_masked_images", observe)
    return seen


def test_source_pins_and_inverse_AST_prove_only_one_assignment_changed():
    proposed.require_frozen_parent_sources()
    original = proposed._forward_tree()
    changed = copy.deepcopy(original)
    old = ast.parse("masked_images = images.masked_fill(~effective_valid[:, :, None, None, None, None], 0.0)").body[0]
    new = ast.parse("masked_images = select_masked_images(self, images, effective_valid)").body[0]
    positions = [index for index, node in enumerate(changed.body[0].body) if ast.dump(node) == ast.dump(old)]
    assert len(positions) == 1
    changed.body[0].body[positions[0]] = new
    namespace = dict(TemporalModelV2.forward.__globals__, select_masked_images=proposed.select_masked_images)
    exec(compile(ast.fix_missing_locations(changed), str(Path(proposed.__file__)), "exec"), namespace)
    candidate = proposed.AllValidMaskedCopyTemporalModelV2.forward
    assert candidate.__code__.co_code == namespace["forward"].__code__.co_code
    assert candidate.__code__.co_names == namespace["forward"].__code__.co_names
    assert candidate._research_inverse_ast_equal is True
    assert candidate._research_original_ast_sha256 == hashlib.sha256(ast.dump(original, include_attributes=False).encode()).hexdigest()
    for pair in PAIRS:
        assert pair[1].forward_cached is pair[0].forward_cached
        assert pair[1]._validate_inputs is pair[0]._validate_inputs
        assert pair[1]._state_identity is pair[0]._state_identity
        mro = pair[1].__mro__
        assert mro.index(CachedTemporalModelV2) < mro.index(proposed.AllValidMaskedCopyTemporalModelV2) < mro.index(TemporalModelV2)


def test_source_drift_is_fail_closed_before_model_construction(monkeypatch):
    monkeypatch.setitem(proposed.PARENT_SOURCE_PINS, "temporal_model_v2.py", "0" * 64)
    with pytest.raises(ContractError, match="frozen parent source"):
        proposed.AllValidMaskedCopyCachedTemporalModelV2()


@pytest.mark.parametrize("pair", PAIRS)
def test_cold_warm_shift_interleaved_cache_misses_exact_bytes_and_no_source_state_mutation(pair, monkeypatch):
    old, new = models(pair)
    state_before = {key: raw_bytes(value) for key, value in new.state_dict().items()}
    seen = observed_selection(monkeypatch)
    expected_counts = [30, 0, 6, 30, 6, 0]
    for step, expected_count in zip((0, 0, 2, 1, 3, 3), expected_counts):
        inputs, window = example(step)
        before = {key: raw_bytes(value) for key, value in inputs.items()}
        with torch.no_grad():
            expected = old.forward_cached(inputs, window=window)
            actual = new.forward_cached(inputs, window=window)
        assert_exact(expected, actual)
        assert old.cache_stats == new.cache_stats
        assert new.cache_stats.cnn_images == expected_count
        assert before == {key: raw_bytes(value) for key, value in inputs.items()}
        assert new.image_encoder.request is None
        for entry in new.image_encoder.entries.values():
            assert entry.features.data_ptr() != inputs["images"].data_ptr()
            assert not entry.features.requires_grad
        if isinstance(new, RouteRecoveryTemporalModelV3):
            assert new._recovery_active is False and new._recovery_features is None
    assert seen == [(True, True)] * 6
    assert state_before == {key: raw_bytes(value) for key, value in new.state_dict().items()}


@pytest.mark.parametrize("kind", ["ordinary", "current_only", "missing_history"])
def test_ordinary_and_masked_paths_use_original_owned_allocation_and_exact_outputs(kind, monkeypatch):
    old, new = models()
    inputs, window = example()
    if kind == "missing_history":
        inputs["time_valid"][0, 0] = False
        window = replace(window, frames=(None,) + window.frames[1:])
    seen = observed_selection(monkeypatch)
    with torch.no_grad():
        if kind == "ordinary":
            expected, actual = old(**inputs), new(**inputs)
        else:
            mode = "current_only" if kind == "current_only" else "temporal"
            expected = old.forward_cached(inputs, window=window, history_mode=mode)
            actual = new.forward_cached(inputs, window=window, history_mode=mode)
    assert_exact(expected, actual)
    assert seen == [(False, False)]


def test_both_full_CPU_finite_scans_remain_and_second_scan_sees_input_mutation(monkeypatch):
    _, new = models()
    inputs, window = example()
    original = finite.cpu_float32_all_finite
    calls = []
    def observed(tensor):
        calls.append(tuple(tensor.shape))
        return original(tensor)
    monkeypatch.setattr(finite, "cpu_float32_all_finite", observed)
    with torch.no_grad(): new.forward_cached(inputs, window=window)
    assert len(calls) == 12 and calls.count((1, 5, 6, 3, 180, 320)) == 2
    calls.clear()
    def mutate_after_first_validation(tensor):
        result = original(tensor)
        calls.append(tuple(tensor.shape))
        if len(calls) == 6: inputs["images"][0, 0, 0, 0, 0, 0] = float("nan")
        return result
    monkeypatch.setattr(finite, "cpu_float32_all_finite", mutate_after_first_validation)
    with torch.no_grad(), pytest.raises(ContractError, match="only finite values"):
        new.forward_cached(inputs, window=window)
    assert new.image_encoder.request is None and not new.image_encoder.entries
    assert new._recovery_active is False and new._recovery_features is None


@pytest.mark.parametrize("scope", ["episode_id", "split", "rig_id", "source_sha256", "calibration"])
def test_scope_changes_reset_original_cache_in_both_models(scope):
    old, new = models()
    inputs, window = example()
    with torch.no_grad():
        for model in (old, new): model.forward_cached(inputs, window=window)
        if scope == "calibration": inputs["calibration"][0, 0, 0] = .5
        else: window = replace(window, **{scope: "val" if scope == "split" else "b" * 64})
        expected = old.forward_cached(inputs, window=window)
        actual = new.forward_cached(inputs, window=window)
    assert_exact(expected, actual)
    assert old.cache_stats == new.cache_stats
    assert new.cache_stats.reset_reason == "episode_split_rig_or_source_changed"


def test_pixel_file_SHA_and_weight_data_mutations_keep_original_guards():
    old, new = models()
    inputs, window = example()
    with torch.no_grad():
        for model in (old, new): model.forward_cached(inputs, window=window)
        inputs["images"][0, 0, 0, 0, 0, 0] += .03
        assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
        assert new.cache_stats.cnn_images == 6 and new.cache_stats.reused_frame_groups == 4
        frames = list(window.frames)
        frames[1] = replace(frames[1], camera_sha256=("c" * 64,) * 6)
        window = replace(window, frames=tuple(frames))
        assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
        assert new.cache_stats.cnn_images == 6
        for model in (old, new): model.curvature_head.bias.data.add_(.001)
        assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
        assert new.cache_stats.reset_reason == "model_state_changed" and new.cache_stats.cold_start


@pytest.mark.parametrize("kind", ["future", "bad_camera_SHA", "anchor", "no_grad", "training", "extra_field", "reentrant"])
def test_original_cached_and_recovery_rejections_remain(kind):
    old, new = models()
    inputs, window = example()
    if kind == "future": window = replace(window, frames=window.frames[:-1] + (replace(window.frames[-1], timestamp_ns=window.anchor_timestamp_ns + 1),))
    if kind == "bad_camera_SHA": window = replace(window, frames=(replace(window.frames[0], camera_sha256=("not-SHA",) * 6),) + window.frames[1:])
    if kind == "anchor": window = replace(window, anchor_timestamp_ns=True)
    if kind == "extra_field": inputs["synthetic_extra"] = torch.zeros(1)
    messages = []
    for model in (old, new):
        if kind == "training": model.train()
        if kind == "reentrant": model._recovery_active = True
        with torch.set_grad_enabled(kind == "no_grad"), pytest.raises(ContractError) as error:
            model.forward_cached(inputs, window=window)
        messages.append(str(error.value))
    assert messages[0] == messages[1]


@pytest.mark.parametrize("target", ["encoder", "features", "first_conv", "pool"])
def test_image_mutating_local_hooks_bypass_reuse_and_preserve_source_bytes(target, monkeypatch):
    old, new = models()
    inputs, window = example()
    before = raw_bytes(inputs["images"])
    def mutate(module, arguments): arguments[0].add_(.025)
    handles = []
    for model in (old, new):
        child = {"encoder": model.image_encoder, "features": model.image_encoder.features,
                 "first_conv": model.image_encoder.features[0], "pool": model.image_encoder.pool}[target]
        handles.append(child.register_forward_pre_hook(mutate))
    seen = observed_selection(monkeypatch)
    try:
        with torch.no_grad():
            assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
        assert seen == [(False, False)]
        assert raw_bytes(inputs["images"]) == before
    finally:
        for handle in handles: handle.remove()


@pytest.mark.parametrize("kind", ["instance_forward", "conv_subclass", "inplace_SiLU", "global_hook"])
def test_unexpected_image_mutating_paths_bypass_reuse_and_match_original_allocation(kind, monkeypatch):
    old, new = models()
    inputs, window = example()
    before = raw_bytes(inputs["images"])
    handle = None
    if kind == "instance_forward":
        for model in (old, new):
            original = model.image_encoder.forward
            def mutate(self, images, original=original):
                images.add_(.0125)
                return original(images)
            model.image_encoder.forward = types.MethodType(mutate, model.image_encoder)
    elif kind == "conv_subclass":
        class MutatingConv(nn.Conv2d):
            def forward(self, images):
                images.add_(.0125)
                return super().forward(images)
        for model in (old, new): model.image_encoder.features[0].__class__ = MutatingConv
    elif kind == "inplace_SiLU":
        for model in (old, new): model.image_encoder.features[2].inplace = True
    else:
        def mutate(module, args):
            if type(module) is _CachedSpatialImageEncoderV2: args[0].add_(.0125)
        if hasattr(torch_module, "register_module_forward_pre_hook"):
            handle = torch_module.register_module_forward_pre_hook(mutate)
        else:
            monkeypatch.setitem(torch_module._global_forward_pre_hooks, 900104, mutate)
    seen = observed_selection(monkeypatch)
    try:
        with torch.no_grad():
            assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
        assert seen == [(False, False)]
        assert raw_bytes(inputs["images"]) == before
    finally:
        if handle is not None: handle.remove()


def test_missing_or_extra_recovery_head_hook_is_rejected_before_encoder_work():
    _, model = models()
    inputs, window = example()
    handle = model.curvature_head.register_forward_hook(lambda module, args, output: output)
    try:
        with torch.no_grad(), pytest.raises(ContractError, match="permanent recovery logit hook"):
            model.forward_cached(inputs, window=window)
        assert model.image_encoder.request is None and not model.image_encoder.entries
    finally: handle.remove()


def test_no_copy_gate_requires_active_request_eval_no_grad_and_all_effective_valid():
    _, model = models(PAIRS[0])
    inputs, window = example()
    images, validity = inputs["images"], inputs["time_valid"]
    with torch.no_grad():
        assert proposed.select_masked_images(model, images, validity) is not images
        model.image_encoder.request = window.frames
        assert proposed.select_masked_images(model, images, validity) is images
        partial = validity.clone(); partial[:, 0] = False
        selected = proposed.select_masked_images(model, images, partial)
        assert selected is not images and selected.data_ptr() != images.data_ptr()
        assert bool((selected[:, 0] == 0).all())
        assert raw_bytes(selected[:, 1:]) == raw_bytes(images[:, 1:])
        model.image_encoder.features[0].train()
        assert proposed.select_masked_images(model, images, validity) is not images
        model.image_encoder.features[0].eval()
    assert proposed.select_masked_images(model, images, validity) is not images
    model.image_encoder.request = None


@pytest.mark.parametrize("field", ["images", "calibration", "ego_history", "route_xy", "relative_pose", "time_offsets_s"])
def test_nonfinite_masked_fields_remain_rejected_before_any_reuse(field):
    old, new = models()
    inputs, window = example()
    inputs["time_valid"][0, 0] = False
    inputs["ego_history_mask"][0, 0] = False
    inputs["route_mask"][0, 127] = False
    inputs[field].reshape(-1)[-1 if field == "route_xy" else 0] = float("inf")
    window = replace(window, frames=(None,) + window.frames[1:])
    for model in (old, new):
        with torch.no_grad(), pytest.raises(ContractError, match="finite values, including masked slots"):
            model.forward_cached(inputs, window=window)
        assert model.image_encoder.request is None and not model.image_encoder.entries


@pytest.mark.parametrize("kind", ["encoder_class_forward", "spatial_class_forward", "conv_class_forward", "extra_child", "encoder_subclass"])
def test_encoder_behavior_and_topology_changes_never_receive_borrowed_storage(kind, monkeypatch):
    _, model = models(PAIRS[0])
    inputs, window = example()
    if kind in ("encoder_class_forward", "spatial_class_forward", "conv_class_forward"):
        cls = {"encoder_class_forward": _CachedSpatialImageEncoderV2,
               "spatial_class_forward": proposed.SpatialImageEncoderV2,
               "conv_class_forward": nn.Conv2d}[kind]
        original = cls.forward
        def mutate(self, value):
            value.add_(.01)
            return original(self, value)
        monkeypatch.setattr(cls, "forward", mutate)
    elif kind == "extra_child": model.image_encoder.features.add_module("unexpected", nn.Identity())
    else:
        class ChangedEncoder(_CachedSpatialImageEncoderV2): pass
        model.image_encoder.__class__ = ChangedEncoder
    model.image_encoder.request = window.frames
    before = raw_bytes(inputs["images"])
    with torch.no_grad():
        selected = proposed.select_masked_images(model, inputs["images"], inputs["time_valid"])
    assert selected is not inputs["images"] and selected.data_ptr() != inputs["images"].data_ptr()
    assert raw_bytes(inputs["images"]) == before
    model.image_encoder.request = None


def test_noncontiguous_inputs_preserve_original_allocated_CNN_layout_and_exact_outputs(monkeypatch):
    old, new = models()
    inputs, window = example()
    images = inputs["images"].transpose(-1, -2).contiguous().transpose(-1, -2)
    assert images.shape == inputs["images"].shape and not images.is_contiguous()
    inputs["images"] = images
    before = raw_bytes(images)
    seen = observed_selection(monkeypatch)
    with torch.no_grad():
        assert_exact(old.forward_cached(inputs, window=window), new.forward_cached(inputs, window=window))
    assert seen == [(False, False)] and raw_bytes(images) == before
