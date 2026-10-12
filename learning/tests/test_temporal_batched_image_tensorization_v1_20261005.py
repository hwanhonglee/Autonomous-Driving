"""HH_261005 - Prove synthetic batch tensorization parity without weights, camera archives or neural execution."""

import ast
from collections import OrderedDict
from dataclasses import replace
import hashlib
from io import BytesIO
import re

import numpy as np
from PIL import Image
import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.model import ModelConfig
from portable_e2e import runtime
from portable_e2e import temporal_batched_image_tensorization_v1_20261005 as candidate
from portable_e2e import temporal_owned_camera_history_v2_20261001 as owned
from portable_e2e.temporal_single_copy_history_research_20261001 import SingleCopyOwnedHeadroomPolicyV2
from portable_e2e.temporal_simulation_research_v2_20261001 import ObservedTemporalFrameV2


IDENTITY = dict(episode_id="synthetic-batch-cache", split="train", rig_id="synthetic-rig", source_sha256="a"*64)


@pytest.fixture(autouse=True)
def bounded_cpu_threads():
    # HH_261005 - Synthetic correctness is not a timing benchmark; restore the caller's CPU setting.
    before = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(before)


def synthetic_inputs(pattern="ramp", encoding="bgra8", readonly=True, *, config=None):
    """HH_261005 - Generate six role-distinct full640x360 sources; no recorded payload is accessed."""
    config = ModelConfig() if config is None else config
    cameras = []
    rng = np.random.RandomState(261005)
    y, x = np.indices((360, 640))
    for index in range(6):
        if pattern == "extremes":
            rgb = np.empty((360, 640, 3), dtype=np.uint8)
            rgb[:] = [(index % 2)*255, ((index//2) % 2)*255, ((index//3) % 2)*255]
        elif pattern == "checker":
            rgb = np.stack((((x//3+y//5+index) % 2)*255, (x+17*index) % 256,
                            (y*3+x+31*index) % 256), axis=-1).astype(np.uint8)
        elif pattern == "noise":
            rgb = rng.randint(0, 256, (360, 640, 3), dtype=np.uint8)
        else:
            rgb = np.stack(((x+index) % 256, (y+index*23) % 256,
                            (x+y*7+index*41) % 256), axis=-1).astype(np.uint8)
        if encoding == "jpeg":
            stream = BytesIO()
            Image.fromarray(rgb, mode="RGB").save(stream, format="JPEG", quality=95)
            payload = stream.getvalue()
        else:
            payload = rgb if encoding == "rgb8" else rgb[..., ::-1].copy()
            if encoding == "bgra8":
                payload = np.concatenate((payload, np.full((360, 640, 1), 77+index, dtype=np.uint8)), axis=-1)
            if readonly:
                payload = np.frombuffer(payload.tobytes(), dtype=np.uint8).reshape(payload.shape)
        cameras.append(runtime.RuntimeCameraFrame(1_000_000_000, payload, encoding))
    calibration = (.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.)
    state = (1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.)
    return runtime.RuntimeInputs(tuple(cameras), (calibration,)*6, (state,)*config.ego_history_frames,
                                 (True,)*config.ego_history_frames, ((0., 0.), (60., 0.), (100., 1.)))


def tensor_bytes(value):
    return value.detach().cpu().contiguous().numpy().tobytes()


def frame(index):
    stamp = 1_000_000_000 + index*100_000_000
    inputs = synthetic_inputs("extremes", "bgra8")
    inputs = replace(inputs, camera_frames=tuple(replace(row, timestamp_ns=stamp) for row in inputs.camera_frames))
    return ObservedTemporalFrameV2(stamp, inputs, (index*.1, 0., 0.), (0., 0., 0., 1.), stamp, **IDENTITY)


def call(policy, method, current, past):
    return getattr(policy, method)(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
                                  ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)


def pixels(frame_value):
    payload = frame_value.payload
    return payload.tobytes() if isinstance(payload, np.ndarray) else bytes(payload)


def assert_outputs_equal(first, second):
    assert set(first) == set(second)
    for name in first:
        assert first[name].dtype == second[name].dtype
        assert first[name].shape == second[name].shape
        assert first[name].stride() == second[name].stride()
        assert tensor_bytes(first[name]) == tensor_bytes(second[name]), name
        assert torch.equal(first[name], second[name]), name
    assert owned.normalized_tensor_hashes(first) == owned.normalized_tensor_hashes(second)


def test_inverse_is_full_original_builder_except_one_image_provider_block():
    original, changed, restored = candidate.image_block_trees()
    assert ast.dump(original, include_attributes=False) == ast.dump(restored, include_attributes=False)
    assert ast.dump(original, include_attributes=False) != ast.dump(changed, include_attributes=False)
    assert "image_tensors = _research_image_provider(inputs, config)" in ast.unparse(changed)
    # HH_261005 - Independent inverse at the exact replacement, not a trusted boolean from the helper.
    old_body, new_body = original.body[0].body, changed.body[0].body
    start = next(i for i, node in enumerate(old_body) if isinstance(node, ast.AnnAssign)
                 and isinstance(node.target, ast.Name) and node.target.id == "image_tensors")
    assert ast.dump(ast.Module(body=old_body[:start], type_ignores=[])) == \
        ast.dump(ast.Module(body=new_body[:start], type_ignores=[]))
    assert ast.dump(ast.Module(body=old_body[start+3:], type_ignores=[])) == \
        ast.dump(ast.Module(body=new_body[start+1:], type_ignores=[]))


def test_compilation_does_not_mutate_runtime_or_single_copy_and_source_hashing_is_setup_only(monkeypatch):
    before = {name: id(value) for name, value in vars(runtime).items()}
    value = candidate.BatchedImageTensorizer()
    assert before == {name: id(item) for name, item in vars(runtime).items()}
    assert candidate.single_copy.build_runtime_tensors is runtime.build_runtime_tensors
    assert value.runtime_builder.__globals__ is not runtime.build_runtime_tensors.__globals__
    for name, item in runtime.build_runtime_tensors.__globals__.items():
        if name != "build_runtime_tensors":
            assert value.runtime_builder.__globals__[name] is item, name
    monkeypatch.setattr(candidate, "verify_frozen_sources", lambda: pytest.fail("per-call source rehash"))
    assert_outputs_equal(runtime.build_runtime_tensors(synthetic_inputs(), ModelConfig()),
                         value(synthetic_inputs(), ModelConfig()))


@pytest.mark.parametrize("pattern", ["extremes", "checker", "noise", "ramp"])
@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "bgra8", "jpeg"])
@pytest.mark.parametrize("readonly", [False, True])
def test_full_six_role_bitwise_parity_and_sources_unchanged(pattern, encoding, readonly):
    inputs = synthetic_inputs(pattern, encoding, readonly)
    before = [pixels(row) for row in inputs.camera_frames]
    value = candidate.BatchedImageTensorizer()
    original = runtime.build_runtime_tensors(inputs, ModelConfig())
    actual = value(inputs, ModelConfig())
    assert_outputs_equal(original, actual)
    assert [pixels(row) for row in inputs.camera_frames] == before
    actual["images"].zero_()
    assert [pixels(row) for row in inputs.camera_frames] == before
    assert value.last_timing["status"] == "PASS" and not value.last_timing["fallback_to_original"]


@pytest.mark.parametrize("size", [(16, 16), (64, 32), (320, 180), (640, 360)])
def test_output_sizes_keep_original_resize_and_model_abi(size):
    config = replace(ModelConfig(), image_width=size[0], image_height=size[1],
                     image_grid_width=1, image_grid_height=1)
    inputs = synthetic_inputs("checker", "bgra8")
    assert_outputs_equal(runtime.build_runtime_tensors(inputs, config), candidate.BatchedImageTensorizer()(inputs, config))


@pytest.mark.parametrize("fault", ["count", "timestamp_bool", "skew", "dtype", "shape", "encoding", "JPEG",
                                   "calibration", "history_nan", "history_overflow", "mask", "mask_suffix", "route"])
def test_original_error_type_message_and_failed_timing_are_preserved(fault):
    inputs = synthetic_inputs()
    cameras = list(inputs.camera_frames)
    if fault == "count": inputs = replace(inputs, camera_frames=tuple(cameras[:-1]))
    elif fault == "timestamp_bool": cameras[2] = replace(cameras[2], timestamp_ns=True)
    elif fault == "skew": cameras[2] = replace(cameras[2], timestamp_ns=1_001_000_001)
    elif fault == "dtype": cameras[2] = replace(cameras[2], payload=cameras[2].payload.astype(np.int16))
    elif fault == "shape": cameras[2] = replace(cameras[2], payload=np.zeros((12, 12, 4), dtype=np.uint8))
    elif fault == "encoding": cameras[2] = replace(cameras[2], encoding="unknown")
    elif fault == "JPEG": cameras[2] = replace(cameras[2], encoding="jpeg", payload=b"notJPEG")
    elif fault == "calibration": inputs = replace(inputs, calibration=((0.,)*16,)*5)
    elif fault in ("history_nan", "history_overflow"):
        row = (float("nan") if fault == "history_nan" else 1.e100,) + inputs.ego_history[0][1:]
        inputs = replace(inputs, ego_history=(row,)*10)
    elif fault == "mask": inputs = replace(inputs, ego_history_mask=(1,)*10)
    elif fault == "mask_suffix": inputs = replace(inputs, ego_history_mask=(True, False)+(True,)*8)
    elif fault == "route": inputs = replace(inputs, route_xy_base_m=((0., 0.), (float("nan"), 0.)))
    if fault in {"timestamp_bool", "skew", "dtype", "shape", "encoding", "JPEG"}:
        inputs = replace(inputs, camera_frames=tuple(cameras))
    with pytest.raises(ContractError) as old:
        runtime.build_runtime_tensors(inputs, ModelConfig())
    value = candidate.BatchedImageTensorizer()
    with pytest.raises(type(old.value)) as new:
        value(inputs, ModelConfig())
    # HH_261005 - PIL embeds each newly created BytesIO object's address in malformed-JPEG errors.
    # Preserve that genuine message and cause; compare only its inherently nondeterministic address semantically.
    if fault == "JPEG":
        assert re.sub(r"0x[0-9a-f]+", "0xOBJECT", str(new.value)) == \
            re.sub(r"0x[0-9a-f]+", "0xOBJECT", str(old.value))
        assert type(new.value.__cause__) is type(old.value.__cause__)
    else:
        assert str(new.value) == str(old.value)
    assert value.last_timing["status"] == "FAILED"
    assert value.last_timing["error_type"] == type(old.value).__name__
    assert value._current_timing is None


@pytest.mark.parametrize("binding", ["rgb", "mean", "resize"])
def test_unknown_implementation_falls_back_to_exact_original(binding, monkeypatch):
    value = candidate.BatchedImageTensorizer()
    if binding == "rgb":
        original = runtime._rgb_array
        monkeypatch.setattr(runtime, "_rgb_array", lambda *a, **k: original(*a, **k))
    elif binding == "mean": monkeypatch.setattr(runtime, "RGB_MEAN", (.4, .5, .6))
    else:
        original = Image.Image.resize
        monkeypatch.setattr(Image.Image, "resize", lambda *a, **k: original(*a, **k))
    inputs = synthetic_inputs()
    assert_outputs_equal(runtime.build_runtime_tensors(inputs, ModelConfig()), value(inputs, ModelConfig()))
    assert value.last_timing["fallback_to_original"] is True
    assert value.last_timing["camera_preprocessing"] == []


def test_nonfinite_fallback_retains_original_final_float32_guard(monkeypatch):
    value = candidate.BatchedImageTensorizer()
    monkeypatch.setattr(runtime, "RGB_MEAN", (float("inf"), .4, .5))
    for function in (runtime.build_runtime_tensors, value):
        with pytest.raises(ContractError, match="runtime images is outside the finite float32 range"):
            function(synthetic_inputs(), ModelConfig())
    assert value.last_timing["fallback_to_original"] and value.last_timing["status"] == "FAILED"


def test_scalar_timings_are_strictly_ordered_and_failed_research_receipt_does_not_hide_exception(monkeypatch):
    value = candidate.BatchedImageTensorizer()
    value(synthetic_inputs(), ModelConfig())
    receipt = value.last_timing
    current = receipt["started_monotonic_ns"]
    for index, row in enumerate(receipt["camera_preprocessing"]):
        assert row["role_index"] == index and row["role"] == runtime.CAMERA_ORDER[index]
        assert current <= row["started_monotonic_ns"] <= row["finished_monotonic_ns"]
        current = row["finished_monotonic_ns"]
    stage = receipt["batch_normalization"]
    assert current <= stage["started_monotonic_ns"] <= stage["finished_monotonic_ns"] <= receipt["finished_monotonic_ns"]
    assert receipt["research_only"] and not receipt["actual_speedup_proven"]
    # HH_261005 - An original normalization exception is retained, not converted into a success or a fallback retry.
    monkeypatch.setattr(value, "_images", lambda *a: (_ for _ in ()).throw(ArithmeticError("synthetic stage failure")))
    value.runtime_builder.__globals__["_research_image_provider"] = value._images
    with pytest.raises(ArithmeticError, match="synthetic stage failure"):
        value(synthetic_inputs(), ModelConfig())
    assert value.last_timing["call_ordinal"] == 2 and value.last_timing["status"] == "FAILED"


def no_model_policy():
    # HH_261005 - Exercise genuine builder/MRO/cache/hash methods without constructing any neural network.
    value = object.__new__(SingleCopyOwnedHeadroomPolicyV2)
    value.identity = tuple(IDENTITY[name] for name in ("episode_id", "split", "rig_id", "source_sha256"))
    value._images = OrderedDict()
    value._owned_camera_hashes = OrderedDict()
    value._last_owned_input_audit = None
    return value


@pytest.mark.parametrize("fail_exit", [False, True])
def test_actual_single_copy_alias_and_genuine_cache_full_tensor_hashes_are_preserved(fail_exit):
    sources = [owned.own_observed_camera_bytes(frame(index)) for index in range(12)]
    expected_policy, actual_policy = no_model_policy(), no_model_policy()
    expected = [call(expected_policy, "build_inputs", sources[index], sources[index-9:index]) for index in (9, 10, 11)]
    original_alias = candidate.single_copy.build_runtime_tensors
    original_builder = runtime.build_runtime_tensors
    try:
        with candidate.activated() as provider:
            assert candidate._SINGLE_COPY_METHOD.__globals__["build_runtime_tensors"] is provider
            assert runtime.build_runtime_tensors is original_builder
            for position, index in enumerate((9, 10, 11)):
                actual = call(actual_policy, "build_inputs", sources[index], sources[index-9:index])
                assert actual[1] == expected[position][1]
                assert_outputs_equal(actual[0], expected[position][0])
                assert actual_policy._last_owned_input_audit["normalized_input_tensor_sha256"] == \
                    owned.normalized_tensor_hashes(expected[position][0])
                assert actual_policy._last_owned_input_audit["every_source_camera_SHA_preserved"] is True
                assert actual_policy._last_owned_input_audit["inside_unchanged_hard_policy_deadline"] is True
            assert list(actual_policy._images) == list(expected_policy._images)
            for key in actual_policy._images:
                assert tensor_bytes(actual_policy._images[key]) == tensor_bytes(expected_policy._images[key])
            if fail_exit: raise LookupError("synthetic inner failure")
    except LookupError:
        assert fail_exit
    assert candidate.single_copy.build_runtime_tensors is original_alias
    assert runtime.build_runtime_tensors is original_builder
    assert not candidate._ACTIVE


def test_nested_and_wrong_provider_and_changed_alias_are_denied(monkeypatch):
    with candidate.activated():
        with pytest.raises(ContractError, match="cannot nest"):
            with candidate.activated(): pass
    with pytest.raises(ContractError, match="exact batched"):
        with candidate.activated(provider=lambda *args: None): pass
    monkeypatch.setattr(candidate.single_copy, "build_runtime_tensors", lambda *args: None)
    with pytest.raises(ContractError, match="actual SingleCopy"):
        with candidate.activated(): pass


def test_reentrant_calls_cannot_overwrite_inflight_receipt():
    value = candidate.BatchedImageTensorizer()
    value._current_timing = {"original": "retained"}
    with pytest.raises(ContractError, match="cannot recurse or overlap"):
        value(synthetic_inputs(), ModelConfig())
    assert value._current_timing == {"original": "retained"}
