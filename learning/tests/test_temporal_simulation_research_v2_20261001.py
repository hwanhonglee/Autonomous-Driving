"""HH_261001 - Verify causal V2 live inputs, data-only loading, startup prefill and strict deadlines."""

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.model import ModelConfig
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs, build_runtime_tensors
from portable_e2e.simulation_research import ControllerConfig
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e import temporal_simulation_research_v2_20261001 as live


IDENTITY = dict(episode_id="observed-startup", split="train", rig_id="six-camera-rig", source_sha256="a" * 64)


@pytest.fixture(scope="module", autouse=True)
def one_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def model():
    torch.manual_seed(2)
    result = CachedTemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    with torch.no_grad():
        result.curvature_head.weight.zero_(); result.curvature_head.bias.zero_()
        result.speed_head.weight.zero_(); result.speed_head.bias.fill_(math.log(math.expm1(2.)))
    result._research_temporal_checkpoint_sha256 = "1" * 64
    result._research_temporal_export_npz_sha256 = "2" * 64
    result._research_temporal_metadata_sha256 = "3" * 64
    return result


def frame(index, *, vx=1., vy=0., yaw=0., route_length=100.):
    stamp = 1_000_000_000 + index * 100_000_000
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    image[..., 0] = index % 256
    calibration = (.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.)
    state = (1., vx, vy, 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.)
    inputs = RuntimeInputs(tuple(RuntimeCameraFrame(stamp, image.copy(), "rgb8") for _ in range(6)),
        (calibration,) * 6, (state,) * 10, (True,) * 10, ((0., 0.), (route_length, 0.)))
    return live.ObservedTemporalFrameV2(stamp, inputs, (index * .1, 0., 0.),
        (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.)), stamp, **IDENTITY)


def policy(**kwargs):
    return live.TemporalSimulationResearchPolicyV2(model(), ControllerConfig(2.7, .6), **IDENTITY, **kwargs)


def build(policy, current, past=()):
    return policy.build_inputs(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
                              ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)


def prefill(policy, current, past=()):
    return policy.prefill(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
                         ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)


def step(policy, current, past=(), **kwargs):
    return policy.step(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
                       ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns, **kwargs)


def ready(policy, monkeypatch):
    monkeypatch.setattr(live.time, "perf_counter", lambda: 1.)
    frames = [frame(index) for index in range(11)]
    prefill(policy, frames[8], frames[:8])
    prefill(policy, frames[9], frames[1:9])
    return frames


def test_current_preprocessing_equals_unchanged_runtime_exactly():
    value = frame(8)
    tensors, window = build(policy(), value)
    original = build_runtime_tensors(value.inputs, ModelConfig())
    assert torch.equal(tensors["images"][0, -1], original["images"][0])
    for name in original:
        if name != "images":
            assert torch.equal(tensors[name], original[name])
    assert tensors["time_valid"].tolist() == [[False, False, False, False, True]]
    assert torch.count_nonzero(tensors["images"][:, :-1]) == 0
    assert window.frames[:-1] == (None,) * 4
    assert window.frames[-1].timestamp_ns == value.timestamp_ns


def test_real_past_pose_slots_and_image_identity_are_causal():
    frames = [frame(index) for index in range(9)]
    tensors, window = build(policy(), frames[-1], frames[:-1])
    assert tensors["time_valid"].tolist() == [[True] * 5]
    assert [row.timestamp_ns for row in window.frames] == [frames[i].timestamp_ns for i in (0, 2, 4, 6, 8)]
    assert tensors["relative_pose"][0, :, 0].tolist() == pytest.approx([-.8, -.6, -.4, -.2, 0.])
    assert tensors["time_offsets_s"].tolist()[0] == pytest.approx([-.8, -.6, -.4, -.2, 0.])
    for slot, index in enumerate((0, 2, 4, 6, 8)):
        expected = build_runtime_tensors(frames[index].inputs, ModelConfig())["images"][0]
        assert torch.equal(tensors["images"][0, slot], expected)


def test_relative_observed_pose_uses_current_vehicle_axes_not_map_delta():
    first, current = frame(0), frame(8, yaw=math.pi / 2.)
    tensors, _ = build(policy(), current, (first,))
    assert tensors["relative_pose"][0, 0].tolist() == pytest.approx([0., .8, -math.pi / 2.], abs=1.e-6)


@pytest.mark.parametrize("bad", ["future", "duplicate", "reversed", "cross_episode", "cross_split",
                                "cross_source", "cross_rig", "pose_nan", "quaternion", "camera_skew",
                                "future_past_pose", "old_past_pose"])
def test_invalid_observed_history_or_pose_is_rejected(bad):
    p, current, past = policy(), frame(8), [frame(0), frame(2)]
    if bad == "future": past.append(frame(9))
    if bad == "duplicate": past.append(past[-1])
    if bad == "reversed": past.reverse()
    if bad == "cross_episode": past[0] = replace(past[0], episode_id="another")
    if bad == "cross_split": past[0] = replace(past[0], split="val")
    if bad == "cross_source": past[0] = replace(past[0], source_sha256="b" * 64)
    if bad == "cross_rig": past[0] = replace(past[0], rig_id="another")
    if bad == "pose_nan": current = replace(current, position_map_m=(float("nan"), 0., 0.))
    if bad == "quaternion": current = replace(current, orientation_xyzw=(0., 0., 0., .5))
    if bad == "camera_skew": current = replace(current, timestamp_ns=current.timestamp_ns + 3001)
    if bad == "future_past_pose": past[0] = replace(past[0], pose_timestamp_ns=past[0].timestamp_ns + 1)
    if bad == "old_past_pose": past[0] = replace(past[0], pose_timestamp_ns=past[0].timestamp_ns - 1001)
    with pytest.raises(ContractError):
        build(p, current, past)


def test_changed_historical_source_pixels_are_rehashed_not_returned_from_cache():
    p, current, past = policy(), frame(8), frame(6)
    original, identity = build(p, current, (past,))
    past.inputs.camera_frames[0].payload[..., 1] = 127
    changed, new_identity = build(p, current, (past,))
    assert identity.frames[3].camera_sha256 != new_identity.frames[3].camera_sha256
    assert not torch.equal(original["images"][0, 3], changed["images"][0, 3])


def test_causal_nav_support_matches_arc_not_expert_distance_or_origin_distance():
    points = torch.tensor([[[100., 0.], [103., 4.], [109., 4.], [999., 999.]]])
    mask = torch.tensor([[True, True, True, False]])
    assert live.causal_route_support(points, mask).item() == 11.
    assert live.causal_route_support(torch.tensor([[[0., 0.], [100., 0.]]]), torch.tensor([[True, True]])).item() == 64.
    with pytest.raises(ContractError):
        live.causal_route_support(points, torch.tensor([[True, False, True, False]]))


def test_current_planar_speed_matches_trainer_without_clipping():
    p = policy()
    tensors, _ = build(p, frame(8, vx=1.2, vy=1.6))
    assert live.measured_current_speed(tensors).item() == 2.
    over, _ = build(p, frame(8, vx=2.50001))
    with pytest.raises(ContractError, match="no clipping"):
        live.measured_current_speed(over)
    reverse, _ = build(p, frame(8, vx=-.01))
    with pytest.raises(ContractError, match="reverse"):
        live.measured_current_speed(reverse)


def test_cold_control_is_full_brake_not_deadline_waiver(monkeypatch):
    p = policy()
    monkeypatch.setattr(live.time, "perf_counter", lambda: 1.)
    result = step(p, frame(8))
    assert result["control"]["brake"] == 1. and result["learned_control_proposed"] is False
    assert "prefill" in result["emergency_reason"]
    assert result["raw_output"] is None


def test_two_real_prefills_enable_one_actual_cached_raw_decoded_proposal(monkeypatch):
    p = policy()
    frames = ready(p, monkeypatch)
    assert len(p.prefill_records) == 2
    assert all(row["control_proposed"] is False and row["actuator_acknowledged"] is False for row in p.prefill_records)
    assert p.prefill_records[0]["cache"]["cnn_images"] == 30
    result = step(p, frames[10], frames[2:10])
    assert result["learned_control_proposed"] is True, result["emergency_reason"]
    assert result["learned_control_applied"] is False and result["control_source"] == "learned_trajectory_tracker"
    assert result["selected"]["candidate_index"] == 0
    assert len(result["decoded_output"]["candidate_xy"]) == 1
    assert result["cache"]["cnn_images"] == 6
    assert result["decoded_output"]["instantaneous_speed"][0] == 1.
    assert result["raw_output"]["intent_logits"] is not None
    assert result["runtime_intent"] == "explicit_cruise_GO_not_head"
    assert result["stop_intent_trained"] is False
    assert result["sensor_callback_or_actuator_ACK_included"] is False
    assert result["ten_hz_end_to_end_proven"] is False


@pytest.mark.parametrize("cause", ["emergency", "short_nav", "overspeed", "cadence", "missing_parity", "mutation"])
def test_rejected_proposals_preserve_full_brake_and_require_new_prefill(monkeypatch, cause):
    p = policy(longitudinal_pi=True)
    frames = ready(p, monkeypatch)
    current, past, kwargs = frames[10], frames[2:10], {}
    if cause == "emergency": kwargs["emergency_reason"] = "collision event"
    if cause == "short_nav": current = frame(10, route_length=17.)
    if cause == "overspeed": current = frame(10, vx=2.51)
    if cause == "cadence": current = frame(11)
    if cause == "missing_parity":
        p.model.reset_cache("unexpected_reset")
    if cause == "mutation":
        with torch.no_grad(): p.model.curvature_head.bias.add_(.01)
    result = step(p, current, past, **kwargs)
    assert result["learned_control_proposed"] is False
    assert result["control"]["brake"] == 1. and result["control"]["throttle"] == 0.
    assert p._prefill_count == 0 and p.pi.integral_error == 0.
    assert "prefill" in step(p, frame(12))["emergency_reason"]


def test_deadline_includes_preprocessing_decoder_gate_and_control(monkeypatch):
    p = policy(longitudinal_pi=True)
    frames = ready(p, monkeypatch)
    # HH_261001 - Clock order is total-start, NN-start, NN-end, full-pipeline deadline, reported-total.
    clock = iter((1., 1.01, 1.02, 1.10001, 1.10002))
    monkeypatch.setattr(live.time, "perf_counter", lambda: next(clock))
    result = step(p, frames[10], frames[2:10])
    assert result["raw_nn_seconds"] == pytest.approx(.01)
    assert result["total_seconds"] == pytest.approx(.10002)
    assert "deadline" in result["emergency_reason"]
    assert result["control"]["brake"] == 1. and p.pi.integral_error == 0.


def test_image_cache_remains_nine_real_groups_on_interleaved_stream(monkeypatch):
    p = policy()
    frames = ready(p, monkeypatch)
    for index in range(10, 18):
        if index >= len(frames): frames.append(frame(index))
        result = step(p, frames[index], frames[max(0, index-8):index])
        assert result["learned_control_proposed"] is True, result["emergency_reason"]
        assert len(p._images) <= 9 and result["cache"]["cnn_images"] == 6


def export(tmp_path):
    value = model()
    arrays = {name: tensor.cpu().numpy() for name, tensor in value.state_dict().items()}
    npz, metadata = tmp_path / "weights.npz", tmp_path / "metadata.json"
    np.savez(npz, **arrays)
    description = dict(schema=live.EXPORT_SCHEMA, model_config=value.config.to_dict(),
        checkpoint_sha256="1" * 64, npz_sha256=hashlib.sha256(npz.read_bytes()).hexdigest(),
        state_tensor_shapes={name: list(array.shape) for name, array in arrays.items()},
        state_tensor_dtypes={name: str(array.dtype) for name, array in arrays.items()},
        source_sha256={name: hashlib.sha256((live.ROOT / name).read_bytes()).hexdigest()
                       for name in live.REQUIRED_SOURCE_PINS},
        optimizer_updates=500, raw_spatial_only=False, decoder_integrated=True, stop_intent_trained=False,
        runtime_intent="explicit_cruise_GO_not_head", simulation_only=True, model_promoted=False,
        vehicle_control_approved=False, parent_checkpoint_sha256=live.PARENT_CHECKPOINT_SHA256,
        parent_source_optimizer_updates=200, known_total_optimizer_updates=700,
        training_dataset_fingerprint_sha256="b" * 64, one_episode_screening=True,
        main_training=False, independent_validation=False)
    metadata.write_text(json.dumps(description))
    return npz, metadata, description, arrays


def load(npz, metadata):
    return live.load_npz(npz, metadata, weights_sha256=hashlib.sha256(npz.read_bytes()).hexdigest(),
                         metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest())


def test_data_only_export_loading_exact_tensor_parity(tmp_path):
    npz, metadata, description, arrays = export(tmp_path)
    loaded = load(npz, metadata)
    assert loaded._research_temporal_checkpoint_sha256 == description["checkpoint_sha256"]
    assert loaded.training is False and all(not x.requires_grad for x in loaded.parameters())
    assert all(np.array_equal(loaded.state_dict()[name].cpu().numpy(), array) for name, array in arrays.items())


@pytest.mark.parametrize("field,bad", [("schema", "raw_only"), ("raw_spatial_only", True),
    ("decoder_integrated", False), ("stop_intent_trained", True), ("runtime_intent", "predicted_GO"),
    ("simulation_only", False), ("model_promoted", True), ("vehicle_control_approved", True),
    ("optimizer_updates", False), ("checkpoint_sha256", "unpinned"), ("optimizer_updates", 3001),
    ("parent_checkpoint_sha256", "f" * 64), ("parent_source_optimizer_updates", 201),
    ("known_total_optimizer_updates", 500), ("one_episode_screening", False),
    ("main_training", True), ("independent_validation", True)])
def test_wrong_export_boundaries_are_rejected(tmp_path, field, bad):
    npz, metadata, description, _ = export(tmp_path)
    description[field] = bad
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError): load(npz, metadata)


def test_wrong_caller_sha_rejected_before_loading(tmp_path):
    npz, metadata, _, _ = export(tmp_path)
    with pytest.raises(ContractError, match="identity"):
        live.load_npz(npz, metadata, weights_sha256="f" * 64,
                      metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest())


@pytest.mark.parametrize("kind", ["missing_tensor", "wrong_shape", "wrong_dtype", "nonfinite"])
def test_invalid_numeric_tensor_export_is_rejected(tmp_path, kind):
    npz, metadata, description, arrays = export(tmp_path)
    name = next(iter(arrays))
    if kind == "missing_tensor": arrays.pop(name)
    if kind == "wrong_shape": arrays[name] = np.zeros((1,), dtype=np.float32)
    if kind == "wrong_dtype": arrays[name] = arrays[name].astype(np.float64)
    if kind == "nonfinite": arrays[name] = arrays[name].copy(); arrays[name].flat[0] = float("nan")
    np.savez(npz, **arrays)
    description["npz_sha256"] = hashlib.sha256(npz.read_bytes()).hexdigest()
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError): load(npz, metadata)


@pytest.mark.parametrize("name", ["../outside.py", "/tmp/outside.py", "learning/../outside.py"])
def test_exported_source_paths_cannot_escape_root(tmp_path, name):
    npz, metadata, description, _ = export(tmp_path)
    description["source_sha256"][name] = "a" * 64
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError, match="relative"):
        load(npz, metadata)


def test_missing_critical_training_source_pin_rejected(tmp_path):
    npz, metadata, description, _ = export(tmp_path)
    description["source_sha256"].pop("learning/portable_e2e/path_time_decoder_v2.py")
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError, match="critical"):
        load(npz, metadata)


def deterministic_export(tmp_path):
    npz, metadata, description, arrays = export(tmp_path)
    description["schema"] = live.DETERMINISTIC_EXPORT_SCHEMA
    numerical_sources = {name: hashlib.sha256((live.ROOT / name).read_bytes()).hexdigest()
                         for name in live.DETERMINISTIC_SOURCE_PINS}
    description["source_sha256"].update(numerical_sources)
    description["numerical_revision"] = dict(
        kernel_id="fixed33_to257_align_corners_dense_gemm.20261001.v1", source_sha256=numerical_sources,
        strict_deterministic_algorithms=True, tf32_matrix_multiplication=False,
        frozen_decoder_sha256=live.FROZEN_DECODER_SHA256,
        frozen_training_orchestrator_sha256=live.FROZEN_TRAINER_SHA256,
        interpolation_math="same33-to257 weights denseGEMM", finite_precision_bitwise_equivalence_claimed=False)
    metadata.write_text(json.dumps(description))
    return npz, metadata, description, arrays


def test_explicit_deterministic_training_revision_loads_original_cpu_inference_model(tmp_path):
    npz, metadata, description, arrays = deterministic_export(tmp_path)
    loaded = load(npz, metadata)
    assert isinstance(loaded, CachedTemporalModelV2)
    assert loaded._research_temporal_export_metadata["numerical_revision"] == description["numerical_revision"]
    assert all(np.array_equal(loaded.state_dict()[name].cpu().numpy(), value) for name, value in arrays.items())


@pytest.mark.parametrize("field,value", [("kernel_id", "undeclared"),
    ("strict_deterministic_algorithms", False), ("tf32_matrix_multiplication", True),
    ("finite_precision_bitwise_equivalence_claimed", True),
    ("frozen_decoder_sha256", "f" * 64), ("frozen_training_orchestrator_sha256", "f" * 64)])
def test_malformed_deterministic_numerical_revision_rejected(tmp_path, field, value):
    npz, metadata, description, _ = deterministic_export(tmp_path)
    description["numerical_revision"][field] = value
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError, match="revision"):
        load(npz, metadata)


def test_missing_deterministic_kernel_source_pin_rejected(tmp_path):
    npz, metadata, description, _ = deterministic_export(tmp_path)
    description["source_sha256"].pop("learning/portable_e2e/deterministic_linear_interpolation_v2.py")
    metadata.write_text(json.dumps(description))
    with pytest.raises(ContractError, match="kernel"):
        load(npz, metadata)


def test_runtime_geometry_gate_failure_is_retained_not_reranked(monkeypatch):
    p = policy()
    frames = ready(p, monkeypatch)
    def rejected(*_args, **_kwargs):
        raise ContractError("deliberate independent runtime geometry rejection")
    monkeypatch.setattr(live, "validate_and_select_trajectory", rejected)
    result = step(p, frames[10], frames[2:10])
    assert result["control"]["brake"] == 1.
    assert result["raw_output"] is not None and result["decoded_output"] is not None
    assert result["candidate_gates"] == [dict(index=0, passed=False,
        reason="deliberate independent runtime geometry rejection")]
    assert result["selected"] is None and result["learned_control_proposed"] is False
