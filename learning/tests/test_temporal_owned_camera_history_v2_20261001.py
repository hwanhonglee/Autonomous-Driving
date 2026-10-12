"""HH_261001 - Prove immutable source ownership, bounded SHA reuse and frozen neural/control numerical parity."""

from dataclasses import replace
import hashlib
import math

import numpy as np
import pytest
import torch

from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.simulation_research import ControllerConfig
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e import temporal_owned_camera_history_v2_20261001 as owned
from portable_e2e import temporal_simulation_research_v2_20261001 as frozen
from scripts.e2e import collect_temporal_v2_owned_camera_research_20261001 as collector


IDENTITY = dict(episode_id="unit-real-boundary-fixture", split="train", rig_id="rig", source_sha256="a"*64)


@pytest.fixture(autouse=True)
def one_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def model():
    torch.manual_seed(7)
    value = CachedTemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    with torch.no_grad():
        value.curvature_head.weight.zero_(); value.curvature_head.bias.zero_()
        value.speed_head.weight.zero_(); value.speed_head.bias.fill_(math.log(math.expm1(2.)))
    value._research_temporal_checkpoint_sha256 = "1"*64
    value._research_temporal_export_npz_sha256 = "2"*64
    value._research_temporal_metadata_sha256 = "3"*64
    return value


def frame(index):
    stamp = 1_000_000_000 + index*100_000_000
    calibration = (.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.)
    state = (1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.)
    cameras = []
    for channel in range(6):
        pixels = np.full((360, 640, 4), index+channel, dtype=np.uint8)
        cameras.append(RuntimeCameraFrame(stamp, pixels, "bgra8"))
    inputs = RuntimeInputs(tuple(cameras), (calibration,)*6, (state,)*10, (True,)*10, ((0., 0.), (100., 0.)))
    return frozen.ObservedTemporalFrameV2(stamp, inputs, (index*.1, 0., 0.), (0., 0., 0., 1.), stamp, **IDENTITY)


def policy(kind=owned.OwnedCameraTemporalSimulationResearchPolicyV2, **kwargs):
    return kind(model(), ControllerConfig(2.7, .6), **IDENTITY, longitudinal_pi=True, **kwargs)


def call(policy, method, current, past):
    return getattr(policy, method)(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
                                  ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns)


def ready(p, frames):
    call(p, "prefill", frames[8], frames[:8])
    call(p, "prefill", frames[9], frames[:9])


def test_own_source_preserves_every_pixel_and_makes_bytes_backing_nonmutable():
    original = frame(2)
    value = owned.own_observed_camera_bytes(original)
    assert value.timestamp_ns == original.timestamp_ns and value.inputs.ego_history == original.inputs.ego_history
    for first, second in zip(original.inputs.camera_frames, value.inputs.camera_frames):
        assert np.array_equal(first.payload, second.payload)
        assert owned.immutable_camera_backing(second.payload) is not None
        first.payload[..., 0] = 255
        assert second.payload[0, 0, 0] != 255
        with pytest.raises(ValueError): second.payload.setflags(write=True)
        with pytest.raises(ValueError): second.payload[0, 0, 0] = 255


@pytest.mark.parametrize("kind", ["mutable", "readonly_flag", "readonly_memoryview", "dtype", "shape", "encoding"])
def test_unknown_or_flag_only_sources_never_gain_immutable_hash_reuse(kind):
    camera = frame(0).inputs.camera_frames[0]
    source = camera.payload
    if kind == "readonly_flag": source.flags.writeable = False
    elif kind == "readonly_memoryview":
        source = np.frombuffer(memoryview(bytearray(source.tobytes())).toreadonly(), dtype=np.uint8).reshape(360, 640, 4)
    elif kind == "dtype": source = source.astype(np.int16)
    elif kind == "shape": source = source.transpose(1, 0, 2)
    elif kind == "encoding": camera = replace(camera, encoding="rgb8")
    assert owned.immutable_camera_backing(source) is None
    p = policy()
    p._admit_camera(replace(camera, payload=source))
    assert len(p._owned_camera_hashes) == 0


def test_immutable_source_hash_and_normalized_tensors_equal_original_exactly():
    original = [frame(i) for i in range(10)]
    sources = [owned.own_observed_camera_bytes(f) for f in original]
    p, base = policy(), policy(frozen.TemporalSimulationResearchPolicyV2)
    tensors, window = call(p, "build_inputs", sources[-1], sources[:-1])
    expected, expected_window = call(base, "build_inputs", original[-1], original[:-1])
    assert window == expected_window
    for name in tensors: assert torch.equal(tensors[name], expected[name]), name
    assert p._last_owned_input_audit["normalized_input_tensor_sha256"] == owned.normalized_tensor_hashes(expected)
    assert p._last_owned_input_audit["source_hashes_new"] == 60
    assert p._last_owned_input_audit["source_hashes_reused"] == 60


def test_rawNN_decoder_gate_selected_and_control_are_numerically_identical(monkeypatch):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    originals = [frame(i) for i in range(11)]
    sources = [owned.own_observed_camera_bytes(f) for f in originals]
    base, proposed = policy(frozen.TemporalSimulationResearchPolicyV2), policy()
    ready(base, originals); ready(proposed, sources)
    first = call(base, "step", originals[10], originals[1:10])
    second = call(proposed, "step", sources[10], sources[1:10])
    assert first["learned_control_proposed"] is True and second["learned_control_proposed"] is True
    for name in ("raw_output", "decoded_output", "candidate_gates", "selected", "control", "longitudinal_pi_research"):
        assert first[name] == second[name], name
    audit = second["owned_camera_history"]
    assert audit["source_hashes_new"] == 6 and audit["source_hashes_reused"] == 60
    assert audit["mutable_or_unknown_source_rehashes"] == 0
    assert audit["bounded_strong_reference_camera_count"] == 60
    assert len(audit["normalized_input_tensor_sha256"]) == 9


def test_bounded_strong_cache_eviction_and_reset_keep_no_prior_epoch_authority():
    p = policy()
    for index in range(20):
        value = owned.own_observed_camera_bytes(frame(index))
        call(p, "build_inputs", value, ())
        assert len(p._owned_camera_hashes) <= 60
    p.reset("new_source_epoch")
    assert len(p._owned_camera_hashes) == 0 and p._prefill_count == 0


@pytest.mark.parametrize("kind", ["cross_source", "duplicate", "shape_mutation", "dtype_mutation", "cadence"])
def test_source_epoch_duplicate_payload_mutation_or_cadence_rejects_and_resets(monkeypatch, kind):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    frames = [owned.own_observed_camera_bytes(frame(i)) for i in range(11)]
    p = policy(); ready(p, frames)
    current, past = frames[10], list(frames[1:10])
    if kind == "cross_source": current = replace(current, source_sha256="b"*64)
    elif kind == "duplicate": past[-1] = past[-2]
    elif kind == "shape_mutation": past[-2].inputs.camera_frames[0].payload.shape = (640, 360, 4)
    elif kind == "dtype_mutation": past[-2].inputs.camera_frames[0].payload.dtype = np.int16
    elif kind == "cadence":
        current = owned.own_observed_camera_bytes(frame(11))
    result = call(p, "step", current, past)
    assert result["learned_control_proposed"] is False and result["control"]["brake"] == 1.
    assert p._prefill_count == 0 and len(p._owned_camera_hashes) == 0


def test_readonly_mutable_backing_changes_rehash_and_change_exact_source_identity():
    originals = [frame(i) for i in range(10)]
    p = policy()
    for value in originals:
        for camera in value.inputs.camera_frames: camera.payload.flags.writeable = False
    first, old = call(p, "build_inputs", originals[-1], originals[:-1])
    originals[7].inputs.camera_frames[0].payload.flags.writeable = True
    originals[7].inputs.camera_frames[0].payload[..., 0] = 199
    originals[7].inputs.camera_frames[0].payload.flags.writeable = False
    second, new = call(p, "build_inputs", originals[-1], originals[:-1])
    assert old.frames[3].camera_sha256 != new.frames[3].camera_sha256
    assert not torch.equal(first["images"], second["images"])
    assert p._last_owned_input_audit["mutable_or_unknown_source_rehashes"] == 60
    assert p._last_owned_input_audit["source_hashes_new"] == 0


def test_ownership_and_tensor_hashing_stay_inside_original_hard_deadline(monkeypatch):
    frames = [owned.own_observed_camera_bytes(frame(i)) for i in range(11)]
    p = policy()
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    ready(p, frames)
    original_build = p.build_inputs
    def slow(*args, **kwargs):
        result = original_build(*args, **kwargs)
        monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.101)
        return result
    monkeypatch.setattr(p, "build_inputs", slow)
    result = call(p, "step", frames[10], frames[1:10])
    assert result["learned_control_proposed"] is False
    assert result["emergency_reason"] == "temporal inference/control deadline exceeded"
    assert result["control"]["brake"] == 1. and p._prefill_count == 0


@pytest.mark.parametrize("cause", ["cadence", "external"])
def test_early_rejection_never_attaches_previous_frame_owned_input_proof(monkeypatch, cause):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    frames = [owned.own_observed_camera_bytes(frame(i)) for i in range(13)]
    p = policy(); ready(p, frames)
    first = call(p, "step", frames[10], frames[1:10])
    assert first["owned_camera_history"]["frame_timestamp_ns"] == frames[10].timestamp_ns
    current = frames[12] if cause == "cadence" else frames[11]
    result = p.step(current, past_frames=tuple(frames[3:12]) if cause == "cadence" else tuple(frames[2:11]),
        now_ns=current.timestamp_ns, ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns,
        emergency_reason="outside emergency" if cause == "external" else None)
    assert result["learned_control_proposed"] is False and result["owned_camera_history"] is None


def test_owned_collector_class_activation_restores_on_nested_attempt_and_exception():
    first = collector.frozen.TemporalLiveResearchSessionV2
    second = collector.frozen.TemporalSimulationResearchPolicyV2
    with pytest.raises(RuntimeError, match="stop"):
        with collector.activated(None, None):
            assert collector.frozen.TemporalLiveResearchSessionV2 is collector.OwnedCameraTemporalLiveResearchSessionV2
            assert collector.frozen.TemporalSimulationResearchPolicyV2 is owned.OwnedCameraTemporalSimulationResearchPolicyV2
            with pytest.raises(RuntimeError, match="nest"):
                with collector.activated(None, None): pass
            raise RuntimeError("stop")
    assert collector.frozen.TemporalLiveResearchSessionV2 is first
    assert collector.frozen.TemporalSimulationResearchPolicyV2 is second
