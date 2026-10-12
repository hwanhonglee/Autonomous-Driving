"""HH_261001 - Check real JPEG parity and byte validation in the offline decoded cache."""

from dataclasses import replace
import hashlib
import os
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import torch

from portable_e2e.contract import ContractError, MAX_JPEG_FILE_BYTES
from portable_e2e.dataset import TrainingExample
from portable_e2e.model import ModelConfig
from portable_e2e.temporal_dataset_v2 import TemporalCommon10TorchDataset, TemporalSampleMetadata
from portable_e2e.temporal_decoded_cache_v2 import (
    DecodedCacheTemporalDatasetV2,
    FROZEN_CONTRACT_SOURCE_SHA256,
    FROZEN_TEMPORAL_DATASET_SOURCE_SHA256,
    FROZEN_TORCH_DATASET_SOURCE_SHA256,
    MAX_CACHED_IMAGES,
)


@pytest.fixture(scope="module", autouse=True)
def _one_cpu_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


@pytest.fixture
def jpeg_stream(tmp_path):
    """HH_261001 - Actual 640x360 JPEG files with spatial and camera-specific pixel variation."""
    y, x = np.indices((360, 640))

    def build(count=12, *, episode="episode-a", rig="rig-one", split="train", source="b" * 64):
        samples, raw = [], {}
        for index in range(count):
            paths, hashes = [], []
            for camera in range(6):
                path = tmp_path / episode / f"{index:06d}-{camera}.jpg"
                path.parent.mkdir(parents=True, exist_ok=True)
                pixels = np.stack(((x + index * 13) % 256, (y + camera * 23) % 256,
                                   (x // 3 + y // 2 + index * 7 + camera * 11) % 256), axis=-1).astype(np.uint8)
                Image.fromarray(pixels, "RGB").save(path, format="JPEG", quality=87)
                paths.append(path)
                hashes.append(hashlib.sha256(path.read_bytes()).hexdigest())
            sample = TrainingExample(
                token=f"{episode}:{index:06d}", episode_id=episode,
                features=(1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.),
                targets_xy=tuple(((future + 1) * .1, 0.) for future in range(64)),
                target_yaw_rad=(0.,) * 64, target_speed_mps=(1.,) * 64,
                sequence_index=index, anchor_timestamp_ns=1_000_000_000 + index * 100_000_000,
                camera_paths=tuple(paths), camera_sha256=tuple(hashes),
                camera_calibration=((0.,) * 16,) * 6,
                route_points_base_xy_m=((0., 0.), (100., 0.)), rig_id=rig,
                source_dataset_id="fixture", source_dataset_version="v1", source_manifest_sha256="c" * 64,
            )
            samples.append(sample)
            raw[sample.token] = TemporalSampleMetadata(
                sample_id=sample.token, episode_id=episode, split=split, sequence_index=index,
                anchor_timestamp_ns=sample.anchor_timestamp_ns, ego_timestamp_ns=sample.anchor_timestamp_ns,
                position_map_m=(float(index), 0., 0.), orientation_xyzw=(0., 0., 0., 1.),
                camera_sha256=sample.camera_sha256, sample_jsonl_sha256=source,
            )
        return samples, raw

    return build


def _dataset(samples, raw, *, cached=True, **kwargs):
    config = kwargs.pop("model_config", ModelConfig(image_height=32, image_width=32,
                                                  image_grid_height=2, image_grid_width=2))
    cls = DecodedCacheTemporalDatasetV2 if cached else TemporalCommon10TorchDataset
    if cached:
        kwargs.setdefault("mode", "evaluation")
    return cls(samples, config, raw_metadata=raw, **kwargs)


def _assert_equal(actual, expected):
    assert actual.keys() == expected.keys()
    for name in actual:
        if isinstance(actual[name], torch.Tensor):
            assert torch.equal(actual[name], expected[name]), name
        else:
            assert actual[name] == expected[name], name


def test_real_jpeg_bit_exact_frozen_outputs_and_interleaved_history_reuse(jpeg_stream):
    samples, raw = jpeg_stream(15)
    cached, original = _dataset(samples, raw), _dataset(samples, raw, cached=False)
    assert cached.fingerprint_sha256 == original.fingerprint_sha256
    assert cached.split == "train" and cached.allow_training is False
    assert cached.cache_source_pin["adapter_sha256"] != cached.fingerprint_sha256
    for index in range(8, 15):
        _assert_equal(cached[index], original[index])
        stats = cached.cache_stats
        assert stats.last_call_hash_checks == 30
        assert stats.last_call_misses == (30 if index < 10 else 6)
        assert stats.last_call_hits == (0 if index < 10 else 24)
        assert stats.retained_images <= 54
        assert stats.retained_bytes <= 54 * 3 * 32 * 32 * 4
    assert stats.successful_calls == 7 and stats.failed_calls == 0
    assert stats.reset_count == 0 and stats.peak_retained_images == 54


def test_returned_image_tensors_cannot_corrupt_cache(jpeg_stream):
    samples, raw = jpeg_stream(9)
    cached, original = _dataset(samples, raw), _dataset(samples, raw, cached=False)
    first = cached[8]
    first["images"].fill_(float("nan"))
    second = cached[8]
    _assert_equal(second, original[8])
    assert torch.isfinite(second["images"]).all()
    assert cached.cache_stats.last_call_hits == 30
    second["images"].zero_()
    _assert_equal(cached[8], original[8])


def test_every_warm_image_access_reads_original_file(jpeg_stream, monkeypatch):
    samples, raw = jpeg_stream(9)
    cached = _dataset(samples, raw)
    cached[8]
    read = cached._read_regular_file
    accesses = []

    def recording_read(path):
        accesses.append(path)
        return read(path)

    monkeypatch.setattr(cached, "_read_regular_file", recording_read)
    cached[8]
    assert len(accesses) == 30
    assert set(accesses) == {path for index in (0, 2, 4, 6, 8) for path in samples[index].camera_paths}
    assert cached.cache_stats.last_call_hash_checks == 30


def test_warm_bytes_changed_with_same_size_and_restored_mtime_are_rejected(jpeg_stream):
    samples, raw = jpeg_stream(9)
    cached = _dataset(samples, raw)
    cached[8]
    path = samples[8].camera_paths[0]
    before = path.stat()
    payload = bytearray(path.read_bytes())
    payload[-1] ^= 1
    path.write_bytes(payload)
    os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
    assert path.stat().st_size == before.st_size and path.stat().st_mtime_ns == before.st_mtime_ns
    with pytest.raises(ContractError, match="changed after validation"):
        cached[8]
    assert cached.cache_stats.retained_images == 0
    assert cached.cache_stats.failed_calls == 1 and cached.cache_stats.reset_reason == "failed_access"


@pytest.mark.parametrize("replacement", ["symlink", "directory", "empty", "oversized"])
def test_warm_file_type_and_bounded_read_gates_are_preserved(jpeg_stream, replacement):
    samples, raw = jpeg_stream(1)
    cached = _dataset(samples, raw)
    cached[0]
    path = samples[0].camera_paths[0]
    backup = path.with_suffix(".original")
    path.rename(backup)
    if replacement == "symlink":
        path.symlink_to(backup)
        message = "symbolic link"
    elif replacement == "directory":
        path.mkdir()
        message = "regular file"
    elif replacement == "empty":
        path.touch()
        message = "empty"
    else:
        with path.open("wb") as stream:
            stream.truncate(MAX_JPEG_FILE_BYTES + 1)
        message = "exceeds limit"
    with pytest.raises(ContractError, match=message):
        cached[0]
    assert cached.cache_stats.retained_images == 0


@pytest.mark.parametrize("kwargs", [
    {"verify_image_sha256": False}, {"verify_image_sha256": 1},
    {"allow_training": True}, {"allow_training": 0},
    {"mode": "training"}, {"mode": "eval"},
    {"max_cached_images": 55}, {"max_cached_images": 0},
    {"max_cached_images": True}, {"max_cached_images": 5.5},
])
def test_unsafe_or_implicit_cache_configuration_rejected(jpeg_stream, kwargs):
    samples, raw = jpeg_stream(1)
    with pytest.raises(ContractError):
        _dataset(samples, raw, **kwargs)


def test_explicit_evaluation_mode_is_required(jpeg_stream):
    samples, raw = jpeg_stream(1)
    with pytest.raises(TypeError, match="mode"):
        DecodedCacheTemporalDatasetV2(samples, raw_metadata=raw)


@pytest.mark.parametrize("name,value", [("verify_image_sha256", False), ("allow_training", True), ("mode", "training")])
def test_mutated_usage_flags_cannot_weaken_warm_cache(jpeg_stream, name, value):
    samples, raw = jpeg_stream(1)
    cached = _dataset(samples, raw)
    cached[0]
    setattr(cached, name, value)
    with pytest.raises(ContractError):
        cached[0]
    assert cached.cache_stats.retained_images == 0


def test_wrong_recorded_hash_is_not_replaced_with_current_file_hash(jpeg_stream):
    samples, raw = jpeg_stream(1)
    sample = replace(samples[0], camera_sha256=("a" * 64,) * 6)
    raw[sample.token] = replace(raw[sample.token], camera_sha256=sample.camera_sha256)
    cached = _dataset([sample], raw)
    with pytest.raises(ContractError, match="changed after validation"):
        cached[0]
    assert cached.cache_stats.hits == 0 and cached.cache_stats.retained_images == 0


def test_episode_scope_change_and_clock_reversal_reset(jpeg_stream):
    first, raw_first = jpeg_stream(10)
    other, raw_other = jpeg_stream(9, episode="episode-b", rig="rig-two", source="d" * 64)
    samples, raw = first + other, {**raw_first, **raw_other}
    cached, original = _dataset(samples, raw), _dataset(samples, raw, cached=False)
    cached[8]
    cached[9]
    _assert_equal(cached[8], original[8])
    assert cached.cache_stats.reset_reason == "clock_reversed"
    assert cached.cache_stats.last_call_misses == 30
    _assert_equal(cached[18], original[18])
    assert cached.cache_stats.reset_reason == "provenance_changed"
    assert cached.cache_stats.last_call_misses == 30 and cached.cache_stats.reset_count == 2


@pytest.mark.parametrize("scope", ["source", "rig", "split"])
def test_changed_provenance_cannot_reuse_warm_images(jpeg_stream, scope):
    samples, raw = jpeg_stream(9)
    cached = _dataset(samples, raw)
    cached[8]
    if scope == "source":
        cached.raw_metadata = {key: replace(value, sample_jsonl_sha256="d" * 64)
                               for key, value in cached.raw_metadata.items()}
    elif scope == "rig":
        cached.examples = tuple(replace(sample, rig_id="rig-two") for sample in cached.examples)
    else:
        cached.split = "val"
        cached.raw_metadata = {key: replace(value, split="val") for key, value in cached.raw_metadata.items()}
    cached[8]
    assert cached.cache_stats.last_call_misses == 30
    assert cached.cache_stats.last_call_hits == 0
    assert cached.cache_stats.reset_count == 1 and cached.cache_stats.reset_reason == "provenance_changed"


def test_configuration_change_resets_and_matches_frozen_decoder(jpeg_stream):
    samples, raw = jpeg_stream(9)
    cached = _dataset(samples, raw)
    cached[8]
    config = ModelConfig(image_height=48, image_width=48, image_grid_height=3, image_grid_width=3)
    cached.config = config
    expected = _dataset(samples, raw, cached=False, model_config=config)
    _assert_equal(cached[8], expected[8])
    assert cached.cache_stats.reset_reason == "configuration_changed"
    assert cached.cache_stats.last_call_misses == 30


@pytest.mark.parametrize("scope", ["source", "rig"])
def test_historical_only_provenance_change_resets_cache(jpeg_stream, scope):
    samples, raw = jpeg_stream(9)
    cached = _dataset(samples, raw)
    cached[8]
    if scope == "source":
        token = samples[0].token
        cached.raw_metadata[token] = replace(cached.raw_metadata[token], sample_jsonl_sha256="d" * 64)
    else:
        cached.examples = (replace(cached.examples[0], rig_id="rig-two"), *cached.examples[1:])
    cached[8]
    assert cached.cache_stats.last_call_misses == 30
    assert cached.cache_stats.last_call_hits == 0
    assert cached.cache_stats.reset_count == 1 and cached.cache_stats.reset_reason == "provenance_changed"


def test_historical_provenance_swap_preserving_scope_set_still_resets(jpeg_stream):
    samples, raw = jpeg_stream(9)
    raw[samples[2].token] = replace(raw[samples[2].token], sample_jsonl_sha256="d" * 64)
    cached = _dataset(samples, raw)
    cached[8]
    cached.raw_metadata[samples[0].token] = replace(cached.raw_metadata[samples[0].token], sample_jsonl_sha256="d" * 64)
    cached.raw_metadata[samples[2].token] = replace(cached.raw_metadata[samples[2].token], sample_jsonl_sha256="b" * 64)
    cached[8]
    assert cached.cache_stats.last_call_hits == 0 and cached.cache_stats.last_call_misses == 30
    assert cached.cache_stats.reset_reason == "provenance_changed"


def test_explicit_reset_clears_tensors_and_records_next_cold_call_reason(jpeg_stream):
    samples, raw = jpeg_stream(1)
    cached = _dataset(samples, raw)
    cached[0]
    cached.reset_cache()
    assert cached.cache_stats.retained_images == 0 and cached.cache_stats.retained_bytes == 0
    cached[0]
    assert cached.cache_stats.last_call_misses == 6 and cached.cache_stats.last_call_hits == 0
    assert cached.cache_stats.reset_count == 1 and cached.cache_stats.reset_reason == "explicit"


def test_missing_history_stays_zero_and_small_lru_bound_is_enforced(jpeg_stream):
    samples, raw = jpeg_stream(3)
    cached, original = _dataset(samples, raw, max_cached_images=6), _dataset(samples, raw, cached=False)
    for index in range(3):
        _assert_equal(cached[index], original[index])
        assert cached.cache_stats.retained_images <= 6
        assert cached.cache_stats.peak_retained_images <= 6
    row = cached[0]
    assert row["time_valid"].tolist() == [False, False, False, False, True]
    assert not row["images"][:-1].any()
    assert cached.cache_stats.retained_bytes == 6 * 3 * 32 * 32 * 4


def test_frozen_sources_and_separate_cache_source_pin(jpeg_stream):
    samples, raw = jpeg_stream(1)
    cached = _dataset(samples, raw)
    root = Path(__file__).resolve().parents[1] / "portable_e2e"
    pins = {"temporal_dataset_v2.py": FROZEN_TEMPORAL_DATASET_SOURCE_SHA256,
            "torch_dataset.py": FROZEN_TORCH_DATASET_SOURCE_SHA256,
            "contract.py": FROZEN_CONTRACT_SOURCE_SHA256}
    assert cached.cache_source_pin["frozen_source_sha256"] == pins
    for filename, expected in pins.items():
        assert hashlib.sha256((root / filename).read_bytes()).hexdigest() == expected
    assert cached.cache_source_sha256 == hashlib.sha256((root / "temporal_decoded_cache_v2.py").read_bytes()).hexdigest()
    assert MAX_CACHED_IMAGES == 54
