"""HH_261001 - Guard real temporal provenance and observed spatial prefixes."""

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.dataset import TrainingExample
from portable_e2e.model import ModelConfig
from portable_e2e.temporal_dataset_v2 import (
    TemporalCommon10TorchDataset,
    TemporalSampleMetadata,
    load_common10_temporal_metadata,
    spatial_expert_labels,
)


def example(index=0, *, episode="episode-a", timestamp_ns=None, speed=1.0):
    token = f"{episode}:{index:06d}"
    return TrainingExample(
        token=token,
        episode_id=episode,
        features=(1.0, speed, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        targets_xy=tuple(((i + 1) * 0.1, 0.0) for i in range(64)),
        target_yaw_rad=(0.0,) * 64,
        target_speed_mps=(speed,) * 64,
        sequence_index=index,
        anchor_timestamp_ns=1_000_000_000 + index * 100_000_000 if timestamp_ns is None else timestamp_ns,
        camera_paths=tuple(Path(f"/unused/{episode}/{index}/{camera}.jpg") for camera in range(6)),
        camera_sha256=("a" * 64,) * 6,
        camera_calibration=((0.0,) * 16,) * 6,
        route_points_base_xy_m=((0.0, 0.0), (100.0, 0.0)),
        rig_id="rig-one",
    )


def metadata(sample, *, yaw=0.0, position=None, split="train"):
    return TemporalSampleMetadata(
        sample_id=sample.token,
        episode_id=sample.episode_id,
        split=split,
        sequence_index=sample.sequence_index,
        anchor_timestamp_ns=sample.anchor_timestamp_ns,
        ego_timestamp_ns=sample.anchor_timestamp_ns,
        position_map_m=(float(sample.sequence_index), 0.0, 0.0) if position is None else position,
        orientation_xyzw=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        camera_sha256=sample.camera_sha256,
        sample_jsonl_sha256="b" * 64,
    )


def dataset(samples, raw=None, *, split="train", **kwargs):
    raw = raw or {sample.token: metadata(sample, split=split) for sample in samples}
    cfg = ModelConfig(image_height=32, image_width=32, image_grid_height=2, image_grid_width=2)
    value = TemporalCommon10TorchDataset(samples, cfg, raw_metadata=raw, split=split, **kwargs)
    # HH_261001 - Decode identity is explicit in the fixture; production decoder still checks JPEG hashes.
    value._decode_image = lambda path, sha: torch.full((3, 32, 32), float(path.parent.name))
    return value


def test_true_five_camera_histories_and_original_targets():
    samples = [example(i) for i in range(11)]
    row = dataset(samples)[10]
    assert row["images"].shape == (5, 6, 3, 32, 32)
    assert row["images"][:, 0, 0, 0, 0].tolist() == [2.0, 4.0, 6.0, 8.0, 10.0]
    assert row["time_valid"].tolist() == [True] * 5
    assert row["ego_history_mask"].tolist() == [True] * 10
    assert row["target_xy"].shape == (64, 2)
    assert row["target_valid"].all()
    assert row["spatial_curvature"].shape == (33,)
    assert row["calibration"].shape == (6, 16)
    assert row["route_xy"].shape == (128, 2)
    assert row["relative_pose"][:, 0].tolist() == [-8.0, -6.0, -4.0, -2.0, 0.0]
    assert torch.allclose(row["time_offsets_s"], torch.tensor([-.8, -.6, -.4, -.2, 0.]))


def test_missing_past_is_zero_not_current_copy():
    row = dataset([example(0)])[0]
    assert row["time_valid"].tolist() == [False, False, False, False, True]
    assert not row["images"][:-1].any()
    assert not row["relative_pose"].any()
    assert row["ego_history_mask"].sum() == 1


def test_measured_pose_in_rotated_current_frame_and_wrapped_yaw():
    samples = [example(0), example(8)]
    raw = {
        samples[0].token: metadata(samples[0], position=(8.0, 18.0, 0.0), yaw=math.pi / 2.0 + .2),
        samples[1].token: metadata(samples[1], position=(10.0, 20.0, 0.0), yaw=math.pi / 2.0),
    }
    row = dataset(samples, raw)[1]
    assert torch.allclose(row["relative_pose"][0], torch.tensor([-2.0, 2.0, .2]))
    # HH_261001 - Vehicle velocities do not substitute for independently recorded poses.
    changed = [replace(sample, features=(1., 999., *sample.features[2:])) for sample in samples]
    assert torch.allclose(dataset(changed, raw)[1]["relative_pose"], row["relative_pose"])


def test_yaw_wrap_does_not_cross_two_pi():
    samples = [example(0), example(8)]
    raw = {samples[0].token: metadata(samples[0], yaw=math.pi - .01), samples[1].token: metadata(samples[1], yaw=-math.pi + .01)}
    assert dataset(samples, raw)[1]["relative_pose"][0, 2].item() == pytest.approx(-.02)


def test_gap_does_not_select_wrong_sequence_as_history():
    samples = [example(0), example(1), example(2, timestamp_ns=1_800_000_000)]
    row = dataset(samples)[2]
    assert row["images"][:, 0, 0, 0, 0].tolist() == [0.0, 0.0, 0.0, 0.0, 2.0]
    assert row["time_valid"].tolist() == [True, False, False, False, True]
    assert row["ego_history_mask"].tolist() == [False, True, True, False, False, False, False, False, False, True]


def test_other_episode_never_supplies_missing_past():
    samples = [example(0, episode="other"), example(8)]
    row = dataset(samples)[1]
    assert row["time_valid"].tolist() == [False, False, False, False, True]


def test_same_episode_rig_change_rejected():
    samples = [example(0), replace(example(8), rig_id="another-rig")]
    with pytest.raises(ContractError, match="rig change"):
        dataset(samples)


def test_future_source_never_selected_and_explicit_positive_offset_rejected():
    samples = [example(8), example(9)]
    value = dataset(samples)
    row = value[0]
    assert row["images"][-1, 0, 0, 0, 0].item() == 8.0
    assert row["time_valid"].tolist() == [False, False, False, False, True]
    with pytest.raises(ContractError, match="future temporal images"):
        value._past_index(samples[0], 1)


def test_true_timestamp_offset_retained_with_sub_ms_clock_noise():
    samples = [example(0, timestamp_ns=1_000_400_000), example(8)]
    row = dataset(samples)[1]
    assert row["time_valid"][0]
    assert row["time_offsets_s"][0].item() == pytest.approx(-.7996)


def test_timestamp_outside_tolerance_masked():
    samples = [example(0, timestamp_ns=1_002_000_000), example(8)]
    assert not dataset(samples)[1]["time_valid"][0]


@pytest.mark.parametrize("patch", [
    {"split": "val"},
    {"sample_id": "another"},
    {"episode_id": "other"},
    {"sequence_index": 7},
    {"anchor_timestamp_ns": 123},
    {"ego_timestamp_ns": 1_000_000_001},
    {"ego_timestamp_ns": 900_000_000},
    {"position_map_m": (float("nan"), 0., 0.)},
    {"position_map_m": ("1.0", 0., 0.)},
    {"position_map_m": (True, 0., 0.)},
    {"orientation_xyzw": (0., 0., 0., .2)},
    {"camera_sha256": ("c" * 64,) * 6},
    {"sample_jsonl_sha256": "wrong"},
])
def test_untrusted_metadata_rejected(patch):
    sample = example(0)
    with pytest.raises(ContractError):
        dataset([sample], {sample.token: replace(metadata(sample), **patch)})


def test_duplicate_timestamp_rejected():
    with pytest.raises(ContractError, match="duplicate temporal episode timestamp"):
        dataset([example(0), example(1, timestamp_ns=1_000_000_000)])


def test_ambiguous_lookup_rejected():
    samples = [example(0), example(1, timestamp_ns=1_000_500_000), example(8)]
    with pytest.raises(ContractError, match="ambiguous"):
        dataset(samples)[2]


@pytest.mark.parametrize("tolerance", [-1, True, 10_000_001, 1.2])
def test_excessive_or_invalid_tolerance_rejected(tolerance):
    with pytest.raises(ContractError, match="tolerance"):
        dataset([example()], timestamp_tolerance_ns=tolerance)


def test_missing_metadata_token_rejected():
    sample = example()
    with pytest.raises(ContractError, match="token sets"):
        TemporalCommon10TorchDataset([sample], raw_metadata={})


def test_metadata_pose_and_policy_bound_into_fingerprint():
    samples = [example()]
    first = dataset(samples)
    changed_pose = {samples[0].token: metadata(samples[0], position=(1., 0., 0.))}
    assert dataset(samples, changed_pose).fingerprint_sha256 != first.fingerprint_sha256
    assert dataset(samples, timestamp_tolerance_ns=0).fingerprint_sha256 != first.fingerprint_sha256


def test_spatial_prefix_never_extrapolates_to_64_m():
    labels = spatial_expert_labels(example())
    assert labels["spatial_valid"].sum() == 4
    assert labels["spatial_valid"].tolist() == [True] * 4 + [False] * 29
    assert not labels["spatial_preferred_speed"][4:].any()
    assert labels["observed_spatial_distance_m"].item() == pytest.approx(6.4)
    assert labels["spatial_curvature_valid"][:4].all()
    assert not labels["spatial_curvature"].any()


def test_stationary_future_has_no_curvature_supervision_or_invented_stop_reason():
    sample = replace(example(speed=0.), targets_xy=((0., 0.),) * 64)
    row = dataset([sample])[0]
    assert row["spatial_valid"].sum() == 1
    assert not row["spatial_curvature_valid"].any()
    assert not row["spatial_preferred_speed"].any()
    assert not row["intent_valid"]
    assert not row["stop_distance_valid"]


def test_label_prefix_mask_limits_observed_distance():
    sample = example()
    sample = replace(sample, targets_xy=sample.targets_xy[:20] + (None,) * 44,
                     target_yaw_rad=sample.target_yaw_rad[:20] + (None,) * 44,
                     target_speed_mps=sample.target_speed_mps[:20] + (None,) * 44)
    labels = spatial_expert_labels(sample)
    assert labels["observed_spatial_distance_m"].item() == pytest.approx(2.)
    assert labels["spatial_valid"].sum() == 2


def test_curved_expert_labels_preserve_unwrapped_geometry():
    kappa = .05
    distances = [(i + 1) * .5 for i in range(64)]
    sample = replace(example(speed=5.),
                     targets_xy=tuple((math.sin(s * kappa) / kappa, (1. - math.cos(s * kappa)) / kappa) for s in distances),
                     target_yaw_rad=tuple(s * kappa for s in distances), target_speed_mps=(5.,) * 64)
    labels = spatial_expert_labels(sample)
    values = labels["spatial_curvature"][labels["spatial_curvature_valid"]]
    assert torch.allclose(values, torch.full_like(values, kappa), rtol=1e-3, atol=1e-5)


def test_dataset_permutation_does_not_change_temporal_images():
    samples = [example(i) for i in range(9)]
    ordered = dataset(samples)[8]
    permuted = dataset(samples[::-1])[0]
    assert torch.allclose(ordered["images"], permuted["images"])
    assert torch.allclose(ordered["relative_pose"], permuted["relative_pose"])


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metadata_root(tmp_path):
    sample = example()
    raw = {"sample_id": sample.token, "episode_id": sample.episode_id, "sequence_index": 0,
           "anchor_timestamp_ns": sample.anchor_timestamp_ns,
           "ego": {"timestamp_ns": sample.anchor_timestamp_ns, "position_m": [0., 0., 0.], "orientation_xyzw": [0., 0., 0., 1.]},
           "camera_bundle": [{"model_index": i, "sha256": sha} for i, sha in enumerate(sample.camera_sha256)]}
    samples_path = tmp_path / "episodes" / "a" / "samples.jsonl"
    samples_hash = write_json(samples_path, raw)
    episode_path = samples_path.parent / "episode.json"
    episode_hash = write_json(episode_path, {"episode_id": sample.episode_id, "split": "train",
                                          "sample_jsonl": "samples.jsonl", "sample_jsonl_sha256": samples_hash})
    write_json(tmp_path / "dataset.json", {"episodes": [{"manifest": "episodes/a/episode.json", "sha256": episode_hash}]})
    return sample, samples_path, episode_path


def test_metadata_loader_reads_original_pose_and_hash(tmp_path):
    sample, samples_path, _ = metadata_root(tmp_path)
    raw = load_common10_temporal_metadata(tmp_path, split="train", expected_tokens=[sample.token])
    assert raw[sample.token].position_map_m == (0., 0., 0.)
    assert raw[sample.token].camera_sha256 == sample.camera_sha256
    assert raw[sample.token].sample_jsonl_sha256 == hashlib.sha256(samples_path.read_bytes()).hexdigest()


def test_metadata_loader_detects_changed_samples(tmp_path):
    _, samples_path, _ = metadata_root(tmp_path)
    samples_path.write_text(samples_path.read_text() + "\n", encoding="utf-8")
    with pytest.raises(ContractError):
        load_common10_temporal_metadata(tmp_path, split="train")


def test_metadata_loader_detects_changed_episode(tmp_path):
    _, _, episode_path = metadata_root(tmp_path)
    episode_path.write_text(episode_path.read_text() + "\n", encoding="utf-8")
    with pytest.raises(ContractError, match="manifest changed"):
        load_common10_temporal_metadata(tmp_path, split="train")


def test_metadata_loader_rejects_token_subset_and_missing_split(tmp_path):
    metadata_root(tmp_path)
    with pytest.raises(ContractError, match="validated examples"):
        load_common10_temporal_metadata(tmp_path, split="train", expected_tokens=["wrong"])
    with pytest.raises(ContractError, match="no samples"):
        load_common10_temporal_metadata(tmp_path, split="val")
