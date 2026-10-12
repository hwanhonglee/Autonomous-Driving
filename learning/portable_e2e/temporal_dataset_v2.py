"""HH_261001 - Build causal real camera history and masked spatial expert labels.

This research adapter does not promote the legacy corpus or infer a stop reason.
Only recorded future distances are supervised; no 64 m labels are extrapolated.
"""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import torch
from torch import Tensor

from .contract import (
    ContractError,
    _iter_jsonl,
    _mapping,
    _read_json_and_sha256,
    _require_admission_not_denied,
    _safe_file,
    _sha256,
)
from .dataset import TrainingExample
from .model import ModelConfig
from .torch_dataset import Common10TorchDataset


TIME_OFFSETS_NS = (-800_000_000, -600_000_000, -400_000_000, -200_000_000, 0)
SPATIAL_KNOT_COUNT = 33
SPATIAL_SPACING_M = 2.0


@dataclass(frozen=True)
class TemporalSampleMetadata:
    """HH_261001 - Preserve measured map pose omitted by the legacy example ABI."""

    sample_id: str
    episode_id: str
    split: str
    sequence_index: int
    anchor_timestamp_ns: int
    ego_timestamp_ns: int
    position_map_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    camera_sha256: tuple[str, ...]
    sample_jsonl_sha256: str


def load_common10_temporal_metadata(
    root: Path | str,
    *,
    split: str,
    expected_tokens: Sequence[str] | None = None,
) -> dict[str, TemporalSampleMetadata]:
    """HH_261001 - Recheck manifest/sample bytes; pair with the validated loader.

    Call ``load_training_examples`` first for full camera, rig, provenance and
    physical-contract validation. This function does not replace that gate.
    """
    if split not in ("train", "val", "test"):
        raise ContractError("temporal metadata split must be train, val, or test")
    root_path = Path(root).expanduser().resolve(strict=True)
    manifest, _ = _read_json_and_sha256(root_path / "dataset.json")
    _require_admission_not_denied(manifest, "temporal dataset")
    result: dict[str, TemporalSampleMetadata] = {}
    for index, raw_reference in enumerate(manifest["episodes"]):
        reference = _mapping(raw_reference, f"dataset.episodes[{index}]")
        episode_path = _safe_file(root_path, reference["manifest"], "episode manifest")
        episode, episode_hash = _read_json_and_sha256(episode_path)
        if episode_hash != _sha256(reference["sha256"], "episode hash"):
            raise ContractError("temporal episode manifest changed")
        if episode["split"] != split:
            continue
        _require_admission_not_denied(episode, str(episode_path))
        sample_path = _safe_file(episode_path.parent, episode["sample_jsonl"], "samples")
        sample_hash = _sha256(episode["sample_jsonl_sha256"], "sample hash")
        for line_number, sample in _iter_jsonl(sample_path, expected_sha256=sample_hash):
            ego = _mapping(sample["ego"], f"{sample_path}:{line_number}.ego")
            token = sample["sample_id"]
            if token in result:
                raise ContractError("duplicate temporal metadata sample ID")
            if sample["episode_id"] != episode["episode_id"]:
                raise ContractError("temporal sample and episode IDs differ")
            cameras = sample["camera_bundle"]
            if len(cameras) != 6 or tuple(camera["model_index"] for camera in cameras) != tuple(range(6)):
                raise ContractError("temporal metadata camera order differs")
            result[token] = TemporalSampleMetadata(
                sample_id=token,
                episode_id=sample["episode_id"],
                split=split,
                sequence_index=sample["sequence_index"],
                anchor_timestamp_ns=sample["anchor_timestamp_ns"],
                ego_timestamp_ns=ego["timestamp_ns"],
                position_map_m=tuple(float(value) for value in ego["position_m"]),
                orientation_xyzw=tuple(float(value) for value in ego["orientation_xyzw"]),
                camera_sha256=tuple(camera["sha256"] for camera in cameras),
                sample_jsonl_sha256=sample_hash,
            )
    if not result:
        raise ContractError("temporal metadata split contains no samples")
    if expected_tokens is not None:
        expected = tuple(expected_tokens)
        if len(expected) != len(set(expected)) or set(result) != set(expected):
            raise ContractError("temporal metadata does not match the validated examples")
    return result


def _yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _relative_pose(current: TemporalSampleMetadata, past: TemporalSampleMetadata) -> tuple[float, float, float]:
    """HH_261001 - Express the measured historical pose in current base XY."""
    yaw = _yaw_from_quaternion(current.orientation_xyzw)
    dx = past.position_map_m[0] - current.position_map_m[0]
    dy = past.position_map_m[1] - current.position_map_m[1]
    delta = _yaw_from_quaternion(past.orientation_xyzw) - yaw
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
        math.atan2(math.sin(delta), math.cos(delta)),
    )


def spatial_expert_labels(example: TrainingExample) -> dict[str, Tensor]:
    """HH_261001 - Resample only the observed temporal future onto distance knots.

    Curvature uses a finite observed distance stencil and is not supervised at
    near-standstill. Speed and curvature validity are deliberately separate.
    The returned masks are loss labels, never model-input preview confidence.
    """
    distances = [0.0]
    yaws = [0.0]
    speeds = [math.hypot(float(example.features[1]), float(example.features[2]))]
    previous_xy = (0.0, 0.0)
    observed_arc = 0.0
    for xy, yaw, speed in zip(example.targets_xy, example.target_yaw_rad, example.target_speed_mps):
        if xy is None:
            break
        if yaw is None or speed is None:
            raise ContractError("spatial label has inconsistent temporal fields")
        observed_arc += math.dist(previous_xy, xy)
        previous_xy = xy
        # HH_261001 - Duplicate stationary positions do not create an interpolation interval.
        if observed_arc - distances[-1] <= 1.0e-6:
            continue
        distances.append(observed_arc)
        yaws.append(float(yaw))
        speeds.append(float(speed))
    knots = np.arange(SPATIAL_KNOT_COUNT, dtype=np.float64) * SPATIAL_SPACING_M
    valid = knots <= observed_arc + 1.0e-8
    curvature = np.zeros(SPATIAL_KNOT_COUNT, dtype=np.float32)
    preferred_speed = np.zeros(SPATIAL_KNOT_COUNT, dtype=np.float32)
    curvature_valid = np.zeros(SPATIAL_KNOT_COUNT, dtype=np.bool_)
    preferred_speed[valid] = np.interp(knots[valid], distances, speeds).astype(np.float32)
    if len(distances) > 1:
        unwrapped_yaw = np.unwrap(np.asarray(yaws, dtype=np.float64))
        for index, distance in enumerate(knots):
            if not valid[index]:
                continue
            lower = max(0.0, float(distance) - SPATIAL_SPACING_M)
            upper = min(observed_arc, float(distance) + SPATIAL_SPACING_M)
            if upper - lower < SPATIAL_SPACING_M - 1.0e-8:
                continue
            stencil_speeds = np.interp([lower, distance, upper], distances, speeds)
            if float(stencil_speeds.min()) <= 0.05:
                continue
            heading = np.interp([lower, upper], distances, unwrapped_yaw)
            curvature[index] = float((heading[1] - heading[0]) / (upper - lower))
            curvature_valid[index] = True
    return {
        "spatial_curvature": torch.from_numpy(curvature),
        "spatial_preferred_speed": torch.from_numpy(preferred_speed),
        "spatial_valid": torch.from_numpy(valid),
        "spatial_speed_valid": torch.from_numpy(valid.copy()),
        "spatial_curvature_valid": torch.from_numpy(curvature_valid),
        "observed_spatial_distance_m": torch.tensor(observed_arc, dtype=torch.float32),
    }


class TemporalCommon10TorchDataset(Common10TorchDataset):
    """HH_261001 - Preserve causal same-split camera and ego histories without copying images."""

    def __init__(
        self,
        examples: Sequence[TrainingExample],
        model_config: ModelConfig | None = None,
        *,
        raw_metadata: Mapping[str, TemporalSampleMetadata],
        verify_image_sha256: bool = True,
        split: str = "train",
        timestamp_tolerance_ns: int = 1_000_000,
    ) -> None:
        super().__init__(examples, model_config or ModelConfig(),
                         verify_image_sha256=verify_image_sha256, split=split)
        if isinstance(timestamp_tolerance_ns, bool) or not isinstance(timestamp_tolerance_ns, int) or not 0 <= timestamp_tolerance_ns <= 10_000_000:
            raise ContractError("temporal timestamp tolerance must be at most 10 ms")
        self.timestamp_tolerance_ns = timestamp_tolerance_ns
        if set(raw_metadata) != {example.token for example in self.examples}:
            raise ContractError("temporal metadata and example token sets differ")
        self.raw_metadata = dict(raw_metadata)
        episode_rows: dict[str, list[tuple[int, int]]] = {}
        episode_rigs: dict[str, str] = {}
        for index, example in enumerate(self.examples):
            metadata = self.raw_metadata[example.token]
            self._validate_metadata(example, metadata)
            previous_rig = episode_rigs.setdefault(example.episode_id, example.rig_id)
            if previous_rig != example.rig_id:
                raise ContractError("temporal history must not cross a rig change")
            episode_rows.setdefault(example.episode_id, []).append((metadata.anchor_timestamp_ns, index))
        self._time_lookup: dict[str, tuple[tuple[int, ...], tuple[int, ...]]] = {}
        for episode_id, rows in episode_rows.items():
            rows.sort()
            timestamps = tuple(timestamp for timestamp, _ in rows)
            if len(timestamps) != len(set(timestamps)):
                raise ContractError("duplicate temporal episode timestamp")
            self._time_lookup[episode_id] = (timestamps, tuple(index for _, index in rows))
        # HH_261001 - Bind measured pose, timing, split and spatial-label policy into the new identity.
        digest = hashlib.sha256()
        digest.update(self.fingerprint_sha256.encode("ascii"))
        provenance = {
            "schema": "portable_e2e.temporal_dataset.v2",
            "time_offsets_ns": TIME_OFFSETS_NS,
            "timestamp_tolerance_ns": timestamp_tolerance_ns,
            "spatial_label_policy": "observed_6p4s_arc_prefix_curvature_2m_stencil_no_stop_intent_v1",
            "metadata": [asdict(self.raw_metadata[example.token]) for example in self.examples],
        }
        digest.update(json.dumps(provenance, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8"))
        self.fingerprint_sha256 = digest.hexdigest()

    def _validate_metadata(self, example: TrainingExample, metadata: TemporalSampleMetadata) -> None:
        if not isinstance(metadata, TemporalSampleMetadata):
            raise ContractError("temporal metadata must be measured TemporalSampleMetadata")
        identities = (
            metadata.sample_id == example.token,
            metadata.episode_id == example.episode_id,
            metadata.split == self.split,
            metadata.sequence_index == example.sequence_index,
            metadata.anchor_timestamp_ns == example.anchor_timestamp_ns,
            metadata.camera_sha256 == example.camera_sha256,
        )
        if not all(identities):
            raise ContractError("temporal metadata identity, split, time or camera hashes differ")
        if isinstance(metadata.ego_timestamp_ns, bool) or not isinstance(metadata.ego_timestamp_ns, int) or not 0 <= metadata.ego_timestamp_ns <= metadata.anchor_timestamp_ns:
            raise ContractError("temporal ego timestamp is invalid or in the future")
        if metadata.anchor_timestamp_ns - metadata.ego_timestamp_ns > 30_000_000:
            raise ContractError("temporal ego state is more than 30 ms stale")
        if len(metadata.position_map_m) != 3 or len(metadata.orientation_xyzw) != 4:
            raise ContractError("temporal measured pose dimensions differ")
        pose_values = (*metadata.position_map_m, *metadata.orientation_xyzw)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in pose_values):
            raise ContractError("temporal measured pose must contain real numbers")
        if not all(math.isfinite(float(value)) for value in pose_values):
            raise ContractError("temporal measured pose contains NaN or Inf")
        if abs(sum(value * value for value in metadata.orientation_xyzw) - 1.0) > 2.0e-3:
            raise ContractError("temporal measured quaternion is not normalized")
        _sha256(metadata.sample_jsonl_sha256, "temporal sample JSONL SHA")

    def _past_index(self, example: TrainingExample, offset_ns: int) -> int | None:
        if offset_ns > 0:
            raise ContractError("future temporal images are forbidden")
        target = example.anchor_timestamp_ns + offset_ns
        timestamps, indices = self._time_lookup[example.episode_id]
        lower = bisect_left(timestamps, target - self.timestamp_tolerance_ns)
        upper = bisect_right(timestamps, min(target + self.timestamp_tolerance_ns, example.anchor_timestamp_ns))
        candidates = indices[lower:upper]
        if len(candidates) > 1:
            raise ContractError("temporal timestamp lookup is ambiguous")
        return candidates[0] if candidates else None

    def _ego_history(self, example: TrainingExample) -> tuple[Tensor, Tensor]:
        history = torch.zeros(self.config.ego_history_frames, self.config.ego_features, dtype=torch.float32)
        valid = torch.zeros(self.config.ego_history_frames, dtype=torch.bool)
        for index in range(self.config.ego_history_frames):
            offset = -(self.config.ego_history_frames - 1 - index) * 100_000_000
            source_index = self._past_index(example, offset)
            if source_index is not None:
                history[index] = torch.tensor(self.examples[source_index].features, dtype=torch.float32)
                valid[index] = True
        if not bool(valid[-1]):
            raise ContractError("temporal current ego history is missing")
        return history, valid

    def __getitem__(self, index: int) -> dict[str, Tensor | str]:
        example = self.examples[index]
        output = super().__getitem__(index)
        current_metadata = self.raw_metadata[example.token]
        images = torch.zeros((5, *output["images"].shape), dtype=output["images"].dtype)
        time_valid = torch.zeros(5, dtype=torch.bool)
        relative_pose = torch.zeros(5, 3, dtype=torch.float32)
        offsets = torch.tensor(TIME_OFFSETS_NS, dtype=torch.float32) * 1.0e-9
        for time_index, offset_ns in enumerate(TIME_OFFSETS_NS):
            source_index = self._past_index(example, offset_ns)
            if source_index is None:
                continue
            source = self.examples[source_index]
            metadata = self.raw_metadata[source.token]
            if offset_ns == 0:
                if source.token != example.token:
                    raise ContractError("temporal current image identity differs")
                images[time_index] = output["images"]
            else:
                images[time_index] = torch.stack([
                    self._decode_image(path, sha) for path, sha in zip(source.camera_paths, source.camera_sha256)
                ])
            time_valid[time_index] = True
            relative_pose[time_index] = torch.tensor(_relative_pose(current_metadata, metadata), dtype=torch.float32)
            offsets[time_index] = (metadata.anchor_timestamp_ns - current_metadata.anchor_timestamp_ns) * 1.0e-9
        if not bool(time_valid[-1]):
            raise ContractError("temporal current camera bundle is missing")
        output.update(images=images, time_valid=time_valid, relative_pose=relative_pose, time_offsets_s=offsets)
        output.update(spatial_expert_labels(example))
        # HH_261001 - This corpus has no verified semantic stop reason; zero is a masked placeholder, not GO truth.
        output.update(
            intent_target=torch.tensor(0, dtype=torch.int64),
            intent_valid=torch.tensor(False),
            stop_distance_target=torch.tensor(0.0, dtype=torch.float32),
            stop_distance_valid=torch.tensor(False),
        )
        return output
