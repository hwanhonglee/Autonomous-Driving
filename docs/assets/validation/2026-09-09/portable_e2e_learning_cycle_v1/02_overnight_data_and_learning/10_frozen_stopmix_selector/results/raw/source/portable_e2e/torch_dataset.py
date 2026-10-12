"""PyTorch image decoding and tensorization for validated common10 examples."""

from __future__ import annotations

from io import BytesIO
import hashlib
import math
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image
import torch
from torch import Tensor
from torch.utils.data import Dataset

from .contract import ContractError
from .contract import MAX_JPEG_FILE_BYTES
from .contract import _read_regular_file_bounded
from .dataset import (
    CALIBRATION_FEATURE_NAMES,
    FEATURE_NAMES,
    TrainingExample,
    fingerprint_examples,
)
from .model import ModelConfig


RGB_MEAN = (0.485, 0.456, 0.406)
RGB_STD = (0.229, 0.224, 0.225)


def _linear_route_samples(
    points: Sequence[tuple[float, float]], *, limit: int, spacing_m: float = 1.0
) -> tuple[tuple[float, float], ...]:
    cleaned: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        if len(point) != 2:
            raise ContractError(f"training route point {index} must contain x and y")
        parsed = (float(point[0]), float(point[1]))
        if not all(math.isfinite(value) for value in parsed):
            raise ContractError("training route contains a non-finite point")
        if not cleaned or math.dist(cleaned[-1], parsed) > 1.0e-9:
            cleaned.append(parsed)
    points = cleaned
    if len(points) < 2:
        raise ContractError("a training route needs at least two points")
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        segment = math.dist(first, second)
        if not math.isfinite(segment):
            raise ContractError("training route contains a non-finite segment")
        cumulative.append(cumulative[-1] + segment)
    total = cumulative[-1]
    if total <= 0.0:
        raise ContractError("a training route must span a nonzero distance")
    requested = [min(float(index) * spacing_m, total) for index in range(limit)]
    if requested[-1] >= total:
        requested = requested[: int(math.floor(total / spacing_m)) + 1]
        if not requested or total - requested[-1] > 1.0e-6:
            requested.append(total)
    output: list[tuple[float, float]] = []
    segment_index = 0
    for distance in requested[:limit]:
        while (
            segment_index + 1 < len(cumulative) - 1
            and cumulative[segment_index + 1] < distance
        ):
            segment_index += 1
        start_distance = cumulative[segment_index]
        end_distance = cumulative[segment_index + 1]
        ratio = (distance - start_distance) / (end_distance - start_distance)
        first = points[segment_index]
        second = points[segment_index + 1]
        output.append(
            (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        )
    return tuple(output)


class Common10TorchDataset(Dataset):
    """Decode validated samples lazily while preserving causal ego history."""

    def __init__(
        self,
        examples: Sequence[TrainingExample],
        model_config: ModelConfig,
        *,
        verify_image_sha256: bool = True,
        split: str = "train",
    ) -> None:
        model_config.validate()
        if not examples:
            raise ContractError("PyTorch dataset needs at least one example")
        if not isinstance(verify_image_sha256, bool):
            raise ContractError("verify_image_sha256 must be boolean")
        if split not in ("train", "val", "test"):
            raise ContractError("PyTorch dataset split must be train, val, or test")
        self.examples = tuple(examples)
        self.config = model_config
        self.verify_image_sha256 = verify_image_sha256
        self.split = split
        # Bind the tensor dataset identity to the examples actually held by this
        # object.  Accepting a caller-provided digest would let an accidental or
        # hostile library caller attach false provenance to a checkpoint.
        digest = hashlib.sha256()
        digest.update(fingerprint_examples(self.examples).encode("ascii"))
        digest.update(b"\0")
        digest.update(split.encode("ascii"))
        self.fingerprint_sha256 = digest.hexdigest()
        self._history_lookup: dict[tuple[str, int], int] = {}
        sample_tokens: set[str] = set()
        for index, example in enumerate(self.examples):
            self._validate_example(example, index)
            if example.token in sample_tokens:
                raise ContractError(f"duplicate sample token: {example.token}")
            sample_tokens.add(example.token)
            key = (example.episode_id, example.sequence_index)
            if key in self._history_lookup:
                raise ContractError(f"duplicate episode/sequence pair: {key}")
            self._history_lookup[key] = index

    def _validate_example(self, example: TrainingExample, index: int) -> None:
        context = f"examples[{index}]"
        cfg = self.config
        if not isinstance(example.token, str) or not example.token:
            raise ContractError(f"{context} has an invalid sample token")
        if not isinstance(example.episode_id, str) or not example.episode_id:
            raise ContractError(f"{context} has an invalid episode ID")
        if not isinstance(example.rig_id, str) or not example.rig_id:
            raise ContractError(f"{context} has an invalid rig ID")
        if len(example.features) != cfg.ego_features:
            raise ContractError(f"{context} has the wrong ego feature count")
        if not all(math.isfinite(float(value)) for value in example.features):
            raise ContractError(f"{context} has a non-finite ego feature")
        if (
            isinstance(example.sequence_index, bool)
            or not isinstance(example.sequence_index, int)
            or example.sequence_index < 0
        ):
            raise ContractError(f"{context} has an invalid sequence index")
        if (
            isinstance(example.anchor_timestamp_ns, bool)
            or not isinstance(example.anchor_timestamp_ns, int)
            or example.anchor_timestamp_ns < 0
        ):
            raise ContractError(f"{context} has an invalid anchor timestamp")
        if len(example.camera_paths) != cfg.camera_count:
            raise ContractError(f"{context} does not contain all camera paths")
        if len(example.camera_sha256) != cfg.camera_count:
            raise ContractError(f"{context} does not contain all camera hashes")
        if any(
            len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
            for value in example.camera_sha256
        ):
            raise ContractError(f"{context} contains an invalid camera hash")
        if len(example.camera_calibration) != cfg.camera_count:
            raise ContractError(f"{context} does not contain all camera calibration")
        if any(
            len(values) != len(CALIBRATION_FEATURE_NAMES)
            for values in example.camera_calibration
        ):
            raise ContractError(f"{context} camera calibration has the wrong width")
        if any(
            not math.isfinite(float(value))
            for values in example.camera_calibration
            for value in values
        ):
            raise ContractError(f"{context} camera calibration contains NaN or Inf")
        target_lengths = (
            len(example.targets_xy),
            len(example.target_yaw_rad),
            len(example.target_speed_mps),
        )
        if target_lengths != (cfg.future_points,) * 3:
            raise ContractError(f"{context} does not contain the complete future-label ABI")
        invalid_seen = False
        for target_index, (point, yaw, speed) in enumerate(
            zip(example.targets_xy, example.target_yaw_rad, example.target_speed_mps)
        ):
            if point is None:
                invalid_seen = True
                if yaw is not None or speed is not None:
                    raise ContractError(
                        f"{context} target {target_index} has inconsistent null fields"
                    )
                continue
            if invalid_seen:
                raise ContractError(f"{context} target mask is not a valid prefix")
            if len(point) != 2:
                raise ContractError(
                    f"{context} target {target_index} must contain x and y"
                )
            values = (*point, yaw, speed)
            if yaw is None or speed is None or not all(
                math.isfinite(float(value)) for value in values
            ):
                raise ContractError(f"{context} target {target_index} contains NaN or Inf")
            if float(speed) < 0.0:
                raise ContractError(f"{context} target {target_index} has negative speed")
        if len(example.route_points_base_xy_m) < 2:
            raise ContractError(f"{context} route has fewer than two points")
        _linear_route_samples(example.route_points_base_xy_m, limit=cfg.route_points)

    @staticmethod
    def _read_regular_file(path: Path) -> bytearray:
        payload = _read_regular_file_bounded(
            path,
            MAX_JPEG_FILE_BYTES,
            f"training image {path}",
        )
        if not payload:
            raise ContractError(f"training image is empty: {path}")
        return payload

    def __len__(self) -> int:
        return len(self.examples)

    def _decode_image(self, path: Path, expected_sha256: str) -> Tensor:
        payload = self._read_regular_file(path)
        if self.verify_image_sha256:
            actual_sha256 = hashlib.sha256(payload).hexdigest()
            if actual_sha256 != expected_sha256:
                raise ContractError(f"training image changed after validation: {path}")
        try:
            with Image.open(BytesIO(payload)) as image:
                image.load()
                rgb = image.convert("RGB")
                if rgb.size != (640, 360):
                    raise ContractError(f"training image has unexpected decoded size: {path}")
                resampling = getattr(Image, "Resampling", Image).BILINEAR
                resized = rgb.resize(
                    (self.config.image_width, self.config.image_height), resample=resampling
                )
                array = np.asarray(resized, dtype=np.float32).copy()
        except MemoryError as error:
            raise ContractError(
                f"training image decode exceeded the bounded memory budget: {path}"
            ) from error
        except (OSError, ValueError) as error:
            raise ContractError(f"cannot decode training image {path}: {error}") from error
        tensor = torch.from_numpy(array).permute(2, 0, 1).contiguous().div_(255.0)
        mean = tensor.new_tensor(RGB_MEAN).view(3, 1, 1)
        std = tensor.new_tensor(RGB_STD).view(3, 1, 1)
        return (tensor - mean) / std

    def _ego_history(self, example: TrainingExample) -> tuple[Tensor, Tensor]:
        cfg = self.config
        history = torch.zeros(cfg.ego_history_frames, len(FEATURE_NAMES), dtype=torch.float32)
        mask = torch.zeros(cfg.ego_history_frames, dtype=torch.bool)
        for history_index in range(cfg.ego_history_frames):
            offset = cfg.ego_history_frames - 1 - history_index
            lookup = (example.episode_id, example.sequence_index - offset)
            source_index = self._history_lookup.get(lookup)
            if source_index is None:
                continue
            source = self.examples[source_index]
            if source.anchor_timestamp_ns > example.anchor_timestamp_ns:
                raise ContractError("ego history lookup selected a future sample")
            history[history_index] = torch.tensor(source.features, dtype=torch.float32)
            mask[history_index] = True
        if not bool(mask[-1].item()):
            raise ContractError("current ego state is missing from its own history")
        return history, mask

    def _route(self, example: TrainingExample) -> tuple[Tensor, Tensor]:
        sampled = _linear_route_samples(
            example.route_points_base_xy_m, limit=self.config.route_points
        )
        route = torch.zeros(self.config.route_points, 2, dtype=torch.float32)
        mask = torch.zeros(self.config.route_points, dtype=torch.bool)
        route[: len(sampled)] = torch.tensor(sampled, dtype=torch.float32)
        mask[: len(sampled)] = True
        return route, mask

    def _targets(self, example: TrainingExample) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        cfg = self.config
        xy = torch.zeros(cfg.future_points, 2, dtype=torch.float32)
        yaw = torch.zeros(cfg.future_points, dtype=torch.float32)
        speed = torch.zeros(cfg.future_points, dtype=torch.float32)
        valid = torch.zeros(cfg.future_points, dtype=torch.bool)
        for index, (point, point_yaw, point_speed) in enumerate(
            zip(example.targets_xy, example.target_yaw_rad, example.target_speed_mps)
        ):
            if point is None:
                if point_yaw is not None or point_speed is not None:
                    raise ContractError("invalid target point has non-null yaw or speed")
                continue
            if point_yaw is None or point_speed is None:
                raise ContractError("valid target point is missing yaw or speed")
            xy[index] = torch.tensor(point, dtype=torch.float32)
            yaw[index] = float(point_yaw)
            speed[index] = float(point_speed)
            valid[index] = True
        if not bool(valid.any().item()):
            raise ContractError("training sample has no valid future target")
        return xy, yaw, speed, valid

    def __getitem__(self, index: int) -> dict[str, Tensor | str]:
        example = self.examples[index]
        images = torch.stack(
            [
                self._decode_image(path, expected_sha256)
                for path, expected_sha256 in zip(
                    example.camera_paths, example.camera_sha256
                )
            ]
        )
        ego_history, ego_history_mask = self._ego_history(example)
        route_xy, route_mask = self._route(example)
        target_xy, target_yaw, target_speed, target_valid = self._targets(example)
        return {
            "sample_id": example.token,
            "episode_id": example.episode_id,
            "images": images,
            "calibration": torch.tensor(example.camera_calibration, dtype=torch.float32),
            "ego_history": ego_history,
            "ego_history_mask": ego_history_mask,
            "route_xy": route_xy,
            "route_mask": route_mask,
            "target_xy": target_xy,
            "target_yaw_rad": target_yaw,
            "target_speed_mps": target_speed,
            "target_valid": target_valid,
        }
