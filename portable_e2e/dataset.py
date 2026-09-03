"""Portable dataset loading primitives that do not import ML or simulator stacks."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .contract import ContractError
from .contract import _iter_jsonl
from .contract import _mapping
from .contract import _read_json_and_sha256
from .contract import _safe_file
from .contract import _sequence
from .contract import _sha256
from .contract import validate_dataset


FEATURE_NAMES = (
    "bias",
    "velocity_x_mps",
    "velocity_y_mps",
    "acceleration_x_mps2",
    "acceleration_y_mps2",
    "yaw_rate_radps",
    "steering_tire_angle_rad",
    "command_left",
    "command_right",
    "command_straight",
    "command_lane_follow",
    "command_change_left",
    "command_change_right",
)
CALIBRATION_FEATURE_NAMES = (
    "fx_over_width",
    "fy_over_height",
    "cx_over_width",
    "cy_over_height",
    "rotation_00",
    "rotation_01",
    "rotation_02",
    "translation_x_m",
    "rotation_10",
    "rotation_11",
    "rotation_12",
    "translation_y_m",
    "rotation_20",
    "rotation_21",
    "rotation_22",
    "translation_z_m",
)


@dataclass(frozen=True)
class TrainingExample:
    """Small framework-neutral view of one planning-training sample."""

    token: str
    episode_id: str
    features: tuple[float, ...]
    targets_xy: tuple[tuple[float, float] | None, ...]
    target_yaw_rad: tuple[float | None, ...] = ()
    target_speed_mps: tuple[float | None, ...] = ()
    sequence_index: int = 0
    anchor_timestamp_ns: int = 0
    camera_paths: tuple[Path, ...] = ()
    camera_sha256: tuple[str, ...] = ()
    camera_calibration: tuple[tuple[float, ...], ...] = ()
    route_points_base_xy_m: tuple[tuple[float, float], ...] = ()
    rig_id: str = ""
    domain: str = ""
    source_dataset_id: str = ""
    source_dataset_version: str = ""
    license_id: str = ""
    scenario_tags: tuple[str, ...] = ()
    map_id: str = ""
    site_id: str = ""
    route_id: str = ""
    source_manifest_sha256: str = ""

    @property
    def valid_point_count(self) -> int:
        return sum(target is not None for target in self.targets_xy)


@dataclass(frozen=True)
class LoadedDataset:
    examples: tuple[TrainingExample, ...]
    fingerprint_sha256: str
    validation_report: Mapping[str, Any]
    split: str


@dataclass(frozen=True)
class _EpisodeMetadata:
    """Normalized immutable provenance copied onto every episode sample."""

    domain: str = ""
    source_dataset_id: str = ""
    source_dataset_version: str = ""
    license_id: str = ""
    scenario_tags: tuple[str, ...] = ()
    map_id: str = ""
    site_id: str = ""
    route_id: str = ""
    source_manifest_sha256: str = ""


def _metadata_text(document: Mapping[str, Any], key: str, context: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f"{context}.{key} must be a non-empty string")
    return value


def _episode_metadata(
    episode: Mapping[str, Any],
    source_manifest: Mapping[str, Any],
    source_manifest_sha256: str,
    context: str,
) -> _EpisodeMetadata:
    # The source-manifest v1 identity fields are deliberately the only part of
    # its adapter-specific document shape exposed here.  Scenario and location
    # metadata continue to come from the normalized episode contract.
    return _EpisodeMetadata(
        domain=_metadata_text(episode, "domain", context),
        source_dataset_id=_metadata_text(
            source_manifest, "source_dataset_id", f"{context}.source_manifest"
        ),
        source_dataset_version=_metadata_text(
            source_manifest, "source_dataset_version", f"{context}.source_manifest"
        ),
        license_id=_metadata_text(
            source_manifest, "license_id", f"{context}.source_manifest"
        ),
        scenario_tags=tuple(str(tag) for tag in episode["scenario_tags"]),
        map_id=_metadata_text(episode, "map_id", context),
        site_id=_metadata_text(episode, "site_id", context),
        route_id=_metadata_text(episode, "route_id", context),
        source_manifest_sha256=source_manifest_sha256,
    )


def _sample_to_example(
    sample: Mapping[str, Any],
    context: str,
    episode_root: Path | None = None,
    rig: Mapping[str, Any] | None = None,
    episode_metadata: _EpisodeMetadata | None = None,
) -> TrainingExample:
    token = sample.get("sample_id")
    episode_id = sample.get("episode_id")
    if not isinstance(token, str) or not token:
        raise ContractError(f"{context}.sample_id must be set")
    if not isinstance(episode_id, str) or not episode_id:
        raise ContractError(f"{context}.episode_id must be set")

    ego = _mapping(sample.get("ego"), f"{context}.ego")
    velocity = _sequence(
        ego.get("linear_velocity_base_mps"), f"{context}.ego.linear_velocity_base_mps"
    )
    acceleration = _sequence(
        ego.get("linear_acceleration_base_mps2"),
        f"{context}.ego.linear_acceleration_base_mps2",
    )
    angular_velocity = _sequence(
        ego.get("angular_velocity_base_radps"),
        f"{context}.ego.angular_velocity_base_radps",
    )
    navigation = _mapping(sample.get("navigation"), f"{context}.navigation")
    command = int(navigation["command"])
    one_hot = tuple(float(index == command) for index in range(6))
    features = (
        1.0,
        float(velocity[0]),
        float(velocity[1]),
        float(acceleration[0]),
        float(acceleration[1]),
        float(angular_velocity[2]),
        float(ego["steering_tire_angle_rad"]),
        *one_hot,
    )

    labels = _mapping(sample.get("labels"), f"{context}.labels")
    planning = _mapping(labels.get("planning"), f"{context}.labels.planning")
    positions = _sequence(
        planning.get("positions_base_xy_m"), f"{context}.labels.planning.positions_base_xy_m"
    )
    valid = _sequence(planning.get("valid"), f"{context}.labels.planning.valid")
    yaws = _sequence(planning.get("yaw_rad"), f"{context}.labels.planning.yaw_rad")
    speeds = _sequence(planning.get("speed_mps"), f"{context}.labels.planning.speed_mps")
    targets: list[tuple[float, float] | None] = []
    target_yaws: list[float | None] = []
    target_speeds: list[float | None] = []
    for index, flag in enumerate(valid):
        if flag:
            point = _sequence(positions[index], f"{context}.labels.planning.positions[{index}]")
            targets.append((float(point[0]), float(point[1])))
            target_yaws.append(float(yaws[index]))
            target_speeds.append(float(speeds[index]))
        else:
            targets.append(None)
            target_yaws.append(None)
            target_speeds.append(None)
    route_points = tuple(
        (float(point[0]), float(point[1]))
        for point in _sequence(
            navigation.get("route_polyline_base_m"),
            f"{context}.navigation.route_polyline_base_m",
        )
    )
    camera_paths: list[Path] = []
    camera_hashes: list[str] = []
    if episode_root is not None:
        for index, raw_camera in enumerate(
            _sequence(sample.get("camera_bundle"), f"{context}.camera_bundle")
        ):
            camera = _mapping(raw_camera, f"{context}.camera_bundle[{index}]")
            camera_paths.append(
                _safe_file(
                    episode_root,
                    camera.get("path"),
                    f"{context}.camera_bundle[{index}].path",
                )
            )
            camera_hashes.append(str(camera["sha256"]))
    camera_calibration: list[tuple[float, ...]] = []
    if rig is not None:
        rig_cameras = _sequence(rig.get("cameras"), f"{context}.rig.cameras")
        for camera_index, raw_camera in enumerate(rig_cameras):
            camera = _mapping(raw_camera, f"{context}.rig.cameras[{camera_index}]")
            width = float(camera["width_px"])
            height = float(camera["height_px"])
            intrinsics = _sequence(camera.get("K"), f"{context}.rig.cameras[{camera_index}].K")
            transform = _sequence(
                camera.get("T_base_from_camera"),
                f"{context}.rig.cameras[{camera_index}].T_base_from_camera",
            )
            camera_calibration.append(
                (
                    float(intrinsics[0]) / width,
                    float(intrinsics[4]) / height,
                    float(intrinsics[2]) / width,
                    float(intrinsics[5]) / height,
                    *(float(transform[index]) for index in range(12)),
                )
            )
    metadata = episode_metadata or _EpisodeMetadata()
    return TrainingExample(
        token=token,
        episode_id=episode_id,
        features=features,
        targets_xy=tuple(targets),
        target_yaw_rad=tuple(target_yaws),
        target_speed_mps=tuple(target_speeds),
        sequence_index=int(sample.get("sequence_index", 0)),
        anchor_timestamp_ns=int(sample.get("anchor_timestamp_ns", 0)),
        camera_paths=tuple(camera_paths),
        camera_sha256=tuple(camera_hashes),
        camera_calibration=tuple(camera_calibration),
        route_points_base_xy_m=route_points,
        rig_id=str(sample.get("rig_id", "")),
        domain=metadata.domain,
        source_dataset_id=metadata.source_dataset_id,
        source_dataset_version=metadata.source_dataset_version,
        license_id=metadata.license_id,
        scenario_tags=metadata.scenario_tags,
        map_id=metadata.map_id,
        site_id=metadata.site_id,
        route_id=metadata.route_id,
        source_manifest_sha256=metadata.source_manifest_sha256,
    )


def fingerprint_examples(examples: Sequence[TrainingExample]) -> str:
    digest = hashlib.sha256()
    for example in examples:
        value = {
            "token": example.token,
            "episode_id": example.episode_id,
            "features": example.features,
            "targets_xy": example.targets_xy,
            "target_yaw_rad": example.target_yaw_rad,
            "target_speed_mps": example.target_speed_mps,
            "sequence_index": example.sequence_index,
            "anchor_timestamp_ns": example.anchor_timestamp_ns,
            "camera_sha256": example.camera_sha256,
            "camera_calibration": example.camera_calibration,
            "route_points_base_xy_m": example.route_points_base_xy_m,
            "rig_id": example.rig_id,
            "domain": example.domain,
            "source_dataset_id": example.source_dataset_id,
            "source_dataset_version": example.source_dataset_version,
            "license_id": example.license_id,
            "scenario_tags": example.scenario_tags,
            "map_id": example.map_id,
            "site_id": example.site_id,
            "route_id": example.route_id,
            "source_manifest_sha256": example.source_manifest_sha256,
        }
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
    return digest.hexdigest()


def load_training_examples(
    root: Path | str,
    *,
    split: str,
    contract_path: Path | str | None = None,
    contract: Mapping[str, Any] | None = None,
    mode: str = "planning",
    check_image_hashes: bool = True,
) -> LoadedDataset:
    """Validate a dataset, then load one whole-episode split in file order."""
    report = validate_dataset(
        root,
        contract_path=contract_path,
        contract=contract,
        mode=mode,
        check_image_hashes=check_image_hashes,
    )
    root_path = Path(root).expanduser().resolve()
    manifest, manifest_hash = _read_json_and_sha256(root_path / "dataset.json")
    if manifest_hash != report["dataset_manifest_sha256"]:
        raise ContractError("dataset manifest changed after validation")
    rigs: dict[str, Mapping[str, Any]] = {}
    for reference_index, raw_reference in enumerate(manifest["rigs"]):
        reference = _mapping(raw_reference, f"dataset.rigs[{reference_index}]")
        rig_path = _safe_file(
            root_path, reference.get("manifest"), f"dataset.rigs[{reference_index}].manifest"
        )
        rig, rig_hash = _read_json_and_sha256(rig_path)
        if rig_hash != _sha256(
            reference.get("sha256"), f"dataset.rigs[{reference_index}].sha256"
        ):
            raise ContractError(f"{rig_path} changed after validation")
        rig_id = str(reference.get("rig_id", ""))
        if rig.get("rig_id") != rig_id:
            raise ContractError(f"{rig_path} rig_id changed after validation")
        rigs[rig_id] = rig

    examples: list[TrainingExample] = []
    for reference_index, raw_reference in enumerate(manifest["episodes"]):
        reference = _mapping(raw_reference, f"dataset.episodes[{reference_index}]")
        episode_path = _safe_file(
            root_path, reference.get("manifest"), f"dataset.episodes[{reference_index}].manifest"
        )
        episode, episode_hash = _read_json_and_sha256(episode_path)
        if episode_hash != _sha256(
            reference.get("sha256"), f"dataset.episodes[{reference_index}].sha256"
        ):
            raise ContractError(f"{episode_path} changed after validation")
        if episode["split"] != split:
            continue
        rig_id = str(episode.get("rig_id", ""))
        if rig_id not in rigs:
            raise ContractError(f"{episode_path} references an unavailable rig")
        provenance = _mapping(
            episode.get("source_provenance"), f"{episode_path}.source_provenance"
        )
        source_manifest_path = _safe_file(
            episode_path.parent,
            provenance.get("source_manifest_file"),
            f"{episode_path}.source_provenance.source_manifest_file",
        )
        source_manifest, source_manifest_hash = _read_json_and_sha256(
            source_manifest_path
        )
        expected_source_manifest_hash = _sha256(
            provenance.get("source_manifest_sha256"),
            f"{episode_path}.source_provenance.source_manifest_sha256",
        )
        if source_manifest_hash != expected_source_manifest_hash:
            raise ContractError(f"{source_manifest_path} changed after validation")
        episode_metadata = _episode_metadata(
            episode,
            source_manifest,
            source_manifest_hash,
            str(episode_path),
        )
        sample_path = _safe_file(
            episode_path.parent, episode.get("sample_jsonl"), f"{episode_path}.sample_jsonl"
        )
        expected_samples_hash = _sha256(
            episode.get("sample_jsonl_sha256"), f"{episode_path}.sample_jsonl_sha256"
        )
        for line_number, sample in _iter_jsonl(
            sample_path, expected_sha256=expected_samples_hash
        ):
            examples.append(
                _sample_to_example(
                    sample,
                    f"{sample_path}:{line_number}",
                    episode_root=episode_path.parent,
                    rig=rigs[rig_id],
                    episode_metadata=episode_metadata,
                )
            )
    if not examples:
        raise ContractError(f"dataset split {split!r} contains no samples")
    split_fingerprint = hashlib.sha256()
    split_fingerprint.update(str(report["dataset_fingerprint_sha256"]).encode("ascii"))
    split_fingerprint.update(b"\0")
    split_fingerprint.update(split.encode("utf-8"))
    split_fingerprint.update(b"\0")
    split_fingerprint.update(fingerprint_examples(examples).encode("ascii"))
    return LoadedDataset(tuple(examples), split_fingerprint.hexdigest(), report, split)


def deterministic_indices(size: int, *, seed: int, epoch: int) -> tuple[int, ...]:
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        raise ContractError("dataset size must be positive")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ContractError("batch seed must be a nonnegative integer")
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
        raise ContractError("epoch must be nonnegative")

    def stable_key(index: int) -> tuple[bytes, int]:
        token = f"portable-e2e-v1\0{seed}\0{epoch}\0{index}".encode("ascii")
        return hashlib.sha256(token).digest(), index

    return tuple(sorted(range(size), key=stable_key))


def batch_indices(
    size: int,
    *,
    batch_size: int,
    seed: int,
    epoch: int,
    drop_last: bool = False,
) -> tuple[tuple[int, ...], ...]:
    if isinstance(batch_size, bool) or not isinstance(batch_size, int) or batch_size <= 0:
        raise ContractError("batch_size must be positive")
    if not isinstance(drop_last, bool):
        raise ContractError("drop_last must be boolean")
    indices = deterministic_indices(size, seed=seed, epoch=epoch)
    batches: list[tuple[int, ...]] = []
    for offset in range(0, size, batch_size):
        batch = indices[offset : offset + batch_size]
        if len(batch) < batch_size and drop_last:
            continue
        batches.append(batch)
    if not batches:
        raise ContractError("batch configuration produces no batches")
    return tuple(batches)


def iter_examples(
    examples: Sequence[TrainingExample], indices: Iterable[int]
) -> tuple[TrainingExample, ...]:
    return tuple(examples[index] for index in indices)
