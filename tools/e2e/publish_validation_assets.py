#!/usr/bin/env python3
"""Publish verified validation evidence into ``docs/assets``.

The packaged-map sweep records CARLA BasicAgent evidence.  This publisher
deliberately keeps that evidence separate from explicit or terminal-matrix-
discovered, successful full-stack Autoware VAD trials.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any, Mapping, Sequence

from zoneinfo import ZoneInfo

from PIL import Image, ImageDraw, ImageFont

try:
    from carla_basicagent_sweep_report import (
        EVIDENCE_KIND,
        SUCCESS_STATUSES,
        validated_job,
    )
    from autoware_vad_town_matrix import (
        CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT as MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT,
        CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR as MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR,
        CAMERA_SOURCE_5HZ_CONTRACT as MATRIX_CAMERA_SOURCE_5HZ_CONTRACT,
        MatrixError as MatrixValidationError,
        _camera_source_5hz_evidence as _matrix_camera_source_5hz_evidence,
    )
    from export_carla_vad_expert import read_jsonl
    from render_carla_vad_expert import (
        _overview_index,
        render_episode,
        render_frame,
    )
except ModuleNotFoundError:
    from scripts.e2e.carla_basicagent_sweep_report import (
        EVIDENCE_KIND,
        SUCCESS_STATUSES,
        validated_job,
    )
    from scripts.e2e.autoware_vad_town_matrix import (
        CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT as MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT,
        CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR as MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR,
        CAMERA_SOURCE_5HZ_CONTRACT as MATRIX_CAMERA_SOURCE_5HZ_CONTRACT,
        MatrixError as MatrixValidationError,
        _camera_source_5hz_evidence as _matrix_camera_source_5hz_evidence,
    )
    from scripts.e2e.export_carla_vad_expert import read_jsonl
    from scripts.e2e.render_carla_vad_expert import (
        _overview_index,
        render_episode,
        render_frame,
    )


CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
MAP_ID_PATTERN = re.compile(r"^[a-z0-9_]+$")
TRIAL_ID_PATTERN = re.compile(r"^[a-z0-9_]+$")
MANAGED_BASICAGENT_NAMES = (
    "expert_overview_1920x1080.png",
    "expert_drive.gif",
)
MANAGED_VAD_NAMES = (
    "autoware_vad_result.json",
    "autoware_vad_route_result.png",
    "autoware_vad_turn_path_control.gif",
)
SPEED_30KPH_EVIDENCE_NAMES = (
    "speed_profile.json",
    "speed_profile.png",
)
SPEED_30KPH_CENTERED_CAPTURE_NAMES = (
    "autoware_rviz_candidate.png",
    "runtime.env",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
CAMERA_SOURCE_5HZ_PROVENANCE_NAMES = (
    "matrix_validation.json",
    "launch_args.txt",
    "recorder.log",
    "stack.log",
    "vad_route_manager.params.yaml",
    "sensor_mapping_provenance/sensor_mapping.yaml",
    "sensor_mapping_provenance/SHA256SUMS",
)
CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_PROVENANCE_NAMES = (
    *CAMERA_SOURCE_5HZ_PROVENANCE_NAMES,
    "vad_model_override_provenance/model_override.param.yaml",
    "vad_model_override_provenance/SHA256SUMS",
)
SPEED_30KPH_VISUAL_EVIDENCE_NAMES = (
    "desktop_capture.json",
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_drive.gif",
    "autoware_rviz_candidate.png",
    "autoware_rviz_capture.mkv",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
SPEED_30KPH_FULLSCREEN_DIMENSIONS = (1920, 1080)
SPEED_30KPH_DRIVE_GIF_DIMENSIONS = (960, 540)
OWNED_WINDOW_VISUAL_AUDIT_DIR = "owned_window_visual_audit"
OWNED_WINDOW_VISUAL_AUDIT_NAMES = (
    "v16_owned_window_visual_audit.json",
    "v16_owned_window_visual_audit.md",
    "v16_owned_window_visual_review.json",
    "v16_owned_window_contact_sheet.png",
)
PUBLICATION_SELECTION_SCOPE = (
    "autoware_vad_publications selected map_id/trial_id/trial_directory/source "
    "records; not a full-file publication manifest digest"
)
PUBLICATION_SELECTION_CANONICALIZATION = (
    "UTF-8 JSON with sort_keys=True, separators=(',', ':'), allow_nan=False"
)
CAMERA_SOURCE_DIAGNOSTICS_DIR = "campaign_diagnostics"
CAMERA_SOURCE_RETRY_RAW_NAMES = (
    "result.json",
    "source_route.json",
    "aligned_route.json",
    "launch_args.txt",
    "runtime.env",
    "latency/e2e_latency.json",
    "stack.log",
    "recorder.log",
    "sensor_mapping_provenance/sensor_mapping.yaml",
    "sensor_mapping_provenance/SHA256SUMS",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
SPEED_30KPH_RUNNABLE_MAP_IDS = frozenset(
    {
        "town01",
        "town02_opt",
        "town03",
        "town04",
        "town05_opt",
        "town06",
        "town07",
        "town10hd_opt",
        "c_track_1_0_7",
    }
)
SPEED_30KPH_BLOCKED_MAP_IDS = frozenset(
    {
        "town02",
        "town05",
        "town08",
        "town09",
        "town10hd",
        "town11",
        "town12",
        "town13",
        "town15",
        "woraksan_1_0_3",
    }
)
LABELED_VAD_ROUTE_NAME = "autoware_vad_route.json"
LABELED_VAD_EVIDENCE_NAMES = (
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_drive.gif",
    "desktop_capture.json",
    "diagnosis.json",
    "path_vs_control.png",
    "steering_tracking.png",
    "latency/e2e_latency.json",
)
VISUAL_REFRESH_MEDIA_NAMES = (
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_drive.gif",
    "autoware_rviz_candidate.png",
    "desktop_capture.json",
)
VISUAL_REFRESH_ARCHIVE_NAMES = (
    "result.json",
    "source_route.json",
    "runtime.env",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
MANAGED_IME_NAMES = (
    "vscode/vscode_ime_before_ascii.png",
    "vscode/vscode_ime_after_hangul.png",
)
DASHBOARD_NAME = "all_maps_basicagent_status_1920x1080.png"
BASICAGENT_AGGREGATE_NAME = "carla_basicagent_sweep_aggregate.json"
VAD_MATRIX_AGGREGATE_NAME = "autoware_vad_matrix_aggregate.json"
VAD_MATRIX_PLAN_NAME = "autoware_vad_matrix_plan.json"
CAMERA_SOURCE_5HZ_SELECTOR = "speed_30kph_camera_source_5hz"
CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR = (
    MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR
)
CAMERA_SOURCE_5HZ_SELECTORS = frozenset(
    {
        CAMERA_SOURCE_5HZ_SELECTOR,
        CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR,
    }
)
SPEED_30KPH_RUNTIME_PROFILE_SELECTORS = frozenset(
    {"speed_30kph", *CAMERA_SOURCE_5HZ_SELECTORS}
)
VAD_RUNTIME_PROFILE_SELECTORS = (
    "recommended",
    "speed_30kph",
    CAMERA_SOURCE_5HZ_SELECTOR,
    CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR,
)
VAD_RUNTIME_WRAPPER_OPTIONS = {
    "recommended": ["--recommended", "--visualize", "--capture-desktop"],
    "speed_30kph": [
        "--recommended",
        "--speed-30kph",
        "--visualize",
        "--capture-desktop",
    ],
    CAMERA_SOURCE_5HZ_SELECTOR: [
        "--recommended",
        "--speed-30kph",
        "--camera-source-5hz",
        "--visualize",
        "--capture-desktop",
    ],
    CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR: [
        "--recommended",
        "--speed-30kph",
        "--camera-source-5hz",
        "--visualize",
        "--capture-desktop",
    ],
}
SPEED_30KPH_RUNTIME_PROFILE_IDS = {
    "speed_30kph": "vad_carla_30kph_visualized_v2",
    CAMERA_SOURCE_5HZ_SELECTOR: (
        "vad_carla_30kph_camera_source_5hz_visualized_v1"
    ),
    CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR: (
        "vad_carla_30kph_camera_source_5hz_best_effort_image_visualized_v1"
    ),
}
SPEED_30KPH_PROFILE_ID = "carla_vad_30kph_v2"
SPEED_30KPH_VALIDATION_STATE = "carla_30kph_v2_screening"
SPEED_30KPH_PUBLICATION_INTERPRETATION = {
    "classification": "simulation screening",
    "profile_id": SPEED_30KPH_PROFILE_ID,
    "planning_architecture": "vad_route_manager_hybrid",
    "geometry_source": "VAD candidate geometry",
    "longitudinal_speed_source": "explicit CARLA simulation speed overlay",
    "vad_velocity_evaluated": False,
    "vad_geometry_evaluated": True,
    "real_vehicle_ready": False,
}
CAMERA_SOURCE_5HZ_CONTRACT = dict(MATRIX_CAMERA_SOURCE_5HZ_CONTRACT)
CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT = dict(
    MATRIX_CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT
)


class PublicationError(RuntimeError):
    """Raised when evidence is incomplete, stale, or mislabeled."""


@dataclass(frozen=True)
class BasicAgentRun:
    map_id: str
    canonical_name: str
    episode: Path
    export: Path
    episode_manifest: Mapping[str, Any]
    export_manifest: Mapping[str, Any]
    sample_count: int


@dataclass(frozen=True)
class VadTrial:
    map_id: str
    trial_id: str | None
    directory: Path
    result: Mapping[str, Any]
    route: Mapping[str, Any] | None
    desktop_capture: Mapping[str, Any] | None
    speed_profile: Mapping[str, Any] | None
    centered_capture_provenance: Mapping[str, Any] | None
    source: str
    runtime_profile_selector: str = "recommended"


@dataclass(frozen=True)
class VadTrialSpec:
    map_id: str
    trial_id: str | None
    directory: Path
    source: str = "explicit"
    evidence_root: Path | None = None
    runtime_profile_selector: str = "recommended"
    matrix_speed_contract: Mapping[str, Any] | None = None
    matrix_speed_evidence: Mapping[str, Any] | None = None
    matrix_visual_evidence: Mapping[str, Any] | None = None
    matrix_campaign_rviz_sha256: str | None = None


@dataclass(frozen=True)
class VadVisualRefresh:
    map_id: str
    trial_id: str
    directory: Path
    result: Mapping[str, Any]
    route: Mapping[str, Any]
    desktop_capture: Mapping[str, Any]
    source_route_sha256: str
    validation_source_route_sha256: str
    runtime_profile: Mapping[str, str]


@dataclass(frozen=True)
class ImeProof:
    before: Path
    after: Path
    before_size: tuple[int, int]
    after_size: tuple[int, int]


def _read_object_with_sha256(
    path: Path, label: str
) -> tuple[dict[str, Any], str]:
    try:
        raw = path.read_bytes()
        payload = json.loads(raw)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PublicationError(f"cannot read {label} JSON {path}: {error}") from error
    if not isinstance(payload, dict):
        raise PublicationError(f"{label} JSON root must be an object: {path}")
    return payload, hashlib.sha256(raw).hexdigest()


def _read_object(path: Path, label: str) -> dict[str, Any]:
    return _read_object_with_sha256(path, label)[0]


def _parse_timestamp(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.strip():
        raise PublicationError(f"{label} has no timestamp")
    normalized = value.strip()
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as error:
        raise PublicationError(f"{label} has an invalid timestamp: {value!r}") from error
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise PublicationError(f"{label} timestamp must include a UTC offset")
    return parsed


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _bag_manifest(path: Path, label: str) -> dict[str, Any]:
    root = path.expanduser()
    if root.is_symlink():
        raise PublicationError(f"{label} rosbag root must not be a symlink: {root}")
    root = root.resolve()
    if not root.is_dir():
        raise PublicationError(f"{label} rosbag directory is missing: {root}")
    files: list[dict[str, Any]] = []
    for source in sorted(root.rglob("*")):
        if source.is_symlink():
            raise PublicationError(f"{label} rosbag contains a symlink: {source}")
        if not source.is_file():
            continue
        files.append(
            {
                "path": source.relative_to(root).as_posix(),
                "size_bytes": source.stat().st_size,
                "sha256": _sha256(source),
            }
        )
    if not files or not any(item["path"] == "metadata.yaml" for item in files):
        raise PublicationError(f"{label} rosbag has no metadata.yaml")
    manifest = {"schema_version": 1, "root": str(root), "files": files}
    manifest["sha256"] = _sha256_json(
        {"schema_version": manifest["schema_version"], "files": files}
    )
    return manifest


def _inside(root: Path, candidate: Path, label: str) -> Path:
    root = root.expanduser().resolve()
    candidate = candidate.expanduser().resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise PublicationError(f"{label} escapes artifact root: {candidate}") from error
    return candidate


def _job_path(value: Any, plan_path: Path, artifact_root: Path, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise PublicationError(f"suite job is missing {label}")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = plan_path.parent / candidate
    return _inside(artifact_root, candidate, label)


def _status_matches_aggregate(
    aggregate_entry: Mapping[str, Any], status: Mapping[str, Any]
) -> bool:
    for key in ("map_id", "selected", "updated_at"):
        if aggregate_entry.get(key) != status.get(key):
            return False
    if aggregate_entry.get("status") == status.get("status"):
        return aggregate_entry.get("stage") == status.get("stage")
    return (
        aggregate_entry.get("status") == "FAILED"
        and aggregate_entry.get("stage") == "artifact_revalidation_failed"
        and aggregate_entry.get("recorded_status") == status.get("status")
    )


def load_sweep_snapshot(
    artifact_root: Path,
    expected_map_count: int = 19,
    expected_selected_map_count: int = 9,
) -> dict[str, Any]:
    artifact_root = artifact_root.expanduser().resolve()
    aggregate_path = artifact_root / "aggregate.json"
    aggregate, aggregate_sha256 = _read_object_with_sha256(
        aggregate_path, "sweep aggregate"
    )
    if aggregate.get("evidence_kind") != EVIDENCE_KIND:
        raise PublicationError(
            "aggregate evidence_kind is not the CARLA BasicAgent sweep contract"
        )
    maps = aggregate.get("maps")
    if not isinstance(maps, list) or len(maps) != expected_map_count:
        raise PublicationError(
            f"aggregate must contain exactly {expected_map_count} map records"
        )
    if aggregate.get("canonical_map_count") != expected_map_count:
        raise PublicationError("aggregate canonical_map_count is inconsistent")

    map_ids: set[str] = set()
    statuses: dict[str, Mapping[str, Any]] = {}
    for index, entry in enumerate(maps):
        if not isinstance(entry, dict):
            raise PublicationError(f"aggregate maps[{index}] must be an object")
        map_id = entry.get("map_id")
        if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
            raise PublicationError(f"aggregate maps[{index}] has an unsafe map_id")
        if map_id in map_ids:
            raise PublicationError(f"aggregate contains duplicate map_id {map_id}")
        map_ids.add(map_id)
        status_path = artifact_root / "maps" / map_id / "status.json"
        status = _read_object(status_path, f"{map_id} status")
        if status.get("evidence_kind") != EVIDENCE_KIND:
            raise PublicationError(f"{map_id} status has the wrong evidence_kind")
        if not _status_matches_aggregate(entry, status):
            raise PublicationError(
                f"{map_id} aggregate/status mismatch; regenerate aggregate.json after the sweep"
            )
        statuses[map_id] = status

    computed_counts = dict(sorted(Counter(str(item.get("status")) for item in maps).items()))
    if aggregate.get("status_counts") != computed_counts:
        raise PublicationError("aggregate status_counts is inconsistent with map records")
    selected_count = sum(entry.get("selected") is True for entry in maps)
    if (
        aggregate.get("selected_map_count") != selected_count
        or selected_count != expected_selected_map_count
    ):
        raise PublicationError(
            "aggregate must contain exactly "
            f"{expected_selected_map_count} selected packaged-map sweep records"
        )
    if any(
        entry.get("status") not in SUCCESS_STATUSES | {"FAILED"}
        for entry in maps
        if entry.get("selected") is True
    ):
        raise PublicationError("selected-map sweep still contains non-terminal records")
    selected_success = sum(
        entry.get("status") in SUCCESS_STATUSES
        for entry in maps
        if entry.get("selected") is True
    )
    if aggregate.get("selected_success_count") != selected_success:
        raise PublicationError("aggregate selected_success_count is inconsistent")
    snapshot = dict(aggregate)
    snapshot["artifact_root"] = artifact_root
    snapshot["aggregate_path"] = aggregate_path
    snapshot["aggregate_sha256"] = aggregate_sha256
    snapshot["statuses"] = statuses
    return snapshot


def _validate_render_inputs(episode: Path, export: Path, sample_count: Any) -> int:
    route = _read_object(episode / "route.json", "episode route")
    route_points = route.get("route")
    if not isinstance(route_points, list) or len(route_points) < 2:
        raise PublicationError(f"episode route has fewer than two points: {episode}")
    try:
        states = read_jsonl(episode / "states.jsonl")
        samples = read_jsonl(export / "samples.jsonl")
    except (OSError, RuntimeError, ValueError) as error:
        raise PublicationError(f"invalid episode/export JSONL for {episode}: {error}") from error
    if not states:
        raise PublicationError(f"episode contains no state records: {episode}")
    if not isinstance(sample_count, int) or isinstance(sample_count, bool):
        raise PublicationError(f"export manifest has invalid sample_count: {export}")
    if sample_count != len(samples):
        raise PublicationError(
            f"export sample_count mismatch for {export}: manifest={sample_count}, actual={len(samples)}"
        )
    for sample_index, sample in enumerate(samples):
        cameras = sample.get("cameras")
        if not isinstance(cameras, Mapping):
            raise PublicationError(f"sample {sample_index} has no camera mapping")
        for camera_name in CAMERA_NAMES:
            camera = cameras.get(camera_name)
            path_value = camera.get("path") if isinstance(camera, Mapping) else None
            if not isinstance(path_value, str) or not path_value:
                raise PublicationError(
                    f"sample {sample_index} is missing {camera_name} image path"
                )
            image_path = _inside(episode, episode / path_value, "camera image")
            if not image_path.is_file():
                raise PublicationError(f"camera image is missing: {image_path}")
    return len(samples)


def collect_basicagent_runs(snapshot: Mapping[str, Any]) -> list[BasicAgentRun]:
    artifact_root = Path(snapshot["artifact_root"])
    runs: list[BasicAgentRun] = []
    for entry in snapshot["maps"]:
        if entry.get("status") not in SUCCESS_STATUSES:
            continue
        map_id = entry["map_id"]
        smoke = entry.get("smoke")
        if not isinstance(smoke, Mapping):
            raise PublicationError(f"successful map {map_id} has no validated smoke record")
        _, validation_error = validated_job(artifact_root, map_id)
        if validation_error:
            raise PublicationError(
                f"successful map {map_id} failed fresh artifact validation: "
                f"{validation_error}"
            )
        plan_value = smoke.get("plan_path")
        if not isinstance(plan_value, str):
            raise PublicationError(f"successful map {map_id} has no collection plan path")
        plan_path = _inside(artifact_root, Path(plan_value), "collection plan")
        expected_plan_path = (
            artifact_root / "maps" / map_id / "smoke" / "collection_plan.json"
        ).resolve()
        if plan_path != expected_plan_path:
            raise PublicationError(f"{map_id} aggregate points to an unexpected plan path")
        plan = _read_object(plan_path, f"{map_id} collection plan")
        jobs = plan.get("jobs")
        if plan.get("status") != "COMPLETE" or not isinstance(jobs, list) or len(jobs) != 1:
            raise PublicationError(f"{map_id} collection plan is not one complete smoke job")
        job = jobs[0]
        if not isinstance(job, Mapping) or job.get("status") not in (
            "COMPLETE",
            "SKIP_RESUME_VALIDATED",
        ):
            raise PublicationError(f"{map_id} smoke job is not complete")
        paths = job.get("paths")
        if not isinstance(paths, Mapping):
            raise PublicationError(f"{map_id} smoke job paths are missing")
        episode = _job_path(paths.get("episode"), plan_path, artifact_root, "episode")
        export = _job_path(paths.get("export"), plan_path, artifact_root, "export")
        episode_manifest = _read_object(episode / "manifest.json", "episode manifest")
        export_manifest = _read_object(export / "manifest.json", "export manifest")
        if episode_manifest.get("status") != "complete":
            raise PublicationError(f"{map_id} episode manifest is not complete")
        if export_manifest.get("status") != "validated":
            raise PublicationError(f"{map_id} export manifest is not validated")
        result = episode_manifest.get("result")
        if not isinstance(result, Mapping) or result.get("goal_reached") is not True:
            raise PublicationError(f"{map_id} BasicAgent episode did not reach its goal")
        actual_count = _validate_render_inputs(
            episode, export, export_manifest.get("sample_count")
        )
        runs.append(
            BasicAgentRun(
                map_id=map_id,
                canonical_name=str(entry.get("canonical_name", map_id)),
                episode=episode,
                export=export,
                episode_manifest=episode_manifest,
                export_manifest=export_manifest,
                sample_count=actual_count,
            )
        )
    return runs


def _parse_vad_trial_specs(values: Sequence[str]) -> list[VadTrialSpec]:
    result: list[VadTrialSpec] = []
    seen: set[tuple[str, str | None]] = set()
    for value in values:
        identity, separator, path_text = value.partition("=")
        map_id, trial_separator, trial_id = identity.partition(":")
        if (
            not separator
            or not MAP_ID_PATTERN.fullmatch(map_id)
            or not path_text
            or (trial_separator and not TRIAL_ID_PATTERN.fullmatch(trial_id))
        ):
            raise PublicationError(
                "--vad-trial must use MAP_ID=TRIAL_DIRECTORY or "
                "MAP_ID:TRIAL_ID=TRIAL_DIRECTORY"
            )
        key = (map_id, trial_id if trial_separator else None)
        if key in seen:
            label = f"{map_id}:{trial_id}" if trial_separator else map_id
            raise PublicationError(f"duplicate --vad-trial id: {label}")
        seen.add(key)
        result.append(
            VadTrialSpec(
                map_id=map_id,
                trial_id=trial_id if trial_separator else None,
                directory=Path(path_text),
            )
        )
    return result


def _parse_vad_visual_refresh_specs(values: Sequence[str]) -> list[VadTrialSpec]:
    specs = _parse_vad_trial_specs(values)
    for spec in specs:
        if spec.trial_id is None:
            raise PublicationError(
                "--vad-visual-refresh must use MAP_ID:TRIAL_ID=TRIAL_DIRECTORY"
            )
    return [
        VadTrialSpec(
            map_id=spec.map_id,
            trial_id=spec.trial_id,
            directory=spec.directory,
            source="same_route_vehicle_centered_visual_refresh",
        )
        for spec in specs
    ]


def _normalize_vad_trial_specs(
    values: Mapping[str, Path] | Sequence[VadTrialSpec],
) -> list[VadTrialSpec]:
    if isinstance(values, Mapping):
        return [
            VadTrialSpec(map_id=str(map_id), trial_id=None, directory=Path(path))
            for map_id, path in values.items()
        ]
    result = list(values)
    if any(not isinstance(value, VadTrialSpec) for value in result):
        raise PublicationError("VAD trial sequence must contain VadTrialSpec values")
    return result


def _exact_finite_number(value: Any, expected: float, label: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or not math.isclose(
            float(value), expected, rel_tol=0.0, abs_tol=1.0e-9
        )
    ):
        raise PublicationError(f"{label} must be exactly {expected!r}")


def _is_speed_30kph_profile(selector: str) -> bool:
    return selector in SPEED_30KPH_RUNTIME_PROFILE_SELECTORS


def _is_camera_source_5hz_profile(selector: str) -> bool:
    return selector in CAMERA_SOURCE_5HZ_SELECTORS


def _camera_source_5hz_contract(selector: str) -> dict[str, Any]:
    if selector == CAMERA_SOURCE_5HZ_SELECTOR:
        return dict(CAMERA_SOURCE_5HZ_CONTRACT)
    if selector == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR:
        return dict(CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT)
    raise PublicationError(
        f"runtime profile {selector!r} is not a camera-source 5 Hz profile"
    )


def _camera_source_5hz_provenance_names(
    camera_contract: Mapping[str, Any],
) -> tuple[str, ...]:
    if camera_contract == CAMERA_SOURCE_5HZ_CONTRACT:
        return CAMERA_SOURCE_5HZ_PROVENANCE_NAMES
    if camera_contract == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT:
        return CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_PROVENANCE_NAMES
    raise PublicationError("camera-source 5 Hz runtime contract is not pinned")


def _speed_30kph_publication_interpretation(
    selector: str,
) -> dict[str, Any]:
    interpretation = dict(SPEED_30KPH_PUBLICATION_INTERPRETATION)
    if _is_camera_source_5hz_profile(selector):
        camera_contract = _camera_source_5hz_contract(selector)
        interpretation.update(
            {
                "camera_source_profile_id": camera_contract[
                    "profile_id"
                ],
                "camera_sensor_count": camera_contract[
                    "sensor_count"
                ],
                "camera_sensor_tick_sec": camera_contract[
                    "sensor_tick_sec"
                ],
                "camera_source_frequency_hz": camera_contract[
                    "source_frequency_hz"
                ],
                "camera_ros_publish_frequency_hz": camera_contract[
                    "ros_publish_frequency_hz"
                ],
                "maximum_camera_stamp_gap_sec": camera_contract[
                    "maximum_stamp_gap_sec"
                ],
            }
        )
        if selector == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_SELECTOR:
            interpretation.update(
                {
                    "maximum_raw_image_stamp_span_sec": camera_contract[
                        "maximum_raw_image_stamp_span_sec"
                    ],
                    "maximum_raw_image_supersession_percent": camera_contract[
                        "maximum_raw_image_supersession_percent"
                    ],
                    "camera_image_publish_qos": camera_contract[
                        "camera_image_publish_qos"
                    ],
                    "camera_info_publish_qos": camera_contract[
                        "camera_info_publish_qos"
                    ],
                    "vad_image_subscription_qos": camera_contract[
                        "vad_image_subscription_qos"
                    ],
                    "rviz_image_subscription_qos": camera_contract[
                        "rviz_image_subscription_qos"
                    ],
                    "sensor_mapping_file": camera_contract["sensor_mapping_file"],
                    "vad_model_override_file": camera_contract[
                        "vad_model_override_file"
                    ],
                    "real_vehicle_ready": camera_contract["real_vehicle_ready"],
                }
            )
    return interpretation


def _validate_matrix_runtime_profile(
    plan: Mapping[str, Any],
    aggregate: Mapping[str, Any],
    requested_selector: str,
) -> tuple[Mapping[str, Any], dict[str, Any] | None]:
    """Validate and classify the immutable matrix runtime profile.

    Historical recommended-profile matrices predate ``runtime_profile_selector``;
    they remain readable as recommended evidence.  The speed profile never gets
    that compatibility fallback: both plan and aggregate must explicitly identify
    it, and callers must opt in with the same selector.
    """
    if requested_selector not in VAD_RUNTIME_PROFILE_SELECTORS:
        raise PublicationError(
            f"unsupported Autoware VAD runtime profile selector: {requested_selector!r}"
        )
    plan_has_selector = "runtime_profile_selector" in plan
    aggregate_has_selector = "runtime_profile_selector" in aggregate
    plan_selector = plan.get("runtime_profile_selector", "recommended")
    aggregate_selector = aggregate.get("runtime_profile_selector", "recommended")
    if (
        plan_selector not in VAD_RUNTIME_PROFILE_SELECTORS
        or aggregate_selector not in VAD_RUNTIME_PROFILE_SELECTORS
        or plan_selector != aggregate_selector
    ):
        raise PublicationError(
            "Autoware VAD matrix runtime_profile_selector is missing, unknown, or "
            "inconsistent between plan and aggregate"
        )
    if plan_selector != requested_selector:
        raise PublicationError(
            "Autoware VAD matrix runtime profile does not match the explicitly "
            f"requested selector {requested_selector!r}"
        )
    if _is_speed_30kph_profile(requested_selector) and not (
        plan_has_selector and aggregate_has_selector
    ):
        raise PublicationError(
            "speed_30kph publication requires an explicit runtime_profile_selector "
            "in both matrix plan and aggregate"
        )

    runtime_profile = plan.get("runtime_profile")
    expected_options = VAD_RUNTIME_WRAPPER_OPTIONS[requested_selector]
    if (
        not isinstance(runtime_profile, Mapping)
        or runtime_profile.get("wrapper_options") != expected_options
        or runtime_profile.get("client_map_loading_allowed") is not False
        or aggregate.get("runtime_profile") != runtime_profile
    ):
        raise PublicationError(
            "Autoware VAD matrix did not use the selected fixed visualized profile"
        )
    if requested_selector == "recommended":
        return runtime_profile, None

    if (
        runtime_profile.get("id")
        != SPEED_30KPH_RUNTIME_PROFILE_IDS[requested_selector]
        or runtime_profile.get("trial_wrapper") != "run_recorded_route_trial.sh"
        or runtime_profile.get("map_lifecycle")
        != "cold_start_owned_process_group_per_trial"
    ):
        raise PublicationError(
            "speed_30kph runtime identity or CARLA lifecycle is not pinned"
        )
    contract = runtime_profile.get("speed_contract")
    if not isinstance(contract, Mapping):
        raise PublicationError("speed_30kph runtime profile has no speed_contract")
    required_contract = {
        "profile_id": SPEED_30KPH_PROFILE_ID,
        "validation_state": SPEED_30KPH_VALIDATION_STATE,
        "longitudinal_speed_source": "explicit_simulation_profile",
        "longitudinal_acceleration_role": "trajectory_internal_curve_exit_cap",
        "vad_geometry_source": True,
        "vad_cruise_velocity_evaluated": False,
        "vad_hard_stop_sentinel_preserved": True,
        "vad_velocity_evaluated": False,
        "vad_geometry_evaluated": True,
    }
    for field, expected in required_contract.items():
        if contract.get(field) != expected:
            raise PublicationError(
                f"speed_30kph speed_contract.{field} is not pinned to {expected!r}"
            )
    camera_contract = runtime_profile.get("camera_source_contract")
    if _is_camera_source_5hz_profile(requested_selector):
        if camera_contract != _camera_source_5hz_contract(requested_selector):
            raise PublicationError(
                "camera-source 5 Hz runtime contract is not pinned"
            )
    elif camera_contract is not None:
        raise PublicationError(
            "non-camera-source runtime profile unexpectedly carries a camera contract"
        )
    _exact_finite_number(
        contract.get("target_speed_mps"),
        8.333333333333334,
        "speed_30kph speed_contract.target_speed_mps",
    )
    route_manager = contract.get("route_manager_parameters")
    if (
        not isinstance(route_manager, Mapping)
        or route_manager.get("longitudinal_velocity_source")
        != "explicit_simulation_nominal"
    ):
        raise PublicationError(
            "speed_30kph must use the explicit CARLA simulation speed overlay"
        )
    for component_name in ("vehicle_cmd_gate", "longitudinal_controller"):
        component = contract.get(component_name)
        if (
            not isinstance(component, Mapping)
            or component.get("profile_id") != SPEED_30KPH_PROFILE_ID
            or component.get("speed_limit_source")
            != "explicit_simulation_profile"
            or component.get("real_vehicle_ready") is not False
        ):
            raise PublicationError(
                f"speed_30kph {component_name} is mislabeled or real-vehicle ready"
            )
    return runtime_profile, _speed_30kph_publication_interpretation(
        requested_selector
    )


def _validate_speed_matrix_campaign_contract(
    plan: Mapping[str, Any], aggregate: Mapping[str, Any]
) -> str:
    """Return the campaign-pinned RViz configuration digest.

    The speed publication is deliberately stricter than the historical
    recommended-profile publisher.  A self-consistent capture-side checksum is
    not enough: the capture must use the RViz file admitted into the immutable
    matrix execution contract.
    """
    campaign = plan.get("campaign_execution_contract")
    recorded_sha256 = plan.get("campaign_execution_contract_sha256")
    admission_sha256 = plan.get("admission_contract_sha256")
    if (
        not isinstance(campaign, Mapping)
        or campaign.get("schema_version") != 1
        or campaign.get("hash_algorithm") != "sha256"
        or not isinstance(recorded_sha256, str)
        or re.fullmatch(r"[0-9a-f]{64}", recorded_sha256) is None
        or _sha256_json(campaign) != recorded_sha256
        or aggregate.get("campaign_execution_contract_sha256") != recorded_sha256
        or not isinstance(admission_sha256, str)
        or re.fullmatch(r"[0-9a-f]{64}", admission_sha256) is None
        or aggregate.get("admission_contract_sha256") != admission_sha256
    ):
        raise PublicationError(
            "speed_30kph matrix campaign/admission contract is missing or stale"
        )
    files = campaign.get("files")
    if not isinstance(files, list):
        raise PublicationError(
            "speed_30kph matrix campaign execution contract has no file records"
        )
    rviz_records = [
        item
        for item in files
        if isinstance(item, Mapping)
        and item.get("path")
        == "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
    ]
    if len(rviz_records) != 1:
        raise PublicationError(
            "speed_30kph matrix campaign does not pin the centered RViz config"
        )
    rviz_sha256 = rviz_records[0].get("sha256")
    if (
        not isinstance(rviz_sha256, str)
        or re.fullmatch(r"[0-9a-f]{64}", rviz_sha256) is None
    ):
        raise PublicationError(
            "speed_30kph matrix centered RViz config digest is invalid"
        )
    return rviz_sha256


def _validate_matrix_trial_lifecycle(
    attempt: Path,
    validation: Mapping[str, Any],
    *,
    map_id: str,
    trial_id: str,
    canonical_name: str,
    expected_campaign_sha256: str,
    expected_admission_sha256: str,
) -> None:
    if (
        validation.get("campaign_execution_contract_sha256")
        != expected_campaign_sha256
        or validation.get("admission_contract_sha256")
        != expected_admission_sha256
    ):
        raise PublicationError(
            f"{map_id}:{trial_id} matrix validation is detached from its "
            "campaign/admission contract"
        )
    lifecycle = validation.get("carla_lifecycle")
    expected_generation = f"{map_id}_{trial_id}_{attempt.name}"
    owner_pid = lifecycle.get("owner_pid") if isinstance(lifecycle, Mapping) else None
    owner_pgid = lifecycle.get("owner_pgid") if isinstance(lifecycle, Mapping) else None
    if (
        not isinstance(lifecycle, Mapping)
        or lifecycle.get("schema_version") != 1
        or lifecycle.get("status") != "PASS"
        or lifecycle.get("lifecycle")
        != "cold_start_owned_process_group_per_trial"
        or lifecycle.get("generation_id") != expected_generation
        or lifecycle.get("expected_map") != canonical_name
        or not isinstance(owner_pid, int)
        or isinstance(owner_pid, bool)
        or owner_pid <= 0
        or owner_pgid != owner_pid
        or lifecycle.get("post_completion_exit_policy")
        != (
            "completion RPC PASS preserves a completed drive; the next trial "
            "always receives a new cold-start generation"
        )
    ):
        raise PublicationError(
            f"{map_id}:{trial_id} matrix CARLA lifecycle contract is invalid"
        )

    log_record = lifecycle.get("server_log")
    expected_log = (attempt / "carla_server.log").resolve()
    recorded_log = None
    if isinstance(log_record, Mapping) and isinstance(log_record.get("path"), str):
        recorded_log = Path(str(log_record["path"])).expanduser().resolve()
    if (
        not isinstance(log_record, Mapping)
        or recorded_log != expected_log
        or not expected_log.is_file()
        or log_record.get("size_bytes") != expected_log.stat().st_size
        or log_record.get("sha256") != _sha256(expected_log)
    ):
        raise PublicationError(
            f"{map_id}:{trial_id} matrix CARLA server log binding changed"
        )

    health_contracts = (
        (
            "preflight_health_sha256",
            "carla_preflight_health.json",
            "trial_preflight",
            "running",
        ),
        (
            "completion_health_sha256",
            "carla_completion_health.json",
            "trial_completion",
            "running",
        ),
        (
            "cleanup_health_sha256",
            "carla_cleanup_health.json",
            "trial_cleanup",
            "stopped",
        ),
    )
    for digest_field, file_name, stage, mode in health_contracts:
        health_path = attempt / file_name
        if (
            not health_path.is_file()
            or lifecycle.get(digest_field) != _sha256(health_path)
        ):
            raise PublicationError(
                f"{map_id}:{trial_id} matrix CARLA {stage} evidence changed"
            )
        health = _read_object(health_path, f"{map_id}:{trial_id} CARLA {stage}")
        if (
            health.get("schema_version") != 1
            or health.get("status") != "PASS"
            or health.get("stage") != stage
            or health.get("mode") != mode
            or health.get("generation_id") != expected_generation
            or health.get("expected_map") != canonical_name
            or health.get("owner_pid") != owner_pid
            or health.get("owner_pgid") != owner_pgid
            or health.get("read_only") is not True
            or (mode == "stopped" and health.get("port_released") is not True)
        ):
            raise PublicationError(
                f"{map_id}:{trial_id} matrix CARLA {stage} contract is invalid"
            )


def _validate_speed_matrix_visual_binding(
    attempt: Path,
    validation: Mapping[str, Any],
    *,
    map_id: str,
    trial_id: str,
    campaign_rviz_sha256: str,
) -> dict[str, Any]:
    label = f"{map_id}:{trial_id}"
    binding = validation.get("visual_evidence")
    files = binding.get("files") if isinstance(binding, Mapping) else None
    if (
        not isinstance(binding, Mapping)
        or binding.get("schema_version") != 1
        or binding.get("binding") != "matrix_validation_sha256_v1"
        or not isinstance(files, Mapping)
        or set(files) != set(SPEED_30KPH_VISUAL_EVIDENCE_NAMES)
        or binding.get("fullscreen_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or binding.get("candidate_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or binding.get("drive_gif_dimensions")
        != list(SPEED_30KPH_DRIVE_GIF_DIMENSIONS)
        or binding.get("vehicle_centered") is not True
        or binding.get("target_frame") != "base_link"
    ):
        raise PublicationError(
            f"{label} speed_30kph matrix has no complete centered visual binding"
        )
    for name in SPEED_30KPH_VISUAL_EVIDENCE_NAMES:
        path = attempt / name
        record = files.get(name)
        if (
            not path.is_file()
            or path.is_symlink()
            or not isinstance(record, Mapping)
            or record.get("size_bytes") != path.stat().st_size
            or record.get("sha256") != _sha256(path)
        ):
            raise PublicationError(
                f"{label} matrix visual evidence changed: {name}"
            )
    config_record = files[
        "rviz_capture_provenance/autoware_vad_carla.rviz"
    ]
    if config_record.get("sha256") != campaign_rviz_sha256:
        raise PublicationError(
            f"{label} centered RViz config differs from the admitted campaign file"
        )
    capture = _read_object(attempt / "desktop_capture.json", f"{label} desktop capture")
    if validation.get("desktop_capture") != capture:
        raise PublicationError(
            f"{label} desktop capture differs from matrix validation"
        )
    return dict(binding)


def discover_vad_matrix_trial_specs(
    snapshot: Mapping[str, Any],
    matrix_root: Path,
    runtime_profile_selector: str = "recommended",
) -> tuple[list[VadTrialSpec], dict[str, Any]]:
    """Discover only status-backed PASS trials from a finished Town matrix.

    Discovery is intentionally read-only.  The matrix runner performs its own
    strict validation and records the selected attempt in each map status;
    this function checks that status/aggregate/validation chain afresh before
    handing the attempt to the publication validator.
    """
    # A new matrix may intentionally live in a newer validation-date directory
    # than the immutable BasicAgent sweep.  Treat the user-selected matrix root as
    # its own read boundary and confine every recorded attempt beneath it.
    matrix_root = matrix_root.expanduser().resolve()
    plan_path = matrix_root / "matrix_plan.json"
    aggregate_path = matrix_root / "aggregate.json"
    plan, plan_sha256 = _read_object_with_sha256(
        plan_path, "Autoware VAD matrix plan"
    )
    aggregate, aggregate_sha256 = _read_object_with_sha256(
        aggregate_path, "Autoware VAD matrix aggregate"
    )
    matrix_id = plan.get("matrix_id")
    if (
        plan.get("schema_version") != 1
        or aggregate.get("schema_version") != 1
        or not isinstance(matrix_id, str)
        or not matrix_id.strip()
        or aggregate.get("matrix_id") != matrix_id
    ):
        raise PublicationError("Autoware VAD matrix plan/aggregate identity mismatch")
    if aggregate.get("status") not in {"COMPLETE", "FAILED"}:
        raise PublicationError(
            "Autoware VAD matrix is not terminal; wait for COMPLETE or FAILED"
        )
    plan_generated_at = plan.get("generated_at")
    aggregate_generated_at = aggregate.get("generated_at")
    plan_generated_time = _parse_timestamp(
        plan_generated_at, "Autoware VAD matrix plan"
    )
    aggregate_generated_time = _parse_timestamp(
        aggregate_generated_at, "Autoware VAD matrix aggregate"
    )
    if aggregate_generated_time < plan_generated_time:
        raise PublicationError(
            "Autoware VAD matrix aggregate predates its matrix plan"
        )

    runtime_profile, profile_interpretation = _validate_matrix_runtime_profile(
        plan, aggregate, runtime_profile_selector
    )
    campaign_rviz_sha256 = None
    if _is_speed_30kph_profile(runtime_profile_selector):
        campaign_rviz_sha256 = _validate_speed_matrix_campaign_contract(
            plan, aggregate
        )
    route_contract = plan.get("route_contract")
    contract_trials = (
        route_contract.get("trials") if isinstance(route_contract, Mapping) else None
    )
    if (
        not isinstance(contract_trials, list)
        or [item.get("id") for item in contract_trials if isinstance(item, Mapping)]
        != ["straight", "turn"]
        or aggregate.get("route_contract") != route_contract
    ):
        raise PublicationError(
            "Autoware VAD matrix must define separate straight and turn trials"
        )

    plan_maps = plan.get("maps")
    aggregate_maps = aggregate.get("maps")
    known_maps = {
        entry["map_id"]: str(entry.get("canonical_name", entry["map_id"]))
        for entry in snapshot["maps"]
    }
    if not isinstance(plan_maps, list) or not isinstance(aggregate_maps, list):
        raise PublicationError("Autoware VAD matrix map records are missing")
    if (
        plan.get("canonical_map_count") != len(plan_maps)
        or aggregate.get("canonical_map_count") != len(aggregate_maps)
        or len(plan_maps) != len(known_maps)
    ):
        raise PublicationError("Autoware VAD matrix canonical map count mismatch")

    def indexed_maps(values: Sequence[Any], label: str) -> dict[str, Mapping[str, Any]]:
        indexed: dict[str, Mapping[str, Any]] = {}
        for index, value in enumerate(values):
            if not isinstance(value, Mapping):
                raise PublicationError(f"{label} maps[{index}] must be an object")
            map_id = value.get("map_id")
            if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
                raise PublicationError(f"{label} maps[{index}] has an unsafe map_id")
            if map_id in indexed:
                raise PublicationError(f"{label} contains duplicate map_id {map_id}")
            indexed[map_id] = value
        return indexed

    plan_by_map = indexed_maps(plan_maps, "matrix plan")
    aggregate_by_map = indexed_maps(aggregate_maps, "matrix aggregate")
    if set(plan_by_map) != set(known_maps) or set(aggregate_by_map) != set(known_maps):
        raise PublicationError("Autoware VAD matrix map ids differ from the canonical sweep")
    if _is_speed_30kph_profile(runtime_profile_selector):
        canonical_ids = set(known_maps)
        if canonical_ids != (
            set(SPEED_30KPH_RUNNABLE_MAP_IDS) | set(SPEED_30KPH_BLOCKED_MAP_IDS)
        ):
            raise PublicationError(
                "speed_30kph canonical map inventory differs from the pinned 19-map set"
            )
        planned_runnable = {
            map_id
            for map_id, entry in plan_by_map.items()
            if entry.get("runnable") is True
        }
        planned_blocked = canonical_ids - planned_runnable
        if (
            planned_runnable != set(SPEED_30KPH_RUNNABLE_MAP_IDS)
            or planned_blocked != set(SPEED_30KPH_BLOCKED_MAP_IDS)
        ):
            raise PublicationError(
                "speed_30kph matrix must admit the exact 9 runnable maps and retain "
                "the exact 10 canonical BLOCKED maps"
            )

    def matrix_attempt_path(
        value: Any, map_id: str, trial_id: str
    ) -> Path | None:
        if value is None:
            return None
        if not isinstance(value, str) or not value.strip():
            raise PublicationError(
                f"{map_id}:{trial_id} has an invalid matrix attempt path"
            )
        candidate = Path(value).expanduser()
        if not candidate.is_absolute():
            candidate = matrix_root / candidate
        attempt = _inside(matrix_root, candidate, "matrix trial attempt")
        expected_parent = (
            matrix_root / "maps" / map_id / "trials" / trial_id
        ).resolve()
        if (
            attempt.parent != expected_parent
            or re.fullmatch(r"attempt_[0-9]+", attempt.name) is None
        ):
            raise PublicationError(
                f"{map_id}:{trial_id} attempt is outside its canonical trial directory"
            )
        return attempt

    def failed_attempt_route(
        attempt: Path | None, map_id: str, trial_id: str
    ) -> dict[str, Any] | None:
        if attempt is None or not (attempt / "result.json").is_file():
            return None
        result = _read_object(
            attempt / "result.json", f"{map_id}:{trial_id} failed matrix result"
        )
        if not isinstance(result.get("route_file"), str):
            return None
        return _labeled_trial_route(
            attempt, result, known_maps[map_id], f"{map_id}:{trial_id} failed matrix"
        )

    status_counts: Counter[str] = Counter()
    specs: list[VadTrialSpec] = []
    failed_maps: list[dict[str, Any]] = []
    matrix_maps: list[dict[str, Any]] = []
    for map_id in known_maps:
        plan_entry = plan_by_map[map_id]
        if plan_entry.get("canonical_name") != known_maps[map_id]:
            raise PublicationError(f"{map_id} matrix canonical name mismatch")
        status_path = matrix_root / "maps" / map_id / "status.json"
        status = _read_object(status_path, f"{map_id} matrix status")
        if status != aggregate_by_map[map_id]:
            raise PublicationError(
                f"{map_id} matrix aggregate/status mismatch; regenerate the matrix summary"
            )
        if (
            status.get("matrix_id") != matrix_id
            or status.get("map_id") != map_id
            or status.get("canonical_name") != known_maps[map_id]
            or status.get("runnable") != (plan_entry.get("runnable") is True)
        ):
            raise PublicationError(f"{map_id} matrix status identity mismatch")
        if plan_entry.get("runnable") is True:
            if status.get("status") not in {"PASS", "FAILED"}:
                raise PublicationError(
                    "Autoware VAD matrix still has non-terminal runnable maps"
                )
        elif status.get("status") != "BLOCKED":
            raise PublicationError(f"{map_id} non-runnable matrix map is not BLOCKED")
        _parse_timestamp(status.get("updated_at"), f"{map_id} matrix status")
        if not isinstance(status.get("stage"), str) or not status["stage"].strip():
            raise PublicationError(f"{map_id} matrix status has no stage")
        if not isinstance(status.get("reason"), str) or not status["reason"].strip():
            raise PublicationError(f"{map_id} matrix status has no reason")
        status_counts[str(status.get("status"))] += 1
        trials = status.get("trials")
        if not isinstance(trials, Mapping) or set(trials) != {"straight", "turn"}:
            raise PublicationError(f"{map_id} matrix status has invalid trial records")
        non_pass_trials: list[dict[str, Any]] = []
        matrix_trial_records: dict[str, dict[str, Any]] = {}
        for trial_id in ("straight", "turn"):
            trial = trials[trial_id]
            if not isinstance(trial, Mapping):
                raise PublicationError(f"{map_id}:{trial_id} status must be an object")
            trial_status = trial.get("status")
            allowed_trial_statuses = (
                {"PASS", "FAILED"}
                if plan_entry.get("runnable") is True
                else {"BLOCKED"}
            )
            if trial_status not in allowed_trial_statuses:
                raise PublicationError(
                    f"{map_id}:{trial_id} has a non-terminal or inconsistent "
                    "matrix trial status"
                )
            trial_reason = trial.get("reason")
            if not isinstance(trial_reason, str) or not trial_reason.strip():
                raise PublicationError(f"{map_id}:{trial_id} has no matrix reason")
            attempt = matrix_attempt_path(
                trial.get("attempt_directory"), map_id, trial_id
            )
            if trial_status == "BLOCKED" and attempt is not None:
                raise PublicationError(
                    f"{map_id}:{trial_id} BLOCKED trial unexpectedly has an attempt"
                )
            if trial_status != "PASS":
                non_pass_record = {
                    "trial_id": trial_id,
                    "status": str(trial_status),
                    "reason": trial_reason,
                    "attempt_directory": str(attempt) if attempt is not None else None,
                    "route": failed_attempt_route(attempt, map_id, trial_id),
                }
                non_pass_trials.append(non_pass_record)
                matrix_trial_records[trial_id] = dict(non_pass_record)
                continue
            validation_value = trial.get("validation")
            if attempt is None or not isinstance(validation_value, str):
                raise PublicationError(
                    f"{map_id}:{trial_id} PASS status lacks attempt/validation paths"
                )
            validation_candidate = Path(validation_value).expanduser()
            if not validation_candidate.is_absolute():
                validation_candidate = matrix_root / validation_candidate
            validation_path = _inside(
                matrix_root, validation_candidate, "matrix trial validation"
            )
            if validation_path != attempt / "matrix_validation.json":
                raise PublicationError(
                    f"{map_id}:{trial_id} status points to an unexpected validation file"
                )
            validation = _read_object(
                validation_path, f"{map_id}:{trial_id} matrix validation"
            )
            result = validation.get("result")
            recorded_directory = validation.get("trial_directory")
            recorded_attempt = None
            if isinstance(recorded_directory, str):
                recorded_candidate = Path(recorded_directory).expanduser()
                if not recorded_candidate.is_absolute():
                    recorded_candidate = matrix_root / recorded_candidate
                recorded_attempt = _inside(
                    matrix_root, recorded_candidate, "recorded matrix trial attempt"
                )
            if (
                validation.get("status") != "PASS"
                or validation.get("matrix_id") != matrix_id
                or validation.get("map_id") != map_id
                or validation.get("trial_id") != trial_id
                or recorded_attempt != attempt
                or validation.get(
                    "runtime_profile_selector", "recommended"
                )
                != runtime_profile_selector
                or validation.get("runtime_profile") != runtime_profile
                or not isinstance(result, Mapping)
                or result.get("success") is not True
                or result.get("execution_mode") != "full_stack"
                or result.get("planning_architecture")
                != "vad_route_manager_hybrid"
                or result.get("route_completion") != "PASS"
                or result.get("goal_reached") is not True
            ):
                raise PublicationError(
                    f"{map_id}:{trial_id} matrix validation is stale or incomplete"
                )
            visual_binding = None
            if _is_speed_30kph_profile(runtime_profile_selector):
                if campaign_rviz_sha256 is None:  # pragma: no cover - set above.
                    raise PublicationError("speed_30kph campaign RViz digest was lost")
                _validate_matrix_trial_lifecycle(
                    attempt,
                    validation,
                    map_id=map_id,
                    trial_id=trial_id,
                    canonical_name=known_maps[map_id],
                    expected_campaign_sha256=str(
                        plan["campaign_execution_contract_sha256"]
                    ),
                    expected_admission_sha256=str(plan["admission_contract_sha256"]),
                )
                visual_binding = _validate_speed_matrix_visual_binding(
                    attempt,
                    validation,
                    map_id=map_id,
                    trial_id=trial_id,
                    campaign_rviz_sha256=campaign_rviz_sha256,
                )
                attempt_result = _read_object(
                    attempt / "result.json", f"{map_id}:{trial_id} matrix result"
                )
                pass_route = _labeled_trial_route(
                    attempt,
                    attempt_result,
                    known_maps[map_id],
                    f"{map_id}:{trial_id} matrix",
                )
                route_scenario = pass_route["scenario"]
                scenario_is_valid = (
                    trial_id == "straight" and route_scenario == "straight"
                ) or (
                    trial_id == "turn" and route_scenario in {"left", "right"}
                )
                expected_turn_direction = (
                    route_scenario if trial_id == "turn" else None
                )
                if (
                    not scenario_is_valid
                    or validation.get("catalog_scenario") != route_scenario
                    or validation.get("turn_direction") != expected_turn_direction
                ):
                    raise PublicationError(
                        f"{map_id}:{trial_id} route scenario does not match its "
                        "straight/turn matrix slot"
                    )
            validation_speed = validation.get("speed_contract")
            if _is_speed_30kph_profile(runtime_profile_selector):
                if (
                    not isinstance(validation_speed, Mapping)
                    or validation_speed.get("status") != "PASS"
                    or validation_speed.get("profile_id")
                    != SPEED_30KPH_PROFILE_ID
                ):
                    raise PublicationError(
                        f"{map_id}:{trial_id} speed_30kph matrix validation lacks "
                        "passing speed-contract evidence"
                    )
            elif validation_speed is not None:
                raise PublicationError(
                    f"{map_id}:{trial_id} recommended matrix unexpectedly carries "
                    "speed-profile evidence"
                )
            validation_camera = validation.get("camera_source_contract")
            if _is_camera_source_5hz_profile(runtime_profile_selector):
                _validated_camera_source_5hz_evidence(
                    attempt,
                    runtime_profile,
                    validation_camera,
                    f"{map_id}:{trial_id}",
                )
            elif validation_camera is not None:
                raise PublicationError(
                    f"{map_id}:{trial_id} non-camera-source matrix unexpectedly "
                    "carries camera-source evidence"
                )
            specs.append(
                VadTrialSpec(
                    map_id=map_id,
                    trial_id=trial_id,
                    directory=attempt,
                    source="matrix_pass_auto_discovery",
                    evidence_root=matrix_root,
                    runtime_profile_selector=runtime_profile_selector,
                    matrix_speed_contract=(
                        dict(runtime_profile["speed_contract"])
                        if _is_speed_30kph_profile(runtime_profile_selector)
                        else None
                    ),
                    matrix_speed_evidence=(
                        dict(validation_speed)
                        if isinstance(validation_speed, Mapping)
                        else None
                    ),
                    matrix_visual_evidence=(
                        visual_binding
                        if isinstance(visual_binding, Mapping)
                        else None
                    ),
                    matrix_campaign_rviz_sha256=campaign_rviz_sha256,
                )
            )
            matrix_trial_records[trial_id] = {
                "trial_id": trial_id,
                "status": "PASS",
                "reason": trial_reason,
                "attempt_directory": str(attempt),
                "route": None,
                "speed_contract": (
                    dict(validation_speed)
                    if isinstance(validation_speed, Mapping)
                    else None
                ),
                "camera_source_contract": (
                    dict(validation_camera)
                    if isinstance(validation_camera, Mapping)
                    else None
                ),
            }
        trial_statuses = {
            trial_id: record["status"]
            for trial_id, record in matrix_trial_records.items()
        }
        if status.get("status") == "PASS" and set(trial_statuses.values()) != {"PASS"}:
            raise PublicationError(f"{map_id} PASS map does not have two PASS trials")
        if status.get("status") == "BLOCKED" and set(trial_statuses.values()) != {"BLOCKED"}:
            raise PublicationError(f"{map_id} BLOCKED map has non-BLOCKED trials")
        matrix_maps.append(
            {
                "map_id": map_id,
                "canonical_name": known_maps[map_id],
                "runnable": status.get("runnable") is True,
                "status": str(status.get("status")),
                "stage": status["stage"],
                "reason": status["reason"],
                "block_code": status.get("block_code"),
                "updated_at": status["updated_at"],
                "trials": matrix_trial_records,
            }
        )
        if status.get("status") == "FAILED":
            failed_maps.append(
                {
                    "map_id": map_id,
                    "canonical_name": known_maps[map_id],
                    "stage": str(status.get("stage") or ""),
                    "reason": str(status.get("reason") or ""),
                    "trials": non_pass_trials,
                }
            )

    computed_counts = dict(sorted(status_counts.items()))
    if aggregate.get("status_counts") != computed_counts:
        raise PublicationError("Autoware VAD matrix status_counts is inconsistent")
    runnable_count = sum(entry.get("runnable") is True for entry in plan_maps)
    runnable_pass_count = sum(
        status.get("runnable") is True and status.get("status") == "PASS"
        for status in aggregate_maps
    )
    blocked_count = sum(status.get("status") == "BLOCKED" for status in aggregate_maps)
    if (
        aggregate.get("runnable_map_count") != runnable_count
        or aggregate.get("runnable_pass_count") != runnable_pass_count
        or aggregate.get("blocked_map_count") != blocked_count
    ):
        raise PublicationError("Autoware VAD matrix aggregate counts are inconsistent")
    expected_overall = (
        "COMPLETE"
        if runnable_count > 0 and runnable_pass_count == runnable_count
        else "FAILED"
    )
    if aggregate.get("status") != expected_overall:
        raise PublicationError("Autoware VAD matrix overall status is inconsistent")
    if _is_speed_30kph_profile(runtime_profile_selector):
        runnable_statuses = {
            item["map_id"]: item["status"]
            for item in matrix_maps
            if item["map_id"] in SPEED_30KPH_RUNNABLE_MAP_IDS
        }
        blocked_statuses = {
            item["map_id"]: item["status"]
            for item in matrix_maps
            if item["map_id"] in SPEED_30KPH_BLOCKED_MAP_IDS
        }
        if (
            aggregate.get("status") != "COMPLETE"
            or runnable_count != 9
            or runnable_pass_count != 9
            or blocked_count != 10
            or computed_counts != {"BLOCKED": 10, "PASS": 9}
            or set(runnable_statuses.values()) != {"PASS"}
            or set(blocked_statuses.values()) != {"BLOCKED"}
            or len(specs) != 18
        ):
            raise PublicationError(
                "speed_30kph publication requires final_state COMPLETE, 9/9 "
                "runnable map PASS, 10 canonical BLOCKED maps, and 18 bound "
                "straight/turn PASS trials"
            )
    return specs, {
        "matrix_root": str(matrix_root),
        "matrix_id": matrix_id,
        "status": aggregate["status"],
        "generated_at": aggregate_generated_at,
        "plan_generated_at": plan_generated_at,
        "plan": str(plan_path),
        "plan_sha256": plan_sha256,
        "aggregate": str(aggregate_path),
        "aggregate_sha256": aggregate_sha256,
        "runtime_profile_selector": runtime_profile_selector,
        "runtime_profile": dict(runtime_profile),
        "evidence_interpretation": profile_interpretation,
        "discovered_pass_trial_count": len(specs),
        "canonical_map_count": len(plan_maps),
        "runnable_map_count": runnable_count,
        "runnable_pass_count": runnable_pass_count,
        "blocked_map_count": blocked_count,
        "status_counts": computed_counts,
        "failed_map_count": len(failed_maps),
        "failed_trial_count": sum(
            trial["status"] == "FAILED"
            for item in failed_maps
            for trial in item["trials"]
        ),
        "failed_maps": failed_maps,
        "maps": matrix_maps,
        "scope": (
            "terminal matrix PASS trials auto-discovered from fresh "
            "status + matrix_validation records"
        ),
    }


def resolve_vad_trial_specs(
    snapshot: Mapping[str, Any],
    explicit_values: Mapping[str, Path] | Sequence[VadTrialSpec],
    matrix_root: Path | None,
    runtime_profile_selector: str = "recommended",
) -> tuple[list[VadTrialSpec], dict[str, Any] | None]:
    explicit = _normalize_vad_trial_specs(explicit_values)
    if runtime_profile_selector not in VAD_RUNTIME_PROFILE_SELECTORS:
        raise PublicationError(
            f"unsupported Autoware VAD runtime profile selector: "
            f"{runtime_profile_selector!r}"
        )
    if runtime_profile_selector != "recommended" and matrix_root is None:
        raise PublicationError(
            "speed_30kph publication requires --vad-matrix-root; standalone explicit "
            "trials cannot establish the immutable simulation-screening contract"
        )
    if _is_speed_30kph_profile(runtime_profile_selector) and explicit:
        raise PublicationError(
            "speed_30kph publication accepts only status-backed matrix PASS trials; "
            "do not mix unbound explicit trials into simulation-screening evidence"
        )
    discovered: list[VadTrialSpec] = []
    matrix_record = None
    if matrix_root is not None:
        discovered, matrix_record = discover_vad_matrix_trial_specs(
            snapshot, matrix_root, runtime_profile_selector
        )
        failed_identities = {
            (item["map_id"], trial["trial_id"])
            for item in matrix_record["failed_maps"]
            for trial in item["trials"]
            if trial["status"] == "FAILED"
        }
        explicit = [
            VadTrialSpec(
                map_id=spec.map_id,
                trial_id=spec.trial_id,
                directory=spec.directory,
                source=(
                    "explicit_recovery_for_matrix_failure"
                    if (spec.map_id, spec.trial_id) in failed_identities
                    else spec.source
                ),
                evidence_root=spec.evidence_root,
                runtime_profile_selector=spec.runtime_profile_selector,
                matrix_speed_contract=spec.matrix_speed_contract,
                matrix_speed_evidence=spec.matrix_speed_evidence,
                matrix_visual_evidence=spec.matrix_visual_evidence,
                matrix_campaign_rviz_sha256=spec.matrix_campaign_rviz_sha256,
            )
            for spec in explicit
        ]
    combined = explicit + discovered
    seen: set[tuple[str, str | None]] = set()
    for spec in combined:
        identity = (spec.map_id, spec.trial_id)
        if identity in seen:
            label = f"{spec.map_id}:{spec.trial_id or 'default'}"
            raise PublicationError(
                f"duplicate explicit/matrix Autoware VAD trial id: {label}"
            )
        seen.add(identity)
    return combined, matrix_record


def _labeled_trial_route(
    directory: Path,
    result: Mapping[str, Any],
    canonical_name: str,
    label: str,
) -> dict[str, Any]:
    route_value = result.get("route_file")
    if not isinstance(route_value, str) or not route_value.strip():
        raise PublicationError(f"{label} labeled trial result has no route_file")
    route_path = Path(route_value).expanduser()
    if not route_path.is_absolute():
        route_path = directory / route_path
    route_path = _inside(directory, route_path, f"{label} route")
    route = _read_object(route_path, f"{label} route")
    town = route.get("town")
    scenario = route.get("scenario")
    length = route.get("route_length_m")
    if town != canonical_name:
        raise PublicationError(
            f"{label} route town {town!r} does not match {canonical_name!r}"
        )
    if not isinstance(scenario, str) or not scenario.strip():
        raise PublicationError(f"{label} route has no scenario")
    endpoint_source = route.get("endpoint_source", "spawn_points")
    endpoint_record: dict[str, Any]
    if endpoint_source == "spawn_points":
        start = route.get("start_spawn_index")
        goal = route.get("goal_spawn_index")
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value < 0
            for value in (start, goal)
        ):
            raise PublicationError(f"{label} route has invalid spawn-point indices")
        endpoint_record = {
            "endpoint_source": "spawn_points",
            "start_index": start,
            "goal_index": goal,
            "start_spawn_index": start,
            "goal_spawn_index": goal,
        }
    elif endpoint_source == "generated_waypoints":
        start = route.get("start_endpoint_index")
        goal = route.get("goal_endpoint_index")
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value < 0
            for value in (start, goal)
        ):
            raise PublicationError(
                f"{label} route has invalid generated-waypoint endpoint indices"
            )
        spacing = route.get("endpoint_waypoint_spacing_m")
        if (
            isinstance(spacing, bool)
            or not isinstance(spacing, (int, float))
            or not math.isfinite(float(spacing))
            or float(spacing) <= 0.0
        ):
            raise PublicationError(
                f"{label} route has invalid generated-waypoint endpoint spacing"
            )
        spawn_height = route.get("spawn_height_contract")
        if (
            not isinstance(spawn_height, Mapping)
            or spawn_height.get("offset_owner")
            != "autoware_carla_interface_bridge"
        ):
            raise PublicationError(
                f"{label} route lacks the bridge-owned spawn-height contract"
            )
        _exact_finite_number(
            spawn_height.get("bridge_z_offset_m"),
            2.0,
            f"{label} route bridge_z_offset_m",
        )
        _exact_finite_number(
            spawn_height.get("catalog_z_offset_m"),
            0.0,
            f"{label} route catalog_z_offset_m",
        )
        endpoint_record = {
            "endpoint_source": "generated_waypoints",
            "start_index": start,
            "goal_index": goal,
            "start_endpoint_index": start,
            "goal_endpoint_index": goal,
            "endpoint_waypoint_spacing_m": float(spacing),
            "spawn_height_contract": dict(spawn_height),
        }
    else:
        raise PublicationError(
            f"{label} route has unsupported endpoint_source {endpoint_source!r}"
        )
    if (
        not isinstance(length, (int, float))
        or isinstance(length, bool)
        or not math.isfinite(float(length))
        or float(length) <= 0.0
    ):
        raise PublicationError(f"{label} route has invalid route_length_m")
    return {
        "source": str(route_path),
        "town": town,
        "scenario": scenario,
        "route_length_m": float(length),
        **endpoint_record,
    }


def _image_dimensions(path: Path, label: str, expected_format: str) -> tuple[int, int]:
    try:
        with Image.open(path) as image:
            if image.format != expected_format:
                raise PublicationError(
                    f"{label} must be {expected_format}, got {image.format!r}"
                )
            dimensions = image.size
            image.verify()
    except (OSError, ValueError) as error:
        raise PublicationError(f"cannot validate {label} {path}: {error}") from error
    return dimensions


def _dimension_pair(value: Any, label: str) -> tuple[int, int]:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(
            not isinstance(item, int) or isinstance(item, bool) or item <= 0
            for item in value
        )
    ):
        raise PublicationError(f"{label} must be [positive_width, positive_height]")
    return value[0], value[1]


def _labeled_desktop_capture(directory: Path, label: str) -> dict[str, Any]:
    for name in LABELED_VAD_EVIDENCE_NAMES:
        if not (directory / name).is_file():
            raise PublicationError(f"{label} VAD trial is missing {name}")
    capture = _read_object(
        directory / "desktop_capture.json", f"{label} desktop capture"
    )
    if capture.get("schema_version") != 1:
        raise PublicationError(f"{label} desktop capture schema_version must be 1")
    if capture.get("candidate_observed") is not True:
        raise PublicationError(f"{label} desktop capture did not observe a VAD candidate")
    topic = capture.get("candidate_topic")
    if (
        not isinstance(topic, str)
        or topic.lstrip("/") != "planning/vad/candidate_trajectories"
    ):
        raise PublicationError(f"{label} desktop capture has the wrong candidate topic")
    if capture.get("capture_started_after_candidate") is not True:
        raise PublicationError(
            f"{label} desktop capture was not started after a VAD candidate"
        )
    expected_files = {
        "png_file": "autoware_rviz_fullscreen.png",
        "gif_file": "autoware_rviz_drive.gif",
    }
    for key, expected in expected_files.items():
        if capture.get(key) != expected:
            raise PublicationError(f"{label} desktop capture {key} must be {expected!r}")
    captured_at = capture.get("captured_at")
    display = capture.get("display")
    _parse_timestamp(captured_at, f"{label} desktop capture")
    if not isinstance(display, str) or not display.strip():
        raise PublicationError(f"{label} desktop capture has no display")

    source_dimensions = _dimension_pair(
        capture.get("source_dimensions"), f"{label} source_dimensions"
    )
    png_dimensions = _dimension_pair(
        capture.get("png_dimensions"), f"{label} png_dimensions"
    )
    gif_dimensions = _dimension_pair(
        capture.get("gif_dimensions"), f"{label} gif_dimensions"
    )
    actual_png = _image_dimensions(
        directory / "autoware_rviz_fullscreen.png",
        f"{label} Autoware/RViz fullscreen",
        "PNG",
    )
    actual_gif = _image_dimensions(
        directory / "autoware_rviz_drive.gif",
        f"{label} Autoware/RViz drive",
        "GIF",
    )
    if source_dimensions != png_dimensions or actual_png != png_dimensions:
        raise PublicationError(
            f"{label} fullscreen PNG dimensions do not match the captured display"
        )
    if actual_gif != gif_dimensions or gif_dimensions[0] != 960:
        raise PublicationError(
            f"{label} drive GIF dimensions must match metadata at 960 px width"
        )
    for name in ("route_result.png", "path_vs_control.png", "steering_tracking.png"):
        _image_dimensions(directory / name, f"{label} {name}", "PNG")
    _image_dimensions(
        directory / "turn_path_control.gif", f"{label} turn_path_control.gif", "GIF"
    )
    _read_object(directory / "diagnosis.json", f"{label} diagnosis")
    _read_object(
        directory / "latency/e2e_latency.json", f"{label} latency analysis"
    )
    return dict(capture)


def _validated_speed_profile(
    directory: Path,
    result: Mapping[str, Any],
    matrix_speed_evidence: Mapping[str, Any] | None,
    label: str,
) -> dict[str, Any]:
    json_path = directory / "speed_profile.json"
    plot_path = directory / "speed_profile.png"
    if not json_path.is_file() or not plot_path.is_file():
        raise PublicationError(
            f"{label} speed_30kph trial is missing speed_profile.json/png"
        )
    profile, profile_sha256 = _read_object_with_sha256(
        json_path, f"{label} speed profile"
    )
    if (
        profile.get("schema_version") != 1
        or profile.get("analysis") != "carla_speed_source_evidence"
        or profile.get("status") != "complete"
        or profile.get("outputs")
        != {"json": "speed_profile.json", "plot": "speed_profile.png"}
    ):
        raise PublicationError(f"{label} speed-profile artifact identity is invalid")
    inputs = profile.get("inputs")
    if (
        not isinstance(inputs, Mapping)
        or inputs.get("profile_id") != SPEED_30KPH_PROFILE_ID
        or inputs.get("longitudinal_speed_source")
        != "explicit_simulation_nominal"
    ):
        raise PublicationError(f"{label} speed-profile input contract is invalid")
    _exact_finite_number(
        inputs.get("target_speed_mps"),
        8.333333333333334,
        f"{label} speed-profile target_speed_mps",
    )
    interpretation = profile.get("interpretation")
    if (
        not isinstance(interpretation, Mapping)
        or interpretation.get("cruise_velocity_source")
        != "explicit CARLA simulation profile"
        or interpretation.get("planning_geometry")
        != "VAD route-manager hybrid"
        or interpretation.get("raw_vad_velocity_is_cruise_target") is not False
        or interpretation.get("real_vehicle_ready") is not False
    ):
        raise PublicationError(
            f"{label} speed profile does not prove VAD geometry plus the explicit "
            "CARLA simulation overlay"
        )
    quality = profile.get("quality")
    required_series = {
        "raw_selected_vad",
        "explicit_overlaid_planning",
        "gated_control_command",
        "actual_odometry",
    }
    series = profile.get("series")
    declared_series = quality.get("required_series") if isinstance(quality, Mapping) else None
    if (
        not isinstance(quality, Mapping)
        or quality.get("problems") != []
        or not isinstance(declared_series, list)
        or not all(isinstance(name, str) for name in declared_series)
        or set(declared_series) != required_series
        or not isinstance(series, Mapping)
        or any(
            not isinstance(series.get(name), list) or not series[name]
            for name in required_series
        )
    ):
        raise PublicationError(
            f"{label} speed profile lacks complete four-source speed evidence"
        )
    profile_context = result.get("profile_context")
    exposure = result.get("speed_exposure")
    expected_context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_geometry_evaluated": True,
        "vad_velocity_evaluated": False,
    }
    if (
        profile_context != expected_context
        or not isinstance(exposure, Mapping)
        or exposure.get("status") != "PASS"
        or any(exposure.get(field) != expected for field, expected in expected_context.items())
    ):
        raise PublicationError(
            f"{label} result does not retain passing speed-screening context"
        )
    identity = profile.get("source_identity")
    if not isinstance(identity, Mapping) or identity.get("schema_version") != 1:
        raise PublicationError(f"{label} speed profile has no source identity")
    identity_sha256 = identity.get("sha256")
    if (
        not isinstance(identity_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", identity_sha256)
        or identity_sha256
        != _sha256_json(
            {key: value for key, value in identity.items() if key != "sha256"}
        )
    ):
        raise PublicationError(f"{label} speed-profile source identity changed")
    route_identity = identity.get("effective_route")
    result_identity = identity.get("route_result")
    bag_identity = identity.get("rosbag")
    if (
        not isinstance(route_identity, Mapping)
        or not isinstance(result_identity, Mapping)
        or not isinstance(bag_identity, Mapping)
    ):
        raise PublicationError(f"{label} speed-profile source identity is incomplete")
    result_path = (directory / "result.json").resolve()
    route_value = result.get("route_file")
    if not isinstance(route_value, str) or not route_value:
        raise PublicationError(f"{label} result has no effective route")
    route_path = Path(route_value).expanduser()
    if not route_path.is_absolute():
        route_path = directory / route_path
    route_path = _inside(directory, route_path, f"{label} effective route")
    recorded_route = Path(str(route_identity.get("path", ""))).expanduser()
    recorded_result = Path(str(result_identity.get("path", ""))).expanduser()
    scenario = "straight" if route_identity.get("trial_id") == "straight" else (
        route_identity.get("scenario")
    )
    if (
        not route_path.is_file()
        or recorded_route.resolve() != route_path
        or recorded_result.resolve() != result_path
        or route_identity.get("sha256") != _sha256(route_path)
        or result_identity.get("sha256") != _sha256(result_path)
        or route_identity.get("trial_id") not in {"straight", "turn"}
        or route_identity.get("scenario") not in {"straight", "left", "right"}
        or scenario not in {"straight", "left", "right"}
        or result_identity.get("success") is not True
        or result_identity.get("execution_mode") != "full_stack"
        or result_identity.get("profile_context") != expected_context
    ):
        raise PublicationError(
            f"{label} speed profile is bound to a different route/result"
        )
    bag = _bag_manifest(directory / "bag", f"{label} speed profile")
    if bag_identity != bag:
        raise PublicationError(
            f"{label} speed-profile rosbag manifest changed after analysis"
        )
    if Path(str(inputs.get("bag", ""))).expanduser().resolve() != (
        directory / "bag"
    ).resolve():
        raise PublicationError(f"{label} speed-profile input bag path changed")

    if not isinstance(matrix_speed_evidence, Mapping):
        raise PublicationError(
            f"{label} speed profile has no immutable matrix-validation binding"
        )
    plot_sha256 = _sha256(plot_path)
    immutable_bindings = {
        "speed_profile_json_sha256": profile_sha256,
        "speed_profile_plot_sha256": plot_sha256,
        "speed_profile_source_identity_sha256": identity_sha256,
        "speed_profile_result_sha256": result_identity.get("sha256"),
        "speed_profile_route_sha256": route_identity.get("sha256"),
        "speed_profile_bag_manifest_sha256": bag.get("sha256"),
    }
    for field, expected in immutable_bindings.items():
        if matrix_speed_evidence.get(field) != expected:
            raise PublicationError(
                f"{label} matrix validation does not bind {field}"
            )
    plot_dimensions = _image_dimensions(
        plot_path, f"{label} speed-profile plot", "PNG"
    )
    return {
        "analysis": "carla_speed_source_evidence",
        "profile_id": SPEED_30KPH_PROFILE_ID,
        "classification": "simulation screening",
        "longitudinal_speed_source": "explicit CARLA simulation speed overlay",
        "geometry_source": "VAD candidate geometry",
        "real_vehicle_ready": False,
        "json_source": str(json_path),
        "json_sha256": profile_sha256,
        "plot_source": str(plot_path),
        "plot_sha256": plot_sha256,
        "plot_dimensions": list(plot_dimensions),
        "source_identity": dict(identity),
        "matrix_validation_bindings": immutable_bindings,
    }


def _validated_speed_centered_capture(
    directory: Path,
    capture: Mapping[str, Any],
    speed_contract: Mapping[str, Any] | None,
    matrix_speed_evidence: Mapping[str, Any] | None,
    matrix_visual_evidence: Mapping[str, Any] | None,
    trial_id: str | None,
    scenario: str | None,
    label: str,
) -> dict[str, Any]:
    """Revalidate and pin the matrix trial's own vehicle-centered capture."""
    capture_path = directory / "desktop_capture.json"
    on_disk_capture = _read_object(capture_path, f"{label} desktop capture")
    if on_disk_capture != capture:
        raise PublicationError(
            f"{label} desktop capture changed during centered-capture validation"
        )
    if (
        capture.get("source_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or capture.get("png_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or capture.get("candidate_png_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or capture.get("gif_dimensions")
        != list(SPEED_30KPH_DRIVE_GIF_DIMENSIONS)
    ):
        raise PublicationError(
            f"{label} speed_30kph centered capture must be a 1920x1080 owned-window "
            "with a 960x540 drive GIF"
        )
    if not isinstance(matrix_visual_evidence, Mapping):
        raise PublicationError(
            f"{label} speed_30kph centered capture lacks its matrix SHA binding"
        )
    rviz_contract, runtime_profile = _validate_centered_capture(
        directory,
        capture,
        label,
        speed_contract=speed_contract,
        matrix_speed_evidence=matrix_speed_evidence,
        trial_id=trial_id,
        scenario=scenario,
    )
    source_files: dict[str, dict[str, str]] = {}
    for name in tuple(
        dict.fromkeys(
            SPEED_30KPH_VISUAL_EVIDENCE_NAMES
            + SPEED_30KPH_CENTERED_CAPTURE_NAMES
        )
    ):
        source = directory / name
        source_files[name] = {
            "source": str(source),
            "sha256": _sha256(source),
        }
    recording = directory / "autoware_rviz_capture.mkv"
    return {
        "status": "PASS",
        "scope": (
            "matrix trial's own vehicle-centered owned-RViz-window capture; "
            "base_link fixed at RViz X/Y 0"
        ),
        "rviz_view_contract": rviz_contract,
        "runtime_profile": runtime_profile,
        "matrix_visual_evidence": dict(matrix_visual_evidence),
        "source_files": source_files,
        "recording_source": str(recording),
        "recording_sha256": _sha256(recording),
        "recording_size_bytes": recording.stat().st_size,
    }


def collect_vad_trials(
    snapshot: Mapping[str, Any],
    trial_paths: Mapping[str, Path] | Sequence[VadTrialSpec],
) -> list[VadTrial]:
    artifact_root = Path(snapshot["artifact_root"])
    known_maps = {
        entry["map_id"]: str(entry.get("canonical_name", entry["map_id"]))
        for entry in snapshot["maps"]
    }
    trials: list[VadTrial] = []
    seen: set[tuple[str, str | None]] = set()
    specs = sorted(
        _normalize_vad_trial_specs(trial_paths),
        key=lambda value: (value.map_id, value.trial_id or ""),
    )
    for spec in specs:
        map_id = spec.map_id
        trial_id = spec.trial_id
        identity = (map_id, trial_id)
        label = f"{map_id}:{trial_id}" if trial_id is not None else map_id
        if identity in seen:
            raise PublicationError(f"duplicate Autoware VAD trial id: {label}")
        seen.add(identity)
        if not MAP_ID_PATTERN.fullmatch(map_id) or (
            trial_id is not None and not TRIAL_ID_PATTERN.fullmatch(trial_id)
        ):
            raise PublicationError(f"Autoware VAD trial has an unsafe id: {label}")
        if map_id not in known_maps:
            raise PublicationError(f"Autoware VAD trial uses unknown map id: {map_id}")
        if spec.runtime_profile_selector not in VAD_RUNTIME_PROFILE_SELECTORS:
            raise PublicationError(
                f"{label} has an unknown runtime-profile selector"
            )
        evidence_root = spec.evidence_root or artifact_root
        directory = _inside(evidence_root, spec.directory, "Autoware VAD trial")
        result = _read_object(directory / "result.json", "Autoware VAD result")
        assessment = result.get("assessment")
        final = result.get("final")
        if (
            result.get("success") is not True
            or result.get("execution_mode") != "full_stack"
            or not isinstance(assessment, Mapping)
            or assessment.get("planning_architecture") != "vad_route_manager_hybrid"
            or assessment.get("route_completion") != "PASS"
            or not isinstance(final, Mapping)
            or final.get("goal_reached") is not True
            or final.get("route_status") != "goal_reached"
        ):
            raise PublicationError(
                f"{label} trial is not a successful full-stack Autoware VAD result"
            )
        for name in ("route_result.png", "turn_path_control.gif"):
            if not (directory / name).is_file():
                raise PublicationError(f"{label} VAD trial is missing {name}")
        route = None
        desktop_capture = None
        if trial_id is not None:
            route = _labeled_trial_route(
                directory, result, known_maps[map_id], label
            )
            desktop_capture = _labeled_desktop_capture(directory, label)
        speed_profile = None
        centered_capture_provenance = None
        if _is_speed_30kph_profile(spec.runtime_profile_selector):
            speed_profile = _validated_speed_profile(
                directory, result, spec.matrix_speed_evidence, label
            )
            if desktop_capture is None:  # pragma: no cover - labeled matrix trials only.
                raise PublicationError(
                    f"{label} speed_30kph trial has no centered desktop capture"
                )
            centered_capture_provenance = _validated_speed_centered_capture(
                directory,
                desktop_capture,
                spec.matrix_speed_contract,
                spec.matrix_speed_evidence,
                spec.matrix_visual_evidence,
                trial_id,
                str(route["scenario"]) if route is not None else None,
                label,
            )
        trials.append(
            VadTrial(
                map_id=map_id,
                trial_id=trial_id,
                directory=directory,
                result=result,
                route=route,
                desktop_capture=desktop_capture,
                speed_profile=speed_profile,
                centered_capture_provenance=centered_capture_provenance,
                source=spec.source,
                runtime_profile_selector=spec.runtime_profile_selector,
            )
        )
    return trials


def _read_runtime_environment(path: Path, label: str) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise PublicationError(f"cannot read {label} {path}: {error}") from error
    environment: dict[str, str] = {}
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if (
            not separator
            or not re.fullmatch(r"[A-Z][A-Z0-9_]*", key)
            or key in environment
        ):
            raise PublicationError(
                f"{label} has an invalid or duplicate entry on line {line_number}"
            )
        environment[key] = value
    return environment


def _fresh_camera_source_5hz_evidence(
    directory: Path,
    runtime_profile: Mapping[str, Any],
    label: str,
) -> dict[str, Any]:
    """Recompute the candidate cadence and loss gates from immutable raw files."""
    runtime = _read_runtime_environment(
        directory / "runtime.env", f"{label} camera-source runtime"
    )
    latency = _read_object(
        directory / "latency/e2e_latency.json",
        f"{label} camera-source latency",
    )
    try:
        evidence = _matrix_camera_source_5hz_evidence(
            directory, runtime, runtime_profile, latency
        )
    except MatrixValidationError as error:
        raise PublicationError(
            f"{label} camera-source raw evidence failed fresh validation: {error}"
        ) from error
    if not isinstance(evidence, Mapping) or evidence.get("status") != "PASS":
        raise PublicationError(
            f"{label} camera-source raw evidence did not produce a PASS record"
        )
    return dict(evidence)


def _validated_camera_source_5hz_evidence(
    directory: Path,
    runtime_profile: Mapping[str, Any],
    recorded_evidence: Any,
    label: str,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    camera_contract = runtime_profile.get("camera_source_contract")
    provenance = _validated_camera_source_5hz_provenance_files(
        directory,
        recorded_evidence,
        label,
        (
            camera_contract if isinstance(camera_contract, Mapping) else None
        ),
    )
    fresh = _fresh_camera_source_5hz_evidence(
        directory, runtime_profile, label
    )
    if (
        not isinstance(recorded_evidence, Mapping)
        or dict(recorded_evidence) != fresh
    ):
        raise PublicationError(
            f"{label} camera-source matrix validation does not match freshly "
            "recomputed cadence/integrity evidence"
        )
    return fresh, provenance


def _finite_number(value: Any, label: str, *, positive: bool = False) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or (positive and float(value) <= 0.0)
    ):
        qualifier = "positive " if positive else "finite "
        raise PublicationError(f"{label} must be a {qualifier}number")
    return float(value)


def _runtime_exact_number(
    runtime: Mapping[str, str], field: str, expected: Any, label: str
) -> None:
    try:
        actual = float(runtime[field])
    except (KeyError, TypeError, ValueError) as error:
        raise PublicationError(f"{label} runtime has no finite {field}") from error
    expected_number = _finite_number(expected, f"{label} expected {field}")
    if not math.isfinite(actual) or not math.isclose(
        actual, expected_number, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise PublicationError(
            f"{label} runtime {field} mismatch: expected={expected_number!r} "
            f"actual={runtime.get(field)!r}"
        )


def _read_sha256_manifest(path: Path, label: str) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise PublicationError(f"cannot read {label} {path}: {error}") from error
    values: dict[str, str] = {}
    for line in lines:
        digest, separator, name = line.partition("  ")
        if (
            not separator
            or not re.fullmatch(r"[0-9a-f]{64}", digest)
            or not name
            or name in values
        ):
            raise PublicationError(f"{label} has a malformed line: {line!r}")
        values[name] = digest
    return values


def _validated_camera_source_5hz_provenance_files(
    directory: Path,
    recorded_evidence: Any,
    label: str,
    camera_contract: Mapping[str, Any] | None = None,
) -> dict[str, dict[str, Any]]:
    if not isinstance(recorded_evidence, Mapping):
        raise PublicationError(
            f"{label} camera-source matrix validation has no evidence object"
        )
    effective_contract = (
        CAMERA_SOURCE_5HZ_CONTRACT
        if camera_contract is None
        else camera_contract
    )
    provenance_names = _camera_source_5hz_provenance_names(effective_contract)
    paths: dict[str, Path] = {}
    for name in (
        *provenance_names,
        "runtime.env",
        "latency/e2e_latency.json",
    ):
        source = directory / name
        path = _inside(
            directory, source, f"{label} camera-source raw evidence"
        )
        if source.is_symlink() or not path.is_file():
            raise PublicationError(
                f"{label} camera-source raw evidence is missing or symlinked: {name}"
            )
        paths[name] = path
    mapping_name = "sensor_mapping_provenance/sensor_mapping.yaml"
    mapping_manifest_name = "sensor_mapping_provenance/SHA256SUMS"
    mapping_sha256 = _sha256(paths[mapping_name])
    if (
        recorded_evidence.get("mapping_sha256") != mapping_sha256
        or _read_sha256_manifest(
            paths[mapping_manifest_name],
            f"{label} camera-source sensor-mapping SHA256SUMS",
        )
        != {"sensor_mapping.yaml": mapping_sha256}
    ):
        raise PublicationError(
            f"{label} camera-source sensor-mapping provenance changed"
        )
    if effective_contract == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT:
        model_name = "vad_model_override_provenance/model_override.param.yaml"
        model_manifest_name = "vad_model_override_provenance/SHA256SUMS"
        model_sha256 = _sha256(paths[model_name])
        transport = recorded_evidence.get("transport_provenance")
        raw_integrity = recorded_evidence.get("raw_six_image_queue_integrity")
        expected_transport = {
            "profile_id": effective_contract["profile_id"],
            "sensor_mapping_sha256": mapping_sha256,
            "vad_model_override_sha256": model_sha256,
            "camera_image_publish_qos": effective_contract[
                "camera_image_publish_qos"
            ],
            "camera_info_publish_qos": effective_contract[
                "camera_info_publish_qos"
            ],
            "vad_image_subscription_qos": effective_contract[
                "vad_image_subscription_qos"
            ],
            "rviz_image_subscription_qos": effective_contract[
                "rviz_image_subscription_qos"
            ],
        }
        if (
            recorded_evidence.get("contract") != effective_contract
            or transport != expected_transport
            or not isinstance(raw_integrity, Mapping)
            or raw_integrity.get("status") != "PASS"
            or _read_sha256_manifest(
                paths[model_manifest_name],
                f"{label} camera-source VAD-model SHA256SUMS",
            )
            != {"model_override.param.yaml": model_sha256}
        ):
            raise PublicationError(
                f"{label} best-effort camera-source transport/model provenance changed"
            )
    return {
        name: {
            "source": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        for name, path in paths.items()
    }


def _full_stack_result_pass(result: Mapping[str, Any]) -> bool:
    assessment = result.get("assessment")
    final = result.get("final")
    return bool(
        result.get("success") is True
        and result.get("execution_mode") == "full_stack"
        and isinstance(assessment, Mapping)
        and assessment.get("planning_architecture") == "vad_route_manager_hybrid"
        and assessment.get("route_completion") == "PASS"
        and isinstance(final, Mapping)
        and final.get("goal_reached") is True
        and final.get("route_status") == "goal_reached"
    )


def _camera_queue_snapshot(directory: Path, label: str) -> dict[str, Any]:
    try:
        stack_lines = (directory / "stack.log").read_text(
            encoding="utf-8", errors="replace"
        ).splitlines()
        recorder_lines = (directory / "recorder.log").read_text(
            encoding="utf-8", errors="replace"
        ).splitlines()
    except OSError as error:
        raise PublicationError(f"cannot read {label} queue diagnostics: {error}") from error
    queue_pattern = re.compile(
        r"\[INFO\s+([0-9]+(?:\.[0-9]+)?)\].*?VAD frame queued: "
        r"source_stamp_ns=(\d+).*?assembled=(\d+).*?"
        r"capacity_pruned=(\d+).*?superseded=(\d+).*?"
        r"mailbox_submitted=(\d+).*?coalesced_drops=(\d+).*?"
        r"received_images_min=(\d+).*?received_images_max=(\d+)"
    )
    inference_pattern = re.compile(
        r"published_count=(\d+).*?mailbox_taken=(\d+).*?coalesced_drops=(\d+)"
    )

    def queue_record(line_number: int, values: Sequence[str]) -> dict[str, Any]:
        return {
            "line_number": line_number,
            "wall_sec": values[0],
            "source_stamp_ns": int(values[1]),
            "assembled": int(values[2]),
            "capacity_pruned": int(values[3]),
            "superseded": int(values[4]),
            "mailbox_submitted": int(values[5]),
            "coalesced_drops": int(values[6]),
            "received_images_min": int(values[7]),
            "received_images_max": int(values[8]),
        }

    queue_records: list[dict[str, Any]] = []
    inference_records: list[dict[str, Any]] = []
    for line_number, line in enumerate(stack_lines, start=1):
        queue_match = queue_pattern.search(line)
        if queue_match is not None:
            queue_records.append(queue_record(line_number, queue_match.groups()))
        inference_match = inference_pattern.search(line)
        if inference_match is not None:
            published, mailbox_taken, coalesced = (
                int(value) for value in inference_match.groups()
            )
            inference_records.append(
                {
                    "line_number": line_number,
                    "published_count": published,
                    "mailbox_taken": mailbox_taken,
                    "coalesced_drops": coalesced,
                }
            )
    recorder_marker = None
    recorder_pattern = re.compile(
        r"\[INFO\s+([0-9]+(?:\.[0-9]+)?)\].*?Recording\.\.\."
    )
    for line_number, line in enumerate(recorder_lines, start=1):
        match = recorder_pattern.search(line)
        if match is not None:
            recorder_marker = {
                "line_number": line_number,
                "wall_sec": match.group(1),
            }
            break
    if not queue_records or not inference_records or recorder_marker is None:
        raise PublicationError(f"{label} lacks timestamped queue/recorder counters")
    initial_superseded = queue_records[0]["superseded"]
    transition = next(
        (
            record
            for record in queue_records[1:]
            if record["superseded"] != initial_superseded
        ),
        None,
    )
    transition_after_recorder = bool(
        transition is not None
        and float(transition["wall_sec"]) >= float(recorder_marker["wall_sec"])
    )
    return {
        "queue_sample_count": len(queue_records),
        "first": queue_records[0],
        "first_superseded_transition": transition,
        "last": queue_records[-1],
        "final_inference": inference_records[-1],
        "recorder_start": recorder_marker,
        "transition_after_recorder": transition_after_recorder,
    }


def _source_record(
    root: Path, source: Path, published_file: str, label: str
) -> dict[str, Any]:
    resolved = _inside(root, source, label)
    if source.is_symlink() or not resolved.is_file():
        raise PublicationError(f"{label} is missing or symlinked: {source}")
    return {
        "source": str(resolved),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
        "published_file": published_file,
    }


def _telemetry_window_diagnostics(
    matrix_root: Path, trials: Sequence[VadTrial]
) -> dict[str, Any] | None:
    telemetry_root = matrix_root / "host_telemetry"
    if not telemetry_root.exists():
        return None
    sources = {
        "vmstat": _inside(
            matrix_root, telemetry_root / "vmstat.log", "campaign vmstat telemetry"
        ),
        "nvidia_smi_dmon": _inside(
            matrix_root,
            telemetry_root / "nvidia_smi_dmon.log",
            "campaign NVIDIA telemetry",
        ),
        "pidstat": _inside(
            matrix_root, telemetry_root / "pidstat.log", "campaign pidstat telemetry"
        ),
    }
    if any(path.is_symlink() or not path.is_file() for path in sources.values()):
        raise PublicationError("camera-source campaign host telemetry is incomplete")
    ranked: list[tuple[float, VadTrial]] = []
    for trial in trials:
        metrics = trial.result.get("metrics")
        if not isinstance(metrics, Mapping):
            continue
        sim_elapsed = _finite_number(
            metrics.get("sim_elapsed_sec"), "diagnostic simulation elapsed", positive=True
        )
        wall_elapsed = _finite_number(
            metrics.get("wall_elapsed_sec"), "diagnostic wall elapsed", positive=True
        )
        ranked.append((sim_elapsed / wall_elapsed, trial))
    if not ranked:
        raise PublicationError("camera-source campaign has no measurable trial RTF")
    rtf, trial = min(ranked, key=lambda item: item[0])
    if trial.trial_id is None:  # pragma: no cover - matrix trials are labeled.
        raise PublicationError("lowest-RTF diagnostic trial has no trial id")
    kst = ZoneInfo("Asia/Seoul")
    started = _parse_timestamp(
        trial.result.get("started_at"), "lowest-RTF trial started_at"
    ).astimezone(kst)
    finished = _parse_timestamp(
        trial.result.get("finished_at"), "lowest-RTF trial finished_at"
    ).astimezone(kst)
    window_start = started.replace(microsecond=0)
    window_end = finished.replace(microsecond=0)
    if window_end < window_start:
        raise PublicationError("lowest-RTF telemetry window is reversed")

    vmstat_lines = sources["vmstat"].read_text(
        encoding="utf-8", errors="replace"
    ).splitlines()
    vmstat_rows: list[str] = []
    vmstat_timestamps: list[datetime] = []
    idle_values: list[int] = []
    iowait_values: list[int] = []
    for line in vmstat_lines:
        parts = line.split()
        if len(parts) < 19 or re.fullmatch(r"\d{4}-\d{2}-\d{2}", parts[-2]) is None:
            continue
        try:
            timestamp = datetime.fromisoformat(
                f"{parts[-2]}T{parts[-1]}"
            ).replace(tzinfo=kst)
            idle = int(parts[-5])
            iowait = int(parts[-4])
        except (ValueError, IndexError) as error:
            raise PublicationError(f"invalid vmstat telemetry row: {line!r}") from error
        if window_start <= timestamp <= window_end:
            vmstat_rows.append(line)
            vmstat_timestamps.append(timestamp)
            idle_values.append(idle)
            iowait_values.append(iowait)

    dmon_lines = sources["nvidia_smi_dmon"].read_text(
        encoding="utf-8", errors="replace"
    ).splitlines()
    dmon_rows: list[str] = []
    dmon_timestamps: list[datetime] = []
    gpu_sm_values: list[int] = []
    for line in dmon_lines:
        parts = line.split()
        if len(parts) < 8 or re.fullmatch(r"\d{8}", parts[0]) is None:
            continue
        try:
            timestamp = datetime.strptime(
                f"{parts[0]} {parts[1]}", "%Y%m%d %H:%M:%S"
            ).replace(tzinfo=kst)
            sm_percent = int(parts[6])
        except (ValueError, IndexError) as error:
            raise PublicationError(f"invalid NVIDIA telemetry row: {line!r}") from error
        if window_start <= timestamp <= window_end:
            dmon_rows.append(line)
            dmon_timestamps.append(timestamp)
            gpu_sm_values.append(sm_percent)
    if not vmstat_rows or not dmon_rows:
        raise PublicationError("lowest-RTF telemetry window has no CPU/GPU samples")

    pidstat_lines = sources["pidstat"].read_text(
        encoding="utf-8", errors="replace"
    ).splitlines()
    host_header = next((line for line in pidstat_lines if line.strip()), None)
    cpu_match = re.search(r"\((\d+) CPU\)", host_header or "")
    if host_header is None or cpu_match is None:
        raise PublicationError("pidstat telemetry has no host CPU-count header")

    identity = f"{trial.map_id}_{trial.trial_id}"
    generated_text = {
        f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/{identity}_vmstat_window.log": (
            "\n".join(vmstat_lines[:2] + vmstat_rows) + "\n"
        ),
        f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/{identity}_nvidia_smi_dmon_window.log": (
            "\n".join(dmon_lines[:2] + dmon_rows) + "\n"
        ),
        f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/{identity}_pidstat_host_header.txt": (
            host_header + "\n"
        ),
    }
    window_files = {
        name: {
            "published_file": name,
            "sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
            "size_bytes": len(content.encode("utf-8")),
        }
        for name, content in generated_text.items()
    }
    envelope_seconds = int((window_end - window_start).total_seconds()) + 1

    def timestamp_coverage(values: Sequence[datetime]) -> dict[str, Any]:
        unique = sorted(set(values))
        maximum_gap = max(
            (
                int((current - previous).total_seconds())
                for previous, current in zip(unique, unique[1:])
            ),
            default=0,
        )
        return {
            "raw_row_count": len(values),
            "unique_timestamp_count": len(unique),
            "duplicate_row_count": len(values) - len(unique),
            "missing_timestamp_count": envelope_seconds - len(unique),
            "unique_timestamp_coverage_percent": (
                100.0 * len(unique) / envelope_seconds
            ),
            "maximum_timestamp_gap_sec": maximum_gap,
        }
    result_path = trial.directory / "result.json"
    telemetry_source_records: dict[str, dict[str, Any]] = {}
    for name, path in sources.items():
        source_record = {
            "source": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        if name != "pidstat":
            source_record["published_file"] = (
                f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/host_telemetry/{path.name}"
            )
        telemetry_source_records[name] = source_record
    return {
        "identity": {"map_id": trial.map_id, "trial_id": trial.trial_id},
        "result": {
            "source": str(result_path.resolve()),
            "sha256": _sha256(result_path),
            "published_file": (
                f"{trial.map_id}/autoware_vad/{trial.trial_id}/"
                "autoware_vad_result.json"
            ),
            "started_at": trial.result["started_at"],
            "finished_at": trial.result["finished_at"],
        },
        "window": {
            "timezone": "Asia/Seoul",
            "selection": "whole-second envelope, both endpoints inclusive",
            "start": window_start.isoformat(),
            "end": window_end.isoformat(),
            "envelope_seconds": envelope_seconds,
            "row_weighting": "every raw row; duplicate timestamps retained",
        },
        "host_cpu_count": int(cpu_match.group(1)),
        "real_time_factor": rtf,
        "calculation": {
            "real_time_factor": "metrics.sim_elapsed_sec / metrics.wall_elapsed_sec",
            "cpu_idle_average_percent": "sum(vmstat.id) / vmstat_sample_count",
            "gpu_sm_average_percent": "sum(nvidia_smi_dmon.sm) / gpu_sample_count",
        },
        "measurements": {
            "vmstat_sample_count": len(idle_values),
            "vmstat_timestamp_coverage": timestamp_coverage(vmstat_timestamps),
            "cpu_idle_sum": sum(idle_values),
            "cpu_idle_average_percent": sum(idle_values) / len(idle_values),
            "cpu_idle_minimum_percent": min(idle_values),
            "iowait_average_percent": sum(iowait_values) / len(iowait_values),
            "iowait_maximum_percent": max(iowait_values),
            "gpu_sample_count": len(gpu_sm_values),
            "gpu_timestamp_coverage": timestamp_coverage(dmon_timestamps),
            "gpu_sm_sum": sum(gpu_sm_values),
            "gpu_sm_average_percent": sum(gpu_sm_values) / len(gpu_sm_values),
            "gpu_sm_maximum_percent": max(gpu_sm_values),
        },
        "source_files": telemetry_source_records,
        "pidstat_archival_policy": (
            "188 MB process log is SHA-bound but not copied; only the host header "
            "needed for the 24-CPU claim is published"
        ),
        "window_files": window_files,
        "_generated_text": generated_text,
    }


def collect_camera_source_campaign_diagnostics(
    matrix_root_value: Path | None,
    runtime_profile_selector: str,
    runtime_profile: Mapping[str, Any] | None,
    trials: Sequence[VadTrial],
) -> dict[str, Any] | None:
    if not _is_camera_source_5hz_profile(runtime_profile_selector):
        return None
    if matrix_root_value is None or not isinstance(runtime_profile, Mapping):
        raise PublicationError("camera-source diagnostics require the matrix profile")
    matrix_root = matrix_root_value.expanduser().resolve()
    selected = {
        (trial.map_id, str(trial.trial_id)): trial
        for trial in trials
        if trial.trial_id is not None
    }
    retry_records: list[dict[str, Any]] = []
    for identity, selected_trial in sorted(selected.items()):
        attempt_match = re.fullmatch(r"attempt_(\d+)", selected_trial.directory.name)
        if attempt_match is None:
            continue
        selected_number = int(attempt_match.group(1))
        for candidate in sorted(selected_trial.directory.parent.glob("attempt_*")):
            candidate_match = re.fullmatch(r"attempt_(\d+)", candidate.name)
            if (
                candidate_match is None
                or not candidate.is_dir()
                or int(candidate_match.group(1)) >= selected_number
            ):
                continue
            candidate = _inside(
                matrix_root, candidate, "camera-source rejected retry attempt"
            )
            camera_contract = _camera_source_5hz_contract(runtime_profile_selector)
            retry_raw_names = CAMERA_SOURCE_RETRY_RAW_NAMES
            if camera_contract == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT:
                retry_raw_names = (
                    *retry_raw_names,
                    "vad_model_override_provenance/model_override.param.yaml",
                    "vad_model_override_provenance/SHA256SUMS",
                )
            required = [candidate / name for name in retry_raw_names]
            if any(not path.is_file() or path.is_symlink() for path in required):
                continue
            result = _read_object(candidate / "result.json", "retry result")
            if not _full_stack_result_pass(result):
                continue
            runtime = _read_runtime_environment(
                candidate / "runtime.env", "retry camera-source runtime"
            )
            latency = _read_object(
                candidate / "latency/e2e_latency.json", "retry camera-source latency"
            )
            try:
                _matrix_camera_source_5hz_evidence(
                    candidate, runtime, runtime_profile, latency
                )
            except MatrixValidationError as error:
                gate_error = str(error)
            else:
                continue
            queue = _camera_queue_snapshot(candidate, f"{identity} {candidate.name}")
            transition = queue["first_superseded_transition"]
            if (
                queue["first"]["superseded"] != 0
                or not isinstance(transition, Mapping)
                or transition.get("superseded") != 1
                or queue["last"]["superseded"] != 1
                or queue["transition_after_recorder"] is not True
            ):
                continue
            selected_queue = _camera_queue_snapshot(
                selected_trial.directory, f"{identity} selected"
            )
            selected_evidence = _fresh_camera_source_5hz_evidence(
                selected_trial.directory, runtime_profile, f"{identity} selected"
            )
            equality: dict[str, Any] = {}
            for name in (
                "source_route.json",
                "aligned_route.json",
                "launch_args.txt",
                "sensor_mapping_provenance/sensor_mapping.yaml",
                *(
                    ("vad_model_override_provenance/model_override.param.yaml",)
                    if camera_contract
                    == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT
                    else ()
                ),
                "rviz_capture_provenance/autoware_vad_carla.rviz",
            ):
                rejected_digest = _sha256(candidate / name)
                selected_digest = _sha256(selected_trial.directory / name)
                equality[name] = {
                    "equal": rejected_digest == selected_digest,
                    "sha256": rejected_digest,
                }
            dynamic_runtime_fields = {
                "CARLA_GENERATION_ID",
                "CARLA_OWNER_PID",
                "CARLA_OWNER_PGID",
                "CARLA_SERVER_LOG",
                "EFFECTIVE_ROUTE_FILE",
                "RVIZ_CAPTURE_CONFIG",
                "RVIZ_CAPTURE_WINDOW_PID",
                "RVIZ_CAPTURE_WINDOW_PGID",
                "RVIZ_CAPTURE_STACK_PGID",
            }
            selected_runtime = _read_runtime_environment(
                selected_trial.directory / "runtime.env",
                "selected retry-comparison runtime",
            )
            rejected_projection = {
                key: value
                for key, value in runtime.items()
                if key not in dynamic_runtime_fields
            }
            selected_projection = {
                key: value
                for key, value in selected_runtime.items()
                if key not in dynamic_runtime_fields
            }
            equality["runtime_contract_projection"] = {
                "equal": rejected_projection == selected_projection,
                "excluded_run_identity_fields": sorted(dynamic_runtime_fields),
                "field_count": len(rejected_projection),
                "sha256": _sha256_json(rejected_projection),
                "canonicalization": PUBLICATION_SELECTION_CANONICALIZATION,
            }
            if any(record["equal"] is not True for record in equality.values()):
                raise PublicationError(
                    f"{identity} rejected retry does not share immutable inputs"
                )
            retry_directory = (
                f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/"
                f"{identity[0]}_{identity[1]}_{candidate.name}"
            )
            source_files = {
                name: _source_record(
                    matrix_root,
                    candidate / name,
                    f"{retry_directory}/{name}",
                    f"{identity} {candidate.name} {name}",
                )
                for name in retry_raw_names
            }
            retry_records.append(
                {
                    "identity": {"map_id": identity[0], "trial_id": identity[1]},
                    "attempt": candidate.name,
                    "selected_attempt": selected_trial.directory.name,
                    "full_stack_result_pass": True,
                    "camera_source_matrix_gate": "FAIL",
                    "camera_source_gate_error": gate_error,
                    "failure_classification": "superseded_transition_after_recorder",
                    "immutable_input_equality": equality,
                    "rejected_queue": queue,
                    "selected_queue": selected_queue,
                    "selected_camera_source_evidence": selected_evidence,
                    "selected_published_directory": (
                        f"{identity[0]}/autoware_vad/{identity[1]}"
                    ),
                    "source_files": source_files,
                }
            )
    host = _telemetry_window_diagnostics(matrix_root, trials)
    console_files: dict[str, dict[str, Any]] = {}
    for name in ("campaign_console.log", "campaign_console_retry_001.log"):
        source = matrix_root / name
        if source.is_file() and not source.is_symlink():
            console_files[name] = _source_record(
                matrix_root,
                source,
                f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/{name}",
                f"camera-source {name}",
            )
    return {
        "schema_version": 1,
        "status": "PASS",
        "runtime_profile_selector": runtime_profile_selector,
        "matrix_root": str(matrix_root),
        "retry_attempts": retry_records,
        "host_telemetry": host,
        "campaign_console_files": console_files,
    }


def _copy_camera_source_campaign_diagnostics(
    diagnostics: Mapping[str, Any], staging: Path
) -> dict[str, Any]:
    public = dict(diagnostics)
    public_retries: list[dict[str, Any]] = []
    published_files: list[str] = []
    for retry in diagnostics["retry_attempts"]:
        public_retry = dict(retry)
        public_sources: dict[str, dict[str, Any]] = {}
        for name, source_record in retry["source_files"].items():
            destination = staging / source_record["published_file"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_verified_source(
                Path(source_record["source"]),
                destination,
                source_record["sha256"],
                f"camera-source retry diagnostic {name}",
            )
            public_sources[name] = dict(source_record)
            published_files.append(source_record["published_file"])
        public_retry["source_files"] = public_sources
        public_retries.append(public_retry)
    public["retry_attempts"] = public_retries

    public_console_files: dict[str, dict[str, Any]] = {}
    for name, source_record in diagnostics.get("campaign_console_files", {}).items():
        destination = staging / source_record["published_file"]
        destination.parent.mkdir(parents=True, exist_ok=True)
        _copy_verified_source(
            Path(source_record["source"]),
            destination,
            source_record["sha256"],
            f"camera-source campaign console {name}",
        )
        public_console_files[name] = dict(source_record)
        published_files.append(source_record["published_file"])
    public["campaign_console_files"] = public_console_files

    host = diagnostics.get("host_telemetry")
    if isinstance(host, Mapping):
        public_host = dict(host)
        generated_text = host.get("_generated_text")
        if not isinstance(generated_text, Mapping):
            raise PublicationError("host telemetry diagnostics lost generated excerpts")
        public_host.pop("_generated_text", None)
        for name, source_record in host["source_files"].items():
            published_file = source_record.get("published_file")
            if published_file is None:
                continue
            destination = staging / published_file
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_verified_source(
                Path(source_record["source"]),
                destination,
                source_record["sha256"],
                f"camera-source host telemetry {name}",
            )
            published_files.append(published_file)
        for relative, content in generated_text.items():
            if not isinstance(relative, str) or not isinstance(content, str):
                raise PublicationError("host telemetry excerpt is malformed")
            destination = staging / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(content, encoding="utf-8")
            expected = host["window_files"][relative]
            if (
                _sha256(destination) != expected["sha256"]
                or destination.stat().st_size != expected["size_bytes"]
            ):
                raise PublicationError("host telemetry excerpt changed while publishing")
            published_files.append(relative)
        public["host_telemetry"] = public_host

    summary_relative = f"{CAMERA_SOURCE_DIAGNOSTICS_DIR}/campaign_diagnostics.json"
    summary_path = staging / summary_relative
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(public, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    published_files.append(summary_relative)
    return {
        "status": "PASS",
        "summary_file": summary_relative,
        "summary_sha256": _sha256(summary_path),
        "retry_attempt_count": len(public_retries),
        "host_telemetry_identity": (
            public["host_telemetry"]["identity"]
            if isinstance(public.get("host_telemetry"), Mapping)
            else None
        ),
        "files": sorted(published_files),
    }


def _validate_speed_runtime_binding(
    directory: Path,
    runtime: Mapping[str, str],
    speed_contract: Mapping[str, Any] | None,
    matrix_speed_evidence: Mapping[str, Any] | None,
    trial_id: str | None,
    scenario: str | None,
    label: str,
) -> dict[str, Any]:
    if (
        not isinstance(speed_contract, Mapping)
        or not isinstance(matrix_speed_evidence, Mapping)
        or trial_id not in {"straight", "turn"}
        or scenario not in {"straight", "left", "right"}
    ):
        raise PublicationError(f"{label} lacks its matrix speed-runtime contract")
    trials = speed_contract.get("trials")
    parameters = speed_contract.get("route_manager_parameters")
    gate = speed_contract.get("vehicle_cmd_gate")
    controller = speed_contract.get("longitudinal_controller")
    if (
        not isinstance(trials, Mapping)
        or not isinstance(trials.get(trial_id), Mapping)
        or not isinstance(parameters, Mapping)
        or not isinstance(gate, Mapping)
        or not isinstance(controller, Mapping)
    ):
        raise PublicationError(f"{label} matrix speed-runtime contract is incomplete")
    trial_contract = trials[trial_id]
    expected_strings = {
        "RECOMMENDED": "true",
        "VISUALIZE": "true",
        "CAPTURE_DESKTOP": "true",
        "SPEED_30KPH": "true",
        "TIGHT_CORRIDOR_CANDIDATE": "false",
        "TRAJECTORY_STABILITY_CANDIDATE": "false",
        "SMART_MPC": "false",
        "FP16_HEADS": "false",
        "SPEED_PROFILE_ID": str(speed_contract["profile_id"]),
        "ROUTE_SCENARIO": scenario,
        "SPEED_EXPOSURE_MODE": str(trial_contract["exposure_mode"]),
        "LONGITUDINAL_SPEED_SOURCE": str(
            speed_contract["longitudinal_speed_source"]
        ),
        "LONGITUDINAL_ACCELERATION_ROLE": str(
            speed_contract["longitudinal_acceleration_role"]
        ),
        "VAD_GEOMETRY_SOURCE": "true",
        "VAD_VELOCITY_EVALUATED": "false",
        "VAD_GEOMETRY_EVALUATED": "true",
        "VAD_CRUISE_VELOCITY_EVALUATED": "false",
        "VAD_HARD_STOP_SENTINEL_PRESERVED": "true",
        "VAD_IMU_ACCELERATION_ENABLED": "true",
        "CLOSED_LOOP_VALIDATION_STATE": str(speed_contract["validation_state"]),
        "SPEED_LIMIT_SOURCE": str(gate["speed_limit_source"]),
        "REAL_VEHICLE_READY": "false",
    }
    for field, expected in expected_strings.items():
        if runtime.get(field) != expected:
            raise PublicationError(
                f"{label} runtime {field} mismatch: expected={expected!r} "
                f"actual={runtime.get(field)!r}"
            )
    if runtime.get("VSCODE_SNAP_GUI_ENV_SANITIZED") not in {"true", "false"}:
        raise PublicationError(
            f"{label} runtime does not record VS Code GUI environment isolation"
        )
    expected_numbers = {
        "TARGET_SPEED_MPS": speed_contract["target_speed_mps"],
        "TARGET_SPEED_KPH": 30.0,
        "MINIMUM_SUSTAINED_SPEED_MPS": trial_contract[
            "minimum_sustained_speed_mps"
        ],
        "MINIMUM_SUSTAINED_SPEED_SEC": trial_contract[
            "minimum_sustained_speed_sec"
        ],
        "MAXIMUM_OBSERVED_SPEED_MPS": speed_contract[
            "maximum_observed_speed_mps"
        ],
        "MAXIMUM_SPEED_SAMPLE_GAP_SEC": speed_contract[
            "maximum_speed_sample_gap_sec"
        ],
        "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": trial_contract[
            "maximum_lateral_acceleration_mps2"
        ],
        "MAXIMUM_LONGITUDINAL_ACCELERATION_MPS2": parameters[
            "maximum_longitudinal_acceleration_mps2"
        ],
        "COMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2": gate[
            "longitudinal_acceleration_limit_mps2"
        ],
        "MAXIMUM_LATERAL_ACCELERATION_MPS2": parameters[
            "maximum_lateral_acceleration_mps2"
        ],
        "CONTROLLER_STOP_OFFSET_M": parameters["controller_stop_offset_m"],
        "MANEUVER_LOOKAHEAD_M": parameters["maneuver_lookahead_m"],
        "MANEUVER_EXIT_LOOKAHEAD_M": parameters["maneuver_exit_lookahead_m"],
        "ROUTE_CURVATURE_LOOKAHEAD_M": parameters[
            "route_curvature_lookahead_m"
        ],
        "CURVATURE_SPEED_PREVIEW_M": parameters["curvature_speed_preview_m"],
        "MAX_ROUTE_DEVIATION_M": parameters["max_route_deviation_m"],
        "MAX_CANDIDATE_AGE_SEC": parameters["max_candidate_age_sec"],
        "CANDIDATE_TIMEOUT_SEC": parameters["candidate_timeout_sec"],
        "COMFORTABLE_DECELERATION_MPS2": parameters[
            "comfortable_deceleration_mps2"
        ],
        "LONGITUDINAL_PID_MAX_OUT_MPS2": controller["maximum_output_mps2"],
        "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2": controller[
            "maximum_proportional_effort_mps2"
        ],
    }
    for field, expected in expected_numbers.items():
        _runtime_exact_number(runtime, field, expected, label)

    provenance = directory / "speed_profile_provenance"
    paths = {
        "vehicle_cmd_gate.param.yaml": provenance / "vehicle_cmd_gate.param.yaml",
        "vehicle_cmd_gate.param.yaml.metadata.json": provenance
        / "vehicle_cmd_gate.param.yaml.metadata.json",
        "longitudinal_controller.param.yaml": provenance
        / "longitudinal_controller.param.yaml",
        "longitudinal_controller.param.yaml.metadata.json": provenance
        / "longitudinal_controller.param.yaml.metadata.json",
    }
    actual_sha256 = {}
    for name, path in paths.items():
        if not path.is_file():
            raise PublicationError(f"{label} speed provenance is missing {name}")
        actual_sha256[name] = _sha256(path)
    expected_sha256 = {
        "vehicle_cmd_gate.param.yaml": gate["parameter_sha256"],
        "vehicle_cmd_gate.param.yaml.metadata.json": gate["metadata_sha256"],
        "longitudinal_controller.param.yaml": controller["parameter_sha256"],
        "longitudinal_controller.param.yaml.metadata.json": controller[
            "metadata_sha256"
        ],
    }
    if actual_sha256 != expected_sha256 or _read_sha256_manifest(
        provenance / "SHA256SUMS", f"{label} speed-provenance SHA256SUMS"
    ) != expected_sha256:
        raise PublicationError(f"{label} speed config provenance digest mismatch")
    runtime_sha_fields = {
        "VEHICLE_CMD_GATE_PARAM_SHA256": actual_sha256[
            "vehicle_cmd_gate.param.yaml"
        ],
        "VEHICLE_CMD_GATE_METADATA_SHA256": actual_sha256[
            "vehicle_cmd_gate.param.yaml.metadata.json"
        ],
        "LONGITUDINAL_CONTROLLER_PARAM_SHA256": actual_sha256[
            "longitudinal_controller.param.yaml"
        ],
        "LONGITUDINAL_CONTROLLER_METADATA_SHA256": actual_sha256[
            "longitudinal_controller.param.yaml.metadata.json"
        ],
    }
    for field, expected in runtime_sha_fields.items():
        if runtime.get(field) != expected:
            raise PublicationError(f"{label} runtime {field} digest mismatch")

    immutable_files = {
        "runtime_env_sha256": directory / "runtime.env",
        "route_manager_parameter_dump_sha256": directory
        / "vad_route_manager.params.yaml",
        "vehicle_cmd_gate_parameter_dump_sha256": directory
        / "vehicle_cmd_gate.params.yaml",
        "longitudinal_controller_parameter_dump_sha256": directory
        / "controller.params.yaml",
    }
    for field, path in immutable_files.items():
        if not path.is_file() or matrix_speed_evidence.get(field) != _sha256(path):
            raise PublicationError(
                f"{label} matrix validation does not bind current {field}"
            )
    matrix_provenance = {
        "gate_provenance_sha256": expected_sha256[
            "vehicle_cmd_gate.param.yaml"
        ],
        "gate_metadata_sha256": expected_sha256[
            "vehicle_cmd_gate.param.yaml.metadata.json"
        ],
        "longitudinal_controller_provenance_sha256": expected_sha256[
            "longitudinal_controller.param.yaml"
        ],
        "longitudinal_controller_metadata_sha256": expected_sha256[
            "longitudinal_controller.param.yaml.metadata.json"
        ],
    }
    for field, expected in matrix_provenance.items():
        if matrix_speed_evidence.get(field) != expected:
            raise PublicationError(
                f"{label} matrix validation does not bind current {field}"
            )
    return {
        "profile_id": speed_contract["profile_id"],
        "trial_id": trial_id,
        "scenario": scenario,
        "runtime_env_sha256": matrix_speed_evidence["runtime_env_sha256"],
        "config_sha256": actual_sha256,
        "vscode_snap_gui_env_sanitized": runtime[
            "VSCODE_SNAP_GUI_ENV_SANITIZED"
        ],
    }


def _validate_centered_capture(
    directory: Path,
    capture: Mapping[str, Any],
    label: str,
    *,
    speed_contract: Mapping[str, Any] | None = None,
    matrix_speed_evidence: Mapping[str, Any] | None = None,
    trial_id: str | None = None,
    scenario: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    candidate_name = capture.get("candidate_png_file")
    if candidate_name != "autoware_rviz_candidate.png":
        raise PublicationError(
            f"{label} centered capture candidate_png_file must be "
            "'autoware_rviz_candidate.png'"
        )
    candidate_dimensions = _dimension_pair(
        capture.get("candidate_png_dimensions"),
        f"{label} candidate_png_dimensions",
    )
    actual_candidate_dimensions = _image_dimensions(
        directory / candidate_name,
        f"{label} centered candidate still",
        "PNG",
    )
    if candidate_dimensions != actual_candidate_dimensions:
        raise PublicationError(
            f"{label} centered candidate still dimensions do not match metadata"
        )
    if candidate_dimensions != _dimension_pair(
        capture.get("source_dimensions"), f"{label} source_dimensions"
    ):
        raise PublicationError(
            f"{label} centered candidate still must cover the owned RViz window"
        )
    candidate_observed_at = _parse_timestamp(
        capture.get("candidate_observed_at"),
        f"{label} VAD candidate observation",
    )
    candidate_still_at = _parse_timestamp(
        capture.get("candidate_still_captured_at"),
        f"{label} centered candidate still",
    )

    contract = capture.get("rviz_view_contract")
    if not isinstance(contract, Mapping):
        raise PublicationError(f"{label} has no RViz view contract")
    center = contract.get("center_xy_m")
    if (
        contract.get("vehicle_centered") is not True
        or contract.get("controller")
        != "rviz_default_plugins/TopDownOrtho"
        or contract.get("target_frame") != "base_link"
        or not isinstance(center, list)
        or len(center) != 2
        or any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(float(value))
            or not math.isclose(float(value), 0.0, abs_tol=1e-9)
            for value in center
        )
        or not math.isclose(
            _finite_number(contract.get("angle_rad"), f"{label} RViz angle"),
            0.0,
            abs_tol=1e-9,
        )
        or not math.isclose(
            _finite_number(
                contract.get("scale"), f"{label} RViz scale", positive=True
            ),
            10.0,
            abs_tol=1e-9,
        )
    ):
        raise PublicationError(
            f"{label} RViz view must center base_link at X/Y 0 using "
            "TopDownOrtho angle 0 and scale 10"
        )
    visible_topics = contract.get("visible_path_topics")
    required_topics = {
        "/planning/trajectory",
        "/planning/vad/candidate_trajectories",
        "/planning/vad_route/actual_path",
        "/planning/vad_route/reference_path",
        "/planning/vad_route/selected_raw_trajectory",
    }
    if not isinstance(visible_topics, list) or not required_topics.issubset(
        {item for item in visible_topics if isinstance(item, str)}
    ):
        raise PublicationError(
            f"{label} centered capture does not show every required path topic"
        )
    visual_clarity = contract.get("visual_clarity")
    expected_visual_clarity = {
        "odometry_display": "Kinematic State",
        "odometry_keep": 1,
        "odometry_covariance": False,
        "odometry_orientation": False,
        "odometry_position": False,
        "candidate_path_alpha": 0.22,
        "candidate_path_width": 0.04,
    }
    if not isinstance(visual_clarity, Mapping) or dict(visual_clarity) != (
        expected_visual_clarity
    ):
        raise PublicationError(
            f"{label} RViz visual clarity must disable odometry trails and use "
            "the low-emphasis candidate path style"
        )

    provenance_relative = "rviz_capture_provenance/autoware_vad_carla.rviz"
    if contract.get("config_file") != provenance_relative:
        raise PublicationError(
            f"{label} RViz view contract must use pinned {provenance_relative}"
        )
    provenance_config = directory / provenance_relative
    if not provenance_config.is_file():
        raise PublicationError(f"{label} pinned RViz config is missing")
    config_sha256 = contract.get("config_sha256")
    if (
        not isinstance(config_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", config_sha256)
        or _sha256(provenance_config) != config_sha256
    ):
        raise PublicationError(f"{label} pinned RViz config digest mismatch")
    checksums_path = directory / "rviz_capture_provenance/SHA256SUMS"
    try:
        checksum_lines = checksums_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise PublicationError(
            f"cannot read {label} RViz provenance checksums: {error}"
        ) from error
    expected_checksum = f"{config_sha256}  autoware_vad_carla.rviz"
    if checksum_lines != [expected_checksum]:
        raise PublicationError(f"{label} RViz provenance checksum file mismatch")

    representative = capture.get("representative_frame")
    if not isinstance(representative, Mapping):
        raise PublicationError(f"{label} has no representative frame provenance")
    recording_file = capture.get("recording_file")
    if (
        recording_file != "autoware_rviz_capture.mkv"
        or representative.get("source") != recording_file
        or representative.get("selection") != "route_evaluation_midpoint"
    ):
        raise PublicationError(
            f"{label} representative frame must be the route-evaluation midpoint"
        )
    recording_path = directory / recording_file
    if not recording_path.is_file() or recording_path.stat().st_size <= 0:
        raise PublicationError(f"{label} source desktop recording is missing or empty")
    offset = _finite_number(
        representative.get("offset_sec"),
        f"{label} representative frame offset",
    )
    duration = _finite_number(
        representative.get("recording_duration_sec"),
        f"{label} desktop recording duration",
        positive=True,
    )
    if offset < 0.0 or offset > duration:
        raise PublicationError(
            f"{label} representative frame is outside the desktop recording"
        )
    route_started = _parse_timestamp(
        capture.get("route_evaluation_started_at"),
        f"{label} route evaluation start",
    )
    recording_started = _parse_timestamp(
        capture.get("recording_started_at"),
        f"{label} desktop recording start",
    )
    route_finished = _parse_timestamp(
        capture.get("route_evaluation_finished_at"),
        f"{label} route evaluation finish",
    )
    captured_at = _parse_timestamp(
        capture.get("captured_at"), f"{label} centered capture"
    )
    representative_at = _parse_timestamp(
        representative.get("captured_at"),
        f"{label} representative frame",
    )
    timestamp_offset = (representative_at - recording_started).total_seconds()
    if abs(offset - timestamp_offset) > 0.01:
        raise PublicationError(
            f"{label} representative frame offset does not match its timestamp"
        )
    if not (
        candidate_observed_at
        <= candidate_still_at
        <= recording_started
        <= route_started
    ):
        raise PublicationError(
            f"{label} candidate/capture/route timestamps are out of order"
        )
    if route_finished <= route_started:
        raise PublicationError(f"{label} route evaluation timestamps are inverted")
    route_midpoint = route_started + (route_finished - route_started) / 2
    if (
        captured_at != representative_at
        or captured_at < route_started
        or captured_at > route_finished
        or abs((captured_at - route_midpoint).total_seconds()) > 2.0
    ):
        raise PublicationError(
            f"{label} representative still is not the route-evaluation midpoint"
        )

    runtime = _read_runtime_environment(directory / "runtime.env", f"{label} runtime")
    expected_profile = {
        "RECOMMENDED": "true",
        "VISUALIZE": "true",
        "CAPTURE_DESKTOP": "true",
    }
    for key, expected in expected_profile.items():
        if runtime.get(key) != expected:
            raise PublicationError(
                f"{label} visual refresh did not use the recommended visualized "
                f"profile ({key}={expected})"
            )
    runtime_config = runtime.get("RVIZ_CAPTURE_CONFIG")
    if not isinstance(runtime_config, str) or not runtime_config:
        raise PublicationError(f"{label} runtime has no pinned RViz config")
    runtime_config_path = Path(runtime_config).expanduser()
    if not runtime_config_path.is_absolute():
        runtime_config_path = directory / runtime_config_path
    if (
        runtime_config_path.resolve() != provenance_config.resolve()
        or runtime.get("RVIZ_CAPTURE_CONFIG_SHA256") != config_sha256
    ):
        raise PublicationError(f"{label} runtime/pinned RViz config mismatch")
    if speed_contract is not None or matrix_speed_evidence is not None:
        return dict(contract), _validate_speed_runtime_binding(
            directory,
            runtime,
            speed_contract,
            matrix_speed_evidence,
            trial_id,
            scenario,
            label,
        )
    return dict(contract), expected_profile


def collect_vad_visual_refreshes(
    snapshot: Mapping[str, Any],
    refresh_paths: Sequence[VadTrialSpec],
    selected_trials: Sequence[VadTrial],
) -> dict[tuple[str, str], VadVisualRefresh]:
    specs = _normalize_vad_trial_specs(refresh_paths)
    selected_by_identity = {
        (trial.map_id, trial.trial_id): trial
        for trial in selected_trials
        if trial.trial_id is not None
    }
    for spec in specs:
        identity = (spec.map_id, spec.trial_id)
        if spec.trial_id is None:
            raise PublicationError(
                "VAD visual refreshes require an explicit map and trial id"
            )
        if identity not in selected_by_identity:
            raise PublicationError(
                f"{spec.map_id}:{spec.trial_id} visual refresh has no selected "
                "published validation trial"
            )
    refresh_trials = collect_vad_trials(snapshot, specs)
    refreshes: dict[tuple[str, str], VadVisualRefresh] = {}
    for trial in refresh_trials:
        if trial.trial_id is None:  # pragma: no cover - checked above.
            raise PublicationError("VAD visual refresh lost its trial id")
        identity = (trial.map_id, trial.trial_id)
        label = f"{trial.map_id}:{trial.trial_id} visual refresh"
        selected = selected_by_identity[identity]
        if (
            trial.route is None
            or trial.desktop_capture is None
            or selected.route is None
            or _route_identity(trial.route) != _route_identity(selected.route)
        ):
            raise PublicationError(
                f"{label} route identity differs from the selected validation route"
            )
        refresh_source_route = trial.directory / "source_route.json"
        selected_source_route = selected.directory / "source_route.json"
        refresh_source_payload = _read_object(
            refresh_source_route, f"{label} source route"
        )
        selected_source_payload = _read_object(
            selected_source_route, f"{label} selected validation source route"
        )
        refresh_sha256 = _sha256(refresh_source_route)
        selected_sha256 = _sha256(selected_source_route)
        if (
            refresh_sha256 != selected_sha256
            or refresh_source_payload != selected_source_payload
        ):
            raise PublicationError(
                f"{label} source_route.json does not exactly match the selected "
                "validation route"
            )
        _, runtime_profile = _validate_centered_capture(
            trial.directory, trial.desktop_capture, label
        )
        refreshes[identity] = VadVisualRefresh(
            map_id=trial.map_id,
            trial_id=trial.trial_id,
            directory=trial.directory,
            result=trial.result,
            route=trial.route,
            desktop_capture=trial.desktop_capture,
            source_route_sha256=refresh_sha256,
            validation_source_route_sha256=selected_sha256,
            runtime_profile=runtime_profile,
        )
    return refreshes


def collect_ime_proof(
    snapshot: Mapping[str, Any], before_value: Path | None, after_value: Path | None
) -> ImeProof | None:
    if before_value is None and after_value is None:
        return None
    if before_value is None or after_value is None:
        raise PublicationError("VS Code IME proof requires both before and after PNGs")
    artifact_root = Path(snapshot["artifact_root"])
    before = _inside(artifact_root, before_value, "VS Code IME before proof")
    after = _inside(artifact_root, after_value, "VS Code IME after proof")
    sizes = []
    for label, path in (("before", before), ("after", after)):
        try:
            with Image.open(path) as image:
                if image.format != "PNG":
                    raise PublicationError(f"VS Code IME {label} proof is not PNG: {path}")
                image.verify()
            with Image.open(path) as image:
                sizes.append(image.size)
        except (OSError, ValueError) as error:
            raise PublicationError(
                f"cannot validate VS Code IME {label} proof {path}: {error}"
            ) from error
    return ImeProof(
        before=before,
        after=after,
        before_size=sizes[0],
        after_size=sizes[1],
    )


def collect_owned_window_visual_audit(
    directory_value: Path | None,
    expected_trial_directories: Mapping[tuple[str, str], Path],
) -> dict[str, Any] | None:
    if directory_value is None:
        return None
    expected_identities = set(expected_trial_directories)
    if not expected_identities:
        raise PublicationError(
            "owned-window visual audit requires labeled Autoware VAD trials"
        )
    directory = directory_value.expanduser().resolve()
    if not directory.is_dir():
        raise PublicationError(
            f"owned-window visual audit directory is not a directory: {directory}"
        )
    sources = {name: directory / name for name in OWNED_WINDOW_VISUAL_AUDIT_NAMES}
    missing = [str(path) for path in sources.values() if not path.is_file()]
    if missing:
        raise PublicationError(
            "owned-window visual audit is incomplete: " + ", ".join(missing)
        )

    audit = _read_object(
        sources["v16_owned_window_visual_audit.json"],
        "owned-window visual audit",
    )
    review = _read_object(
        sources["v16_owned_window_visual_review.json"],
        "owned-window visual review",
    )
    if audit.get("schema_version") != 2:
        raise PublicationError(
            "owned-window visual audit schema_version must be 2"
        )
    expected_count = len(expected_identities)
    expected_map_count = len({identity[0] for identity in expected_identities})
    counts = audit.get("counts")
    if (
        audit.get("status") != "PASS"
        or audit.get("mechanical_status") != "PASS"
        or not isinstance(counts, Mapping)
        or counts.get("maps") != expected_map_count
        or counts.get("total") != expected_count
        or counts.get("mechanical_pass") != expected_count
        or counts.get("visual_pass") != expected_count
        or counts.get("visual_flag") != 0
        or counts.get("visual_pending") != 0
    ):
        raise PublicationError(
            "owned-window visual audit is not a complete PASS for the published trials"
        )

    required_visual_fields = (
        "vehicle_visible",
        "reference_route_visible",
        "final_trajectory_visible",
        "vad_trajectories_visible",
        "viewport_centered",
    )

    def validated_scene_identities(
        values: Any, label: str, *, nested: bool
    ) -> dict[tuple[str, str], Mapping[str, Any]]:
        if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
            raise PublicationError(f"{label} scenes must be a list")
        scenes: dict[tuple[str, str], Mapping[str, Any]] = {}
        for index, value in enumerate(values):
            if not isinstance(value, Mapping):
                raise PublicationError(f"{label} scene {index} is not an object")
            identity = (value.get("map_id"), value.get("trial_id"))
            if not all(isinstance(item, str) and item for item in identity):
                raise PublicationError(f"{label} scene {index} has no identity")
            typed_identity = (str(identity[0]), str(identity[1]))
            if typed_identity in scenes:
                raise PublicationError(f"{label} contains duplicate scene {typed_identity}")
            scenes[typed_identity] = value
            visual = value.get("visual_review") if nested else value
            if not isinstance(visual, Mapping) or visual.get("status") != "PASS":
                raise PublicationError(f"{label} scene {typed_identity} is not PASS")
            if any(visual.get(field) is not True for field in required_visual_fields):
                raise PublicationError(
                    f"{label} scene {typed_identity} lacks required visible content"
                )
            if nested:
                trial_directory = expected_trial_directories.get(typed_identity)
                if trial_directory is None:  # Checked again as a complete set below.
                    raise PublicationError(
                        f"{label} scene {typed_identity} is not a published VAD trial"
                    )
                trial_directory = trial_directory.expanduser().resolve()
                selected_directory = value.get("selected_attempt_directory")
                if (
                    not isinstance(selected_directory, str)
                    or Path(selected_directory).expanduser().resolve()
                    != trial_directory
                ):
                    raise PublicationError(
                        f"{label} scene {typed_identity} selected attempt mismatch"
                    )
                capture = value.get("capture")
                if not isinstance(capture, Mapping):
                    raise PublicationError(
                        f"{label} scene {typed_identity} has no capture record"
                    )
                contract = capture.get("rviz_view_contract")
                representative = capture.get("representative_frame")
                frame_verification = (
                    representative.get("frame_verification")
                    if isinstance(representative, Mapping)
                    else None
                )
                if (
                    not isinstance(contract, Mapping)
                    or contract.get("vehicle_centered") is not True
                    or contract.get("target_frame") != "base_link"
                    or contract.get("center_xy_m") != [0.0, 0.0]
                    or not isinstance(frame_verification, Mapping)
                    or frame_verification.get("pixel_exact_match") is not True
                ):
                    raise PublicationError(
                        f"{label} scene {typed_identity} violates the centered-frame contract"
                    )

                def require_source_record(
                    record_value: Any,
                    relative: str,
                    record_label: str,
                    *,
                    expected_format: str | None = None,
                ) -> str:
                    if not isinstance(record_value, Mapping):
                        raise PublicationError(
                            f"{label} scene {typed_identity} lacks {record_label} provenance"
                        )
                    source = trial_directory / relative
                    if not source.is_file():
                        raise PublicationError(
                            f"{label} scene {typed_identity} source is missing: {source}"
                        )
                    digest = _sha256(source)
                    if (
                        record_value.get("sha256") != digest
                        or record_value.get("size_bytes") != source.stat().st_size
                    ):
                        raise PublicationError(
                            f"{label} scene {typed_identity} {record_label} source binding mismatch"
                        )
                    if expected_format is not None:
                        dimensions = _image_dimensions(
                            source,
                            f"{label} scene {typed_identity} {record_label}",
                            expected_format,
                        )
                        if record_value.get("dimensions") != list(dimensions):
                            raise PublicationError(
                                f"{label} scene {typed_identity} {record_label} dimensions mismatch"
                            )
                    return digest

                representative_digest = require_source_record(
                    capture.get("representative_png"),
                    "autoware_rviz_fullscreen.png",
                    "representative PNG",
                    expected_format="PNG",
                )
                require_source_record(
                    capture.get("candidate_png"),
                    "autoware_rviz_candidate.png",
                    "candidate PNG",
                    expected_format="PNG",
                )
                require_source_record(
                    capture.get("drive_gif"),
                    "autoware_rviz_drive.gif",
                    "drive GIF",
                    expected_format="GIF",
                )
                require_source_record(
                    capture.get("recording"),
                    "autoware_rviz_capture.mkv",
                    "recording",
                )
                metadata_path = trial_directory / "desktop_capture.json"
                if capture.get("metadata_sha256") != _sha256(metadata_path):
                    raise PublicationError(
                        f"{label} scene {typed_identity} capture metadata binding mismatch"
                    )
                result_record = value.get("result")
                result_path = trial_directory / "result.json"
                if (
                    not isinstance(result_record, Mapping)
                    or result_record.get("sha256") != _sha256(result_path)
                ):
                    raise PublicationError(
                        f"{label} scene {typed_identity} result binding mismatch"
                    )
                route_record = value.get("source_route")
                route_path = trial_directory / "source_route.json"
                if (
                    not isinstance(route_record, Mapping)
                    or route_record.get("status") != "EXACT_MATCH"
                    or route_record.get("sha256") != _sha256(route_path)
                    or any(
                        not isinstance(route_record.get(key), str)
                        or Path(str(route_record[key])).expanduser().resolve()
                        != route_path
                        for key in ("centered_file", "publication_original_file")
                    )
                ):
                    raise PublicationError(
                        f"{label} scene {typed_identity} source-route binding mismatch"
                    )
                if visual.get("representative_png_sha256") != representative_digest:
                    raise PublicationError(
                        f"{label} scene {typed_identity} visual review PNG binding mismatch"
                    )
        if set(scenes) != expected_identities:
            raise PublicationError(
                f"{label} identities do not match the published VAD trials"
            )
        return scenes

    audit_scenes = validated_scene_identities(
        audit.get("trials"), "visual audit", nested=True
    )
    review_scenes = validated_scene_identities(
        review.get("scenes"), "visual review", nested=False
    )
    for identity, audit_scene in audit_scenes.items():
        if dict(audit_scene["visual_review"]) != dict(review_scenes[identity]):
            raise PublicationError(
                f"visual audit scene {identity} does not match operator review JSON"
            )

    selection = audit.get("publication_selection")
    if not isinstance(selection, Mapping):
        raise PublicationError(
            "owned-window visual audit has no publication-selection binding"
        )
    selection_records = selection.get("records")
    if not isinstance(selection_records, list):
        raise PublicationError(
            "owned-window visual audit publication-selection records must be a list"
        )
    expected_selection_records: list[dict[str, str]] = []
    for map_id, trial_id in sorted(expected_identities):
        scene = audit_scenes[(map_id, trial_id)]
        route = scene.get("source_route")
        source = route.get("publication_source") if isinstance(route, Mapping) else None
        if not isinstance(source, str) or not source:
            raise PublicationError(
                f"visual audit scene {(map_id, trial_id)} has no publication source"
            )
        expected_selection_records.append(
            {
                "map_id": map_id,
                "trial_id": trial_id,
                "trial_directory": str(
                    expected_trial_directories[(map_id, trial_id)]
                    .expanduser()
                    .resolve()
                ),
                "source": source,
            }
        )
    selection_source_file = selection.get("selection_source_file")
    if (
        not isinstance(selection_source_file, str)
        or not selection_source_file
        or selection.get("scope") != PUBLICATION_SELECTION_SCOPE
        or selection.get("canonicalization")
        != PUBLICATION_SELECTION_CANONICALIZATION
        or selection.get("record_count") != expected_count
        or selection_records != expected_selection_records
        or selection.get("sha256") != _sha256_json(selection_records)
    ):
        raise PublicationError(
            "owned-window visual audit publication-selection binding mismatch"
        )

    contact = audit.get("contact_sheet")
    contact_path = sources["v16_owned_window_contact_sheet.png"]
    if not isinstance(contact, Mapping):
        raise PublicationError("owned-window visual audit has no contact-sheet record")
    dimensions = _image_dimensions(
        contact_path, "owned-window visual audit contact sheet", "PNG"
    )
    with Image.open(contact_path) as contact_image:
        extrema = contact_image.convert("RGB").getextrema()
    if all(low == high for low, high in extrema):
        raise PublicationError("owned-window visual audit contact sheet is blank")
    if (
        contact.get("dimensions") != list(dimensions)
        or contact.get("scene_count") != expected_count
        or contact.get("sha256") != _sha256(contact_path)
        or not isinstance(contact.get("rows"), int)
        or not isinstance(contact.get("columns"), int)
        or contact.get("rows", 0) < 1
        or contact.get("columns", 0) < 1
        or contact["rows"] * contact["columns"] < expected_count
    ):
        raise PublicationError(
            "owned-window visual audit contact-sheet provenance mismatch"
        )
    markdown = sources["v16_owned_window_visual_audit.md"].read_text(
        encoding="utf-8"
    )
    if "Overall status: **PASS**" not in markdown:
        raise PublicationError("owned-window visual audit Markdown is not PASS")

    return {
        "status": "PASS",
        "source_directory": str(directory),
        "counts": dict(counts),
        "contact_sheet_dimensions": list(dimensions),
        "files": {
            name: {
                "source": str(source),
                "sha256": _sha256(source),
                "published_file": f"{OWNED_WINDOW_VISUAL_AUDIT_DIR}/{name}",
            }
            for name, source in sources.items()
        },
    }


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    names = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    )
    try:
        return ImageFont.truetype(names, size=size)
    except OSError:
        return ImageFont.load_default()


def _fit_text(draw: ImageDraw.ImageDraw, text: str, width: int, font: ImageFont.ImageFont) -> str:
    if draw.textlength(text, font=font) <= width:
        return text
    suffix = "..."
    candidate = text
    while candidate and draw.textlength(candidate + suffix, font=font) > width:
        candidate = candidate[:-1]
    return candidate.rstrip() + suffix


def render_status_dashboard(snapshot: Mapping[str, Any], output: Path) -> Path:
    width, height = 1920, 1080
    canvas = Image.new("RGB", (width, height), "#eef2f6")
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((36, 28, width - 36, height - 28), 20, fill="#ffffff")
    draw.text((68, 54), "CARLA packaged-map validation status", "#172033", _font(34, True))
    draw.text(
        (68, 101),
        "CARLA BasicAgent six-camera route smoke — not Autoware VAD inference or control",
        "#a43820",
        _font(22, True),
    )
    generated = str(snapshot.get("generated_at", "unknown"))
    overall = str(snapshot.get("status", "UNKNOWN"))
    selected = int(snapshot.get("selected_map_count", 0))
    success = int(snapshot.get("selected_success_count", 0))
    draw.text(
        (68, 146),
        f"Aggregate: {overall}    selected success: {success}/{selected}    generated: {generated}",
        "#4b5565",
        _font(20),
    )

    status_colors = {
        "PASS": "#16794b",
        "SKIP_RESUME_VALIDATED": "#1769aa",
        "FAILED": "#b42318",
        "RUNNING": "#b26a00",
        "PENDING": "#526071",
        "EXCLUDED": "#707782",
        "SOURCE_EDITOR_REQUIRED": "#7546a5",
        "BLOCKED": "#343b45",
    }
    chip_x = 68
    for status, count in sorted(snapshot["status_counts"].items()):
        label = f"{status} {count}"
        label_width = int(draw.textlength(label, font=_font(16, True))) + 28
        draw.rounded_rectangle(
            (chip_x, 187, chip_x + label_width, 220),
            13,
            fill=status_colors.get(status, "#526071"),
        )
        draw.text((chip_x + 14, 194), label, "white", _font(16, True))
        chip_x += label_width + 10

    maps = list(snapshot["maps"])
    columns = (maps[:10], maps[10:])
    left_positions = (68, 992)
    card_width = 860
    card_height = 70
    card_gap = 7
    top = 244
    for column, entries in enumerate(columns):
        left = left_positions[column]
        for row, entry in enumerate(entries):
            card_top = top + row * (card_height + card_gap)
            card_bottom = card_top + card_height
            status = str(entry.get("status", "UNKNOWN"))
            color = status_colors.get(status, "#526071")
            draw.rounded_rectangle(
                (left, card_top, left + card_width, card_bottom),
                10,
                fill="#f8fafc",
                outline="#d9e0e8",
                width=1,
            )
            draw.rounded_rectangle(
                (left, card_top, left + 12, card_bottom), 6, fill=color
            )
            map_name = f"{entry.get('canonical_name', entry['map_id'])}  [{entry['map_id']}]"
            draw.text((left + 28, card_top + 10), map_name, "#172033", _font(18, True))
            status_font = _font(14, True)
            status_width = int(draw.textlength(status, font=status_font)) + 24
            draw.rounded_rectangle(
                (
                    left + card_width - status_width - 16,
                    card_top + 8,
                    left + card_width - 16,
                    card_top + 34,
                ),
                10,
                fill=color,
            )
            draw.text(
                (left + card_width - status_width - 4, card_top + 13),
                status,
                "white",
                status_font,
            )
            reason = str(entry.get("reason") or entry.get("stage") or "no reason recorded")
            reason = _fit_text(draw, reason.replace("\n", " "), card_width - 54, _font(14))
            draw.text((left + 28, card_top + 41), reason, "#596579", _font(14))

    footer = (
        "19 canonical map records; success media are rendered only from complete "
        "episode + validated export data."
    )
    draw.text((68, height - 59), footer, "#4b5565", _font(17))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)
    with Image.open(output) as check:
        if check.size != (1920, 1080):  # pragma: no cover - Pillow contract.
            raise PublicationError(f"dashboard has unexpected dimensions: {check.size}")
    return output


def render_basicagent_run(
    run: BasicAgentRun,
    output_dir: Path,
    gif_fps: float,
    gif_max_frames: int,
) -> dict[str, Any]:
    samples = read_jsonl(run.export / "samples.jsonl")
    states = read_jsonl(run.episode / "states.jsonl")
    route = _read_object(run.episode / "route.json", "episode route")
    output_dir.mkdir(parents=True, exist_ok=True)
    png = output_dir / MANAGED_BASICAGENT_NAMES[0]
    gif = output_dir / MANAGED_BASICAGENT_NAMES[1]
    overview_index = _overview_index(samples)
    frame = render_frame(
        run.episode, route, states, samples[overview_index], 1920, 1080
    )
    frame.save(png)
    with tempfile.TemporaryDirectory(prefix="basicagent_gif_preview_") as temporary:
        temporary_png = Path(temporary) / "overview_960x540.png"
        report = render_episode(
            run.episode,
            run.export,
            temporary_png,
            gif,
            fps=gif_fps,
            maximum_frames=gif_max_frames,
            width=960,
            height=540,
        )
    with Image.open(png) as check:
        if check.size != (1920, 1080):
            raise PublicationError(f"{run.map_id} overview is not 1920x1080")
    with Image.open(gif) as check:
        if check.size != (960, 540):
            raise PublicationError(f"{run.map_id} GIF is not 960x540")
    return {
        "map_id": run.map_id,
        "canonical_name": run.canonical_name,
        "episode": str(run.episode),
        "export": str(run.export),
        "sample_count": run.sample_count,
        "rendered_frame_count": report["rendered_frame_count"],
        "overview_sample_index": overview_index,
        "result": dict(run.episode_manifest.get("result", {})),
        "export_metrics": {
            key: run.export_manifest.get(key)
            for key in (
                "maximum_route_cte_m",
                "collision_event_count",
                "lane_invasion_event_count",
            )
        },
        "files": [
            f"{run.map_id}/{MANAGED_BASICAGENT_NAMES[0]}",
            f"{run.map_id}/{MANAGED_BASICAGENT_NAMES[1]}",
        ],
    }


def _relative_link(path: Path, start: Path) -> str:
    return Path(os.path.relpath(path, start=start)).as_posix()


def _markdown(
    snapshot: Mapping[str, Any],
    basic_records: Sequence[Mapping[str, Any]],
    vad_records: Sequence[Mapping[str, Any]],
    ime_record: Mapping[str, Any] | None,
    link_prefix: str,
    aggregate_link: str,
    vad_matrix_record: Mapping[str, Any] | None,
    docs_assets_root: Path,
    report_path: Path | None,
    owned_window_visual_audit_record: Mapping[str, Any] | None = None,
    report_preamble: Mapping[str, Any] | None = None,
    camera_source_diagnostics_record: Mapping[str, Any] | None = None,
    *,
    embed_report_preamble: bool = False,
) -> str:
    basic_by_map = {record["map_id"]: record for record in basic_records}
    reproduction_parts = [
        "python3 scripts/e2e/publish_validation_assets.py",
        f"  --artifact-root {snapshot['artifact_root']}",
        f"  --docs-assets-root {docs_assets_root}",
        f"  --expected-map-count {snapshot['canonical_map_count']}",
        f"  --expected-selected-map-count {snapshot['selected_map_count']}",
    ]
    if report_path is not None:
        reproduction_parts.append(f"  --report {report_path}")
    if report_preamble is not None:
        reproduction_parts.append(
            f"  --report-preamble {report_preamble['source']}"
        )
    if owned_window_visual_audit_record is not None:
        reproduction_parts.append(
            "  --owned-window-visual-audit-dir "
            f"{owned_window_visual_audit_record['source_directory']}"
        )
    if vad_matrix_record is not None:
        reproduction_parts.append(
            f"  --vad-matrix-root {vad_matrix_record['matrix_root']}"
        )
        selector = vad_matrix_record.get(
            "runtime_profile_selector", "recommended"
        )
        if selector != "recommended":
            reproduction_parts.append(
                f"  --vad-runtime-profile-selector {selector}"
            )
    for record in vad_records:
        if record.get("source") == "matrix_pass_auto_discovery":
            continue
        identity = record["map_id"]
        if record.get("trial_id") is not None:
            identity += f":{record['trial_id']}"
        reproduction_parts.append(
            f"  --vad-trial {identity}={record['trial_directory']}"
        )
    for record in vad_records:
        refresh = record.get("visual_refresh")
        if not isinstance(refresh, Mapping):
            continue
        identity = f"{record['map_id']}:{record['trial_id']}"
        reproduction_parts.append(
            "  --vad-visual-refresh "
            f"{identity}={refresh['refresh_trial_directory']}"
        )
    if ime_record is not None:
        reproduction_parts.extend(
            [
                f"  --vscode-ime-before {ime_record['before_source']}",
                f"  --vscode-ime-after {ime_record['after_source']}",
            ]
        )
    reproduction_command = " \\\n".join(reproduction_parts)

    def asset(relative: str) -> str:
        return f"{link_prefix.rstrip('/')}/{relative}" if link_prefix not in ("", ".") else relative

    def markdown_cell(value: Any) -> str:
        return str(value).replace("\n", " ").replace("|", "\\|")

    lines = [
        "# Validation evidence",
        "",
    ]
    if embed_report_preamble and report_preamble is not None:
        lines.extend(
            [
                "> The following campaign summary is an operator-reviewed, SHA-bound "
                "narrative supplement. The generated provenance and result tables "
                "below remain authoritative.",
                "",
            ]
        )
        lines.extend(str(report_preamble["content"]).strip().splitlines())
        lines.append("")
    lines.extend([
        f"Source: [archived CARLA BasicAgent aggregate JSON]({aggregate_link}) "
        f"(`sha256:{snapshot['aggregate_sha256']}`) — overall "
        f"**{snapshot.get('status')}**, selected success "
        f"**{snapshot.get('selected_success_count')}/{snapshot.get('selected_map_count')}**.",
        "",
        "> Scope boundary: the all-map sweep and `expert_*` media are CARLA BasicAgent "
        "six-camera route-smoke evidence. They are not Autoware VAD inference or "
        "closed-loop-control evidence.",
        "",
        f"![All-map CARLA BasicAgent status]({asset(DASHBOARD_NAME)})",
        "",
        "## CARLA BasicAgent packaged-map sweep",
        "",
        "| Map | Status | Stage | Published media |",
        "|---|---|---|---|",
    ])
    for entry in snapshot["maps"]:
        map_id = entry["map_id"]
        record = basic_by_map.get(map_id)
        media = "—"
        if record:
            media = (
                f"[PNG 1920x1080]({asset(record['files'][0])}), "
                f"[GIF 960x540]({asset(record['files'][1])})"
            )
        lines.append(
            f"| `{map_id}` ({entry.get('canonical_name', map_id)}) | "
            f"**{entry.get('status', 'UNKNOWN')}** | {entry.get('stage', '—')} | {media} |"
        )
    lines.extend(
        [
            "",
            "Only PASS/SKIP_RESUME_VALIDATED rows with a complete episode, validated "
            "export, matching manifests, and all six camera images are published.",
            "",
            "## Autoware VAD full-stack trials",
            "",
        ]
    )
    if vad_matrix_record is not None:
        matrix_status = vad_matrix_record["status"]
        counts = ", ".join(
            f"{name}={count}"
            for name, count in vad_matrix_record["status_counts"].items()
        )
        matrix_aggregate_file = vad_matrix_record.get("published_aggregate_file")
        matrix_plan_file = vad_matrix_record.get("published_plan_file")
        matrix_aggregate_link = (
            f"[archived terminal aggregate JSON]({asset(matrix_aggregate_file)})"
            if isinstance(matrix_aggregate_file, str)
            else "source terminal aggregate JSON"
        )
        matrix_plan_link = (
            f"[archived matrix plan JSON]({asset(matrix_plan_file)})"
            if isinstance(matrix_plan_file, str)
            else "source matrix plan JSON"
        )
        if _is_speed_30kph_profile(
            str(vad_matrix_record.get("runtime_profile_selector"))
        ):
            interpretation = vad_matrix_record.get("evidence_interpretation")
            expected_interpretation = _speed_30kph_publication_interpretation(
                str(vad_matrix_record.get("runtime_profile_selector"))
            )
            if interpretation != expected_interpretation:
                raise PublicationError(
                    "speed_30kph publication lost its simulation-screening boundary"
                )
            lines.extend(
                [
                    "> **30 kph evidence boundary — simulation screening only.** "
                    "`carla_vad_30kph_v2` evaluates VAD candidate geometry through "
                    "`vad_route_manager_hybrid` while an explicit CARLA simulation "
                    "speed overlay supplies longitudinal cruise velocity. It does not "
                    "evaluate raw VAD cruise velocity and `real_vehicle_ready=false`. "
                    "A PASS is therefore not a real-vehicle-readiness claim and does "
                    "not by itself claim that measured speed reached exactly 30 kph.",
                    "",
                ]
            )
            if (
                vad_matrix_record.get("runtime_profile_selector")
                == CAMERA_SOURCE_5HZ_SELECTOR
            ):
                lines.extend(
                    [
                        "> **5 Hz camera-source contract.** All six CARLA RGB "
                        "cameras use source `sensor_tick=0.2 s` and reliable ROS "
                        "publication at 5 Hz; IMU/GNSS settings are unchanged. "
                        "Publication freshly recomputes the six topic stamp rates, "
                        "worst stamp gap, synchronized-bundle coverage, candidate/front "
                        "coverage, and VAD queue/mailbox loss directly from `runtime.env`, "
                        "the pinned sensor mapping, latency JSON, and recorder/stack logs.",
                        "",
                    ]
                )
        lines.extend(
            [
                f"Matrix campaign status: **{matrix_status}**; runnable map PASS "
                f"**{vad_matrix_record['runnable_pass_count']}/"
                f"{vad_matrix_record['runnable_map_count']}**; {counts}.",
                "",
                f"Provenance: {matrix_plan_link} "
                f"(`sha256:{vad_matrix_record['plan_sha256']}`, generated "
                f"`{vad_matrix_record['plan_generated_at']}`); "
                f"{matrix_aggregate_link} "
                f"(`sha256:{vad_matrix_record['aggregate_sha256']}`, generated "
                f"`{vad_matrix_record['generated_at']}`).",
                "",
                "The terminal table below is immutable publication provenance. Matrix "
                "PASS trials are revalidated from status + `matrix_validation.json`; "
                "camera-source trials additionally re-run the raw cadence and loss "
                "validator. "
                "Explicit supplemental PASS rows are reported separately with capture "
                "timing; they never change a matrix `FAILED` or `BLOCKED` row.",
                "",
                "Map identities are not aliases: `town05` (Town05) remains distinct "
                "from `town05_opt` (Town05_Opt), and `town10hd` (Town10HD) remains "
                "distinct from `town10hd_opt` (Town10HD_Opt). Evidence for an optimized "
                "variant does not validate its standard-map counterpart.",
                "",
                "### Original terminal matrix: all 19 maps",
                "",
                "| Map | Runnable | Matrix status | Stage | Straight | Turn | Recorded reason |",
                "|---|---|---|---|---|---|---|",
            ]
        )
        for matrix_map in vad_matrix_record["maps"]:
            trials = matrix_map["trials"]
            lines.append(
                f"| `{matrix_map['map_id']}` ({matrix_map['canonical_name']}) | "
                f"{'yes' if matrix_map['runnable'] else 'no'} | "
                f"**{matrix_map['status']}** | "
                f"{markdown_cell(matrix_map['stage'])} | "
                f"{trials['straight']['status']} | {trials['turn']['status']} | "
                f"{markdown_cell(matrix_map['reason'])} |"
            )
        if vad_matrix_record["failed_maps"]:
            lines.extend(
                [
                    "",
                    "### Original terminal matrix failures (unchanged)",
                    "",
                    "These records remain failures even when a later or external "
                    "supplement passes. An alternate-route supplement demonstrates only "
                    "the published alternate route, not recovery of the failed route.",
                    "",
                    "| Map | Stage | Matrix reason | Non-PASS trial records |",
                    "|---|---|---|---|",
                ]
            )
            for failure in vad_matrix_record["failed_maps"]:
                outcome_parts = []
                for trial in failure["trials"]:
                    route = trial.get("route")
                    route_text = ""
                    if route:
                        route_text = (
                            f" (matrix route {_route_endpoint_text(route)}, "
                            f"{route['scenario']})"
                        )
                    outcome_parts.append(
                        f"{trial['trial_id']}={trial['status']}{route_text}: "
                        f"{markdown_cell(trial['reason'])}"
                    )
                outcomes = "; ".join(outcome_parts) or (
                    "map-level failure; both trial records were PASS"
                )
                lines.append(
                    f"| `{failure['map_id']}` ({failure['canonical_name']}) | "
                    f"{markdown_cell(failure['stage'])} | "
                    f"{markdown_cell(failure['reason'])} | {outcomes} |"
                )

        selected_variants = [
            (entry["map_id"], str(entry.get("canonical_name", entry["map_id"])))
            for entry in snapshot["maps"]
            if entry.get("selected") is True
        ]
        paired_coverage: dict[str, dict[str, str | None]] = {
            map_id: {"straight": None, "turn": None}
            for map_id, _ in selected_variants
        }
        for matrix_map in vad_matrix_record["maps"]:
            map_id = matrix_map["map_id"]
            if map_id not in paired_coverage:
                continue
            for trial_id in ("straight", "turn"):
                if matrix_map["trials"][trial_id]["status"] == "PASS":
                    paired_coverage[map_id][trial_id] = "PASS (matrix)"
        for record in vad_records:
            map_id = record["map_id"]
            trial_id = record.get("trial_id")
            if map_id not in paired_coverage or trial_id not in {"straight", "turn"}:
                continue
            context = record.get("matrix_context") or {}
            relation = context.get("relation")
            if relation == "matrix_pass":
                label = "PASS (matrix)"
            elif relation == "explicit_recovery_alternate_route":
                label = "PASS (alternate-route supplement; matrix failure preserved)"
            elif relation == "explicit_recovery_same_route":
                label = "PASS (same-route supplement; matrix failure preserved)"
            elif relation == "explicit_supplement_for_blocked_map":
                label = "PASS (supplement; matrix BLOCKED preserved)"
            else:
                label = "PASS (explicit supplement)"
            paired_coverage[map_id][trial_id] = label
        lines.extend(
            [
                "",
                "### Final paired coverage across executable variants",
                "",
                "This is a union of status-backed matrix PASS trials and separately "
                "validated explicit supplements. It is a publication coverage view, "
                "not a rewritten matrix result. Only exact `straight` and `turn` trial "
                "IDs fill the paired slots; the historical C-track `lane_follow` row is "
                "supplemental and does not count as C-track straight coverage.",
                "",
                "| Executable map variant | Straight | Turn | Final paired outcome |",
                "|---|---|---|---|",
            ]
        )
        for map_id, canonical_name in selected_variants:
            coverage = paired_coverage[map_id]
            straight = coverage["straight"] or "NOT PUBLISHED"
            turn = coverage["turn"] or "NOT PUBLISHED"
            outcome = (
                "**PASS/PASS**"
                if coverage["straight"] is not None and coverage["turn"] is not None
                else "**INCOMPLETE**"
            )
            lines.append(
                f"| `{map_id}` ({canonical_name}) | {straight} | {turn} | {outcome} |"
            )

    if not vad_records:
        if vad_matrix_record is None:
            lines.append(
                "No Autoware VAD trial was supplied to this publication. BasicAgent "
                "results above must not be cited as VAD validation."
            )
        else:
            lines.extend(
                [
                    "",
                    "No successful Autoware VAD trial was supplied or discovered for "
                    "publication. BasicAgent results above must not be cited as VAD "
                    "validation.",
                ]
            )
    else:
        refresh_count = sum(
            isinstance(record.get("visual_refresh"), Mapping)
            for record in vad_records
        )
        lines.extend(
            [
                "",
                "### Published PASS trials" if vad_matrix_record is not None else "Published PASS trials",
                "",
                "Every row passed the full-stack, route-assisted HYBRID "
                "`vad_route_manager_hybrid` evaluator; these rows are not unassisted "
                "end-to-end VAD claims. Every labeled row also requires a candidate-gated "
                "owned-window Autoware/RViz capture and recorded route-analysis bundle.",
                "",
            ]
        )
        if refresh_count:
            lines.extend(
                [
                    f"**{refresh_count}** published row(s) use a separately validated "
                    "vehicle-centered visual refresh. Each refresh is a recommended-"
                    "profile, same-route repeated full-stack PASS with `base_link` at "
                    "RViz X/Y 0. Only the RViz PNG/GIF/candidate still and capture "
                    "metadata are refreshed; the original result, route/control "
                    "analysis, matrix relation, and terminal status remain authoritative.",
                    "",
                ]
            )
        if vad_matrix_record is not None:
            lines.extend(
                [
                    "Timing compares `desktop_capture.json.captured_at` with the archived "
                    "matrix plan and terminal aggregate timestamps for ordinary rows. "
                    "For a visual-refresh row, timing remains tied to the original "
                    "authoritative validation capture, which is retained separately in "
                    "the manifest and archive. `after terminal "
                    "matrix` is post-matrix evidence; `during matrix window (external)` "
                    "and `before matrix plan` are still explicit supplements outside the "
                    "matrix status chain. None rewrites the terminal table above.",
                    "",
                    "| Map | Trial | Source and matrix relation | Evidence timing | Route | "
                    "Result | Owned RViz window | Drive GIF | Route/control analysis |",
                    "|---|---|---|---|---|---|---|---|---|",
                ]
            )
        else:
            lines.extend(
                [
                    "| Map | Trial | Route | Result | Owned RViz window | Drive GIF | "
                    "Route/control analysis |",
                    "|---|---|---|---|---|---|---|",
                ]
            )
        for record in vad_records:
            roles = record["file_roles"]
            route = record.get("route")
            route_text = "—"
            if route:
                route_text = (
                    f"`{_route_endpoint_text(route)}` "
                    f"({route['scenario']}, {route['route_length_m']:.2f} m; "
                    f"[JSON]({asset(roles['route'])}))"
                )
            capture = record.get("desktop_capture")
            png_size = capture.get("png_dimensions") if capture else None
            gif_size = capture.get("gif_dimensions") if capture else None
            fullscreen = (
                f"[{'vehicle-centered midpoint ' if record.get('visual_refresh') else ''}"
                f"PNG {png_size[0]}x{png_size[1]}]"
                f"({asset(roles['fullscreen'])})"
                if "fullscreen" in roles
                else "—"
            )
            if "candidate" in roles:
                fullscreen += (
                    f", [vehicle-centered candidate still]"
                    f"({asset(roles['candidate'])})"
                )
            drive = (
                f"[GIF {gif_size[0]}x{gif_size[1]}]"
                f"({asset(roles['drive_gif'])})"
                if "drive_gif" in roles
                else "—"
            )
            analysis = (
                f"[route PNG]({asset(roles['route_result'])}), "
                f"[control GIF]({asset(roles['control_gif'])})"
            )
            if "diagnosis" in roles:
                analysis += (
                    f", [diagnosis]({asset(roles['diagnosis'])}), "
                    f"[tracking]({asset(roles['path_vs_control'])}), "
                    f"[steering]({asset(roles['steering_tracking'])}), "
                    f"[latency]({asset(roles['latency'])})"
                )
            if "speed_profile" in roles:
                analysis += (
                    f", [speed-source JSON]({asset(roles['speed_profile'])}), "
                    f"[speed plot]({asset(roles['speed_plot'])})"
                )
                context = record.get("matrix_context")
                speed_contract = (
                    context.get("speed_contract")
                    if isinstance(context, Mapping)
                    else None
                )
                maximum_speed = (
                    speed_contract.get("maximum_observed_speed_mps")
                    if isinstance(speed_contract, Mapping)
                    else None
                )
                if (
                    isinstance(maximum_speed, (int, float))
                    and not isinstance(maximum_speed, bool)
                    and math.isfinite(float(maximum_speed))
                ):
                    maximum_speed = float(maximum_speed)
                    analysis += (
                        f", observed max `{maximum_speed:.3f} m/s` "
                        f"(`{maximum_speed * 3.6:.2f} kph`)"
                    )
            camera_source = record.get("camera_source_provenance")
            if isinstance(camera_source, Mapping):
                camera_validation = camera_source.get("validation")
                if not isinstance(camera_validation, Mapping):
                    raise PublicationError(
                        "camera-source publication lost its measured validation"
                    )
                coverage = _finite_number(
                    camera_validation.get("bundle_coverage_percent"),
                    "published camera bundle coverage",
                )
                maximum_gap = _finite_number(
                    camera_validation.get("maximum_camera_stamp_gap_sec"),
                    "published maximum camera stamp gap",
                )
                candidate_count = camera_validation.get("candidate_count")
                front_count = camera_validation.get("front_count")
                inference = camera_validation.get("vad_inference")
                if (
                    isinstance(candidate_count, bool)
                    or not isinstance(candidate_count, int)
                    or isinstance(front_count, bool)
                    or not isinstance(front_count, int)
                    or not isinstance(inference, Mapping)
                ):
                    raise PublicationError(
                        "camera-source publication has invalid measured counts"
                    )
                analysis += (
                    f", [camera cadence/integrity]"
                    f"({asset(roles['matrix_validation'])})"
                    f" (bundle `{coverage:.2f}%`, worst gap "
                    f"`{maximum_gap:.3f} s`, candidate/front "
                    f"`{candidate_count}/{front_count}`, queue drops "
                    f"`{inference.get('coalesced_drops')}`), "
                    f"[sensor mapping]({asset(roles['sensor_mapping'])}), "
                    f"[mapping checksum]({asset(roles['sensor_mapping_checksums'])}), "
                    f"[runtime args]({asset(roles['launch_args'])}), "
                    f"[stack log]({asset(roles['stack_log'])}), "
                    f"[recorder log]({asset(roles['recorder_log'])})"
                )
            if "rviz_config" in roles:
                analysis += (
                    f", [capture runtime]({asset(roles['capture_runtime'])}), "
                    f"[pinned RViz config]({asset(roles['rviz_config'])}), "
                    f"[RViz checksum]({asset(roles['rviz_checksums'])})"
                )
            if vad_matrix_record is not None:
                context = record["matrix_context"]
                relation = context["relation"]
                source_labels = {
                    "matrix_pass": "matrix PASS",
                    "explicit_recovery_same_route": (
                        "explicit same-route recovery; matrix failure preserved"
                    ),
                    "explicit_recovery_alternate_route": (
                        "explicit alternate-route supplement; matrix failure preserved"
                    ),
                    "explicit_recovery_route_unavailable": (
                        "explicit recovery; failed matrix route unavailable; failure preserved"
                    ),
                    "explicit_supplement_for_blocked_map": (
                        "explicit supplement for matrix-BLOCKED map; block preserved"
                    ),
                    "explicit_supplement": "explicit supplemental PASS",
                }
                source = source_labels[relation]
                refresh = record.get("visual_refresh")
                if isinstance(refresh, Mapping):
                    repeat_result = refresh["archived_files"]["result.json"]
                    source += (
                        "; same-route vehicle-centered repeated PASS "
                        f"([repeat result]({asset(repeat_result)})); original "
                        "validation provenance preserved"
                    )
                failed_route = context.get("matrix_failed_route")
                if relation == "explicit_recovery_alternate_route" and failed_route:
                    source += (
                        f" (failed matrix route {_route_endpoint_text(failed_route)}, "
                        f"{failed_route['scenario']})"
                    )
                timing_labels = {
                    "matrix_campaign": "matrix status-backed",
                    "before_matrix_plan": "before matrix plan",
                    "during_matrix_window_outside_matrix": (
                        "during matrix window (external)"
                    ),
                    "after_terminal_matrix": "after terminal matrix",
                    "capture_time_unavailable": "capture time unavailable",
                }
                timing = timing_labels[context["timing"]]
                if context.get("captured_at"):
                    timing += f" (`{context['captured_at']}`)"
                refresh = record.get("visual_refresh")
                if isinstance(refresh, Mapping):
                    timing += (
                        "; centered visual refresh "
                        f"(`{refresh['desktop_capture']['captured_at']}`)"
                    )
                lines.append(
                    f"| `{record['map_id']}` | `{record.get('trial_id') or 'default'}` | "
                    f"{source} | {timing} | {route_text} | "
                    f"[JSON]({asset(roles['result'])}) | {fullscreen} | {drive} | "
                    f"{analysis} |"
                )
            else:
                lines.append(
                    f"| `{record['map_id']}` | `{record.get('trial_id') or 'default'}` | "
                    f"{route_text} | [JSON]({asset(roles['result'])}) | "
                    f"{fullscreen} | {drive} | {analysis} |"
                )
    if owned_window_visual_audit_record is not None:
        audit_files = owned_window_visual_audit_record["files"]
        audit_counts = owned_window_visual_audit_record["counts"]
        contact = audit_files["v16_owned_window_contact_sheet.png"][
            "published_file"
        ]
        audit_json = audit_files["v16_owned_window_visual_audit.json"][
            "published_file"
        ]
        audit_markdown = audit_files["v16_owned_window_visual_audit.md"][
            "published_file"
        ]
        review_json = audit_files["v16_owned_window_visual_review.json"][
            "published_file"
        ]
        dimensions = owned_window_visual_audit_record[
            "contact_sheet_dimensions"
        ]
        lines.extend(
            [
                "",
                "## Owned-window vehicle-centered visual audit",
                "",
                f"**PASS**: {audit_counts['maps']} maps / "
                f"{audit_counts['total']} straight+turn trials passed both the "
                "mechanical centered-frame contract and the operator visual review. "
                "Every representative PNG is pixel-exact with its route-midpoint "
                "recording frame.",
                "",
                f"[Contact sheet {dimensions[0]}x{dimensions[1]}]({asset(contact)}), "
                f"[audit JSON]({asset(audit_json)}), "
                f"[audit Markdown]({asset(audit_markdown)}), "
                f"[operator review JSON]({asset(review_json)}).",
                "",
                f"![All owned-window Autoware VAD scenes]({asset(contact)})",
            ]
        )
    if camera_source_diagnostics_record is not None:
        diagnostic_files = camera_source_diagnostics_record["files"]

        def diagnostic_file(suffix: str) -> str:
            matches = [
                value
                for value in diagnostic_files
                if isinstance(value, str) and value.endswith(suffix)
            ]
            if len(matches) != 1:
                raise PublicationError(
                    f"camera-source diagnostics lack one {suffix!r} file"
                )
            return matches[0]

        lines.extend(
            [
                "",
                "## Camera-source retry and host diagnostics",
                "",
                "The retry classification and the lowest-RTF host measurements are "
                "recomputable from archived raw evidence and deterministic "
                "whole-second telemetry excerpts.",
                "",
                f"- [Diagnostic summary JSON]({asset(camera_source_diagnostics_record['summary_file'])})",
                f"- Town02 rejected retry: [result]({asset(diagnostic_file('attempt_001/result.json'))}), "
                f"[stack log]({asset(diagnostic_file('attempt_001/stack.log'))}), "
                f"[recorder log]({asset(diagnostic_file('attempt_001/recorder.log'))}), "
                f"[source route]({asset(diagnostic_file('attempt_001/source_route.json'))})",
                f"- Lowest-RTF window: [vmstat rows]({asset(diagnostic_file('_vmstat_window.log'))}), "
                f"[GPU dmon rows]({asset(diagnostic_file('_nvidia_smi_dmon_window.log'))}), "
                f"[24-CPU host header]({asset(diagnostic_file('_pidstat_host_header.txt'))})",
            ]
        )
    lines.extend(["", "## VS Code Korean IME desktop proof", ""])
    if ime_record is None:
        lines.append("No VS Code IME before/after proof was supplied to this publication.")
    else:
        files = ime_record["files"]
        lines.extend(
            [
                "These screenshots document the editor input-method fix only. They are "
                "not CARLA driving or Autoware VAD evidence.",
                "",
                f"- [Before / ASCII input]({asset(files[0])}) "
                f"({ime_record['before_dimensions'][0]}x{ime_record['before_dimensions'][1]})",
                f"- [After / Hangul input]({asset(files[1])}) "
                f"({ime_record['after_dimensions'][0]}x{ime_record['after_dimensions'][1]})",
            ]
        )
    lines.extend(
        [
            "",
            "## Integrity and reproduction",
            "",
            "Run the checksum command from the published asset directory:",
            "",
            "```bash",
            f"cd {docs_assets_root}",
            "sha256sum -c SHA256SUMS",
            "```",
            "",
            "Rebuild with:",
            "",
            "```bash",
            reproduction_command,
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def _route_endpoint_text(route: Mapping[str, Any]) -> str:
    source = route.get("endpoint_source", "spawn_points")
    start = route.get("start_index", route.get("start_spawn_index"))
    goal = route.get("goal_index", route.get("goal_spawn_index"))
    prefix = "waypoint " if source == "generated_waypoints" else ""
    return f"{prefix}{start}→{goal}"


def _route_identity(route: Mapping[str, Any] | None) -> tuple[Any, ...] | None:
    if route is None:
        return None
    return (
        route.get("town"),
        route.get("scenario"),
        route.get("endpoint_source", "spawn_points"),
        route.get("start_index", route.get("start_spawn_index")),
        route.get("goal_index", route.get("goal_spawn_index")),
    )


def _matrix_publication_context(
    trial: VadTrial, matrix_record: Mapping[str, Any] | None
) -> dict[str, Any] | None:
    if matrix_record is None:
        return None
    matrix_maps = matrix_record.get("maps")
    if not isinstance(matrix_maps, Sequence):  # pragma: no cover - internal contract.
        raise PublicationError("Autoware VAD matrix publication lost its map records")
    map_record = next(
        (
            item
            for item in matrix_maps
            if isinstance(item, Mapping) and item.get("map_id") == trial.map_id
        ),
        None,
    )
    if map_record is None:  # pragma: no cover - discovery already checks this.
        raise PublicationError(f"matrix publication has no map record for {trial.map_id}")

    captured_at = (
        trial.desktop_capture.get("captured_at")
        if trial.desktop_capture is not None
        else None
    )
    timing = "matrix_campaign"
    if trial.source != "matrix_pass_auto_discovery":
        if captured_at is None:
            timing = "capture_time_unavailable"
        else:
            capture_time = _parse_timestamp(
                captured_at, f"{trial.map_id}:{trial.trial_id} supplemental capture"
            )
            plan_time = _parse_timestamp(
                matrix_record.get("plan_generated_at"), "Autoware VAD matrix plan"
            )
            terminal_time = _parse_timestamp(
                matrix_record.get("generated_at"), "Autoware VAD matrix aggregate"
            )
            if capture_time < plan_time:
                timing = "before_matrix_plan"
            elif capture_time <= terminal_time:
                timing = "during_matrix_window_outside_matrix"
            else:
                timing = "after_terminal_matrix"

    trial_record = None
    trials = map_record.get("trials")
    if isinstance(trials, Mapping) and trial.trial_id in trials:
        candidate = trials[trial.trial_id]
        if isinstance(candidate, Mapping):
            trial_record = candidate
    matrix_route = trial_record.get("route") if trial_record is not None else None
    route_comparison = None
    relation = "matrix_pass"
    if trial.source == "explicit_recovery_for_matrix_failure":
        if matrix_route is None:
            route_comparison = "matrix_failed_route_unavailable"
            relation = "explicit_recovery_route_unavailable"
        elif _route_identity(trial.route) == _route_identity(matrix_route):
            route_comparison = "same_route"
            relation = "explicit_recovery_same_route"
        else:
            route_comparison = "alternate_route"
            relation = "explicit_recovery_alternate_route"
    elif trial.source != "matrix_pass_auto_discovery":
        relation = (
            "explicit_supplement_for_blocked_map"
            if map_record.get("status") == "BLOCKED"
            else "explicit_supplement"
        )
    return {
        "relation": relation,
        "timing": timing,
        "captured_at": captured_at,
        "matrix_plan_generated_at": matrix_record.get("plan_generated_at"),
        "matrix_terminal_generated_at": matrix_record.get("generated_at"),
        "matrix_map_status": map_record.get("status"),
        "matrix_trial_status": (
            trial_record.get("status") if trial_record is not None else None
        ),
        "runtime_profile_selector": matrix_record.get(
            "runtime_profile_selector", "recommended"
        ),
        "speed_contract": (
            dict(trial_record["speed_contract"])
            if trial_record is not None
            and isinstance(trial_record.get("speed_contract"), Mapping)
            else None
        ),
        "route_comparison": route_comparison,
        "matrix_failed_route": (
            dict(matrix_route) if isinstance(matrix_route, Mapping) else None
        ),
    }


def _copy_vad_trial(
    trial: VadTrial,
    staging: Path,
    matrix_record: Mapping[str, Any] | None = None,
    visual_refresh: VadVisualRefresh | None = None,
) -> dict[str, Any]:
    relative_dir = Path(trial.map_id)
    if trial.trial_id is not None:
        relative_dir /= Path("autoware_vad") / trial.trial_id
    output_dir = staging / relative_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    centered_sources: Mapping[str, Any] = {}
    if trial.centered_capture_provenance is not None:
        candidate_sources = trial.centered_capture_provenance.get("source_files")
        if not isinstance(candidate_sources, Mapping):  # pragma: no cover - internal.
            raise PublicationError("centered capture lost its pinned source files")
        centered_sources = candidate_sources

    def centered_source(name: str) -> tuple[Path, str]:
        record = centered_sources.get(name)
        if not isinstance(record, Mapping):
            raise PublicationError(
                f"{trial.map_id}:{trial.trial_id} centered capture lacks {name}"
            )
        source_value = record.get("source")
        expected_sha256 = record.get("sha256")
        expected_source = (trial.directory / name).resolve()
        if (
            not isinstance(source_value, str)
            or Path(source_value).expanduser().resolve() != expected_source
            or not isinstance(expected_sha256, str)
            or re.fullmatch(r"[0-9a-f]{64}", expected_sha256) is None
        ):
            raise PublicationError(
                f"{trial.map_id}:{trial.trial_id} centered capture has invalid "
                f"source provenance for {name}"
            )
        return expected_source, expected_sha256

    sources = (
        trial.directory / "result.json",
        trial.directory / "route_result.png",
        trial.directory / "turn_path_control.gif",
    )
    destinations = tuple(output_dir / name for name in MANAGED_VAD_NAMES)
    for source, destination in zip(sources, destinations):
        shutil.copy2(source, destination)
    relative_files = [
        (relative_dir / name).as_posix() for name in MANAGED_VAD_NAMES
    ]
    file_roles = {
        "result": relative_files[0],
        "route_result": relative_files[1],
        "control_gif": relative_files[2],
    }
    if trial.trial_id is not None:
        if trial.route is None or trial.desktop_capture is None:  # pragma: no cover.
            raise PublicationError("labeled VAD trial lost validated evidence metadata")
        route_destination = output_dir / LABELED_VAD_ROUTE_NAME
        shutil.copy2(Path(trial.route["source"]), route_destination)
        route_relative = (relative_dir / LABELED_VAD_ROUTE_NAME).as_posix()
        relative_files.append(route_relative)
        file_roles["route"] = route_relative
        for name in LABELED_VAD_EVIDENCE_NAMES:
            source = trial.directory / name
            destination = output_dir / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            if name in centered_sources:
                verified_source, expected_sha256 = centered_source(name)
                _copy_verified_source(
                    verified_source,
                    destination,
                    expected_sha256,
                    f"{trial.map_id}:{trial.trial_id} {name}",
                )
            else:
                shutil.copy2(source, destination)
            relative = (relative_dir / name).as_posix()
            relative_files.append(relative)
        file_roles.update(
            {
                "fullscreen": (relative_dir / "autoware_rviz_fullscreen.png").as_posix(),
                "drive_gif": (relative_dir / "autoware_rviz_drive.gif").as_posix(),
                "desktop_capture": (relative_dir / "desktop_capture.json").as_posix(),
                "diagnosis": (relative_dir / "diagnosis.json").as_posix(),
                "path_vs_control": (relative_dir / "path_vs_control.png").as_posix(),
                "steering_tracking": (relative_dir / "steering_tracking.png").as_posix(),
                "latency": (relative_dir / "latency/e2e_latency.json").as_posix(),
            }
        )
    published_speed_profile = None
    if trial.speed_profile is not None:
        speed_files: dict[str, str] = {}
        for name, source_key, sha_key in (
            ("speed_profile.json", "json_source", "json_sha256"),
            ("speed_profile.png", "plot_source", "plot_sha256"),
        ):
            destination = output_dir / name
            _copy_verified_source(
                Path(trial.speed_profile[source_key]),
                destination,
                str(trial.speed_profile[sha_key]),
                f"{trial.map_id}:{trial.trial_id} {name}",
            )
            relative = (relative_dir / name).as_posix()
            relative_files.append(relative)
            speed_files[name] = relative
        file_roles.update(
            {
                "speed_profile": speed_files["speed_profile.json"],
                "speed_plot": speed_files["speed_profile.png"],
            }
        )
        published_speed_profile = {
            **dict(trial.speed_profile),
            "files": speed_files,
        }
    published_centered_capture = None
    if trial.centered_capture_provenance is not None:
        if trial.speed_profile is None:  # pragma: no cover - internal invariant.
            raise PublicationError(
                "centered matrix capture publication requires speed-profile evidence"
            )
        centered_files: dict[str, str] = {}
        for name in SPEED_30KPH_CENTERED_CAPTURE_NAMES:
            source, expected_sha256 = centered_source(name)
            destination = output_dir / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_verified_source(
                source,
                destination,
                expected_sha256,
                f"{trial.map_id}:{trial.trial_id} {name}",
            )
            relative = (relative_dir / name).as_posix()
            relative_files.append(relative)
            centered_files[name] = relative
        file_roles.update(
            {
                "candidate": centered_files["autoware_rviz_candidate.png"],
                "capture_runtime": centered_files["runtime.env"],
                "rviz_config": centered_files[
                    "rviz_capture_provenance/autoware_vad_carla.rviz"
                ],
                "rviz_checksums": centered_files[
                    "rviz_capture_provenance/SHA256SUMS"
                ],
            }
        )
        published_centered_capture = {
            **dict(trial.centered_capture_provenance),
            "published_files": centered_files,
        }
    published_camera_source_provenance = None
    if _is_camera_source_5hz_profile(trial.runtime_profile_selector):
        camera_contract = _camera_source_5hz_contract(
            trial.runtime_profile_selector
        )
        camera_provenance_names = _camera_source_5hz_provenance_names(
            camera_contract
        )
        validation = _read_object(
            trial.directory / "matrix_validation.json",
            f"{trial.map_id}:{trial.trial_id} matrix validation",
        )
        camera_evidence = validation.get("camera_source_contract")
        runtime_profile = validation.get("runtime_profile")
        validated_camera_source = (
            _validated_camera_source_5hz_evidence(
                trial.directory,
                runtime_profile,
                camera_evidence,
                f"{trial.map_id}:{trial.trial_id}",
            )
            if isinstance(runtime_profile, Mapping)
            else None
        )
        if validated_camera_source is None:
            raise PublicationError(
                f"{trial.map_id}:{trial.trial_id} lost freshly validated "
                "camera-source evidence"
            )
        fresh_camera_evidence, raw_provenance = validated_camera_source
        provenance_files: dict[str, dict[str, Any]] = {}
        for name in (
            *camera_provenance_names,
            "runtime.env",
            "latency/e2e_latency.json",
        ):
            source_record = raw_provenance[name]
            source = Path(str(source_record["source"]))
            expected_sha256 = str(source_record["sha256"])
            destination = output_dir / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_verified_source(
                source,
                destination,
                expected_sha256,
                f"{trial.map_id}:{trial.trial_id} {name}",
            )
            relative = (relative_dir / name).as_posix()
            if relative not in relative_files:
                relative_files.append(relative)
            provenance_files[name] = {
                "published_file": relative,
                "sha256": expected_sha256,
                "size_bytes": source_record["size_bytes"],
            }
        file_roles.update(
            {
                "matrix_validation": provenance_files[
                    "matrix_validation.json"
                ]["published_file"],
                "stack_log": provenance_files["stack.log"][
                    "published_file"
                ],
                "recorder_log": provenance_files["recorder.log"][
                    "published_file"
                ],
                "launch_args": provenance_files["launch_args.txt"][
                    "published_file"
                ],
                "route_manager_parameters": provenance_files[
                    "vad_route_manager.params.yaml"
                ]["published_file"],
                "sensor_mapping": provenance_files[
                    "sensor_mapping_provenance/sensor_mapping.yaml"
                ]["published_file"],
                "sensor_mapping_checksums": provenance_files[
                    "sensor_mapping_provenance/SHA256SUMS"
                ]["published_file"],
            }
        )
        if camera_contract == CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_CONTRACT:
            file_roles.update(
                {
                    "vad_model_override": provenance_files[
                        "vad_model_override_provenance/model_override.param.yaml"
                    ]["published_file"],
                    "vad_model_override_checksums": provenance_files[
                        "vad_model_override_provenance/SHA256SUMS"
                    ]["published_file"],
                }
            )
        published_camera_source_provenance = {
            "status": "PASS",
            "contract": camera_contract,
            "validation": fresh_camera_evidence,
            "files": provenance_files,
        }
    refresh_record = None
    published_capture = trial.desktop_capture
    if visual_refresh is not None:
        if (
            trial.trial_id is None
            or (trial.map_id, trial.trial_id)
            != (visual_refresh.map_id, visual_refresh.trial_id)
        ):  # pragma: no cover - caller constructs the identity mapping.
            raise PublicationError("VAD visual refresh publication identity mismatch")
        refresh_archive_dir = output_dir / "visual_refresh"
        refresh_archive_dir.mkdir(parents=True, exist_ok=True)
        original_capture_destination = (
            refresh_archive_dir / "original_validation_desktop_capture.json"
        )
        shutil.copy2(trial.directory / "desktop_capture.json", original_capture_destination)
        original_capture_relative = (
            relative_dir
            / "visual_refresh"
            / "original_validation_desktop_capture.json"
        ).as_posix()
        relative_files.append(original_capture_relative)

        for name in VISUAL_REFRESH_MEDIA_NAMES:
            source = visual_refresh.directory / name
            destination = output_dir / name
            shutil.copy2(source, destination)
            relative = (relative_dir / name).as_posix()
            if relative not in relative_files:
                relative_files.append(relative)
        file_roles["candidate"] = (
            relative_dir / "autoware_rviz_candidate.png"
        ).as_posix()

        archived_roles: dict[str, str] = {}
        for name in VISUAL_REFRESH_ARCHIVE_NAMES:
            source = visual_refresh.directory / name
            archive_name = "repeat_result.json" if name == "result.json" else name
            destination = refresh_archive_dir / archive_name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            relative = (relative_dir / "visual_refresh" / archive_name).as_posix()
            relative_files.append(relative)
            archived_roles[name] = relative
        published_capture = visual_refresh.desktop_capture
        refresh_record = {
            "scope": (
                "same-route repeated full-stack PASS used only to refresh "
                "vehicle-centered RViz media; original validation provenance and "
                "result remain authoritative"
            ),
            "status": "PASS",
            "relation": "same_route_vehicle_centered_visual_refresh",
            "refresh_trial_directory": str(visual_refresh.directory),
            "original_validation_trial_directory": str(trial.directory),
            "result_sha256": _sha256(visual_refresh.directory / "result.json"),
            "source_route_sha256": visual_refresh.source_route_sha256,
            "validation_source_route_sha256": (
                visual_refresh.validation_source_route_sha256
            ),
            "route_match": "exact_payload_and_sha256",
            "runtime_profile": dict(visual_refresh.runtime_profile),
            "desktop_capture": dict(visual_refresh.desktop_capture),
            "rviz_view_contract": dict(
                visual_refresh.desktop_capture["rviz_view_contract"]
            ),
            "original_desktop_capture_sha256": _sha256(
                trial.directory / "desktop_capture.json"
            ),
            "original_validation_media_sha256": {
                name: _sha256(trial.directory / name)
                for name in (
                    "autoware_rviz_fullscreen.png",
                    "autoware_rviz_drive.gif",
                )
            },
            "refresh_media_sha256": {
                name: _sha256(visual_refresh.directory / name)
                for name in VISUAL_REFRESH_MEDIA_NAMES
            },
            "archived_original_desktop_capture_file": original_capture_relative,
            "archived_files": archived_roles,
            "published_media": {
                name: (relative_dir / name).as_posix()
                for name in VISUAL_REFRESH_MEDIA_NAMES
            },
        }
    return {
        "map_id": trial.map_id,
        "trial_id": trial.trial_id,
        "trial_directory": str(trial.directory),
        "success": True,
        "execution_mode": "full_stack",
        "planning_architecture": "vad_route_manager_hybrid",
        "evidence_interpretation": (
            _speed_30kph_publication_interpretation(
                trial.runtime_profile_selector
            )
            if trial.speed_profile is not None
            else None
        ),
        "source": trial.source,
        "matrix_context": _matrix_publication_context(trial, matrix_record),
        "route": dict(trial.route) if trial.route is not None else None,
        "desktop_capture": (
            dict(published_capture)
            if published_capture is not None
            else None
        ),
        "validation_desktop_capture": (
            dict(trial.desktop_capture)
            if trial.desktop_capture is not None
            else None
        ),
        "speed_profile": published_speed_profile,
        "centered_capture_provenance": published_centered_capture,
        "camera_source_provenance": published_camera_source_provenance,
        "visual_refresh": refresh_record,
        "file_roles": file_roles,
        "files": relative_files,
    }


def _copy_ime_proof(proof: ImeProof, staging: Path) -> dict[str, Any]:
    destinations = tuple(staging / name for name in MANAGED_IME_NAMES)
    for source, destination in zip((proof.before, proof.after), destinations):
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    return {
        "scope": "VS Code Korean IME desktop proof; not driving evidence",
        "before_source": str(proof.before),
        "after_source": str(proof.after),
        "before_dimensions": list(proof.before_size),
        "after_dimensions": list(proof.after_size),
        "files": list(MANAGED_IME_NAMES),
    }


def _copy_owned_window_visual_audit(
    audit: Mapping[str, Any], staging: Path
) -> dict[str, Any]:
    copied_files: dict[str, dict[str, Any]] = {}
    for name, record in audit["files"].items():
        if not isinstance(record, Mapping):  # pragma: no cover - validated above.
            raise PublicationError("owned-window visual audit lost file provenance")
        destination = staging / str(record["published_file"])
        destination.parent.mkdir(parents=True, exist_ok=True)
        _copy_verified_source(
            Path(str(record["source"])),
            destination,
            str(record["sha256"]),
            f"owned-window visual audit {name}",
        )
        copied_files[str(name)] = dict(record)
    return {
        "status": audit["status"],
        "source_directory": audit["source_directory"],
        "counts": dict(audit["counts"]),
        "contact_sheet_dimensions": list(audit["contact_sheet_dimensions"]),
        "files": copied_files,
    }


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _safe_publication_relative_file(value: Any) -> str | None:
    if not isinstance(value, str) or not value or "\x00" in value:
        return None
    relative = Path(value)
    if relative.is_absolute() or value == "." or ".." in relative.parts:
        return None
    return relative.as_posix()


def _publish_staging(staging: Path, destination: Path, map_ids: Sequence[str]) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    staged_files = {
        path.relative_to(staging).as_posix(): path
        for path in staging.rglob("*")
        if path.is_file()
    }
    managed_candidates = {
        f"{map_id}/{name}"
        for map_id in map_ids
        for name in MANAGED_BASICAGENT_NAMES + MANAGED_VAD_NAMES
    }
    managed_candidates.update(MANAGED_IME_NAMES)
    previous_manifest_path = destination / "publication_manifest.json"
    if previous_manifest_path.is_file():
        try:
            previous = _read_object(previous_manifest_path, "previous publication")
            managed_candidates.update(
                relative
                for value in previous.get("generated_files", [])
                if (relative := _safe_publication_relative_file(value)) is not None
            )
        except PublicationError:
            pass
    destination_resolved = destination.resolve()
    for relative in sorted(managed_candidates - set(staged_files)):
        target = (destination / relative).resolve()
        if not target.is_relative_to(destination_resolved):
            raise PublicationError(
                f"refusing to remove a publication path outside {destination_resolved}"
            )
        if target.is_file() or target.is_symlink():
            target.unlink()
    for relative, source in sorted(staged_files.items()):
        target = (destination / relative).resolve()
        if not target.is_relative_to(destination_resolved):
            raise PublicationError(
                f"refusing to publish outside {destination_resolved}"
            )
        target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(source, target)


def _copy_verified_source(
    source: Path, destination: Path, expected_sha256: str, label: str
) -> None:
    shutil.copy2(source, destination)
    if _sha256(destination) != expected_sha256:
        raise PublicationError(f"{label} changed during publication")


def _published_runtime_profile_selector(
    manifest: Mapping[str, Any],
) -> str | None:
    matrix = manifest.get("autoware_vad_matrix_source")
    if matrix is None:
        return None
    if not isinstance(matrix, Mapping):
        raise PublicationError(
            "previous publication has an invalid Autoware VAD matrix record"
        )
    selector = matrix.get("runtime_profile_selector")
    if selector is None:
        profile = matrix.get("runtime_profile")
        options = profile.get("wrapper_options") if isinstance(profile, Mapping) else None
        if options == VAD_RUNTIME_WRAPPER_OPTIONS["recommended"]:
            return "recommended"
        raise PublicationError(
            "previous publication matrix has no identifiable runtime profile"
        )
    if selector not in VAD_RUNTIME_PROFILE_SELECTORS:
        raise PublicationError(
            "previous publication matrix has an unknown runtime profile"
        )
    return str(selector)


def _protect_existing_publication_profile(
    destination: Path, requested_selector: str
) -> None:
    manifest_path = destination / "publication_manifest.json"
    if not manifest_path.is_file():
        return
    previous = _read_object(manifest_path, "previous publication")
    previous_selector = _published_runtime_profile_selector(previous)
    if previous_selector != requested_selector and (
        _is_speed_30kph_profile(requested_selector)
        or (
            previous_selector is not None
            and _is_speed_30kph_profile(previous_selector)
        )
    ):
        raise PublicationError(
            "refusing to overwrite an existing publication with a different runtime "
            "profile; use a new dated docs/assets destination"
        )


def _load_report_preamble(path: Path | None) -> dict[str, str] | None:
    if path is None:
        return None
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise PublicationError(f"report preamble is not a file: {resolved}")
    try:
        content = resolved.read_text(encoding="utf-8")
    except UnicodeDecodeError as error:
        raise PublicationError(
            f"report preamble is not valid UTF-8: {resolved}"
        ) from error
    content = content.strip()
    if not content:
        raise PublicationError(f"report preamble is empty: {resolved}")
    if "\x00" in content:
        raise PublicationError(f"report preamble contains a NUL byte: {resolved}")
    if any(line.strip().startswith("# ") for line in content.splitlines()):
        raise PublicationError(
            "report preamble must not add a level-one heading"
        )
    return {
        "source": str(resolved),
        "sha256": _sha256(resolved),
        "content": content,
        "validation_scope": (
            "UTF-8, non-empty, no level-one heading, SHA256-bound; "
            "operator-reviewed narrative, not semantically generated"
        ),
    }


def publish_assets(
    artifact_root: Path,
    docs_assets_root: Path,
    report_path: Path | None = None,
    report_preamble_path: Path | None = None,
    owned_window_visual_audit_dir: Path | None = None,
    expected_map_count: int = 19,
    expected_selected_map_count: int = 9,
    vad_trial_paths: Mapping[str, Path] | Sequence[VadTrialSpec] | None = None,
    vad_visual_refresh_paths: Sequence[VadTrialSpec] | None = None,
    vad_matrix_root: Path | None = None,
    vad_runtime_profile_selector: str = "recommended",
    ime_before: Path | None = None,
    ime_after: Path | None = None,
    gif_fps: float = 5.0,
    gif_max_frames: int = 100,
) -> dict[str, Any]:
    if not math.isfinite(gif_fps) or gif_fps <= 0.0:
        raise PublicationError("GIF FPS must be finite and positive")
    if gif_max_frames < 2:
        raise PublicationError("GIF maximum frame count must be at least two")
    if report_preamble_path is not None and report_path is None:
        raise PublicationError("report preamble requires a report path")
    report_preamble = _load_report_preamble(report_preamble_path)
    snapshot = load_sweep_snapshot(
        artifact_root, expected_map_count, expected_selected_map_count
    )
    basic_runs = collect_basicagent_runs(snapshot)
    vad_trial_specs, vad_matrix_record = resolve_vad_trial_specs(
        snapshot,
        vad_trial_paths or (),
        vad_matrix_root,
        vad_runtime_profile_selector,
    )
    if (
        _is_speed_30kph_profile(vad_runtime_profile_selector)
        and vad_visual_refresh_paths
    ):
        raise PublicationError(
            "speed_30kph publication does not accept an unbound visual refresh; "
            "publish the matrix trial's own centered capture"
        )
    vad_trials = collect_vad_trials(snapshot, vad_trial_specs)
    camera_source_diagnostics = collect_camera_source_campaign_diagnostics(
        Path(vad_matrix_record["matrix_root"])
        if vad_matrix_record is not None
        else None,
        vad_runtime_profile_selector,
        (
            vad_matrix_record.get("runtime_profile")
            if vad_matrix_record is not None
            else None
        ),
        vad_trials,
    )
    owned_window_visual_audit = collect_owned_window_visual_audit(
        owned_window_visual_audit_dir,
        {
            (trial.map_id, str(trial.trial_id)): trial.directory
            for trial in vad_trials
            if trial.trial_id is not None
        },
    )
    vad_visual_refreshes = collect_vad_visual_refreshes(
        snapshot, vad_visual_refresh_paths or (), vad_trials
    )
    ime_proof = collect_ime_proof(snapshot, ime_before, ime_after)
    docs_assets_root = docs_assets_root.expanduser().resolve()
    if report_path is not None:
        report_path = report_path.expanduser().resolve()
    _protect_existing_publication_profile(
        docs_assets_root, vad_runtime_profile_selector
    )
    docs_assets_root.parent.mkdir(parents=True, exist_ok=True)
    published_vad_matrix_record = (
        dict(vad_matrix_record) if vad_matrix_record is not None else None
    )

    with tempfile.TemporaryDirectory(
        prefix=".validation_publication_", dir=str(docs_assets_root.parent)
    ) as temporary:
        staging = Path(temporary)
        _copy_verified_source(
            Path(snapshot["aggregate_path"]),
            staging / BASICAGENT_AGGREGATE_NAME,
            str(snapshot["aggregate_sha256"]),
            "CARLA BasicAgent sweep aggregate",
        )
        basic_records = [
            render_basicagent_run(
                run, staging / run.map_id, gif_fps, gif_max_frames
            )
            for run in basic_runs
        ]
        vad_records = [
            _copy_vad_trial(
                trial,
                staging,
                vad_matrix_record,
                vad_visual_refreshes.get((trial.map_id, trial.trial_id)),
            )
            for trial in vad_trials
        ]
        ime_record = _copy_ime_proof(ime_proof, staging) if ime_proof else None
        published_owned_window_visual_audit = (
            _copy_owned_window_visual_audit(owned_window_visual_audit, staging)
            if owned_window_visual_audit is not None
            else None
        )
        published_camera_source_diagnostics = (
            _copy_camera_source_campaign_diagnostics(
                camera_source_diagnostics, staging
            )
            if camera_source_diagnostics is not None
            else None
        )
        render_status_dashboard(snapshot, staging / DASHBOARD_NAME)
        if published_vad_matrix_record is not None:
            _copy_verified_source(
                Path(published_vad_matrix_record["plan"]),
                staging / VAD_MATRIX_PLAN_NAME,
                str(published_vad_matrix_record["plan_sha256"]),
                "Autoware VAD matrix plan",
            )
            _copy_verified_source(
                Path(published_vad_matrix_record["aggregate"]),
                staging / VAD_MATRIX_AGGREGATE_NAME,
                str(published_vad_matrix_record["aggregate_sha256"]),
                "Autoware VAD matrix aggregate",
            )
            published_vad_matrix_record["published_plan_file"] = (
                VAD_MATRIX_PLAN_NAME
            )
            published_vad_matrix_record["published_aggregate_file"] = (
                VAD_MATRIX_AGGREGATE_NAME
            )

        aggregate_link = BASICAGENT_AGGREGATE_NAME
        readme = _markdown(
            snapshot,
            basic_records,
            vad_records,
            ime_record,
            ".",
            aggregate_link,
            published_vad_matrix_record,
            docs_assets_root,
            report_path,
            published_owned_window_visual_audit,
            report_preamble,
            camera_source_diagnostics_record=(
                published_camera_source_diagnostics
            ),
        )
        (staging / "README.md").write_text(readme, encoding="utf-8")

        payload = {
            "schema_version": 1,
            "evidence_boundary": {
                "basicagent": EVIDENCE_KIND,
                "autoware_vad": (
                    "explicit or terminal-matrix-discovered successful "
                    "full-stack trials only"
                ),
                "autoware_vad_visual_refresh": (
                    None
                    if _is_speed_30kph_profile(vad_runtime_profile_selector)
                    else (
                        "same-route recommended-profile repeated full-stack PASS; "
                        "visual media only, original validation result preserved"
                    )
                ),
                "speed_30kph": (
                    _speed_30kph_publication_interpretation(
                        vad_runtime_profile_selector
                    )
                    if _is_speed_30kph_profile(vad_runtime_profile_selector)
                    else None
                ),
            },
            "runtime_profile_selector": vad_runtime_profile_selector,
            "source_aggregate": str(snapshot["aggregate_path"]),
            "source_aggregate_sha256": snapshot["aggregate_sha256"],
            "published_source_aggregate_file": BASICAGENT_AGGREGATE_NAME,
            "source_generated_at": snapshot.get("generated_at"),
            "canonical_map_count": expected_map_count,
            "basicagent_publications": basic_records,
            "autoware_vad_publications": vad_records,
            "autoware_vad_visual_refresh_count": len(vad_visual_refreshes),
            "autoware_vad_matrix_source": published_vad_matrix_record,
            "vscode_ime_proof": ime_record,
            "owned_window_visual_audit": published_owned_window_visual_audit,
            "camera_source_campaign_diagnostics": (
                published_camera_source_diagnostics
            ),
            "report_preamble": (
                {
                    "source": report_preamble["source"],
                    "sha256": report_preamble["sha256"],
                    "validation_scope": report_preamble["validation_scope"],
                    "embedding_requested": True,
                    "report_target": str(report_path),
                }
                if report_preamble is not None
                else None
            ),
        }
        current_files = sorted(
            path.relative_to(staging).as_posix()
            for path in staging.rglob("*")
            if path.is_file()
        )
        payload["generated_files"] = current_files + [
            "publication_manifest.json",
            "SHA256SUMS",
        ]
        manifest_path = staging / "publication_manifest.json"
        manifest_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        checksum_paths = sorted(
            path for path in staging.rglob("*") if path.is_file()
        )
        checksums = "".join(
            f"{_sha256(path)}  {path.relative_to(staging).as_posix()}\n"
            for path in checksum_paths
        )
        (staging / "SHA256SUMS").write_text(checksums, encoding="utf-8")
        _publish_staging(
            staging,
            docs_assets_root,
            [entry["map_id"] for entry in snapshot["maps"]],
        )

    if report_path is not None:
        report_path = report_path.expanduser().resolve()
        link_prefix = _relative_link(docs_assets_root, report_path.parent)
        aggregate_link = (
            f"{link_prefix.rstrip('/')}/{BASICAGENT_AGGREGATE_NAME}"
            if link_prefix not in ("", ".")
            else BASICAGENT_AGGREGATE_NAME
        )
        report = _markdown(
            snapshot,
            basic_records,
            vad_records,
            ime_record,
            link_prefix,
            aggregate_link,
            published_vad_matrix_record,
            docs_assets_root,
            report_path,
            published_owned_window_visual_audit,
            report_preamble,
            camera_source_diagnostics_record=(
                published_camera_source_diagnostics
            ),
            embed_report_preamble=True,
        )
        _atomic_write(report_path, report)
    return payload


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--docs-assets-root", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--report-preamble",
        type=Path,
        help=(
            "UTF-8 Markdown fragment inserted below the generated report title; "
            "the fragment path and digest are recorded for reproducible rebuilds"
        ),
    )
    parser.add_argument(
        "--owned-window-visual-audit-dir",
        type=Path,
        help=(
            "directory containing the complete centered-window audit JSON, "
            "Markdown, operator review, and contact sheet"
        ),
    )
    parser.add_argument("--expected-map-count", type=int, default=19)
    parser.add_argument("--expected-selected-map-count", type=int, default=9)
    parser.add_argument(
        "--vad-trial",
        action="append",
        default=[],
        metavar="MAP_ID[:TRIAL_ID]=DIR",
    )
    parser.add_argument(
        "--vad-matrix-root",
        type=Path,
        help=(
            "auto-discover every PASS straight/turn attempt from a terminal "
            "Autoware VAD Town matrix"
        ),
    )
    parser.add_argument(
        "--vad-runtime-profile-selector",
        choices=VAD_RUNTIME_PROFILE_SELECTORS,
        default="recommended",
        help=(
            "expected immutable matrix runtime profile; speed_30kph is an "
            "explicit opt-in simulation-screening publication"
        ),
    )
    parser.add_argument(
        "--vad-visual-refresh",
        action="append",
        default=[],
        metavar="MAP_ID:TRIAL_ID=DIR",
        help=(
            "replace only the published RViz PNG/GIF/candidate media with a "
            "validated same-route vehicle-centered repeated PASS"
        ),
    )
    parser.add_argument("--vscode-ime-before", type=Path)
    parser.add_argument("--vscode-ime-after", type=Path)
    parser.add_argument("--gif-fps", type=float, default=5.0)
    parser.add_argument("--gif-max-frames", type=int, default=100)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate aggregate/status and raw evidence without writing docs/assets",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        vad_trials = _parse_vad_trial_specs(args.vad_trial)
        vad_visual_refreshes = _parse_vad_visual_refresh_specs(
            args.vad_visual_refresh
        )
        if args.expected_map_count < 1 or args.expected_selected_map_count < 1:
            raise PublicationError("expected map counts must be positive")
        if args.report_preamble is not None and args.report is None:
            raise PublicationError("--report-preamble requires --report")
        report_preamble = _load_report_preamble(args.report_preamble)
        if args.dry_run:
            snapshot = load_sweep_snapshot(
                args.artifact_root,
                args.expected_map_count,
                args.expected_selected_map_count,
            )
            basic = collect_basicagent_runs(snapshot)
            vad_trial_specs, matrix_record = resolve_vad_trial_specs(
                snapshot,
                vad_trials,
                args.vad_matrix_root,
                args.vad_runtime_profile_selector,
            )
            if (
                _is_speed_30kph_profile(args.vad_runtime_profile_selector)
                and vad_visual_refreshes
            ):
                raise PublicationError(
                    "speed_30kph publication does not accept an unbound visual refresh"
                )
            vad = collect_vad_trials(snapshot, vad_trial_specs)
            camera_diagnostics = collect_camera_source_campaign_diagnostics(
                Path(matrix_record["matrix_root"])
                if matrix_record is not None
                else None,
                args.vad_runtime_profile_selector,
                (
                    matrix_record.get("runtime_profile")
                    if matrix_record is not None
                    else None
                ),
                vad,
            )
            visual_audit = collect_owned_window_visual_audit(
                args.owned_window_visual_audit_dir,
                {
                    (trial.map_id, str(trial.trial_id)): trial.directory
                    for trial in vad
                    if trial.trial_id is not None
                },
            )
            visual_refreshes = collect_vad_visual_refreshes(
                snapshot, vad_visual_refreshes, vad
            )
            ime = collect_ime_proof(
                snapshot, args.vscode_ime_before, args.vscode_ime_after
            )
            print(
                f"PASS dry-run maps={len(snapshot['maps'])} "
                f"basicagent_publications={len(basic)} autoware_vad_publications={len(vad)} "
                f"vad_matrix={matrix_record is not None} "
                f"runtime_profile={args.vad_runtime_profile_selector} "
                f"vad_visual_refreshes={len(visual_refreshes)} "
                f"vscode_ime_proof={ime is not None} "
                f"report_preamble={report_preamble is not None} "
                f"owned_window_visual_audit={visual_audit is not None} "
                f"camera_source_diagnostics={camera_diagnostics is not None}"
            )
            return 0
        payload = publish_assets(
            args.artifact_root,
            args.docs_assets_root,
            report_path=args.report,
            report_preamble_path=args.report_preamble,
            owned_window_visual_audit_dir=args.owned_window_visual_audit_dir,
            expected_map_count=args.expected_map_count,
            expected_selected_map_count=args.expected_selected_map_count,
            vad_trial_paths=vad_trials,
            vad_visual_refresh_paths=vad_visual_refreshes,
            vad_matrix_root=args.vad_matrix_root,
            vad_runtime_profile_selector=args.vad_runtime_profile_selector,
            ime_before=args.vscode_ime_before,
            ime_after=args.vscode_ime_after,
            gif_fps=args.gif_fps,
            gif_max_frames=args.gif_max_frames,
        )
        print(
            f"PASS docs_assets={args.docs_assets_root.resolve()} "
            f"basicagent_publications={len(payload['basicagent_publications'])} "
            f"autoware_vad_publications={len(payload['autoware_vad_publications'])}"
        )
    except (OSError, PublicationError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=os.sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
