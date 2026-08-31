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

from PIL import Image, ImageDraw, ImageFont

try:
    from carla_basicagent_sweep_report import (
        EVIDENCE_KIND,
        SUCCESS_STATUSES,
        validated_job,
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
    source: str


@dataclass(frozen=True)
class VadTrialSpec:
    map_id: str
    trial_id: str | None
    directory: Path
    source: str = "explicit"


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


def discover_vad_matrix_trial_specs(
    snapshot: Mapping[str, Any], matrix_root: Path
) -> tuple[list[VadTrialSpec], dict[str, Any]]:
    """Discover only status-backed PASS trials from a finished Town matrix.

    Discovery is intentionally read-only.  The matrix runner performs its own
    strict validation and records the selected attempt in each map status;
    this function checks that status/aggregate/validation chain afresh before
    handing the attempt to the publication validator.
    """
    artifact_root = Path(snapshot["artifact_root"])
    matrix_root = _inside(artifact_root, matrix_root, "Autoware VAD matrix root")
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

    runtime_profile = plan.get("runtime_profile")
    expected_options = ["--recommended", "--visualize", "--capture-desktop"]
    if (
        not isinstance(runtime_profile, Mapping)
        or runtime_profile.get("wrapper_options") != expected_options
        or runtime_profile.get("client_map_loading_allowed") is not False
        or aggregate.get("runtime_profile") != runtime_profile
    ):
        raise PublicationError(
            "Autoware VAD matrix did not use the fixed recommended visualized profile"
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
            specs.append(
                VadTrialSpec(
                    map_id=map_id,
                    trial_id=trial_id,
                    directory=attempt,
                    source="matrix_pass_auto_discovery",
                )
            )
            matrix_trial_records[trial_id] = {
                "trial_id": trial_id,
                "status": "PASS",
                "reason": trial_reason,
                "attempt_directory": str(attempt),
                "route": None,
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
        "runtime_profile": dict(runtime_profile),
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
) -> tuple[list[VadTrialSpec], dict[str, Any] | None]:
    explicit = _normalize_vad_trial_specs(explicit_values)
    discovered: list[VadTrialSpec] = []
    matrix_record = None
    if matrix_root is not None:
        discovered, matrix_record = discover_vad_matrix_trial_specs(
            snapshot, matrix_root
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
    start = route.get("start_spawn_index")
    goal = route.get("goal_spawn_index")
    length = route.get("route_length_m")
    if town != canonical_name:
        raise PublicationError(
            f"{label} route town {town!r} does not match {canonical_name!r}"
        )
    if not isinstance(scenario, str) or not scenario.strip():
        raise PublicationError(f"{label} route has no scenario")
    if (
        not isinstance(start, int)
        or isinstance(start, bool)
        or not isinstance(goal, int)
        or isinstance(goal, bool)
    ):
        raise PublicationError(f"{label} route has invalid spawn indices")
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
        "start_spawn_index": start,
        "goal_spawn_index": goal,
        "route_length_m": float(length),
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
        directory = _inside(artifact_root, spec.directory, "Autoware VAD trial")
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
        trials.append(
            VadTrial(
                map_id=map_id,
                trial_id=trial_id,
                directory=directory,
                result=result,
                route=route,
                desktop_capture=desktop_capture,
                source=spec.source,
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


def _validate_centered_capture(
    directory: Path, capture: Mapping[str, Any], label: str
) -> tuple[dict[str, Any], dict[str, str]]:
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
            f"{label} centered candidate still must cover the full display"
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
    if offset < 0.0 or offset > duration or abs(offset - duration / 2.0) > 2.0:
        raise PublicationError(
            f"{label} representative frame is not near the recording midpoint"
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
) -> str:
    basic_by_map = {record["map_id"]: record for record in basic_records}
    reproduction_parts = [
        "python3 scripts/e2e/publish_validation_assets.py",
        f"  --artifact-root {snapshot['artifact_root']}",
        "  --docs-assets-root docs/assets/validation/2026-08-31",
        "  --report docs/validation-2026-08-31.md",
    ]
    if vad_matrix_record is not None:
        reproduction_parts.append(
            f"  --vad-matrix-root {vad_matrix_record['matrix_root']}"
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
    ]
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
                "PASS trials are revalidated from status + `matrix_validation.json`. "
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
                            f" (matrix route {route['start_spawn_index']}→"
                            f"{route['goal_spawn_index']}, {route['scenario']})"
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
                "full-display Autoware/RViz capture and recorded route-analysis bundle.",
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
                    "Result | Full display | Drive GIF | Route/control analysis |",
                    "|---|---|---|---|---|---|---|---|---|",
                ]
            )
        else:
            lines.extend(
                [
                    "| Map | Trial | Route | Result | Full display | Drive GIF | "
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
                    f"`{route['start_spawn_index']}→{route['goal_spawn_index']}` "
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
                        f" (failed matrix route {failed_route['start_spawn_index']}→"
                        f"{failed_route['goal_spawn_index']}, "
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
            "Run `sha256sum -c SHA256SUMS` in this asset directory. Rebuild with:",
            "",
            "```bash",
            reproduction_command,
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def _route_identity(route: Mapping[str, Any] | None) -> tuple[Any, ...] | None:
    if route is None:
        return None
    return (
        route.get("town"),
        route.get("scenario"),
        route.get("start_spawn_index"),
        route.get("goal_spawn_index"),
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
                str(value)
                for value in previous.get("generated_files", [])
                if isinstance(value, str) and ".." not in Path(value).parts
            )
        except PublicationError:
            pass
    for relative in sorted(managed_candidates - set(staged_files)):
        target = destination / relative
        if target.is_file() or target.is_symlink():
            target.unlink()
    for relative, source in sorted(staged_files.items()):
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(source, target)


def _copy_verified_source(
    source: Path, destination: Path, expected_sha256: str, label: str
) -> None:
    shutil.copy2(source, destination)
    if _sha256(destination) != expected_sha256:
        raise PublicationError(f"{label} changed during publication")


def publish_assets(
    artifact_root: Path,
    docs_assets_root: Path,
    report_path: Path | None = None,
    expected_map_count: int = 19,
    expected_selected_map_count: int = 9,
    vad_trial_paths: Mapping[str, Path] | Sequence[VadTrialSpec] | None = None,
    vad_visual_refresh_paths: Sequence[VadTrialSpec] | None = None,
    vad_matrix_root: Path | None = None,
    ime_before: Path | None = None,
    ime_after: Path | None = None,
    gif_fps: float = 5.0,
    gif_max_frames: int = 100,
) -> dict[str, Any]:
    if not math.isfinite(gif_fps) or gif_fps <= 0.0:
        raise PublicationError("GIF FPS must be finite and positive")
    if gif_max_frames < 2:
        raise PublicationError("GIF maximum frame count must be at least two")
    snapshot = load_sweep_snapshot(
        artifact_root, expected_map_count, expected_selected_map_count
    )
    basic_runs = collect_basicagent_runs(snapshot)
    vad_trial_specs, vad_matrix_record = resolve_vad_trial_specs(
        snapshot, vad_trial_paths or (), vad_matrix_root
    )
    vad_trials = collect_vad_trials(snapshot, vad_trial_specs)
    vad_visual_refreshes = collect_vad_visual_refreshes(
        snapshot, vad_visual_refresh_paths or (), vad_trials
    )
    ime_proof = collect_ime_proof(snapshot, ime_before, ime_after)
    docs_assets_root = docs_assets_root.expanduser().resolve()
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
                    "same-route recommended-profile repeated full-stack PASS; "
                    "visual media only, original validation result preserved"
                ),
            },
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
        )
        _atomic_write(report_path, report)
    return payload


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--docs-assets-root", type=Path, required=True)
    parser.add_argument("--report", type=Path)
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
        if args.dry_run:
            snapshot = load_sweep_snapshot(
                args.artifact_root,
                args.expected_map_count,
                args.expected_selected_map_count,
            )
            basic = collect_basicagent_runs(snapshot)
            vad_trial_specs, matrix_record = resolve_vad_trial_specs(
                snapshot, vad_trials, args.vad_matrix_root
            )
            vad = collect_vad_trials(snapshot, vad_trial_specs)
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
                f"vad_visual_refreshes={len(visual_refreshes)} "
                f"vscode_ime_proof={ime is not None}"
            )
            return 0
        payload = publish_assets(
            args.artifact_root,
            args.docs_assets_root,
            report_path=args.report,
            expected_map_count=args.expected_map_count,
            expected_selected_map_count=args.expected_selected_map_count,
            vad_trial_paths=vad_trials,
            vad_visual_refresh_paths=vad_visual_refreshes,
            vad_matrix_root=args.vad_matrix_root,
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
