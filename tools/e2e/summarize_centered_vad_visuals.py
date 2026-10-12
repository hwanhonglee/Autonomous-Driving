#!/usr/bin/env python3
"""Audit same-route centered RViz reruns and render a review contact sheet."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
from typing import Any, Mapping, Sequence

from PIL import Image, ImageChops, ImageDraw, ImageFont


TRIAL_IDS = ("straight", "turn")
ATTEMPT_PATTERN = re.compile(r"attempt_([0-9]+)")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
REQUIRED_PATH_TOPICS = {
    "/planning/trajectory",
    "/planning/vad/candidate_trajectories",
    "/planning/vad_route/actual_path",
    "/planning/vad_route/reference_path",
    "/planning/vad_route/selected_raw_trajectory",
}
REQUIRED_FILES = (
    "result.json",
    "source_route.json",
    "runtime.env",
    "desktop_capture.json",
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_candidate.png",
    "autoware_rviz_drive.gif",
    "autoware_rviz_capture.mkv",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
PUBLICATION_SELECTION_SCOPE = (
    "autoware_vad_publications selected map_id/trial_id/trial_directory/source "
    "records; not a full-file publication manifest digest"
)
PUBLICATION_SELECTION_CANONICALIZATION = (
    "UTF-8 JSON with sort_keys=True, separators=(',', ':'), allow_nan=False"
)


class AuditError(RuntimeError):
    """Raised when centered visual evidence fails its contract."""


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AuditError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(payload, dict):
        raise AuditError(f"{label} must be a JSON object: {path}")
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise AuditError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _timestamp(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.strip():
        raise AuditError(f"{label} must be a non-empty ISO-8601 timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise AuditError(f"{label} is not an ISO-8601 timestamp: {value!r}") from error
    if parsed.tzinfo is None:
        raise AuditError(f"{label} must include a timezone: {value!r}")
    return parsed


def _finite(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AuditError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted) or (positive and converted <= 0.0):
        qualifier = "positive " if positive else ""
        raise AuditError(f"{label} must be a {qualifier}finite number")
    return converted


def _dimensions(value: Any, label: str) -> tuple[int, int]:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(
            isinstance(item, bool) or not isinstance(item, int) or item <= 0
            for item in value
        )
    ):
        raise AuditError(f"{label} must be [positive_width, positive_height]")
    return value[0], value[1]


def _image_info(path: Path, expected_format: str, label: str) -> dict[str, Any]:
    try:
        with Image.open(path) as image:
            actual_format = image.format
            dimensions = image.size
            frame_count = int(getattr(image, "n_frames", 1))
            image.seek(frame_count - 1)
            image.load()
    except (OSError, ValueError) as error:
        raise AuditError(f"cannot decode {label} {path}: {error}") from error
    if actual_format != expected_format:
        raise AuditError(
            f"{label} must be {expected_format}, got {actual_format!r}: {path}"
        )
    return {
        "dimensions": list(dimensions),
        "format": actual_format,
        "frame_count": frame_count,
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
    }


def _runtime_environment(path: Path, label: str) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise AuditError(f"cannot read {label} {path}: {error}") from error
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
            raise AuditError(
                f"{label} has an invalid or duplicate entry on line {line_number}"
            )
        environment[key] = value
    return environment


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


def _attempt_number(path: Path) -> int:
    match = ATTEMPT_PATTERN.fullmatch(path.name)
    if match is None:
        raise AuditError(f"invalid attempt directory name: {path}")
    return int(match.group(1))


def _manifest_records(
    manifest_path: Path, expected_map_count: int
) -> dict[tuple[str, str], dict[str, Any]]:
    manifest = _read_json_object(manifest_path, "publication manifest")
    publications = manifest.get("autoware_vad_publications")
    if not isinstance(publications, list):
        raise AuditError("publication manifest has no Autoware VAD publication list")
    selected: dict[tuple[str, str], dict[str, Any]] = {}
    for raw_record in publications:
        if not isinstance(raw_record, dict) or raw_record.get("trial_id") not in TRIAL_IDS:
            continue
        map_id = raw_record.get("map_id")
        trial_id = raw_record.get("trial_id")
        if not isinstance(map_id, str):
            raise AuditError("publication manifest has a VAD record without map_id")
        identity = (map_id, trial_id)
        if identity in selected:
            raise AuditError(f"duplicate publication record for {map_id}:{trial_id}")
        selected[identity] = raw_record
    maps = {map_id for map_id, _ in selected}
    expected_identities = {
        (map_id, trial_id) for map_id in maps for trial_id in TRIAL_IDS
    }
    if len(maps) != expected_map_count or set(selected) != expected_identities:
        raise AuditError(
            "publication manifest must select straight and turn for exactly "
            f"{expected_map_count} maps; got {len(maps)} maps/{len(selected)} trials"
        )
    return selected


def _publication_selection(
    manifest_path: Path, expected_map_count: int
) -> dict[str, Any]:
    """Bind only the stable manifest projection consumed by the visual audit.

    The final publication manifest embeds the completed visual audit, so hashing the
    whole input manifest here would create a circular and inevitably stale reference.
    """
    manifest_path = manifest_path.expanduser().resolve()
    selected = _manifest_records(manifest_path, expected_map_count)
    records: list[dict[str, str]] = []
    for map_id, trial_id in sorted(selected):
        record = selected[(map_id, trial_id)]
        directory_value = record.get("trial_directory")
        source = record.get("source")
        if not isinstance(directory_value, str) or not directory_value:
            raise AuditError(
                f"{map_id}:{trial_id} publication record has no selected original"
            )
        if not isinstance(source, str) or not source:
            raise AuditError(
                f"{map_id}:{trial_id} publication record has no selection source"
            )
        directory = Path(directory_value).expanduser()
        if not directory.is_absolute():
            directory = manifest_path.parent / directory
        records.append(
            {
                "map_id": map_id,
                "trial_id": trial_id,
                "trial_directory": str(directory.resolve()),
                "source": source,
            }
        )
    encoded = json.dumps(
        records, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return {
        "selection_source_file": str(manifest_path),
        "scope": PUBLICATION_SELECTION_SCOPE,
        "canonicalization": PUBLICATION_SELECTION_CANONICALIZATION,
        "record_count": len(records),
        "records": records,
        "sha256": hashlib.sha256(encoded).hexdigest(),
    }


def _select_latest_successful_attempt(
    trial_root: Path, label: str
) -> tuple[Path, list[dict[str, Any]]]:
    attempts = sorted(
        (path for path in trial_root.glob("attempt_*") if path.is_dir()),
        key=_attempt_number,
    )
    if not attempts:
        raise AuditError(f"{label} has no attempt directories: {trial_root}")
    candidates: list[Path] = []
    history: list[dict[str, Any]] = []
    for attempt in attempts:
        result_path = attempt / "result.json"
        result: dict[str, Any] | None = None
        error_text = None
        try:
            result = _read_json_object(result_path, f"{label} result")
        except AuditError as error:
            error_text = str(error)
        passed = result is not None and _full_stack_result_pass(result)
        history.append(
            {
                "attempt": attempt.name,
                "attempt_number": _attempt_number(attempt),
                "full_stack_result_pass": passed,
                "desktop_capture_present": (attempt / "desktop_capture.json").is_file(),
                "error": error_text,
            }
        )
        if passed:
            candidates.append(attempt)
    if not candidates:
        raise AuditError(f"{label} has no strict successful attempt")
    return max(candidates, key=_attempt_number), history


def _verify_extracted_frame(
    video_path: Path,
    representative_path: Path,
    offset_sec: float,
    label: str,
) -> dict[str, Any]:
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise AuditError("frame verification requires ffmpeg and ffprobe")
    probe = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "default=nw=1:nk=1",
            str(video_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if probe.returncode != 0:
        raise AuditError(f"{label} ffprobe failed: {probe.stderr.strip()}")
    try:
        probed_duration = float(probe.stdout.strip())
    except ValueError as error:
        raise AuditError(f"{label} ffprobe returned invalid duration") from error
    if not math.isfinite(probed_duration) or probed_duration <= 0.0:
        raise AuditError(f"{label} source recording has invalid duration")

    with tempfile.TemporaryDirectory(prefix="centered_vad_frame_") as temp_directory:
        extracted = Path(temp_directory) / "representative.png"
        command = [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(video_path),
            "-ss",
            f"{offset_sec:.6f}",
            "-frames:v",
            "1",
            "-an",
            str(extracted),
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0 or not extracted.is_file():
            raise AuditError(
                f"{label} representative frame extraction failed: "
                f"{completed.stderr.strip()}"
            )
        try:
            with Image.open(representative_path) as expected_image:
                expected = expected_image.convert("RGB")
            with Image.open(extracted) as actual_image:
                actual = actual_image.convert("RGB")
        except OSError as error:
            raise AuditError(f"{label} cannot compare representative frame") from error
        if expected.size != actual.size or ImageChops.difference(expected, actual).getbbox():
            raise AuditError(
                f"{label} representative PNG does not match its MKV midpoint frame"
            )
    return {
        "status": "PASS",
        "ffmpeg_command": command[:-1] + ["<temporary>/representative.png"],
        "probed_recording_duration_sec": probed_duration,
        "pixel_exact_match": True,
    }


def _validate_capture(
    directory: Path,
    label: str,
    *,
    verify_frame: bool,
) -> dict[str, Any]:
    for relative in REQUIRED_FILES:
        path = directory / relative
        if not path.is_file() or path.stat().st_size <= 0:
            raise AuditError(f"{label} is missing required evidence: {relative}")

    capture = _read_json_object(directory / "desktop_capture.json", f"{label} capture")
    if (
        capture.get("schema_version") != 1
        or capture.get("candidate_observed") is not True
        or capture.get("capture_started_after_candidate") is not True
        or str(capture.get("candidate_topic", "")).lstrip("/")
        != "planning/vad/candidate_trajectories"
    ):
        raise AuditError(f"{label} does not prove post-candidate desktop capture")
    expected_names = {
        "png_file": "autoware_rviz_fullscreen.png",
        "candidate_png_file": "autoware_rviz_candidate.png",
        "gif_file": "autoware_rviz_drive.gif",
        "recording_file": "autoware_rviz_capture.mkv",
    }
    for key, expected in expected_names.items():
        if capture.get(key) != expected:
            raise AuditError(f"{label} capture {key} must be {expected!r}")

    representative_info = _image_info(
        directory / expected_names["png_file"], "PNG", f"{label} representative PNG"
    )
    candidate_info = _image_info(
        directory / expected_names["candidate_png_file"],
        "PNG",
        f"{label} candidate PNG",
    )
    gif_info = _image_info(
        directory / expected_names["gif_file"], "GIF", f"{label} drive GIF"
    )
    source_dimensions = _dimensions(capture.get("source_dimensions"), f"{label} source")
    if tuple(representative_info["dimensions"]) != source_dimensions or tuple(
        candidate_info["dimensions"]
    ) != source_dimensions:
        raise AuditError(f"{label} PNGs do not cover the full captured display")
    if _dimensions(capture.get("png_dimensions"), f"{label} PNG metadata") != tuple(
        representative_info["dimensions"]
    ) or _dimensions(
        capture.get("candidate_png_dimensions"), f"{label} candidate metadata"
    ) != tuple(candidate_info["dimensions"]):
        raise AuditError(f"{label} PNG dimensions disagree with metadata")
    if _dimensions(capture.get("gif_dimensions"), f"{label} GIF metadata") != tuple(
        gif_info["dimensions"]
    ) or gif_info["dimensions"][0] != 960:
        raise AuditError(f"{label} drive GIF must match metadata at width 960")
    if gif_info["frame_count"] <= 1:
        raise AuditError(f"{label} drive GIF is not animated")
    if representative_info["sha256"] == candidate_info["sha256"]:
        raise AuditError(f"{label} route-midpoint PNG equals the initial candidate still")

    contract = capture.get("rviz_view_contract")
    if not isinstance(contract, dict):
        raise AuditError(f"{label} has no RViz view contract")
    center = contract.get("center_xy_m")
    if (
        contract.get("vehicle_centered") is not True
        or contract.get("controller") != "rviz_default_plugins/TopDownOrtho"
        or contract.get("target_frame") != "base_link"
        or not isinstance(center, list)
        or len(center) != 2
        or any(
            not math.isclose(_finite(value, f"{label} center"), 0.0, abs_tol=1e-9)
            for value in center
        )
        or not math.isclose(
            _finite(contract.get("angle_rad"), f"{label} angle"),
            0.0,
            abs_tol=1e-9,
        )
        or not math.isclose(
            _finite(contract.get("scale"), f"{label} scale", positive=True),
            10.0,
            abs_tol=1e-9,
        )
    ):
        raise AuditError(f"{label} RViz view does not center base_link at scale 10")
    visible_topics = contract.get("visible_path_topics")
    if not isinstance(visible_topics, list) or not REQUIRED_PATH_TOPICS.issubset(
        {topic for topic in visible_topics if isinstance(topic, str)}
    ):
        raise AuditError(f"{label} RViz contract omits required path topics")

    config_relative = "rviz_capture_provenance/autoware_vad_carla.rviz"
    config_path = directory / config_relative
    config_sha256 = contract.get("config_sha256")
    if (
        contract.get("config_file") != config_relative
        or not isinstance(config_sha256, str)
        or SHA256_PATTERN.fullmatch(config_sha256) is None
        or _sha256(config_path) != config_sha256
    ):
        raise AuditError(f"{label} pinned RViz config digest mismatch")
    checksum_text = (directory / "rviz_capture_provenance/SHA256SUMS").read_text(
        encoding="utf-8"
    )
    if checksum_text != f"{config_sha256}  autoware_vad_carla.rviz\n":
        raise AuditError(f"{label} RViz SHA256SUMS mismatch")

    representative = capture.get("representative_frame")
    if not isinstance(representative, dict):
        raise AuditError(f"{label} has no representative-frame metadata")
    if (
        representative.get("source") != expected_names["recording_file"]
        or representative.get("selection") != "route_evaluation_midpoint"
    ):
        raise AuditError(f"{label} representative frame is not the route midpoint")
    offset = _finite(representative.get("offset_sec"), f"{label} frame offset")
    duration = _finite(
        representative.get("recording_duration_sec"),
        f"{label} recording duration",
        positive=True,
    )
    if not 0.0 < offset < duration:
        raise AuditError(f"{label} representative offset is outside the recording")

    candidate_at = _timestamp(capture.get("candidate_observed_at"), f"{label} candidate")
    candidate_still_at = _timestamp(
        capture.get("candidate_still_captured_at"), f"{label} candidate still"
    )
    recording_started_at = _timestamp(
        capture.get("recording_started_at"), f"{label} recording start"
    )
    route_started_at = _timestamp(
        capture.get("route_evaluation_started_at"), f"{label} route start"
    )
    route_finished_at = _timestamp(
        capture.get("route_evaluation_finished_at"), f"{label} route finish"
    )
    captured_at = _timestamp(capture.get("captured_at"), f"{label} captured_at")
    representative_at = _timestamp(
        representative.get("captured_at"), f"{label} representative captured_at"
    )
    if not (
        candidate_at
        <= candidate_still_at
        <= recording_started_at
        <= route_started_at
        < route_finished_at
    ):
        raise AuditError(f"{label} capture timestamps are not monotonic")
    route_midpoint = route_started_at + (route_finished_at - route_started_at) / 2
    offset_timestamp = recording_started_at + (representative_at - recording_started_at)
    if (
        captured_at != representative_at
        or abs((representative_at - route_midpoint).total_seconds()) > 0.01
        or abs(
            (representative_at - recording_started_at).total_seconds() - offset
        )
        > 0.01
        or offset_timestamp != representative_at
    ):
        raise AuditError(f"{label} representative PNG is not the route midpoint")

    runtime = _runtime_environment(directory / "runtime.env", f"{label} runtime")
    expected_runtime = {
        "RECOMMENDED": "true",
        "VISUALIZE": "true",
        "CAPTURE_DESKTOP": "true",
    }
    for key, expected in expected_runtime.items():
        if runtime.get(key) != expected:
            raise AuditError(f"{label} runtime contract mismatch: {key}={expected}")
    runtime_config = runtime.get("RVIZ_CAPTURE_CONFIG")
    if not runtime_config:
        raise AuditError(f"{label} runtime has no RVIZ_CAPTURE_CONFIG")
    runtime_config_path = Path(runtime_config)
    if not runtime_config_path.is_absolute():
        runtime_config_path = directory / runtime_config_path
    if (
        runtime_config_path.resolve() != config_path.resolve()
        or runtime.get("RVIZ_CAPTURE_CONFIG_SHA256") != config_sha256
    ):
        raise AuditError(f"{label} runtime RViz config does not match provenance")

    frame_verification: dict[str, Any]
    if verify_frame:
        frame_verification = _verify_extracted_frame(
            directory / expected_names["recording_file"],
            directory / expected_names["png_file"],
            offset,
            label,
        )
        if abs(frame_verification["probed_recording_duration_sec"] - duration) > 0.25:
            raise AuditError(f"{label} probed recording duration disagrees with metadata")
    else:
        frame_verification = {
            "status": "SKIPPED",
            "reason": "--skip-frame-reextract",
        }

    return {
        "metadata_sha256": _sha256(directory / "desktop_capture.json"),
        "candidate_png": candidate_info,
        "representative_png": representative_info,
        "drive_gif": gif_info,
        "recording": {
            "sha256": _sha256(directory / expected_names["recording_file"]),
            "size_bytes": (directory / expected_names["recording_file"]).stat().st_size,
            "duration_sec": duration,
        },
        "representative_frame": {
            "selection": representative["selection"],
            "offset_sec": offset,
            "captured_at": representative_at.isoformat(),
            "route_midpoint_at": route_midpoint.isoformat(),
            "frame_verification": frame_verification,
        },
        "rviz_view_contract": dict(contract),
        "runtime_profile": expected_runtime,
    }


def audit_trials(
    centered_root: Path,
    manifest_path: Path,
    *,
    expected_map_count: int = 9,
    verify_frames: bool = True,
) -> list[dict[str, Any]]:
    centered_root = centered_root.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    selected = _manifest_records(manifest_path, expected_map_count)
    records: list[dict[str, Any]] = []
    for map_id, trial_id in sorted(selected):
        label = f"{map_id}:{trial_id}"
        trial_root = centered_root / "maps" / map_id / "trials" / trial_id
        attempt, history = _select_latest_successful_attempt(trial_root, label)
        result = _read_json_object(attempt / "result.json", f"{label} result")
        if not _full_stack_result_pass(result):  # pragma: no cover - selection guarantees it.
            raise AuditError(f"{label} selected attempt lost full-stack PASS status")

        manifest_record = selected[(map_id, trial_id)]
        original_directory = manifest_record.get("trial_directory")
        if not isinstance(original_directory, str) or not original_directory:
            raise AuditError(f"{label} publication record has no selected original")
        original_path = Path(original_directory).expanduser()
        if not original_path.is_absolute():
            original_path = manifest_path.parent / original_path
        original_path = original_path.resolve()
        centered_route = attempt / "source_route.json"
        original_route = original_path / "source_route.json"
        centered_payload = _read_json_object(centered_route, f"{label} centered route")
        original_payload = _read_json_object(original_route, f"{label} original route")
        centered_route_sha256 = _sha256(centered_route)
        original_route_sha256 = _sha256(original_route)
        if (
            centered_route_sha256 != original_route_sha256
            or centered_payload != original_payload
        ):
            raise AuditError(
                f"{label} source_route.json differs from publication-manifest original"
            )

        capture = _validate_capture(attempt, label, verify_frame=verify_frames)
        records.append(
            {
                "map_id": map_id,
                "trial_id": trial_id,
                "status": "PASS",
                "selected_attempt": attempt.name,
                "selected_attempt_number": _attempt_number(attempt),
                "selected_attempt_directory": str(attempt),
                "attempt_history": history,
                "result": {
                    "sha256": _sha256(attempt / "result.json"),
                    "success": True,
                    "execution_mode": "full_stack",
                    "planning_architecture": "vad_route_manager_hybrid",
                    "route_completion": "PASS",
                    "goal_reached": True,
                    "route_status": "goal_reached",
                },
                "source_route": {
                    "status": "EXACT_MATCH",
                    "sha256": centered_route_sha256,
                    "centered_file": str(centered_route),
                    "publication_original_file": str(original_route),
                    "publication_source": manifest_record.get("source"),
                },
                "capture": capture,
            }
        )
    return records


def _load_visual_review(
    path: Path | None, records: Sequence[Mapping[str, Any]]
) -> tuple[dict[tuple[str, str], dict[str, Any]], dict[str, Any]]:
    identities = {(record["map_id"], record["trial_id"]) for record in records}
    if path is None:
        pending = {
            identity: {
                "status": "NOT_REVIEWED",
                "vehicle_visible": None,
                "reference_route_visible": None,
                "final_trajectory_visible": None,
                "vad_trajectories_visible": None,
                "viewport_centered": None,
                "notes": "Inspect contact_sheet.png.",
            }
            for identity in identities
        }
        return pending, {
            "status": "NOT_REVIEWED",
            "review_file": None,
            "reviewer": None,
            "reviewed_at": None,
        }

    path = path.expanduser().resolve()
    payload = _read_json_object(path, "visual review")
    if payload.get("schema_version") != 1:
        raise AuditError("visual review schema_version must be 1")
    scenes = payload.get("scenes")
    if not isinstance(scenes, list):
        raise AuditError("visual review must contain a scenes list")
    reviewed: dict[tuple[str, str], dict[str, Any]] = {}
    record_by_identity = {
        (record["map_id"], record["trial_id"]): record for record in records
    }
    boolean_fields = (
        "vehicle_visible",
        "reference_route_visible",
        "final_trajectory_visible",
        "vad_trajectories_visible",
        "viewport_centered",
    )
    for raw_scene in scenes:
        if not isinstance(raw_scene, dict):
            raise AuditError("visual review scene entries must be objects")
        identity = (raw_scene.get("map_id"), raw_scene.get("trial_id"))
        if identity not in identities or identity in reviewed:
            raise AuditError(f"visual review has an unknown/duplicate scene: {identity}")
        if any(not isinstance(raw_scene.get(field), bool) for field in boolean_fields):
            raise AuditError(f"visual review scene {identity} lacks boolean judgments")
        expected_sha = record_by_identity[identity]["capture"]["representative_png"][
            "sha256"
        ]
        if raw_scene.get("representative_png_sha256") != expected_sha:
            raise AuditError(f"visual review scene {identity} is stale")
        expected_status = (
            "PASS" if all(raw_scene[field] for field in boolean_fields) else "FLAG"
        )
        if raw_scene.get("status") != expected_status:
            raise AuditError(f"visual review scene {identity} has inconsistent status")
        notes = raw_scene.get("notes", "")
        if not isinstance(notes, str):
            raise AuditError(f"visual review scene {identity} notes must be a string")
        reviewed[identity] = dict(raw_scene)
    if set(reviewed) != identities:
        raise AuditError("visual review must cover every audited scene exactly once")
    reviewed_at = _timestamp(payload.get("reviewed_at"), "visual review timestamp")
    reviewer = payload.get("reviewer")
    if not isinstance(reviewer, str) or not reviewer.strip():
        raise AuditError("visual review must name its reviewer")
    overall = "PASS" if all(item["status"] == "PASS" for item in reviewed.values()) else "FLAG"
    return reviewed, {
        "status": overall,
        "review_file": str(path),
        "review_file_sha256": _sha256(path),
        "reviewer": reviewer,
        "reviewed_at": reviewed_at.isoformat(),
    }


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    if font_path.is_file():
        return ImageFont.truetype(str(font_path), size=size)
    return ImageFont.load_default()


def render_contact_sheet(
    records: Sequence[Mapping[str, Any]],
    reviews: Mapping[tuple[str, str], Mapping[str, Any]],
    output_path: Path,
) -> dict[str, Any]:
    columns = 3
    tile_width = 640
    image_height = 360
    label_height = 42
    rows = math.ceil(len(records) / columns)
    sheet = Image.new(
        "RGB", (columns * tile_width, rows * (image_height + label_height)), "#111820"
    )
    draw = ImageDraw.Draw(sheet)
    font = _font(18)
    small_font = _font(14)
    resampling = getattr(Image, "Resampling", Image).LANCZOS
    for index, record in enumerate(records):
        column = index % columns
        row = index // columns
        x = column * tile_width
        y = row * (image_height + label_height)
        png_path = Path(record["selected_attempt_directory"]) / (
            "autoware_rviz_fullscreen.png"
        )
        with Image.open(png_path) as source:
            frame = source.convert("RGB").resize(
                (tile_width, image_height), resampling
            )
        sheet.paste(frame, (x, y + label_height))
        identity = (record["map_id"], record["trial_id"])
        review = reviews[identity]
        status = review["status"]
        color = "#1f7a4d" if status == "PASS" else "#8a6d1d" if status == "NOT_REVIEWED" else "#a93232"
        draw.rectangle((x, y, x + tile_width, y + label_height), fill=color)
        title = (
            f"{record['map_id']} / {record['trial_id']} / "
            f"{record['selected_attempt']} | visual={status}"
        )
        midpoint = record["capture"]["representative_frame"]
        subtitle = (
            f"route midpoint +{midpoint['offset_sec']:.3f}s | "
            "mechanical PASS"
        )
        draw.text((x + 8, y + 3), title, fill="white", font=font)
        draw.text((x + 8, y + 24), subtitle, fill="#e4edf4", font=small_font)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent, delete=False
    ) as stream:
        temporary = Path(stream.name)
    try:
        sheet.save(temporary, format="PNG", optimize=True)
        temporary.replace(output_path)
    finally:
        temporary.unlink(missing_ok=True)
    return {
        "file": str(output_path),
        "sha256": _sha256(output_path),
        "dimensions": list(sheet.size),
        "columns": columns,
        "rows": rows,
        "scene_count": len(records),
    }


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        stream.write(text)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _markdown(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Centered Autoware VAD visual audit",
        "",
        f"- Overall status: **{summary['status']}**",
        f"- Mechanical audit: **{summary['mechanical_status']}** "
        f"({summary['counts']['mechanical_pass']}/{summary['counts']['total']} PASS)",
        f"- Visual review: **{summary['visual_review']['status']}** "
        f"({summary['counts']['visual_pass']}/{summary['counts']['total']} PASS, "
        f"{summary['counts']['visual_flag']} flagged, "
        f"{summary['counts']['visual_pending']} pending)",
        f"- Contact sheet: `{Path(summary['contact_sheet']['file']).name}` "
        f"({summary['contact_sheet']['dimensions'][0]}×"
        f"{summary['contact_sheet']['dimensions'][1]})",
        "- Publication selection binding: "
        f"`{summary['publication_selection']['sha256']}` "
        f"({summary['publication_selection']['record_count']} stable records; "
        "not a full-file manifest digest)",
        "",
        "Every selected rerun is the highest-numbered full-stack result-PASS attempt "
        "for its map/trial; this result-level label is separate from the camera-source "
        "matrix gate. `source_route.json` is byte-for-byte equal to the selected "
        "original named by the publication selection.",
        "",
        "| Map | Trial | Attempt | Result | Same source route | Midpoint frame | "
        "Centered contract | Visual review | Notes |",
        "|---|---|---:|---|---|---|---|---|---|",
    ]
    for record in summary["trials"]:
        review = record["visual_review"]
        notes = str(review.get("notes", "")).replace("|", "\\|")
        lines.append(
            f"| `{record['map_id']}` | `{record['trial_id']}` | "
            f"`{record['selected_attempt']}` | PASS | EXACT_MATCH | "
            f"PASS (+{record['capture']['representative_frame']['offset_sec']:.3f}s) | "
            f"PASS (`base_link`, X/Y 0, scale 10) | {review['status']} | {notes} |"
        )
    lines.extend(
        [
            "",
            "## Reproduce",
            "",
            "```bash",
            summary["reproduction_command"],
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def build_outputs(
    centered_root: Path,
    manifest_path: Path,
    *,
    output_json: Path,
    output_markdown: Path,
    contact_sheet: Path,
    visual_review_path: Path | None,
    expected_map_count: int,
    verify_frames: bool,
) -> dict[str, Any]:
    centered_root = centered_root.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    records = audit_trials(
        centered_root,
        manifest_path,
        expected_map_count=expected_map_count,
        verify_frames=verify_frames,
    )
    publication_selection = _publication_selection(manifest_path, expected_map_count)
    reviews, review_summary = _load_visual_review(visual_review_path, records)
    for record in records:
        record["visual_review"] = reviews[(record["map_id"], record["trial_id"])]
    contact_info = render_contact_sheet(records, reviews, contact_sheet.resolve())
    visual_pass = sum(record["visual_review"]["status"] == "PASS" for record in records)
    visual_flag = sum(record["visual_review"]["status"] == "FLAG" for record in records)
    visual_pending = len(records) - visual_pass - visual_flag
    status = (
        "PASS"
        if review_summary["status"] == "PASS"
        else "FLAG"
        if review_summary["status"] == "FLAG"
        else "MECHANICAL_PASS_VISUAL_PENDING"
    )

    command = [
        "python3",
        "scripts/e2e/summarize_centered_vad_visuals.py",
        "--centered-root",
        str(centered_root),
        "--publication-manifest",
        str(manifest_path),
    ]
    if visual_review_path is not None:
        command.extend(["--visual-review", str(visual_review_path.resolve())])
    if not verify_frames:
        command.append("--skip-frame-reextract")
    if expected_map_count != 9:
        command.extend(["--expected-map-count", str(expected_map_count)])
    command.extend(
        [
            "--output-json",
            str(output_json.resolve()),
            "--output-markdown",
            str(output_markdown.resolve()),
            "--contact-sheet",
            str(contact_sheet.resolve()),
        ]
    )
    summary = {
        "schema_version": 2,
        "status": status,
        "mechanical_status": "PASS",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "centered_root": str(centered_root),
        "publication_selection": publication_selection,
        "counts": {
            "maps": len({record["map_id"] for record in records}),
            "total": len(records),
            "mechanical_pass": len(records),
            "visual_pass": visual_pass,
            "visual_flag": visual_flag,
            "visual_pending": visual_pending,
        },
        "visual_review": review_summary,
        "contact_sheet": contact_info,
        "reproduction_command": " ".join(shlex.quote(value) for value in command),
        "trials": records,
    }
    _atomic_text(
        output_json.resolve(),
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
    )
    _atomic_text(output_markdown.resolve(), _markdown(summary))
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centered-root", type=Path, required=True)
    parser.add_argument("--publication-manifest", type=Path, required=True)
    parser.add_argument("--visual-review", type=Path)
    parser.add_argument("--expected-map-count", type=int, default=9)
    parser.add_argument("--skip-frame-reextract", action="store_true")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-markdown", type=Path)
    parser.add_argument("--contact-sheet", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.expected_map_count <= 0:
        raise SystemExit("--expected-map-count must be positive")
    root = args.centered_root.expanduser().resolve()
    output_json = args.output_json or root / "summary.json"
    output_markdown = args.output_markdown or root / "SUMMARY.md"
    contact_sheet = args.contact_sheet or root / "contact_sheet.png"
    try:
        summary = build_outputs(
            root,
            args.publication_manifest,
            output_json=output_json,
            output_markdown=output_markdown,
            contact_sheet=contact_sheet,
            visual_review_path=args.visual_review,
            expected_map_count=args.expected_map_count,
            verify_frames=not args.skip_frame_reextract,
        )
    except AuditError as error:
        raise SystemExit(f"centered visual audit failed: {error}") from error
    print(
        f"status={summary['status']} maps={summary['counts']['maps']} "
        f"trials={summary['counts']['total']} contact_sheet={contact_sheet}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
