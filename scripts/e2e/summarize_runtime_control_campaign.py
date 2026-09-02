#!/usr/bin/env python3
"""Build deterministic, fail-closed reports for the runtime/control campaign."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import posixpath
import statistics
import sys
import tempfile
from typing import Any, Callable, Iterable, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMPAIGN_ROOT = (
    REPOSITORY_ROOT
    / "artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1"
)

SCENARIOS: tuple[dict[str, Any], ...] = (
    {
        "id": "town07_straight",
        "town": "Town07",
        "route_scenarios": ("straight",),
        "baseline": "A_baseline_best_effort_health_001",
        "candidate": "B_pid_i40_best_effort_health_001",
        "candidate_id": "pid_i40",
        "comparison": "A_baseline_vs_B_pid_i40_decision",
    },
    {
        "id": "c_track_turn",
        "town": "C_track_1_0_7",
        "route_scenarios": ("left", "right"),
        "baseline": "A_baseline_best_effort_health_001",
        "candidate": "B_turn_preview_5m_best_effort_health_001",
        "candidate_id": "turn_preview_5m",
        "comparison": "A_baseline_vs_B_turn_preview_5m_decision",
    },
    {
        "id": "town03_turn",
        "town": "Town03",
        "route_scenarios": ("left", "right"),
        "baseline": "A_baseline_best_effort_health_001",
        "candidate": "B_turn_preview_5m_best_effort_health_001",
        "candidate_id": "turn_preview_5m",
        "comparison": "A_baseline_vs_B_turn_preview_5m_decision",
    },
)

TRIAL_SOURCE_FILES = (
    "owned_trial_summary.json",
    "result.json",
    "runtime_health.json",
    "runtime.env",
    "source_route.json",
    "speed_profile.json",
    "longitudinal_response.json",
    "diagnosis.json",
    "latency/e2e_latency.json",
    "desktop_capture.json",
)

TRIAL_VISUALS = (
    ("fullscreen", "autoware_rviz_fullscreen.png", "image/png"),
    ("drive", "autoware_rviz_drive.gif", "image/gif"),
    ("turn_path_control", "turn_path_control.gif", "image/gif"),
    ("route_result", "route_result.png", "image/png"),
    ("path_control", "path_vs_control.png", "image/png"),
)

PILOT_DIRECTORY_V1 = "town06_straight_60kph_pilot_best_effort_image_v1"
PILOT_DIRECTORY_V2 = "town06_straight_60kph_pilot_best_effort_image_v2"
_COMPARATOR_MODULE: Any | None = None


class CampaignSummaryError(RuntimeError):
    """Raised when campaign evidence is absent, ambiguous, or contradictory."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CampaignSummaryError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _validate_finite(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise CampaignSummaryError(f"non-finite number in {label}")
    if isinstance(value, list):
        for item in value:
            _validate_finite(item, label)
    elif isinstance(value, dict):
        for item in value.values():
            _validate_finite(item, label)


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=lambda value: (_ for _ in ()).throw(
                CampaignSummaryError(f"non-finite JSON constant {value} in {label}")
            ),
        )
    except CampaignSummaryError:
        raise
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CampaignSummaryError(f"cannot read {label} at {path}: {error}") from error
    if not isinstance(payload, dict):
        raise CampaignSummaryError(f"{label} must be a JSON object: {path}")
    _validate_finite(payload, label)
    return payload


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CampaignSummaryError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise CampaignSummaryError(f"{label} must be finite")
    return result


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CampaignSummaryError(f"{label} must be an integer")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _within(root: Path, path: Path, label: str) -> Path:
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise CampaignSummaryError(f"cannot resolve {label}: {path}: {error}") from error
    if root != resolved and root not in resolved.parents:
        raise CampaignSummaryError(f"{label} escapes campaign root: {path}")
    relative = path.relative_to(root)
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise CampaignSummaryError(f"{label} may not use symlinks: {path}")
    return resolved


def _required_file(root: Path, path: Path, label: str) -> Path:
    if not path.is_file():
        raise CampaignSummaryError(f"missing {label}: {path}")
    resolved = _within(root, path, label)
    if resolved.stat().st_size <= 0:
        raise CampaignSummaryError(f"empty {label}: {path}")
    return resolved


def _reference(root: Path, path: Path, label: str) -> dict[str, Any]:
    resolved = _required_file(root, path, label)
    return {
        "campaign_relative_path": resolved.relative_to(root).as_posix(),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _load_comparator_trial(path: Path, role: str) -> dict[str, Any]:
    global _COMPARATOR_MODULE
    module_path = Path(__file__).with_name("compare_control_ab.py")
    if _COMPARATOR_MODULE is None:
        spec = importlib.util.spec_from_file_location(
            "_campaign_compare_control_ab", module_path
        )
        if spec is None or spec.loader is None:
            raise CampaignSummaryError(f"cannot load comparator: {module_path}")
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except Exception as error:
            raise CampaignSummaryError(f"cannot load comparator: {error}") from error
        _COMPARATOR_MODULE = module
    module = _COMPARATOR_MODULE
    try:
        return module.load_trial(path, role)
    except Exception as error:  # comparator exposes a local exception type
        raise CampaignSummaryError(
            f"{role} trial failed authoritative validation at {path}: {error}"
        ) from error


def _runtime_health_summary(health: Mapping[str, Any], label: str) -> dict[str, Any]:
    if health.get("schema_version") != 1 or health.get("status") != "PASS":
        raise CampaignSummaryError(f"{label} runtime health must be schema-1 PASS")
    sequence = health.get("sequence")
    windows = health.get("windows")
    if not isinstance(sequence, dict) or not isinstance(windows, list):
        raise CampaignSummaryError(f"{label} runtime health sequence is malformed")
    if sequence.get("status") != "PASS" or sequence.get("timed_out") is not False:
        raise CampaignSummaryError(f"{label} runtime health sequence is not PASS")
    indexes = sequence.get("winning_window_indexes")
    if not isinstance(indexes, list) or len(indexes) != 3:
        raise CampaignSummaryError(
            f"{label} runtime health must have exactly three winning windows"
        )
    if any(isinstance(index, bool) or not isinstance(index, int) for index in indexes):
        raise CampaignSummaryError(f"{label} winning-window indexes are invalid")
    if len(set(indexes)) != len(indexes):
        raise CampaignSummaryError(f"{label} winning-window indexes are duplicated")
    rtfs: list[float] = []
    rates: list[float] = []
    bundle_p95: list[float] = []
    for index in indexes:
        if index < 0 or index >= len(windows) or not isinstance(windows[index], dict):
            raise CampaignSummaryError(f"{label} winning-window index is out of range")
        window = windows[index]
        if window.get("status") != "PASS" or window.get("failures") != []:
            raise CampaignSummaryError(f"{label} winning window is not cleanly PASS")
        clock = window.get("clock")
        bundles = window.get("bundles")
        if not isinstance(clock, dict) or not isinstance(bundles, dict):
            raise CampaignSummaryError(f"{label} winning-window metrics are malformed")
        receipt = bundles.get("receipt_span_seconds")
        if not isinstance(receipt, dict):
            raise CampaignSummaryError(f"{label} camera-bundle metrics are malformed")
        rtfs.append(_number(clock.get("rtf"), f"{label} winning RTF"))
        rates.append(
            _number(
                window.get("minimum_observed_camera_wall_rate_hz"),
                f"{label} minimum camera wall rate",
            )
        )
        bundle_p95.append(
            _number(receipt.get("p95"), f"{label} bundle receipt p95")
        )
    return {
        "status": "PASS",
        "winning_window_count": len(indexes),
        "winning_window_indexes": indexes,
        "rtf": {
            "minimum": min(rtfs),
            "median": statistics.median(rtfs),
        },
        "minimum_camera_wall_rate_hz": {
            "minimum": min(rates),
            "median": statistics.median(rates),
        },
        "camera_bundle_receipt_p95_sec": {
            "maximum": max(bundle_p95),
            "median": statistics.median(bundle_p95),
        },
    }


def _validate_desktop_capture(desktop: Mapping[str, Any], label: str) -> None:
    if desktop.get("schema_version") != 1:
        raise CampaignSummaryError(f"{label} desktop capture schema is unsupported")
    if desktop.get("candidate_observed") is not True:
        raise CampaignSummaryError(f"{label} capture is not candidate-backed")
    if desktop.get("capture_started_after_candidate") is not True:
        raise CampaignSummaryError(f"{label} capture started before the candidate")
    if desktop.get("png_file") != "autoware_rviz_fullscreen.png":
        raise CampaignSummaryError(f"{label} fullscreen PNG identity changed")
    if desktop.get("gif_file") != "autoware_rviz_drive.gif":
        raise CampaignSummaryError(f"{label} drive GIF identity changed")
    capture = desktop.get("capture_source")
    overlay = desktop.get("desktop_overlay_check")
    view = desktop.get("rviz_view_contract")
    if not isinstance(capture, dict) or not isinstance(overlay, dict):
        raise CampaignSummaryError(f"{label} owned-window capture contract is missing")
    if (
        capture.get("root_capture") is not False
        or capture.get("shell_surfaces_excluded") is not True
        or overlay.get("passed") is not True
    ):
        raise CampaignSummaryError(f"{label} owned-window capture contract did not pass")
    if not isinstance(view, dict) or view.get("vehicle_centered") is not True:
        raise CampaignSummaryError(f"{label} RViz view is not vehicle-centered")
    topics = view.get("visible_path_topics")
    required_topics = {
        "/planning/trajectory",
        "/planning/vad_route/actual_path",
        "/planning/vad_route/reference_path",
    }
    if not isinstance(topics, list) or not required_topics.issubset(set(topics)):
        raise CampaignSummaryError(f"{label} RViz path visibility contract is incomplete")


def _collect_trial(
    root: Path,
    owner_root: Path,
    role: str,
    spec: Mapping[str, Any],
    trial_loader: Callable[[Path, str], dict[str, Any]],
) -> tuple[
    dict[str, Any],
    dict[str, Any],
    list[dict[str, Any]],
    list[dict[str, Any]],
]:
    owner_path = _required_file(
        root, owner_root / "owned_trial_summary.json", f"{spec['id']} {role} owner summary"
    )
    owner = _read_json(owner_path, f"{spec['id']} {role} owner summary")
    attempts = owner.get("attempts")
    if not isinstance(attempts, list) or len(attempts) != 1:
        raise CampaignSummaryError(
            f"{spec['id']} {role} owner must contain exactly one attempt"
        )
    attempt = attempts[0]
    if not isinstance(attempt, dict) or not isinstance(attempt.get("attempt_id"), str):
        raise CampaignSummaryError(f"{spec['id']} {role} owner attempt is malformed")
    trial_dir = owner_root / "attempts" / attempt["attempt_id"]
    _within(root, trial_dir, f"{spec['id']} {role} trial directory")

    json_sources = (
        "result.json",
        "runtime_health.json",
        "source_route.json",
        "speed_profile.json",
        "longitudinal_response.json",
        "diagnosis.json",
        "latency/e2e_latency.json",
        "desktop_capture.json",
    )
    parsed: dict[str, dict[str, Any]] = {}
    for name in json_sources:
        path = _required_file(root, trial_dir / name, f"{spec['id']} {role} {name}")
        parsed[name] = _read_json(path, f"{spec['id']} {role} {name}")
    _required_file(root, trial_dir / "runtime.env", f"{spec['id']} {role} runtime.env")

    trial = trial_loader(owner_root, role)
    if not isinstance(trial, dict):
        raise CampaignSummaryError(f"{spec['id']} {role} loader returned no trial")
    if Path(str(trial.get("path", ""))).resolve() != trial_dir.resolve():
        raise CampaignSummaryError(f"{spec['id']} {role} resolved trial path changed")
    if trial.get("route_town") != spec["town"]:
        raise CampaignSummaryError(f"{spec['id']} {role} town identity changed")
    if trial.get("route_scenario") not in spec["route_scenarios"]:
        raise CampaignSummaryError(f"{spec['id']} {role} route scenario changed")
    expected_candidate = "baseline" if role == "baseline" else spec["candidate_id"]
    if trial.get("control_ab_candidate") != expected_candidate:
        raise CampaignSummaryError(f"{spec['id']} {role} control identity changed")
    expected_control = (
        expected_candidate,
        "true" if expected_candidate == "pid_i40" else "false",
        "true" if expected_candidate == "turn_preview_5m" else "false",
        "true",
    )
    actual_control = (
        trial.get("control_ab_candidate"),
        trial.get("control_ab_pid_i40"),
        trial.get("control_ab_turn_preview_5m"),
        trial.get("control_ab_isolated_single_knob"),
    )
    if actual_control != expected_control:
        raise CampaignSummaryError(f"{spec['id']} {role} control isolation changed")
    if trial.get("profile_id") != "carla_vad_30kph_v2":
        raise CampaignSummaryError(f"{spec['id']} {role} speed profile changed")

    health_summary = _runtime_health_summary(
        parsed["runtime_health.json"], f"{spec['id']} {role}"
    )
    _validate_desktop_capture(
        parsed["desktop_capture.json"], f"{spec['id']} {role}"
    )

    evidence: list[dict[str, Any]] = []
    owner_ref = _reference(root, owner_path, f"{spec['id']} {role} owner summary")
    owner_ref["kind"] = "owner_summary"
    owner_ref["scenario"] = spec["id"]
    owner_ref["variant"] = role
    evidence.append(owner_ref)
    for name in TRIAL_SOURCE_FILES[1:]:
        ref = _reference(root, trial_dir / name, f"{spec['id']} {role} {name}")
        ref["kind"] = name
        ref["scenario"] = spec["id"]
        ref["variant"] = role
        evidence.append(ref)

    visuals: list[dict[str, Any]] = []
    for kind, filename, mime_type in TRIAL_VISUALS:
        ref = _reference(root, trial_dir / filename, f"{spec['id']} {role} {kind}")
        ref.update(
            {
                "id": f"30kph.{spec['id']}.{role}.{kind}",
                "speed_class": "30kph",
                "scenario": spec["id"],
                "variant": role,
                "kind": kind,
                "mime_type": mime_type,
            }
        )
        visuals.append(ref)

    metrics = trial.get("metrics")
    if not isinstance(metrics, dict):
        raise CampaignSummaryError(f"{spec['id']} {role} control metrics are missing")
    if not isinstance(trial.get("quality_problems"), list):
        raise CampaignSummaryError(f"{spec['id']} {role} quality problems are malformed")
    record = {
        "variant": role,
        "candidate_id": expected_candidate,
        "owner_status": trial.get("owner_status"),
        "route_outcome": trial.get("route_outcome"),
        "success": trial.get("success"),
        "reason": trial.get("reason"),
        "town": trial.get("route_town"),
        "route_scenario": trial.get("route_scenario"),
        "route_sha256": trial.get("route_sha256"),
        "profile_id": trial.get("profile_id"),
        "control_isolation": {
            "candidate_id": trial.get("control_ab_candidate"),
            "pid_i40": trial.get("control_ab_pid_i40"),
            "turn_preview_5m": trial.get("control_ab_turn_preview_5m"),
            "isolated_single_knob": trial.get("control_ab_isolated_single_knob"),
        },
        "runtime_health": health_summary,
        "quality_problems": trial.get("quality_problems"),
        "metrics": metrics,
        "evidence": {
            item["kind"]: {
                key: item[key]
                for key in ("campaign_relative_path", "sha256", "size_bytes")
            }
            for item in evidence
        },
    }
    return record, trial, evidence, visuals


def _comparison_projection(trial: Mapping[str, Any]) -> dict[str, Any]:
    keys = (
        "owner_status",
        "route_outcome",
        "success",
        "reason",
        "health_status",
        "health_rtf",
        "route_sha256",
        "owner_route_sha256",
        "route_town",
        "route_scenario",
        "profile_id",
        "control_ab_candidate",
        "control_ab_pid_i40",
        "control_ab_turn_preview_5m",
        "control_ab_isolated_single_knob",
        "runtime_health_gate_enabled",
        "runtime_health_gate_status",
        "runtime_health_evidence_sha256",
        "runtime_health_recorded_sha256",
        "quality_problems",
        "metrics",
    )
    return {key: trial.get(key) for key in keys}


def _validate_comparison(
    root: Path,
    comparison_path: Path,
    comparison_png: Path,
    spec: Mapping[str, Any],
    baseline_loaded: Mapping[str, Any],
    candidate_loaded: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    directory = comparison_path.parent
    decision_jsons = sorted(directory.glob("*_decision.json"))
    decision_pngs = sorted(directory.glob("*_decision.png"))
    if decision_jsons != [comparison_path] or decision_pngs != [comparison_png]:
        raise CampaignSummaryError(f"{spec['id']} comparison evidence is ambiguous")
    comparison_path = _required_file(
        root, comparison_path, f"{spec['id']} comparison JSON"
    )
    comparison_png = _required_file(
        root, comparison_png, f"{spec['id']} comparison PNG"
    )
    payload = _read_json(comparison_path, f"{spec['id']} comparison")
    if (
        payload.get("schema_version") != 1
        or payload.get("scenario") != spec["id"]
        or payload.get("candidate_id") != spec["candidate_id"]
        or payload.get("real_vehicle_ready") is not False
    ):
        raise CampaignSummaryError(f"{spec['id']} comparison identity is invalid")
    decision = payload.get("decision")
    checks = payload.get("checks")
    if decision not in {"ACCEPT", "HOLD"} or not isinstance(checks, dict) or not checks:
        raise CampaignSummaryError(f"{spec['id']} comparison decision is malformed")
    failed_checks: list[str] = []
    for name in sorted(checks):
        row = checks[name]
        if not isinstance(row, dict) or row.get("status") not in {"PASS", "FAIL"}:
            raise CampaignSummaryError(f"{spec['id']} comparison check {name} is malformed")
        if row["status"] == "FAIL":
            failed_checks.append(name)
    expected_decision = "HOLD" if failed_checks else "ACCEPT"
    if decision != expected_decision:
        raise CampaignSummaryError(f"{spec['id']} comparison decision contradicts checks")
    for role, loaded in (
        ("baseline", baseline_loaded),
        ("candidate", candidate_loaded),
    ):
        stored = payload.get(role)
        if not isinstance(stored, dict):
            raise CampaignSummaryError(f"{spec['id']} comparison lacks {role}")
        if _comparison_projection(stored) != _comparison_projection(loaded):
            raise CampaignSummaryError(
                f"{spec['id']} comparison {role} no longer matches source evidence"
            )

    json_ref = _reference(root, comparison_path, f"{spec['id']} comparison JSON")
    json_ref.update({"kind": "comparison", "scenario": spec["id"]})
    visual_ref = _reference(root, comparison_png, f"{spec['id']} comparison PNG")
    visual_ref.update(
        {
            "id": f"30kph.{spec['id']}.comparison",
            "speed_class": "30kph",
            "scenario": spec["id"],
            "variant": "comparison",
            "kind": "comparison",
            "mime_type": "image/png",
        }
    )
    summary = {
        "decision": decision,
        "passed_check_count": len(checks) - len(failed_checks),
        "failed_check_count": len(failed_checks),
        "failed_checks": failed_checks,
        "evidence": {
            key: json_ref[key]
            for key in ("campaign_relative_path", "sha256", "size_bytes")
        },
    }
    return summary, json_ref, visual_ref


def _observed_median(
    attempt: Mapping[str, Any], name: str, label: str, required: bool = True
) -> float | None:
    observed = attempt.get("observed")
    metric = observed.get(name) if isinstance(observed, dict) else None
    if metric is None and not required:
        return None
    if not isinstance(metric, dict):
        raise CampaignSummaryError(f"{label} {name} summary is missing")
    return _number(metric.get("median"), f"{label} {name} median")


def _collect_stutter(
    root: Path, town07_baseline: Mapping[str, Any]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    case_root = (
        root
        / "10_runtime_stutter"
        / "town07_straight_A_baseline_health_001"
    )
    summary_path = case_root / "runtime_stutter_summary.json"
    if not case_root.exists():
        return {"status": "UNAVAILABLE", "reason": "stutter case directory absent"}, []
    if not summary_path.is_file():
        raise CampaignSummaryError("stutter case exists without runtime_stutter_summary.json")
    payload = _read_json(
        _required_file(root, summary_path, "runtime stutter summary"),
        "runtime stutter summary",
    )
    attempts = payload.get("attempts")
    if (
        payload.get("schema_version") != 1
        or payload.get("status")
        != "PRE_ENGAGEMENT_RUNTIME_HEALTH_FAILED_ALL_ATTEMPTS"
        or payload.get("engagement_blocked_before_motion") is not True
        or payload.get("vehicle_engaged_any_attempt") is not False
        or not isinstance(attempts, list)
        or _integer(payload.get("attempt_count"), "stutter attempt count")
        != len(attempts)
        or not attempts
    ):
        raise CampaignSummaryError("runtime stutter summary contract is invalid")
    if payload.get("route_sha256") != town07_baseline.get("route_sha256"):
        raise CampaignSummaryError("stutter and healthy Town07 route digests differ")

    rows: list[dict[str, Any]] = []
    ids: set[str] = set()
    for attempt in attempts:
        if not isinstance(attempt, dict):
            raise CampaignSummaryError("stutter attempt must be an object")
        attempt_id = attempt.get("attempt_id")
        engagement = attempt.get("engagement")
        if not isinstance(attempt_id, str) or not attempt_id or attempt_id in ids:
            raise CampaignSummaryError("stutter attempt identity is ambiguous")
        ids.add(attempt_id)
        if (
            attempt.get("status") != "PRE_ENGAGEMENT_RUNTIME_HEALTH_FAIL"
            or not isinstance(engagement, dict)
            or engagement.get("blocked_before_engagement") is not True
            or engagement.get("vehicle_engaged") is not False
        ):
            raise CampaignSummaryError(f"stutter {attempt_id} was not blocked safely")
        rows.append(
            {
                "attempt_id": attempt_id,
                "rtf_median": _observed_median(attempt, "rtf", attempt_id),
                "minimum_camera_wall_rate_hz_median": _observed_median(
                    attempt, "minimum_camera_wall_rate", attempt_id
                ),
                "camera_bundle_receipt_p95_sec_median": _observed_median(
                    attempt, "bundle_receipt_window_p95", attempt_id
                ),
                "cam_front_publish_ms_median": _observed_median(
                    attempt,
                    "runtime_timing_cam_front_image_publish",
                    attempt_id,
                    required=False,
                ),
            }
        )
    rows.sort(key=lambda row: row["attempt_id"])
    before = {
        "status": payload["status"],
        "engagement_blocked_before_motion": True,
        "attempt_count": len(rows),
        "attempts": rows,
        "median_of_attempt_medians": {
            "rtf": statistics.median(row["rtf_median"] for row in rows),
            "minimum_camera_wall_rate_hz": statistics.median(
                row["minimum_camera_wall_rate_hz_median"] for row in rows
            ),
            "camera_bundle_receipt_p95_sec": statistics.median(
                row["camera_bundle_receipt_p95_sec_median"] for row in rows
            ),
            "cam_front_publish_ms": (
                statistics.median(
                    row["cam_front_publish_ms_median"]
                    for row in rows
                    if row["cam_front_publish_ms_median"] is not None
                )
                if any(row["cam_front_publish_ms_median"] is not None for row in rows)
                else None
            ),
        },
    }
    after_health = town07_baseline.get("runtime_health")
    if not isinstance(after_health, dict) or after_health.get("status") != "PASS":
        raise CampaignSummaryError("healthy Town07 baseline lacks runtime-health summary")
    after = {
        **after_health,
        "status": "PRE_ENGAGEMENT_RUNTIME_HEALTH_PASS",
        "source": "town07_straight baseline selected attempt",
    }
    before_medians = before["median_of_attempt_medians"]
    deltas = {
        "rtf_ratio_after_over_before": (
            after_health["rtf"]["median"] / before_medians["rtf"]
        ),
        "camera_rate_ratio_after_over_before": (
            after_health["minimum_camera_wall_rate_hz"]["median"]
            / before_medians["minimum_camera_wall_rate_hz"]
        ),
        "bundle_receipt_p95_reduction_percent": (
            100.0
            * (
                before_medians["camera_bundle_receipt_p95_sec"]
                - after_health["camera_bundle_receipt_p95_sec"]["median"]
            )
            / before_medians["camera_bundle_receipt_p95_sec"]
        ),
    }
    ref = _reference(root, summary_path, "runtime stutter summary")
    ref.update({"kind": "runtime_stutter_summary", "scenario": "town07_straight"})
    return {
        "status": "AVAILABLE",
        "before": before,
        "after": after,
        "change": deltas,
        "evidence": {
            key: ref[key]
            for key in ("campaign_relative_path", "sha256", "size_bytes")
        },
    }, [ref]


def _validate_pilot_run(
    root: Path, pilot_root: Path, payload: Mapping[str, Any]
) -> dict[str, Any]:
    if (
        payload.get("schema_version") != 1
        or payload.get("status") not in {"PASS", "FAILED"}
        or payload.get("profile_id") != "carla_vad_60kph_straight_pilot_v1"
        or payload.get("validation_state") != "exploratory_simulation_only"
        or payload.get("real_vehicle_ready") is not False
    ):
        raise CampaignSummaryError("60 kph pilot_run identity is invalid")
    exit_statuses = payload.get("exit_statuses")
    problems = payload.get("problems")
    summary = payload.get("summary")
    route = payload.get("route")
    if (
        not isinstance(exit_statuses, dict)
        or not isinstance(problems, list)
        or not isinstance(summary, dict)
        or not isinstance(route, dict)
    ):
        raise CampaignSummaryError("60 kph pilot_run body is malformed")
    required_statuses = {
        "trial_exit_status",
        "carla_cleanup_status",
        "actuation_coverage_exit_status",
        "telemetry_cleanup_status",
        "camera_integrity_exit_status",
        "runtime_load_analysis_exit_status",
    }
    if not required_statuses.issubset(exit_statuses):
        raise CampaignSummaryError("60 kph pilot_run exit-status set is incomplete")
    for key in required_statuses:
        _integer(exit_statuses[key], f"60 kph {key}")
    if payload["status"] == "PASS":
        if any(exit_statuses[key] != 0 for key in required_statuses) or problems:
            raise CampaignSummaryError("60 kph PASS pilot_run contradicts its gates")
    elif not problems:
        raise CampaignSummaryError("60 kph FAILED pilot_run has no problems")
    route_path_value = route.get("path")
    route_digest = route.get("sha256")
    if not isinstance(route_path_value, str) or not isinstance(route_digest, str):
        raise CampaignSummaryError("60 kph pilot_run route provenance is malformed")
    route_path = Path(route_path_value)
    if not route_path.is_absolute():
        route_path = pilot_root / route_path
    route_path = _required_file(root, route_path, "60 kph pilot route")
    if _sha256(route_path) != route_digest:
        raise CampaignSummaryError("60 kph pilot route digest mismatch")
    sim = summary.get("sim_elapsed_sec")
    wall = summary.get("wall_elapsed_sec")
    rtf = summary.get("real_time_factor")
    if sim is not None or wall is not None or rtf is not None:
        sim_number = _number(sim, "60 kph sim elapsed")
        wall_number = _number(wall, "60 kph wall elapsed")
        rtf_number = _number(rtf, "60 kph real-time factor")
        if wall_number <= 0.0 or not math.isclose(
            rtf_number, sim_number / wall_number, rel_tol=1.0e-12, abs_tol=1.0e-12
        ):
            raise CampaignSummaryError("60 kph pilot_run RTF is inconsistent")
    return {
        "status": payload["status"],
        "profile_id": payload["profile_id"],
        "real_vehicle_ready": False,
        "exit_statuses": {key: exit_statuses[key] for key in sorted(exit_statuses)},
        "problems": problems,
        "summary": summary,
        "route_sha256": route_digest,
    }


def _optional_pilot_attempt(
    root: Path, pilot_root: Path, require_complete: bool
) -> tuple[dict[str, Any] | None, list[dict[str, Any]], list[dict[str, Any]]]:
    attempt = pilot_root / "trial/attempt_001"
    if not attempt.is_dir():
        if require_complete:
            raise CampaignSummaryError("complete 60 kph evidence lacks trial/attempt_001")
        return None, [], []
    result_path = attempt / "result.json"
    if not result_path.exists():
        if require_complete:
            raise CampaignSummaryError("complete 60 kph evidence lacks result.json")
        return None, [], []
    result = _read_json(
        _required_file(root, result_path, "60 kph result"), "60 kph result"
    )
    metrics = result.get("metrics")
    if not isinstance(result.get("success"), bool) or not isinstance(metrics, dict):
        raise CampaignSummaryError("60 kph result contract is malformed")
    record: dict[str, Any] = {
        "present": True,
        "success": result["success"],
        "reason": result.get("reason"),
        "metrics": {
            key: metrics.get(key)
            for key in (
                "maximum_observed_speed_mps",
                "maximum_sustained_speed_duration_sec",
                "maximum_absolute_cte_m",
                "maximum_lateral_acceleration_mps2",
                "traveled_distance_m",
                "sim_elapsed_sec",
                "wall_elapsed_sec",
            )
        },
    }
    evidence: list[dict[str, Any]] = []
    for name in (
        "result.json",
        "runtime.env",
        "runtime_health.json",
        "speed_profile.json",
        "longitudinal_response.json",
    ):
        path = attempt / name
        if path.exists():
            ref = _reference(root, path, f"60 kph {name}")
            ref.update({"kind": name, "scenario": "town06_straight_60kph"})
            evidence.append(ref)
        elif require_complete:
            raise CampaignSummaryError(f"complete 60 kph evidence lacks {name}")
    if (attempt / "runtime_health.json").is_file():
        health = _read_json(attempt / "runtime_health.json", "60 kph runtime health")
        record["runtime_health"] = _runtime_health_summary(health, "60 kph")
    if (attempt / "longitudinal_response.json").is_file():
        longitudinal = _read_json(
            attempt / "longitudinal_response.json", "60 kph longitudinal response"
        )
        record["control_metrics"] = {
            "status": longitudinal.get("status"),
            "quality_problems": (
                longitudinal.get("quality", {}).get("problems")
                if isinstance(longitudinal.get("quality"), dict)
                else None
            ),
            "gate_positive_limit_time_percent": (
                longitudinal.get("saturation_and_duty", {})
                .get("gated_positive_acceleration_limit", {})
                .get("time_fraction_percent")
                if isinstance(longitudinal.get("saturation_and_duty"), dict)
                else None
            ),
        }

    visuals: list[dict[str, Any]] = []
    existence = [(attempt / filename).is_file() for _, filename, _ in TRIAL_VISUALS]
    if require_complete and not all(existence):
        raise CampaignSummaryError("complete 60 kph evidence lacks required visuals")
    if all(existence):
        desktop = _read_json(
            _required_file(root, attempt / "desktop_capture.json", "60 kph desktop capture"),
            "60 kph desktop capture",
        )
        _validate_desktop_capture(desktop, "60 kph")
        desktop_ref = _reference(root, attempt / "desktop_capture.json", "60 kph desktop capture")
        desktop_ref.update({"kind": "desktop_capture.json", "scenario": "town06_straight_60kph"})
        evidence.append(desktop_ref)
        for kind, filename, mime_type in TRIAL_VISUALS:
            ref = _reference(root, attempt / filename, f"60 kph {kind}")
            ref.update(
                {
                    "id": f"60kph.town06_straight.{kind}",
                    "speed_class": "60kph",
                    "scenario": "town06_straight",
                    "variant": "pilot",
                    "kind": kind,
                    "mime_type": mime_type,
                }
            )
            visuals.append(ref)
    elif any(existence):
        record["visual_evidence_status"] = "PARTIAL_NOT_REFERENCED"
    else:
        record["visual_evidence_status"] = "ABSENT"
    return record, evidence, visuals


def _collect_60kph(
    root: Path, pilot_directory: str | None = None
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    category = root / "30_60kph"
    if not category.exists():
        return {"status": "ABSENT", "reason": "30_60kph directory absent"}, [], []
    if category.is_symlink() or not category.is_dir():
        raise CampaignSummaryError("30_60kph is not a regular campaign directory")
    candidates = sorted(path for path in category.iterdir() if path.is_dir())
    if not candidates:
        if pilot_directory is not None:
            raise CampaignSummaryError(
                f"explicit 60 kph pilot directory is absent: {pilot_directory}"
            )
        return {"status": "ABSENT", "reason": "no 60 kph pilot directory"}, [], []
    if pilot_directory is not None:
        requested = Path(pilot_directory)
        if (
            not pilot_directory
            or requested.name != pilot_directory
            or pilot_directory in {".", ".."}
        ):
            raise CampaignSummaryError(
                "--pilot-directory must be one campaign child-directory name"
            )
        matches = [path for path in candidates if path.name == pilot_directory]
        if len(matches) != 1:
            raise CampaignSummaryError(
                f"explicit 60 kph pilot directory is absent: {pilot_directory}"
            )
        pilot_root = matches[0]
        selection = {
            "mode": "explicit_argument",
            "selected_directory": pilot_directory,
            "other_directory_count": len(candidates) - 1,
        }
    elif len(candidates) == 1:
        pilot_root = candidates[0]
        selection = {
            "mode": "sole_active_directory",
            "selected_directory": pilot_root.name,
            "other_directory_count": 0,
        }
    else:
        raise CampaignSummaryError(
            "60 kph pilot evidence is ambiguous: "
            + ", ".join(path.name for path in candidates)
        )
    _within(root, pilot_root, "60 kph pilot directory")
    run_path = pilot_root / "pilot_run.json"
    failure_path = pilot_root / "pilot_failure.json"
    semantic_path = pilot_root / "pilot_summary.json"
    run_exists = run_path.is_file()
    failure_exists = failure_path.is_file()
    semantic_exists = semantic_path.is_file()
    if not (run_exists or failure_exists or semantic_exists):
        return {
            "status": "IN_PROGRESS",
            "pilot_directory": pilot_root.relative_to(root).as_posix(),
            "selection": selection,
            "reason": "no atomic terminal pilot evidence is present",
        }, [], []

    evidence: list[dict[str, Any]] = []
    visuals: list[dict[str, Any]] = []
    run_summary: dict[str, Any] | None = None
    if run_exists:
        run_payload = _read_json(
            _required_file(root, run_path, "60 kph pilot_run"), "60 kph pilot_run"
        )
        run_summary = _validate_pilot_run(root, pilot_root, run_payload)
        ref = _reference(root, run_path, "60 kph pilot_run")
        ref.update({"kind": "pilot_run", "scenario": "town06_straight_60kph"})
        evidence.append(ref)

    console_path = pilot_root / "pilot_console.log"
    completion_found = False
    if console_path.is_file():
        try:
            console_lines = console_path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeDecodeError) as error:
            raise CampaignSummaryError(f"cannot read 60 kph pilot console: {error}") from error
        completion_line = f"60 kph exploratory pilot completed: {pilot_root.resolve()}"
        completion_found = completion_line in console_lines
    if failure_exists and completion_found:
        raise CampaignSummaryError("60 kph failure sentinel contradicts completion line")

    failure_summary: dict[str, Any] | None = None
    if failure_exists:
        failure = _read_json(
            _required_file(root, failure_path, "60 kph pilot failure"),
            "60 kph pilot failure",
        )
        if failure.get("schema_version") != 1 or failure.get("status") != "FAILED":
            raise CampaignSummaryError("60 kph failure sentinel is malformed")
        for key in (
            "process_exit_status",
            "carla_cleanup_status",
            "telemetry_cleanup_status",
        ):
            _integer(failure.get(key), f"60 kph failure {key}")
        if not isinstance(failure.get("phase"), str) or not failure["phase"]:
            raise CampaignSummaryError("60 kph failure phase is missing")
        failure_summary = {
            key: failure.get(key)
            for key in (
                "phase",
                "process_exit_status",
                "carla_cleanup_status",
                "telemetry_cleanup_status",
            )
        }
        ref = _reference(root, failure_path, "60 kph pilot failure")
        ref.update({"kind": "pilot_failure", "scenario": "town06_straight_60kph"})
        evidence.append(ref)

    semantic_summary: dict[str, Any] | None = None
    if semantic_exists:
        semantic = _read_json(
            _required_file(root, semantic_path, "60 kph pilot semantic summary"),
            "60 kph pilot semantic summary",
        )
        if (
            semantic.get("schema_version") != 1
            or semantic.get("status")
            != "EVIDENCE_COMPLETE_TRIAL_FAILED_SPEED_EXPOSURE"
            or semantic.get("trial_verdict") != "FAIL"
            or semantic.get("profile_id") != "carla_vad_60kph_straight_pilot_v1"
            or semantic.get("real_vehicle_ready") is not False
            or semantic.get("speed_exposure_status") != "FAIL"
            or not failure_exists
            or run_summary is None
            or run_summary.get("status") != "FAILED"
        ):
            raise CampaignSummaryError("60 kph semantic summary chain is invalid")
        comparison = semantic.get("comparison")
        if not isinstance(comparison, dict):
            raise CampaignSummaryError("60 kph semantic comparison provenance is missing")
        comparison_path_value = comparison.get("path")
        if not isinstance(comparison_path_value, str):
            raise CampaignSummaryError("60 kph semantic comparison path is invalid")
        comparison_path = Path(comparison_path_value)
        if not comparison_path.is_absolute():
            comparison_path = pilot_root / comparison_path
        comparison_path = _required_file(
            root, comparison_path, "60 kph speed comparison JSON"
        )
        if (
            comparison.get("sha256") != _sha256(comparison_path)
            or comparison.get("size_bytes") != comparison_path.stat().st_size
        ):
            raise CampaignSummaryError("60 kph speed comparison digest mismatch")
        comparison_ref = _reference(
            root, comparison_path, "60 kph speed comparison JSON"
        )
        comparison_ref.update(
            {"kind": "speed_comparison", "scenario": "town06_straight_60kph"}
        )
        evidence.append(comparison_ref)
        semantic_summary = {
            "status": semantic["status"],
            "trial_verdict": semantic["trial_verdict"],
            "reason": semantic.get("reason"),
            "goal_reached": semantic.get("goal_reached"),
            "speed_exposure_status": semantic.get("speed_exposure_status"),
        }
        ref = _reference(root, semantic_path, "60 kph pilot semantic summary")
        ref.update({"kind": "pilot_summary", "scenario": "town06_straight_60kph"})
        evidence.append(ref)

    if run_summary is not None and run_summary["status"] == "PASS":
        if failure_exists or semantic_exists:
            raise CampaignSummaryError("60 kph PASS run has contradictory failure evidence")
        if not completion_found:
            return {
                "status": "IN_PROGRESS",
                "pilot_directory": pilot_root.relative_to(root).as_posix(),
                "selection": selection,
                "reason": "pilot_run PASS exists but completion line is not committed",
            }, evidence, []
        terminal_status = "COMPLETE_PASS"
        require_complete = True
    elif semantic_summary is not None:
        terminal_status = "EVIDENCE_COMPLETE_TRIAL_FAILED_SPEED_EXPOSURE"
        require_complete = True
    elif failure_summary is not None:
        terminal_status = "COMPLETE_FAILED"
        require_complete = False
    else:
        return {
            "status": "IN_PROGRESS",
            "pilot_directory": pilot_root.relative_to(root).as_posix(),
            "selection": selection,
            "reason": "pilot_run is not accompanied by an atomic terminal marker",
        }, evidence, []

    result, result_evidence, result_visuals = _optional_pilot_attempt(
        root, pilot_root, require_complete=require_complete
    )
    evidence.extend(result_evidence)
    visuals.extend(result_visuals)
    return {
        "status": terminal_status,
        "pilot_directory": pilot_root.relative_to(root).as_posix(),
        "selection": selection,
        "validation_boundary": "exploratory_simulation_only",
        "real_vehicle_ready": False,
        "pilot_run": run_summary,
        "failure": failure_summary,
        "semantic_summary": semantic_summary,
        "result": result,
    }, evidence, visuals


def _snapshot_digest(
    evidence: Iterable[Mapping[str, Any]], visuals: Iterable[Mapping[str, Any]]
) -> str:
    records = {
        (item["campaign_relative_path"], item["sha256"], item["size_bytes"])
        for item in (*tuple(evidence), *tuple(visuals))
    }
    canonical = json.dumps(sorted(records), separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _manifest_relative_path(campaign_relative_path: str) -> str:
    return posixpath.relpath(campaign_relative_path, "40_visuals")


def collect_campaign(
    campaign_root: Path,
    trial_loader: Callable[[Path, str], dict[str, Any]] = _load_comparator_trial,
    *,
    pilot_directory: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    supplied_root = campaign_root.expanduser()
    if supplied_root.is_symlink() or not supplied_root.is_dir():
        raise CampaignSummaryError(f"campaign root is not a regular directory: {supplied_root}")
    root = supplied_root.resolve()
    evidence: list[dict[str, Any]] = []
    visuals: list[dict[str, Any]] = []
    scenario_rows: list[dict[str, Any]] = []
    loaded_town07_baseline: dict[str, Any] | None = None

    for spec in SCENARIOS:
        scenario_root = root / "20_30kph_control_ab" / spec["id"]
        baseline_root = scenario_root / spec["baseline"]
        candidate_root = scenario_root / spec["candidate"]
        (
            baseline_record,
            baseline_loaded,
            baseline_evidence,
            baseline_visuals,
        ) = _collect_trial(
            root, baseline_root, "baseline", spec, trial_loader
        )
        (
            candidate_record,
            candidate_loaded,
            candidate_evidence,
            candidate_visuals,
        ) = _collect_trial(
            root, candidate_root, "candidate", spec, trial_loader
        )
        if baseline_record["route_sha256"] != candidate_record["route_sha256"]:
            raise CampaignSummaryError(f"{spec['id']} A/B route digests differ")
        comparison_base = scenario_root / "comparison" / spec["comparison"]
        comparison, comparison_evidence, comparison_visual = _validate_comparison(
            root,
            comparison_base.with_suffix(".json"),
            comparison_base.with_suffix(".png"),
            spec,
            baseline_loaded,
            candidate_loaded,
        )
        evidence.extend(baseline_evidence)
        evidence.extend(candidate_evidence)
        evidence.append(comparison_evidence)
        visuals.extend(baseline_visuals)
        visuals.extend(candidate_visuals)
        visuals.append(comparison_visual)
        scenario_rows.append(
            {
                "scenario": spec["id"],
                "town": spec["town"],
                "route_scenarios": list(spec["route_scenarios"]),
                "candidate_id": spec["candidate_id"],
                "baseline": baseline_record,
                "candidate": candidate_record,
                "comparison": comparison,
            }
        )
        if spec["id"] == "town07_straight":
            loaded_town07_baseline = baseline_record

    if loaded_town07_baseline is None:
        raise CampaignSummaryError("Town07 baseline was not collected")
    stutter, stutter_evidence = _collect_stutter(root, loaded_town07_baseline)
    evidence.extend(stutter_evidence)
    sixty, sixty_evidence, sixty_visuals = _collect_60kph(
        root, pilot_directory=pilot_directory
    )
    evidence.extend(sixty_evidence)
    visuals.extend(sixty_visuals)

    decisions = [row["comparison"]["decision"] for row in scenario_rows]
    overall_decision = "ACCEPT" if all(value == "ACCEPT" for value in decisions) else "HOLD"
    visuals.sort(key=lambda item: item["id"])
    evidence.sort(
        key=lambda item: (
            item["campaign_relative_path"],
            str(item.get("scenario", "")),
            str(item.get("variant", "")),
        )
    )
    if len({item["id"] for item in visuals}) != len(visuals):
        raise CampaignSummaryError("visual asset IDs are duplicated")
    snapshot = _snapshot_digest(evidence, visuals)
    manifest_assets: list[dict[str, Any]] = []
    for item in visuals:
        asset = dict(item)
        asset["path_from_manifest"] = _manifest_relative_path(
            asset["campaign_relative_path"]
        )
        manifest_assets.append(asset)
    manifest = {
        "schema_version": 1,
        "kind": "runtime_control_campaign_visual_manifest",
        "campaign_id": root.name,
        "copy_policy": "reference_only_no_asset_duplication",
        "source_snapshot_sha256": snapshot,
        "asset_count": len(manifest_assets),
        "assets": manifest_assets,
    }
    summary = {
        "schema_version": 1,
        "kind": "autoware_vad_runtime_control_campaign_summary",
        "campaign_id": root.name,
        "source_snapshot_sha256": snapshot,
        "campaign_status": f"COMPLETE_30KPH_60KPH_{sixty['status']}",
        "overall_control_decision": overall_decision,
        "real_vehicle_ready": False,
        "validation_boundary": "CARLA simulation evidence only",
        "30kph_control_ab": {
            "status": "COMPLETE",
            "scenario_count": len(scenario_rows),
            "accept_count": decisions.count("ACCEPT"),
            "hold_count": decisions.count("HOLD"),
            "scenarios": scenario_rows,
        },
        "runtime_stutter_before_after": stutter,
        "60kph_pilot": sixty,
        "visual_manifest": "40_visuals/visual_manifest.json",
        "evidence_file_count": len(evidence),
        "evidence": evidence,
    }
    return summary, manifest


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}"
    return str(value)


def render_markdown(summary: Mapping[str, Any], manifest: Mapping[str, Any]) -> str:
    control = summary["30kph_control_ab"]
    lines = [
        "# Runtime / control campaign summary",
        "",
        "> Deterministic report generated from canonical owner, route-result, runtime-health, control-analysis, and comparison evidence. No visual asset is copied.",
        "",
        "## Campaign verdict",
        "",
        f"- 30 km/h evidence: **{control['status']}** ({control['scenario_count']} scenarios)",
        f"- Control decision: **{summary['overall_control_decision']}** ({control['accept_count']} ACCEPT / {control['hold_count']} HOLD)",
        f"- 60 km/h pilot: **{summary['60kph_pilot']['status']}**",
        "- Real-vehicle readiness: **NO** (CARLA simulation evidence only)",
        f"- Source snapshot SHA-256: `{summary['source_snapshot_sha256']}`",
        "",
        "## 30 km/h control A/B",
        "",
        "| Scenario | A outcome | B outcome | A/B max speed (m/s) | A/B tracking RMSE (m/s) | A/B max CTE (m) | A/B route RTF | Decision | Failed checks |",
        "|---|---|---|---:|---:|---:|---:|---|---|",
    ]
    for row in control["scenarios"]:
        a = row["baseline"]
        b = row["candidate"]
        checks = ", ".join(row["comparison"]["failed_checks"]) or "none"
        lines.append(
            "| {scenario} | {ao} | {bo} | {aspeed} / {bspeed} | {armse} / {brmse} | {acte} / {bcte} | {artf} / {brtf} | **{decision}** | {checks} |".format(
                scenario=row["scenario"],
                ao=a["route_outcome"],
                bo=b["route_outcome"],
                aspeed=_fmt(a["metrics"]["maximum_speed_mps"]),
                bspeed=_fmt(b["metrics"]["maximum_speed_mps"]),
                armse=_fmt(a["metrics"]["target_tracking_rmse_mps"]),
                brmse=_fmt(b["metrics"]["target_tracking_rmse_mps"]),
                acte=_fmt(a["metrics"]["maximum_cte_m"]),
                bcte=_fmt(b["metrics"]["maximum_cte_m"]),
                artf=_fmt(a["metrics"]["route_rtf"]),
                brtf=_fmt(b["metrics"]["route_rtf"]),
                decision=row["comparison"]["decision"],
                checks=checks,
            )
        )

    lines.extend(["", "## Runtime stutter: before / after", ""])
    stutter = summary["runtime_stutter_before_after"]
    if stutter["status"] == "AVAILABLE":
        before = stutter["before"]["median_of_attempt_medians"]
        after = stutter["after"]
        change = stutter["change"]
        lines.extend(
            [
                "| Metric | Before: rejected fresh-CARLA attempts | After: selected healthy baseline | Change |",
                "|---|---:|---:|---:|",
                f"| RTF | {_fmt(before['rtf'])} | {_fmt(after['rtf']['median'])} | {_fmt(change['rtf_ratio_after_over_before'], 2)}× |",
                f"| Minimum camera wall rate | {_fmt(before['minimum_camera_wall_rate_hz'])} Hz | {_fmt(after['minimum_camera_wall_rate_hz']['median'])} Hz | {_fmt(change['camera_rate_ratio_after_over_before'], 2)}× |",
                f"| Six-camera receipt p95 | {_fmt(before['camera_bundle_receipt_p95_sec'] * 1000.0)} ms | {_fmt(after['camera_bundle_receipt_p95_sec']['median'] * 1000.0)} ms | {_fmt(change['bundle_receipt_p95_reduction_percent'], 1)}% reduction |",
                f"| CAM_FRONT publish timing | {_fmt(before['cam_front_publish_ms'])} ms | not emitted by PASS health schema | n/a |",
                "",
                "The before observations were blocked before engagement; they are runtime/camera-delivery evidence, not driving-control measurements.",
            ]
        )
    else:
        lines.append(f"Status: **{stutter['status']}** — {stutter.get('reason', '')}")

    sixty = summary["60kph_pilot"]
    lines.extend(["", "## 60 km/h exploratory pilot", "", f"Status: **{sixty['status']}**."])
    if sixty.get("result"):
        result = sixty["result"]
        lines.extend(
            [
                "",
                f"- Route result: `{'PASS' if result['success'] else 'FAIL'}` — {result.get('reason')}",
                f"- Maximum speed: {_fmt(result['metrics'].get('maximum_observed_speed_mps'))} m/s",
                f"- Maximum CTE: {_fmt(result['metrics'].get('maximum_absolute_cte_m'))} m",
            ]
        )
    elif sixty.get("reason"):
        lines.extend(["", sixty["reason"]])

    lines.extend(
        [
            "",
            "## Visual evidence manifest",
            "",
            f"The reference-only manifest contains **{manifest['asset_count']}** assets. Paths are relative to the canonical campaign; SHA-256 and byte size are recorded for every asset.",
            "",
            "| Scenario | Variant | Kind | Referenced asset |",
            "|---|---|---|---|",
        ]
    )
    for asset in manifest["assets"]:
        path = "../" + asset["campaign_relative_path"]
        lines.append(
            f"| {asset['scenario']} | {asset['variant']} | {asset['kind']} | [{Path(asset['campaign_relative_path']).name}]({path}) |"
        )
    lines.extend(["", "See [`../40_visuals/visual_manifest.json`](../40_visuals/visual_manifest.json) for hashes and exact paths.", ""])
    return "\n".join(lines)


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink() or path.parent.is_symlink():
        raise CampaignSummaryError(f"refusing to replace symlinked output: {path}")
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def write_campaign_reports(
    campaign_root: Path, *, pilot_directory: str | None = None
) -> tuple[Path, Path, Path]:
    root = campaign_root.expanduser().resolve()
    summary, manifest = collect_campaign(root, pilot_directory=pilot_directory)
    json_path = root / "50_reports/runtime_control_campaign_summary.json"
    markdown_path = root / "50_reports/runtime_control_campaign_summary.md"
    manifest_path = root / "40_visuals/visual_manifest.json"
    json_text = json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    manifest_text = json.dumps(
        manifest, indent=2, sort_keys=True, ensure_ascii=False
    ) + "\n"
    markdown_text = render_markdown(summary, manifest)
    _atomic_write(json_path, json_text)
    _atomic_write(markdown_path, markdown_text)
    _atomic_write(manifest_path, manifest_text)
    return json_path, markdown_path, manifest_path


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--campaign-root",
        type=Path,
        default=DEFAULT_CAMPAIGN_ROOT,
        help=f"canonical campaign root (default: {DEFAULT_CAMPAIGN_ROOT})",
    )
    parser.add_argument(
        "--pilot-directory",
        help=(
            "explicit child directory under 30_60kph; required to resolve more "
            "than one active pilot and never inferred by version or mtime"
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        outputs = write_campaign_reports(
            args.campaign_root, pilot_directory=args.pilot_directory
        )
    except CampaignSummaryError as error:
        print(f"campaign summary error: {error}", file=sys.stderr)
        return 2
    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
