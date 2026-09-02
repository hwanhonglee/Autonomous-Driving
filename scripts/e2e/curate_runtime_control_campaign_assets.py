#!/usr/bin/env python3
"""Publish a small, deterministic visual bundle from the canonical campaign.

The campaign summary and its reference-only visual manifest are the authority for
trial selection.  This tool never scans for the newest pilot and never mutates
source evidence.  It copies only review-oriented PNG/GIF files into a readable
hierarchy and seals every managed output in a deterministic manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMPAIGN_ROOT = (
    REPOSITORY_ROOT
    / "artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1"
)
DEFAULT_DOCS_ROOT = (
    REPOSITORY_ROOT
    / "docs/assets/validation/2026-09-02-runtime-control-campaign-v1"
)

SUMMARY_RELATIVE_PATH = Path("50_reports/runtime_control_campaign_summary.json")
REFERENCE_MANIFEST_RELATIVE_PATH = Path("40_visuals/visual_manifest.json")
PUBLICATION_ROOT_RELATIVE_PATH = Path("40_visuals")
PUBLICATION_MANIFEST = "publication_manifest.json"
README = "README.md"
CHECKSUMS = "SHA256SUMS"

SCENARIO_ORDER = ("town07_straight", "c_track_turn", "town03_turn")
SCENARIO_TITLES = {
    "town07_straight": "Town07 straight",
    "c_track_turn": "C-track turn",
    "town03_turn": "Town03 turn",
    "town06_straight": "Town06 straight",
}
SCENARIO_MAPS = {
    "town07_straight": "Town07",
    "c_track_turn": "C_track_1_0_7",
    "town03_turn": "Town03",
}

SELECTED_30KPH_REGRESSION_DIRECTORY = (
    "C_selected_baseline_depth1_loopback_regression_001"
)
SELECTED_30KPH_REGRESSION_ATTEMPT = "attempt_001"
SELECTED_30KPH_REGRESSION_VARIANT = "selected_baseline_regression"
SELECTED_30KPH_TRANSPORT_PROFILE = (
    "carla_vad_camera_source_5hz_best_effort_image_v2"
)

CORE_ASSETS = (
    (
        "fullscreen",
        "fullscreen",
        "01_autoware_vehicle_centered_fullscreen.png",
        "Autoware/RViz vehicle-centered fullscreen",
        "image/png",
    ),
    (
        "drive",
        "drive",
        "02_autoware_drive.gif",
        "Autoware/RViz driving sequence",
        "image/gif",
    ),
    (
        "path_vs_control",
        "path_control",
        "03_path_vs_control.png",
        "Path versus control analysis",
        "image/png",
    ),
)
DERIVED_ASSETS = (
    (
        "steering_tracking",
        "steering_tracking.png",
        "04_steering_tracking.png",
        "Steering tracking analysis",
        "image/png",
    ),
    (
        "speed_profile",
        "speed_profile.png",
        "05_speed_profile.png",
        "Speed profile analysis",
        "image/png",
    ),
)
EXACT_TRIAL_CORE_FILES = {
    "fullscreen": "autoware_rviz_fullscreen.png",
    "drive": "autoware_rviz_drive.gif",
    "path_control": "path_vs_control.png",
}

TERMINAL_60KPH_STATUSES = {
    "COMPLETE_PASS",
    "COMPLETE_FAILED",
    "EVIDENCE_COMPLETE_TRIAL_FAILED_SPEED_EXPOSURE",
}
NONTERMINAL_60KPH_STATUSES = {"ABSENT", "IN_PROGRESS"}
BANNED_SOURCE_SUFFIXES = {".mkv", ".mp4", ".db3", ".mcap", ".log", ".bag"}
HEX_DIGITS = frozenset("0123456789abcdef")


class CurationError(RuntimeError):
    """Raised before publication when an evidence or ownership check fails."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CurationError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _validate_finite(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise CurationError(f"non-finite number in {label}")
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
                CurationError(f"non-finite JSON constant {value} in {label}")
            ),
        )
    except CurationError:
        raise
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CurationError(f"cannot read {label} at {path}: {error}") from error
    if not isinstance(payload, dict):
        raise CurationError(f"{label} must be a JSON object: {path}")
    _validate_finite(payload, label)
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _valid_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and set(value).issubset(HEX_DIGITS)
    )


def _canonical_digest(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return _sha256_bytes(encoded)


def _safe_relative(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise CurationError(f"{label} must be a non-empty relative path")
    candidate = Path(value)
    if (
        candidate.is_absolute()
        or value != candidate.as_posix()
        or any(part in {"", ".", ".."} for part in candidate.parts)
    ):
        raise CurationError(f"unsafe {label}: {value!r}")
    return candidate


def _regular_within(root: Path, relative: Path, label: str) -> Path:
    candidate = root / relative
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise CurationError(f"{label} may not use symlinks: {candidate}")
    if not candidate.is_file():
        raise CurationError(f"missing {label}: {candidate}")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise CurationError(f"cannot resolve {label}: {candidate}: {error}") from error
    if root != resolved and root not in resolved.parents:
        raise CurationError(f"{label} escapes root: {candidate}")
    if resolved.stat().st_size <= 0:
        raise CurationError(f"empty {label}: {candidate}")
    return resolved


def _verify_reference(
    campaign_root: Path, reference: Mapping[str, Any], label: str
) -> Path:
    relative = _safe_relative(reference.get("campaign_relative_path"), f"{label} path")
    expected_sha = reference.get("sha256")
    expected_size = reference.get("size_bytes")
    if not _valid_sha256(expected_sha):
        raise CurationError(f"invalid {label} SHA-256")
    if isinstance(expected_size, bool) or not isinstance(expected_size, int) or expected_size <= 0:
        raise CurationError(f"invalid {label} byte size")
    source = _regular_within(campaign_root, relative, label)
    actual_size = source.stat().st_size
    if actual_size != expected_size:
        raise CurationError(
            f"{label} size drift: expected {expected_size}, observed {actual_size}"
        )
    actual_sha = _sha256(source)
    if actual_sha != expected_sha:
        raise CurationError(
            f"{label} hash drift: expected {expected_sha}, observed {actual_sha}"
        )
    return source


def _validate_media(path: Path, mime_type: str, label: str) -> None:
    with path.open("rb") as stream:
        prefix = stream.read(8)
    if mime_type == "image/png" and prefix != b"\x89PNG\r\n\x1a\n":
        raise CurationError(f"{label} is not a PNG file")
    if mime_type == "image/gif" and prefix[:6] not in {b"GIF87a", b"GIF89a"}:
        raise CurationError(f"{label} is not a GIF file")


def _validate_desktop_capture(
    campaign_root: Path,
    summary_record: Mapping[str, Any],
    trial_directory: Path,
    label: str,
) -> None:
    evidence = summary_record.get("evidence")
    reference = evidence.get("desktop_capture.json") if isinstance(evidence, dict) else None
    if not isinstance(reference, dict):
        raise CurationError(f"{label} lacks hashed desktop-capture evidence")
    desktop_path = _verify_reference(campaign_root, reference, f"{label} desktop capture")
    if desktop_path.parent != trial_directory:
        raise CurationError(f"{label} desktop capture is outside the selected trial")
    desktop = _read_json(desktop_path, f"{label} desktop capture")
    capture = desktop.get("capture_source")
    overlay = desktop.get("desktop_overlay_check")
    view = desktop.get("rviz_view_contract")
    if (
        desktop.get("schema_version") != 1
        or desktop.get("candidate_observed") is not True
        or desktop.get("capture_started_after_candidate") is not True
        or not isinstance(capture, dict)
        or capture.get("root_capture") is not False
        or capture.get("shell_surfaces_excluded") is not True
        or not isinstance(overlay, dict)
        or overlay.get("passed") is not True
        or not isinstance(view, dict)
        or view.get("vehicle_centered") is not True
    ):
        raise CurationError(f"{label} vehicle-centered capture contract did not pass")
    topics = view.get("visible_path_topics")
    required_topics = {
        "/planning/trajectory",
        "/planning/vad_route/actual_path",
        "/planning/vad_route/reference_path",
    }
    if not isinstance(topics, list) or not required_topics.issubset(set(topics)):
        raise CurationError(f"{label} path-visibility contract is incomplete")


def _source_asset(
    *,
    asset_id: str,
    title: str,
    speed_class: str,
    scenario: str,
    variant: str,
    kind: str,
    mime_type: str,
    source: Path,
    campaign_root: Path,
    destination: Path,
    expected_sha256: str | None = None,
    expected_size: int | None = None,
) -> dict[str, Any]:
    if source.suffix.lower() in BANNED_SOURCE_SUFFIXES:
        raise CurationError(f"bulk media/log source is forbidden: {source}")
    relative = source.relative_to(campaign_root)
    source = _regular_within(campaign_root, relative, title)
    _validate_media(source, mime_type, title)
    size = source.stat().st_size
    digest = _sha256(source)
    if expected_size is not None and size != expected_size:
        raise CurationError(
            f"{title} size drift: expected {expected_size}, observed {size}"
        )
    if expected_sha256 is not None and digest != expected_sha256:
        raise CurationError(
            f"{title} hash drift: expected {expected_sha256}, observed {digest}"
        )
    return {
        "id": asset_id,
        "title": title,
        "speed_class": speed_class,
        "scenario": scenario,
        "variant": variant,
        "kind": kind,
        "mime_type": mime_type,
        "source_campaign_relative_path": relative.as_posix(),
        "source_sha256": digest,
        "source_size_bytes": size,
        "published_relative_path": destination.as_posix(),
        "published_sha256": digest,
        "published_size_bytes": size,
        "_source_path": source,
    }


def _find_manifest_asset(
    assets: Sequence[Mapping[str, Any]],
    *,
    speed_class: str,
    scenario: str,
    variant: str,
    kind: str,
) -> Mapping[str, Any]:
    matches = [
        asset
        for asset in assets
        if asset.get("speed_class") == speed_class
        and asset.get("scenario") == scenario
        and asset.get("variant") == variant
        and asset.get("kind") == kind
    ]
    if len(matches) != 1:
        raise CurationError(
            "expected exactly one reference asset for "
            f"{speed_class}/{scenario}/{variant}/{kind}; found {len(matches)}"
        )
    return matches[0]


def _variant_title(role: str, record: Mapping[str, Any]) -> str:
    if role == "baseline":
        return "A baseline"
    candidate_id = record.get("candidate_id")
    if not isinstance(candidate_id, str) or not candidate_id:
        raise CurationError("candidate record lacks candidate_id")
    return "B candidate (" + candidate_id.replace("_", " ") + ")"


def _collect_trial_assets(
    campaign_root: Path,
    manifest_assets: Sequence[Mapping[str, Any]],
    summary_record: Mapping[str, Any],
    *,
    speed_class: str,
    scenario: str,
    variant: str,
    destination_directory: Path,
) -> list[dict[str, Any]]:
    scenario_title = SCENARIO_TITLES.get(scenario, scenario.replace("_", " "))
    variant_title = _variant_title(variant, summary_record) if speed_class == "30kph" else "Selected pilot"
    collected: list[dict[str, Any]] = []
    trial_directory: Path | None = None
    for published_kind, source_kind, filename, asset_title, mime_type in CORE_ASSETS:
        reference = _find_manifest_asset(
            manifest_assets,
            speed_class=speed_class,
            scenario=scenario,
            variant=variant,
            kind=source_kind,
        )
        source = _verify_reference(
            campaign_root,
            reference,
            f"{speed_class} {scenario} {variant} {source_kind}",
        )
        if trial_directory is None:
            trial_directory = source.parent
        elif source.parent != trial_directory:
            raise CurationError(
                f"{speed_class} {scenario} {variant} assets span multiple trials"
            )
        collected.append(
            _source_asset(
                asset_id=f"{speed_class}.{scenario}.{variant}.{published_kind}",
                title=f"{scenario_title} · {variant_title} · {asset_title}",
                speed_class=speed_class,
                scenario=scenario,
                variant=variant,
                kind=published_kind,
                mime_type=mime_type,
                source=source,
                campaign_root=campaign_root,
                destination=destination_directory / filename,
                expected_sha256=reference["sha256"],
                expected_size=reference["size_bytes"],
            )
        )
    assert trial_directory is not None
    _validate_desktop_capture(
        campaign_root, summary_record, trial_directory, f"{speed_class} {scenario} {variant}"
    )
    for kind, source_name, filename, asset_title, mime_type in DERIVED_ASSETS:
        collected.append(
            _source_asset(
                asset_id=f"{speed_class}.{scenario}.{variant}.{kind}",
                title=f"{scenario_title} · {variant_title} · {asset_title}",
                speed_class=speed_class,
                scenario=scenario,
                variant=variant,
                kind=kind,
                mime_type=mime_type,
                source=trial_directory / source_name,
                campaign_root=campaign_root,
                destination=destination_directory / filename,
            )
        )
    return collected


def _selected_30kph_regression(
    campaign_root: Path, scenario: str
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    """Resolve and validate one exact post-fix regression without directory scans."""
    owner_relative = (
        Path("20_30kph_control_ab")
        / scenario
        / SELECTED_30KPH_REGRESSION_DIRECTORY
    )
    owner_summary_path = _regular_within(
        campaign_root,
        owner_relative / "owned_trial_summary.json",
        f"{scenario} selected regression owner summary",
    )
    owner_summary = _read_json(
        owner_summary_path, f"{scenario} selected regression owner summary"
    )
    attempts = owner_summary.get("attempts")
    if (
        owner_summary.get("schema_version") != 1
        or owner_summary.get("status") != "PASS"
        or owner_summary.get("map") != SCENARIO_MAPS[scenario]
        or owner_summary.get("selected_attempt")
        != SELECTED_30KPH_REGRESSION_ATTEMPT
        or not isinstance(attempts, list)
        or len(attempts) != 1
        or not isinstance(attempts[0], dict)
        or attempts[0].get("attempt_id") != SELECTED_30KPH_REGRESSION_ATTEMPT
        or attempts[0].get("process_exit_status") != 0
        or attempts[0].get("runtime_health_status") != "PASS"
    ):
        raise CurationError(f"{scenario} selected regression did not pass exactly once")
    trial_options = owner_summary.get("trial_options")
    required_options = {
        "--speed-30kph",
        "--camera-source-5hz",
        "--visualize",
        "--capture-desktop",
    }
    if not isinstance(trial_options, list) or not required_options.issubset(
        set(trial_options)
    ):
        raise CurationError(f"{scenario} selected regression options are incomplete")

    trial_relative = (
        owner_relative
        / "attempts"
        / SELECTED_30KPH_REGRESSION_ATTEMPT
    )
    result_path = _regular_within(
        campaign_root,
        trial_relative / "result.json",
        f"{scenario} selected regression result",
    )
    result = _read_json(result_path, f"{scenario} selected regression result")
    if result.get("schema_version") != 1 or result.get("success") is not True:
        raise CurationError(f"{scenario} selected regression route result did not pass")

    health_path = _regular_within(
        campaign_root,
        trial_relative / "runtime_health.json",
        f"{scenario} selected regression runtime health",
    )
    health = _read_json(health_path, f"{scenario} selected regression runtime health")
    contract = health.get("contract")
    transport = contract.get("camera_transport") if isinstance(contract, dict) else None
    image_graph = health.get("camera_image_graph")
    if (
        health.get("schema_version") != 1
        or health.get("status") != "PASS"
        or not isinstance(transport, dict)
        or transport.get("profile_id") != SELECTED_30KPH_TRANSPORT_PROFILE
        or transport.get("camera_image_endpoint_depth") != 1
        or transport.get("camera_image_publisher_reliability") != "best_effort"
        or transport.get("vad_image_subscription_reliability") != "best_effort"
        or transport.get("rviz_image_subscription_reliability") != "best_effort"
        or transport.get("cyclonedds_loopback_interface_required") is not True
        or transport.get("ros_localhost_only_expected") != "0"
        or not isinstance(image_graph, dict)
        or image_graph.get("status") != "PASS"
    ):
        raise CurationError(
            f"{scenario} selected regression depth-1 loopback health contract failed"
        )

    desktop_path = _regular_within(
        campaign_root,
        trial_relative / "desktop_capture.json",
        f"{scenario} selected regression desktop capture",
    )
    desktop_reference = {
        "campaign_relative_path": desktop_path.relative_to(campaign_root).as_posix(),
        "sha256": _sha256(desktop_path),
        "size_bytes": desktop_path.stat().st_size,
    }
    selected_record = {"evidence": {"desktop_capture.json": desktop_reference}}
    provenance = {
        "scenario": scenario,
        "status": "PASS",
        "campaign_relative_directory": owner_relative.as_posix(),
        "selected_attempt": SELECTED_30KPH_REGRESSION_ATTEMPT,
        "transport_profile": SELECTED_30KPH_TRANSPORT_PROFILE,
        "owner_summary_sha256": _sha256(owner_summary_path),
        "route_result_sha256": _sha256(result_path),
        "runtime_health_sha256": _sha256(health_path),
        "desktop_capture_sha256": desktop_reference["sha256"],
    }
    return campaign_root / trial_relative, selected_record, provenance


def _collect_exact_trial_assets(
    campaign_root: Path,
    summary_record: Mapping[str, Any],
    *,
    trial_directory: Path,
    speed_class: str,
    scenario: str,
    variant: str,
    variant_title: str,
    destination_directory: Path,
) -> list[dict[str, Any]]:
    scenario_title = SCENARIO_TITLES.get(scenario, scenario.replace("_", " "))
    _validate_desktop_capture(
        campaign_root,
        summary_record,
        trial_directory,
        f"{speed_class} {scenario} {variant}",
    )
    collected: list[dict[str, Any]] = []
    for published_kind, source_kind, filename, asset_title, mime_type in CORE_ASSETS:
        collected.append(
            _source_asset(
                asset_id=f"{speed_class}.{scenario}.{variant}.{published_kind}",
                title=f"{scenario_title} · {variant_title} · {asset_title}",
                speed_class=speed_class,
                scenario=scenario,
                variant=variant,
                kind=published_kind,
                mime_type=mime_type,
                source=trial_directory / EXACT_TRIAL_CORE_FILES[source_kind],
                campaign_root=campaign_root,
                destination=destination_directory / filename,
            )
        )
    for kind, source_name, filename, asset_title, mime_type in DERIVED_ASSETS:
        collected.append(
            _source_asset(
                asset_id=f"{speed_class}.{scenario}.{variant}.{kind}",
                title=f"{scenario_title} · {variant_title} · {asset_title}",
                speed_class=speed_class,
                scenario=scenario,
                variant=variant,
                kind=kind,
                mime_type=mime_type,
                source=trial_directory / source_name,
                campaign_root=campaign_root,
                destination=destination_directory / filename,
            )
        )
    return collected


def _validate_upstream(
    campaign_root: Path,
) -> tuple[dict[str, Any], dict[str, Any], Path, Path]:
    summary_path = _regular_within(
        campaign_root, SUMMARY_RELATIVE_PATH, "campaign summary"
    )
    manifest_path = _regular_within(
        campaign_root, REFERENCE_MANIFEST_RELATIVE_PATH, "reference visual manifest"
    )
    summary = _read_json(summary_path, "campaign summary")
    manifest = _read_json(manifest_path, "reference visual manifest")
    campaign_id = campaign_root.name
    if (
        summary.get("schema_version") != 1
        or summary.get("kind") != "autoware_vad_runtime_control_campaign_summary"
        or summary.get("campaign_id") != campaign_id
        or summary.get("visual_manifest") != REFERENCE_MANIFEST_RELATIVE_PATH.as_posix()
    ):
        raise CurationError("campaign summary identity is invalid")
    if (
        manifest.get("schema_version") != 1
        or manifest.get("kind") != "runtime_control_campaign_visual_manifest"
        or manifest.get("campaign_id") != campaign_id
        or manifest.get("copy_policy") != "reference_only_no_asset_duplication"
    ):
        raise CurationError("reference visual manifest identity is invalid")
    snapshot = summary.get("source_snapshot_sha256")
    if (
        not _valid_sha256(snapshot)
        or manifest.get("source_snapshot_sha256") != snapshot
    ):
        raise CurationError("summary/reference-manifest source snapshot mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or manifest.get("asset_count") != len(assets):
        raise CurationError("reference visual manifest asset count is invalid")
    ids: set[str] = set()
    paths: set[str] = set()
    for index, asset in enumerate(assets):
        if not isinstance(asset, dict):
            raise CurationError(f"reference asset {index} is not an object")
        asset_id = asset.get("id")
        relative = asset.get("campaign_relative_path")
        if not isinstance(asset_id, str) or not asset_id or asset_id in ids:
            raise CurationError("reference visual asset IDs are invalid or duplicated")
        _safe_relative(relative, f"reference asset {asset_id} path")
        if relative in paths:
            raise CurationError("reference visual asset paths are duplicated")
        ids.add(asset_id)
        paths.add(relative)
    return summary, manifest, summary_path, manifest_path


def collect_assets(campaign_root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    supplied = campaign_root.expanduser()
    if supplied.is_symlink() or not supplied.is_dir():
        raise CurationError(f"campaign root is not a regular directory: {supplied}")
    root = supplied.resolve()
    summary, manifest, summary_path, manifest_path = _validate_upstream(root)
    manifest_assets = manifest["assets"]

    control = summary.get("30kph_control_ab")
    rows = control.get("scenarios") if isinstance(control, dict) else None
    if (
        not isinstance(control, dict)
        or control.get("status") != "COMPLETE"
        or control.get("scenario_count") != len(SCENARIO_ORDER)
        or not isinstance(rows, list)
        or len(rows) != len(SCENARIO_ORDER)
    ):
        raise CurationError("30 kph campaign summary is not complete")
    by_scenario: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        scenario = row.get("scenario") if isinstance(row, dict) else None
        if scenario not in SCENARIO_ORDER or scenario in by_scenario:
            raise CurationError("30 kph scenario set is invalid or duplicated")
        by_scenario[scenario] = row
    if set(by_scenario) != set(SCENARIO_ORDER):
        raise CurationError("30 kph scenario set is incomplete")

    collected: list[dict[str, Any]] = []
    for scenario in SCENARIO_ORDER:
        row = by_scenario[scenario]
        for role, directory in (("baseline", "A_baseline"), ("candidate", "B_candidate")):
            record = row.get(role)
            if not isinstance(record, dict):
                raise CurationError(f"{scenario} lacks {role} summary")
            collected.extend(
                _collect_trial_assets(
                    root,
                    manifest_assets,
                    record,
                    speed_class="30kph",
                    scenario=scenario,
                    variant=role,
                    destination_directory=Path("30kph") / scenario / directory,
                )
            )
        comparison = row.get("comparison")
        if not isinstance(comparison, dict) or comparison.get("decision") not in {"ACCEPT", "HOLD"}:
            raise CurationError(f"{scenario} comparison decision is invalid")
        reference = _find_manifest_asset(
            manifest_assets,
            speed_class="30kph",
            scenario=scenario,
            variant="comparison",
            kind="comparison",
        )
        source = _verify_reference(root, reference, f"{scenario} A/B comparison")
        collected.append(
            _source_asset(
                asset_id=f"30kph.{scenario}.comparison",
                title=(
                    f"{SCENARIO_TITLES[scenario]} · A/B control decision "
                    f"({comparison['decision']})"
                ),
                speed_class="30kph",
                scenario=scenario,
                variant="comparison",
                kind="comparison",
                mime_type="image/png",
                source=source,
                campaign_root=root,
                destination=(
                    Path("30kph")
                    / scenario
                    / "comparison"
                    / "06_A_baseline_vs_B_candidate_control_decision.png"
                ),
                expected_sha256=reference["sha256"],
                expected_size=reference["size_bytes"],
            )
        )

    selected_regressions: list[dict[str, Any]] = []
    for scenario in SCENARIO_ORDER:
        trial_directory, selected_record, provenance = _selected_30kph_regression(
            root, scenario
        )
        collected.extend(
            _collect_exact_trial_assets(
                root,
                selected_record,
                trial_directory=trial_directory,
                speed_class="30kph",
                scenario=scenario,
                variant=SELECTED_30KPH_REGRESSION_VARIANT,
                variant_title="Selected baseline depth-1 loopback regression",
                destination_directory=(
                    Path("30kph") / scenario / "C_selected_baseline_regression"
                ),
            )
        )
        selected_regressions.append(provenance)

    sixty = summary.get("60kph_pilot")
    if not isinstance(sixty, dict) or not isinstance(sixty.get("status"), str):
        raise CurationError("60 kph summary is malformed")
    sixty_status = sixty["status"]
    selected_pilot: dict[str, Any] | None = None
    if sixty_status in TERMINAL_60KPH_STATUSES:
        selection = sixty.get("selection")
        pilot_relative_value = sixty.get("pilot_directory")
        pilot_relative = _safe_relative(pilot_relative_value, "selected 60 kph pilot")
        if pilot_relative.parts != ("30_60kph", pilot_relative.name):
            raise CurationError(
                "completed 60 kph pilot_directory must identify one exact campaign child"
            )
        selection_mode = "summary_pilot_directory"
        if selection is not None:
            if (
                not isinstance(selection, dict)
                or selection.get("mode") not in {
                    "explicit_argument",
                    "sole_active_directory",
                }
                or selection.get("selected_directory") != pilot_relative.name
            ):
                raise CurationError(
                    "60 kph selection metadata contradicts pilot_directory"
                )
            selection_mode = str(selection["mode"])
        result = sixty.get("result")
        if not isinstance(result, dict) or result.get("present") is not True:
            raise CurationError("completed 60 kph pilot lacks a selected trial result")
        sixty_record: dict[str, Any] = {"evidence": {}}
        desktop_matches = [
            item
            for item in summary.get("evidence", [])
            if isinstance(item, dict)
            and item.get("kind") == "desktop_capture.json"
            and isinstance(item.get("campaign_relative_path"), str)
            and item["campaign_relative_path"].startswith(pilot_relative.as_posix() + "/")
        ]
        if len(desktop_matches) != 1:
            raise CurationError("selected 60 kph pilot lacks unique desktop evidence")
        sixty_record["evidence"]["desktop_capture.json"] = {
            key: desktop_matches[0][key]
            for key in ("campaign_relative_path", "sha256", "size_bytes")
        }
        pilot_manifest_assets = [
            asset
            for asset in manifest_assets
            if isinstance(asset, dict) and asset.get("speed_class") == "60kph"
        ]
        if not pilot_manifest_assets or any(
            not str(asset.get("campaign_relative_path", "")).startswith(
                pilot_relative.as_posix() + "/"
            )
            for asset in pilot_manifest_assets
        ):
            raise CurationError("60 kph manifest assets do not belong to selected pilot")
        collected.extend(
            _collect_trial_assets(
                root,
                manifest_assets,
                sixty_record,
                speed_class="60kph",
                scenario="town06_straight",
                variant="pilot",
                destination_directory=Path("60kph/town06_straight/selected_pilot"),
            )
        )
        selected_pilot = {
            "status": sixty_status,
            "campaign_relative_directory": pilot_relative.as_posix(),
            "selection_mode": selection_mode,
        }
    elif sixty_status in NONTERMINAL_60KPH_STATUSES:
        if any(
            isinstance(asset, dict) and asset.get("speed_class") == "60kph"
            for asset in manifest_assets
        ):
            raise CurationError("nonterminal 60 kph status has referenced visual assets")
    else:
        raise CurationError(f"unsupported 60 kph status: {sixty_status}")

    ids = [asset["id"] for asset in collected]
    destinations = [asset["published_relative_path"] for asset in collected]
    if len(ids) != len(set(ids)) or len(destinations) != len(set(destinations)):
        raise CurationError("curated asset IDs or destinations are duplicated")
    collected.sort(key=lambda asset: asset["published_relative_path"])
    metadata = {
        "campaign_id": root.name,
        "source_snapshot_sha256": summary["source_snapshot_sha256"],
        "source_summary_sha256": _sha256(summary_path),
        "source_visual_manifest_sha256": _sha256(manifest_path),
        "campaign_status": summary.get("campaign_status"),
        "overall_control_decision": summary.get("overall_control_decision"),
        "selected_30kph_regressions": selected_regressions,
        "selected_60kph_pilot": selected_pilot,
    }
    return metadata, collected


def _render_readme(
    metadata: Mapping[str, Any], assets: Sequence[Mapping[str, Any]], target: str
) -> bytes:
    lines = [
        "# Autoware VAD runtime/control visual evidence",
        "",
        (
            "This is a curated, review-oriented copy of the canonical CARLA campaign. "
            "Source evidence remains immutable; bags, MKV files, logs, and other bulk "
            "runtime files are intentionally excluded."
        ),
        "",
        f"- Campaign: `{metadata['campaign_id']}`",
        f"- Publication target: `{target}`",
        f"- Campaign status: `{metadata.get('campaign_status')}`",
        f"- 30 km/h control decision: `{metadata.get('overall_control_decision')}`",
        f"- Source snapshot SHA-256: `{metadata['source_snapshot_sha256']}`",
        "- Fullscreen images passed the vehicle-centered and visible-path capture contract.",
        "- Every copied byte is sealed in `publication_manifest.json` and `SHA256SUMS`.",
        "",
        "## 30 km/h A/B evidence",
        "",
        "| Scenario | Variant | Evidence | File |",
        "|---|---|---|---|",
    ]
    for asset in assets:
        if (
            asset["speed_class"] != "30kph"
            or asset["variant"] == SELECTED_30KPH_REGRESSION_VARIANT
        ):
            continue
        lines.append(
            f"| {SCENARIO_TITLES.get(asset['scenario'], asset['scenario'])} "
            f"| {asset['variant']} | {asset['kind']} "
            f"| [{Path(asset['published_relative_path']).name}]({asset['published_relative_path']}) |"
        )
    lines.extend(
        [
            "",
            "## 30 km/h selected baseline regressions",
            "",
            (
                "These are the exact post-fix depth-1 loopback regressions; "
                "the curator never scans for a newest-looking directory."
            ),
            "",
            "| Scenario | Evidence | File |",
            "|---|---|---|",
        ]
    )
    for asset in assets:
        if asset["variant"] != SELECTED_30KPH_REGRESSION_VARIANT:
            continue
        lines.append(
            f"| {SCENARIO_TITLES.get(asset['scenario'], asset['scenario'])} "
            f"| {asset['kind']} "
            f"| [{Path(asset['published_relative_path']).name}]({asset['published_relative_path']}) |"
        )
    lines.extend(["", "## 60 km/h selected pilot", ""])
    pilot = metadata.get("selected_60kph_pilot")
    if pilot is None:
        lines.append("No terminal 60 km/h pilot was selected in the campaign summary.")
    else:
        lines.extend(
            [
                f"- Status: `{pilot['status']}`",
                f"- Explicit source: `{pilot['campaign_relative_directory']}`",
                "",
                "| Evidence | File |",
                "|---|---|",
            ]
        )
        for asset in assets:
            if asset["speed_class"] == "60kph":
                lines.append(
                    f"| {asset['kind']} | [{Path(asset['published_relative_path']).name}]({asset['published_relative_path']}) |"
                )
    lines.extend(
        [
            "",
            "## Integrity",
            "",
            "`publication_manifest.json` records the source and published SHA-256 for each asset. "
            "A later run replaces or removes only paths authorized by the previously verified manifest; "
            "unmanaged files and canonical source evidence are never removed.",
            "",
        ]
    )
    return "\n".join(lines).encode("utf-8")


def _manifest_signature(payload: Mapping[str, Any]) -> str:
    unsigned = dict(payload)
    unsigned.pop("manifest_payload_sha256", None)
    return _canonical_digest(unsigned)


def _public_asset(asset: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in asset.items() if not key.startswith("_")}


def _stage_publication(
    output_root: Path,
    metadata: Mapping[str, Any],
    assets: Sequence[Mapping[str, Any]],
    target: str,
) -> tuple[Path, dict[str, Any]]:
    output_root.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".runtime-control-assets.", dir=output_root.parent))
    try:
        for asset in assets:
            destination = stage / asset["published_relative_path"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(asset["_source_path"], destination)
            if (
                destination.stat().st_size != asset["published_size_bytes"]
                or _sha256(destination) != asset["published_sha256"]
            ):
                raise CurationError(f"staged copy verification failed: {destination}")

        readme_bytes = _render_readme(metadata, assets, target)
        (stage / README).write_bytes(readme_bytes)
        checksum_rows = [
            (_sha256_bytes(readme_bytes), README),
            *[
                (asset["published_sha256"], asset["published_relative_path"])
                for asset in assets
            ],
        ]
        checksum_rows.sort(key=lambda row: row[1])
        checksums_bytes = "".join(
            f"{digest}  {relative}\n" for digest, relative in checksum_rows
        ).encode("utf-8")
        (stage / CHECKSUMS).write_bytes(checksums_bytes)

        managed_files: list[dict[str, Any]] = []
        for relative, role in [
            (README, "readme"),
            (CHECKSUMS, "checksums"),
            *[(asset["published_relative_path"], "visual_asset") for asset in assets],
        ]:
            path = stage / relative
            managed_files.append(
                {
                    "relative_path": relative,
                    "role": role,
                    "sha256": _sha256(path),
                    "size_bytes": path.stat().st_size,
                }
            )
        managed_files.sort(key=lambda row: row["relative_path"])
        managed_paths = sorted(
            [row["relative_path"] for row in managed_files] + [PUBLICATION_MANIFEST]
        )
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "kind": "runtime_control_campaign_asset_publication",
            "curator": Path(__file__).name,
            "campaign_id": metadata["campaign_id"],
            "publication_target": target,
            "source_snapshot_sha256": metadata["source_snapshot_sha256"],
            "source_summary": {
                "campaign_relative_path": SUMMARY_RELATIVE_PATH.as_posix(),
                "sha256": metadata["source_summary_sha256"],
            },
            "source_visual_manifest": {
                "campaign_relative_path": REFERENCE_MANIFEST_RELATIVE_PATH.as_posix(),
                "sha256": metadata["source_visual_manifest_sha256"],
            },
            "selected_30kph_regressions": metadata.get(
                "selected_30kph_regressions"
            ),
            "selected_60kph_pilot": metadata.get("selected_60kph_pilot"),
            "asset_count": len(assets),
            "assets": [_public_asset(asset) for asset in assets],
            "managed_files": managed_files,
            "managed_paths": managed_paths,
        }
        manifest["manifest_payload_sha256"] = _manifest_signature(manifest)
        (stage / PUBLICATION_MANIFEST).write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return stage, manifest
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise


def _validate_prior_manifest(
    output_root: Path, campaign_root: Path, expected_target: str
) -> tuple[dict[str, Any] | None, set[str]]:
    manifest_path = output_root / PUBLICATION_MANIFEST
    if manifest_path.is_symlink():
        raise CurationError(
            f"prior publication manifest may not be a symlink: {manifest_path}"
        )
    if not manifest_path.exists():
        reserved = (README, CHECKSUMS, PUBLICATION_MANIFEST)
        collisions = [
            name
            for name in reserved
            if (output_root / name).exists() or (output_root / name).is_symlink()
        ]
        if collisions:
            raise CurationError(
                "publication control files exist without an ownership manifest: "
                + ", ".join(collisions)
            )
        return None, set()
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise CurationError(f"prior publication manifest is not a regular file: {manifest_path}")
    prior = _read_json(manifest_path, "prior publication manifest")
    if (
        prior.get("schema_version") != 1
        or prior.get("kind") != "runtime_control_campaign_asset_publication"
        or prior.get("curator") != Path(__file__).name
        or prior.get("campaign_id") != campaign_root.name
        or prior.get("publication_target") != expected_target
    ):
        raise CurationError("prior publication manifest identity is invalid")
    expected_signature = prior.get("manifest_payload_sha256")
    if not _valid_sha256(expected_signature) or _manifest_signature(prior) != expected_signature:
        raise CurationError("prior publication manifest signature mismatch")
    managed_paths_value = prior.get("managed_paths")
    managed_files = prior.get("managed_files")
    prior_assets = prior.get("assets")
    if (
        not isinstance(managed_paths_value, list)
        or not isinstance(managed_files, list)
        or not isinstance(prior_assets, list)
    ):
        raise CurationError("prior publication ownership body is malformed")
    managed_paths: set[str] = set()
    for value in managed_paths_value:
        relative = _safe_relative(value, "prior managed path")
        relative_text = relative.as_posix()
        if relative_text == REFERENCE_MANIFEST_RELATIVE_PATH.name:
            raise CurationError("prior publication claims canonical visual_manifest.json")
        if relative_text not in {README, CHECKSUMS, PUBLICATION_MANIFEST} and relative.parts[0] not in {"30kph", "60kph"}:
            raise CurationError(f"prior managed path is outside visual categories: {relative_text}")
        if relative_text in managed_paths:
            raise CurationError("prior managed paths are duplicated")
        managed_paths.add(relative_text)
    if PUBLICATION_MANIFEST not in managed_paths:
        raise CurationError("prior publication manifest does not own itself")
    file_paths: set[str] = set()
    for row in managed_files:
        if not isinstance(row, dict):
            raise CurationError("prior managed file record is malformed")
        relative = _safe_relative(row.get("relative_path"), "prior managed file")
        relative_text = relative.as_posix()
        if relative_text == PUBLICATION_MANIFEST or relative_text in file_paths:
            raise CurationError("prior managed file identities are invalid")
        if relative_text not in managed_paths:
            raise CurationError("prior managed file is not authorized by managed_paths")
        expected_sha = row.get("sha256")
        expected_size = row.get("size_bytes")
        if not _valid_sha256(expected_sha) or isinstance(expected_size, bool) or not isinstance(expected_size, int) or expected_size <= 0:
            raise CurationError(f"invalid prior managed-file digest: {relative_text}")
        path = _regular_within(output_root, relative, f"prior managed file {relative_text}")
        if path.stat().st_size != expected_size or _sha256(path) != expected_sha:
            raise CurationError(f"prior managed output drift: {relative_text}")
        file_paths.add(relative_text)
    if managed_paths != file_paths | {PUBLICATION_MANIFEST}:
        raise CurationError("prior managed path/file sets differ")

    for asset in prior_assets:
        if not isinstance(asset, dict):
            raise CurationError("prior source asset record is malformed")
        relative = _safe_relative(
            asset.get("source_campaign_relative_path"), "prior source asset"
        )
        if relative.parts[0] not in {"20_30kph_control_ab", "30_60kph"}:
            raise CurationError("prior publication source points outside canonical evidence")
        source = _regular_within(campaign_root, relative, "prior source asset")
        if (
            source.stat().st_size != asset.get("source_size_bytes")
            or _sha256(source) != asset.get("source_sha256")
        ):
            raise CurationError(
                f"prior publication source hash drift: {relative.as_posix()}"
            )
    return prior, managed_paths


def _planned_paths(manifest: Mapping[str, Any]) -> set[str]:
    paths = manifest.get("managed_paths")
    if not isinstance(paths, list):
        raise CurationError("staged manifest lacks managed paths")
    return set(paths)


def _validate_destination_collisions(
    output_root: Path, planned: Iterable[str], prior_managed: set[str]
) -> None:
    for relative_text in planned:
        relative = _safe_relative(relative_text, "planned output")
        path = output_root / relative
        current = output_root
        for part in relative.parts[:-1]:
            current = current / part
            if current.is_symlink():
                raise CurationError(f"output path uses a symlink: {path}")
            if current.exists() and not current.is_dir():
                raise CurationError(f"output parent is not a directory: {current}")
        if path.is_symlink():
            raise CurationError(f"refusing to replace symlinked output: {path}")
        if path.exists() and relative_text not in prior_managed:
            raise CurationError(f"refusing to replace unmanaged output: {path}")


def _same_publication(
    output_root: Path, prior: Mapping[str, Any] | None, desired: Mapping[str, Any]
) -> bool:
    if prior != desired:
        return False
    staged_signature = desired.get("manifest_payload_sha256")
    if not _valid_sha256(staged_signature):
        return False
    manifest = _read_json(output_root / PUBLICATION_MANIFEST, "publication manifest")
    return manifest == desired


def _install_stage(
    stage: Path,
    output_root: Path,
    desired: Mapping[str, Any],
    prior: Mapping[str, Any] | None,
    prior_managed: set[str],
) -> str:
    planned = _planned_paths(desired)
    _validate_destination_collisions(output_root, planned, prior_managed)
    if prior is not None and _same_publication(output_root, prior, desired):
        shutil.rmtree(stage)
        return "UNCHANGED"
    output_root.mkdir(parents=True, exist_ok=True)
    for relative_text in sorted(planned - {PUBLICATION_MANIFEST}):
        source = stage / relative_text
        destination = output_root / relative_text
        destination.parent.mkdir(parents=True, exist_ok=True)
        os.replace(source, destination)
    # Commit the ownership record last.  A valid prior manifest remains the
    # authority until every desired content file is installed.
    os.replace(stage / PUBLICATION_MANIFEST, output_root / PUBLICATION_MANIFEST)

    for relative_text in sorted(prior_managed - planned, reverse=True):
        path = output_root / relative_text
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise CurationError(f"refusing to remove non-file managed path: {path}")
        path.unlink(missing_ok=True)
    for category in (output_root / "30kph", output_root / "60kph"):
        if not category.exists() or not category.is_dir() or category.is_symlink():
            continue
        directories = sorted(
            (path for path in category.rglob("*") if path.is_dir() and not path.is_symlink()),
            key=lambda path: len(path.parts),
            reverse=True,
        )
        for directory in directories:
            try:
                directory.rmdir()
            except OSError:
                pass
        try:
            category.rmdir()
        except OSError:
            pass
    shutil.rmtree(stage)
    return "UPDATED"


def curate(
    campaign_root: Path,
    *,
    docs_root: Path | None = None,
) -> list[dict[str, Any]]:
    supplied = campaign_root.expanduser()
    if supplied.is_symlink() or not supplied.is_dir():
        raise CurationError(f"campaign root is not a regular directory: {supplied}")
    root = supplied.resolve()
    metadata, assets = collect_assets(root)
    targets = [(root / PUBLICATION_ROOT_RELATIVE_PATH, "campaign")]
    if docs_root is not None:
        docs_supplied = docs_root.expanduser()
        if docs_supplied.is_symlink():
            raise CurationError(f"docs publication root may not be a symlink: {docs_supplied}")
        docs_resolved = docs_supplied.resolve()
        campaign_output = (root / PUBLICATION_ROOT_RELATIVE_PATH).resolve()
        if docs_resolved == campaign_output:
            raise CurationError("docs publication root duplicates campaign output root")
        targets.append((docs_resolved, "docs_mirror"))

    staged: list[tuple[Path, Path, str, dict[str, Any], dict[str, Any] | None, set[str]]] = []
    try:
        # Validate and stage every target before mutating any destination.
        for output_root, target in targets:
            prior, prior_managed = _validate_prior_manifest(output_root, root, target)
            stage, desired = _stage_publication(output_root, metadata, assets, target)
            _validate_destination_collisions(
                output_root, _planned_paths(desired), prior_managed
            )
            staged.append(
                (stage, output_root, target, desired, prior, prior_managed)
            )
        if (
            _sha256(root / SUMMARY_RELATIVE_PATH)
            != metadata["source_summary_sha256"]
            or _sha256(root / REFERENCE_MANIFEST_RELATIVE_PATH)
            != metadata["source_visual_manifest_sha256"]
        ):
            raise CurationError("upstream report changed while assets were staged")
        results: list[dict[str, Any]] = []
        for stage, output_root, target, desired, prior, prior_managed in staged:
            status = _install_stage(
                stage, output_root, desired, prior, prior_managed
            )
            results.append(
                {
                    "target": target,
                    "output_root": str(output_root),
                    "status": status,
                    "asset_count": desired["asset_count"],
                    "manifest_payload_sha256": desired["manifest_payload_sha256"],
                }
            )
        return results
    finally:
        for stage, *_ in staged:
            if stage.exists():
                shutil.rmtree(stage, ignore_errors=True)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--campaign-root",
        type=Path,
        default=DEFAULT_CAMPAIGN_ROOT,
        help=f"canonical campaign root (default: {DEFAULT_CAMPAIGN_ROOT})",
    )
    parser.add_argument(
        "--mirror-docs",
        action="store_true",
        help=f"also publish to the default docs asset root ({DEFAULT_DOCS_ROOT})",
    )
    parser.add_argument(
        "--docs-root",
        type=Path,
        help="explicit docs mirror root (implies --mirror-docs)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    docs_root = args.docs_root
    if args.mirror_docs and docs_root is None:
        docs_root = DEFAULT_DOCS_ROOT
    try:
        results = curate(args.campaign_root, docs_root=docs_root)
    except CurationError as error:
        print(f"asset curation error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(results, indent=2, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
