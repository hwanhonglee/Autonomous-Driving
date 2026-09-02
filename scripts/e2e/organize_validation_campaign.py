#!/usr/bin/env python3
"""Create a pointer-only validation campaign and safely quarantine audited debris.

The default command only creates the campaign directory skeleton plus an
absolute-path pointer index.  It never copies, links, renames, or deletes an
evidence source.  Cleanup is deliberately split into a hash-bound planning
step and an explicitly confirmed, same-filesystem rename step.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import stat
import statistics
import sys
import tempfile
from typing import Any, Iterator, Mapping, Sequence, TextIO


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMPAIGN_ROOT = (
    REPO_ROOT
    / "artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1"
)
DEFAULT_TMP_ROOT = Path("/tmp")
DEFAULT_STUTTER_SOURCE = (
    DEFAULT_CAMPAIGN_ROOT
    / "20_30kph_control_ab/town07_straight/A_baseline_health_001"
)
DEFAULT_STUTTER_OUTPUT = (
    DEFAULT_CAMPAIGN_ROOT
    / "10_runtime_stutter/town07_straight_A_baseline_health_001"
)
MANIFEST_SCHEMA_VERSION = 1
INDEX_SCHEMA_VERSION = 1
JOURNAL_SCHEMA_VERSION = 1
APPLY_CONFIRMATION = "I_UNDERSTAND_RENAME_QUARANTINE"
RESTORE_CONFIRMATION = "I_UNDERSTAND_RESTORE_QUARANTINE"

CATEGORY_DIRECTORIES = (
    "00_index",
    "10_runtime_stutter",
    "10_runtime_stutter/town07_straight_A_baseline_health_001",
    "15_all_towns_30kph",
    "20_30kph_control_ab",
    "20_30kph_control_ab/town07_straight",
    "20_30kph_control_ab/c_track_turn",
    "20_30kph_control_ab/town03_turn",
    "30_60kph",
    "40_visuals",
    "50_reports",
    "90_quarantine",
    "90_quarantine/artifacts",
    "90_quarantine/tmp",
    "99_integrity",
)

TREE_HASH_ALGORITHM = (
    "sha256 over sorted regular files as "
    "relative_posix_path + NUL + decimal_size + NUL + file_sha256 + LF; "
    "internal symlinks add relative_posix_path + NUL + SYMLINK + NUL + "
    "link_target + LF without dereference; a single file uses its byte SHA-256"
)


class CampaignError(RuntimeError):
    """Raised when a campaign or cleanup safety contract is violated."""


@dataclass(frozen=True)
class CandidateSpec:
    candidate_id: str
    scope: str
    relative_path: str
    observed_status: str
    reason: str


ARTIFACT_CANDIDATES = (
    CandidateSpec(
        "artifact_2026_09_01_town01_capture_smoke_v16pre",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town01_capture_smoke_30kph_v16pre",
        "INCOMPLETE",
        "single-map pre-final smoke run superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_town10hd_repro_v15",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town10hd_repro_30kph_v15",
        "INCOMPLETE",
        "single-map pre-final reproduction superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v10_ctrack",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v10_ctrack",
        "FAILED",
        "intermediate failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v11_ctrack_roi",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v11_ctrack_roi",
        "INCOMPLETE",
        "targeted intermediate matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v12_final",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v12_final",
        "FAILED",
        "failed pre-final matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v13_town05_obb_screen",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v13_town05_obb_screen",
        "FAILED",
        "failed screen-only diagnostic superseded by later evidence",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v13b_town05_obb_capture",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v13b_town05_obb_capture",
        "INCOMPLETE",
        "single-map diagnostic capture superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v14_final",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v14_final",
        "FAILED",
        "failed pre-final matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v2",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v2",
        "FAILED",
        "early failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v3",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v3",
        "FAILED",
        "early failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v4",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v4",
        "FAILED",
        "early failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v5_targeted",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v5_targeted",
        "FAILED",
        "targeted failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v6_targeted",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v6_targeted",
        "FAILED",
        "targeted failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v7_targeted",
        "artifacts",
        "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v7_targeted",
        "FAILED",
        "targeted failed matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v8_town05",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v8_town05",
        "INCOMPLETE",
        "single-map intermediate run superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_matrix_v9_final",
        "artifacts",
        "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v9_final",
        "FAILED",
        "failed pre-final matrix superseded by v16 final",
    ),
    CandidateSpec(
        "artifact_2026_09_01_speed_30kph_v1",
        "artifacts",
        "artifacts/validation/2026-09-01/speed_30kph_v1",
        "SUPERSEDED",
        "legacy speed analysis superseded by the bound 2026-09-02 comparison",
    ),
    CandidateSpec(
        "artifact_2026_09_01_speed_30kph_v2",
        "artifacts",
        "artifacts/validation/2026-09-01/speed_30kph_v2",
        "SUPERSEDED",
        "legacy speed run superseded by the bound 2026-09-02 comparison",
    ),
    CandidateSpec(
        "artifact_2026_09_01_speed_30kph_v4_preflight",
        "artifacts",
        "artifacts/validation/2026-09-01/speed_30kph_v4_preflight",
        "SUPERSEDED",
        "preflight-only data superseded by completed trials",
    ),
    CandidateSpec(
        "artifact_2026_08_31_matrix_contract_archives",
        "artifacts",
        "artifacts/validation/2026-08-31/_matrix_contract_archives",
        "ARCHIVED_LEGACY",
        "already-designated legacy contract archive with no external basename references",
    ),
    CandidateSpec(
        "artifact_2026_08_31_centered_vad_v2",
        "artifacts",
        "artifacts/validation/2026-08-31/centered_vad_v2",
        "SUPERSEDED",
        "centered capture superseded by centered_vad_v3",
    ),
    CandidateSpec(
        "artifact_2026_08_31_town01_left_failed_2p5m",
        "artifacts",
        "artifacts/validation/2026-08-31/town01_left_expert_failed_2p5m",
        "FAILED",
        "explicitly failed expert attempt with no external basename references",
    ),
    CandidateSpec(
        "artifact_2026_09_02_campaign_60kph_image_v1",
        "artifacts",
        "artifacts/validation/2026-09-02/"
        "autoware_vad_runtime_control_campaign_v1/30_60kph/"
        "town06_straight_60kph_pilot_best_effort_image_v1",
        "SUPERSEDED_FAILED",
        "failed pre-depth-1 60 kph pilot superseded by the selected v3 evidence",
    ),
    CandidateSpec(
        "artifact_2026_09_02_campaign_60kph_image_v2",
        "artifacts",
        "artifacts/validation/2026-09-02/"
        "autoware_vad_runtime_control_campaign_v1/30_60kph/"
        "town06_straight_60kph_pilot_best_effort_image_v2",
        "SUPERSEDED_FAILED",
        "incomplete depth-1 bring-up pilot superseded by the selected v3 evidence",
    ),
)


TMP_EXACT_CANDIDATES = (
    "autoware_mission_planner_fix.UD6ymT",
    "pytest-of-a",
    "autoware-catalog-preflight-town02.sDoEsc",
    "autoware-catalog-preflight-town06.lYMRqT",
    "autoware-e2e-coverage-preflight.h4TTDS",
    "autoware_compare_failclosed.zO3mbW",
    "autoware_compare_failclosed_final.8Uo1oU",
    "autoware_longitudinal_30.4uk3le",
    "autoware_longitudinal_30_final.agpq6U",
    "autoware_longitudinal_60.nVeS3F",
    "autoware_longitudinal_60_final.4BFUhR",
    "autoware_longitudinal_60_final.UBj0mi",
    "60_panel_0.png",
    "60_panel_1.png",
    "60_panel_2.png",
    "60_panel_3.png",
    "60_panel_4.png",
    "at79.png",
    "repro60.png",
    "repro60x",
    "seek60",
    "town06_30_contact_verify.png",
    "autoware_e2e_all_maps_dashboard_preview.png",
    "autoware_e2e_desktop_after_dnd.png",
    "autoware_e2e_desktop_dismissed.png",
    "autoware_e2e_desktop_notification_clicked.png",
    "autoware_e2e_x11_capture_probe.GJqpV4.png",
    "autoware-e2e-review-map.json",
    "autoware-v12-qa.Z0swrn",
    "autoware-v9-visual-audit.rglqOU",
    "autoware-vad-matrix-profile.u42Y7G",
    "autoware-vad-matrix-v2.WscPdx",
    "autoware_e2e_carla_vad_full_show_args.txt",
    "autoware_e2e_docs_sha_check.txt",
    "autoware_e2e_env.log",
    "autoware_e2e_env_collect.log",
    "autoware_e2e_env_collect_ctrack.log",
    "autoware_e2e_env_export.log",
    "autoware_e2e_env_export_ctrack.log",
    "autoware_e2e_env_route.log",
    "autoware_e2e_frames.tVtIOi",
    "autoware_e2e_lfs_attrs.txt",
    "autoware_e2e_pid2460883_before.txt",
    "autoware_e2e_sha_check.log",
    "autoware_e2e_sha_check.out",
    "autoware_e2e_speed_plot_tmp_path",
    "autoware_e2e_tl_expected_probe",
    "autoware_e2e_vad_preflight.Zdovu2",
    "autoware_sha_check.out",
    "autoware_sha_check_commit.out",
    "autoware_sha_check_final.out",
    "autoware_shutdown_pycache.PNRtp4",
    "autoware_status_final.txt",
    "autoware_v12_qa.DRjnty",
    "town06_directed_preflight.json",
    "town06_turn001_detail_20260831.png",
    "town06_turn001_gif_grid_20260831.png",
    # Older map-conversion, per-Town QA, and visual-review scratch files found
    # during the final 2026-09-02 audit.  Keep this list exact: an ordinary
    # /tmp glob could capture unrelated user or ROS work.
    "c_track_alignment_check.60j8SA",
    "c_track_campaign_preflight.ew86fqec",
    "town05_aeb.aTnxLW",
    "town05-forensic.u1kpYN",
    "town05_v13b_visual_qa.NItMkQ",
    "town02_v16_qa.QNVyRO",
    "town03-v16-qa.q8w1Mc",
    "c_track_autoware_venvmap.osm",
    "c_track_venvmap_report.json",
    "c_track_autoware_system.osm",
    "c_track_system_report.json",
    "c_track_source.osm",
    "c_track_source.log",
    "c_track_docs.osm",
    "c_track_docs.log",
    "town01_inventory.json",
    "c_track_turn003_gif_grid_20260831.png",
    "c_track_turn003_detail_20260831.png",
    "town01_turn001_gif_grid_20260831.png",
    "town01_turn001_detail_20260831.png",
    "town02_opt_turn001_gif_grid_20260831.png",
    "town03_straight001_gif_grid_20260831.png",
    "town05_opt_turn001_gif_grid_20260831.png",
    "town05_opt_turn001_detail_20260831.png",
    "c_track_1_0_7_directed_preflight.json",
    "town01_directed_preflight.json",
    "town02_opt_directed_preflight.json",
    "town03_directed_preflight.json",
    "town04_directed_preflight.json",
    "town05_opt_directed_preflight.json",
    "town07_directed_preflight.json",
    "town06_straight_60kph_pilot_best_effort_image_v1_trial_stack_launch.txt",
    "town06_straight_60kph_pilot_best_effort_image_v2_trial_stack_launch.txt",
)

TMP_LOCK_PATTERNS = (
    "autoware_e2e_runtime.lock",
    "autoware_e2e_semantic_lidar_world_*.lock",
    "autoware_e2e_turn_recorder_*.lock",
    "autoware_e2e_vad_dataset_recorder_*.lock",
    "autoware_e2e_vad_matrix.lock",
)

LAUNCH_PARAM_PATTERN = re.compile(r"^launch_params_[A-Za-z0-9_]+$")
PROJECT_PROCESS_PATTERNS = (
    re.compile(r"(?:^|/)CarlaUE4(?:\.sh|-Linux-Shipping)?(?:\s|$)", re.IGNORECASE),
    re.compile(r"\bros2\s+launch\b.*\bautoware\b", re.IGNORECASE),
    re.compile(r"\brun_autoware_vad_[^\s/]*\.sh\b", re.IGNORECASE),
    re.compile(r"\brun_route_vad_[^\s/]*\.sh\b", re.IGNORECASE),
    re.compile(r"\brun_recorded_route_trial\.sh\b", re.IGNORECASE),
    re.compile(r"\bautoware_vad_town_matrix\.py\b", re.IGNORECASE),
    re.compile(r"\bcolcon\s+(?:build|test)\b", re.IGNORECASE),
)
RUNTIME_TIMING_FRONT_IMAGE_RE = re.compile(
    r"runtime_timing\s+stage=camera_image_publish\s+camera=CAM_FRONT\s+"
    r"duration_ms=(?P<duration>[0-9]+(?:\.[0-9]+)?)\s+"
    r"monotonic_ns=(?P<monotonic>[0-9]+)\s+"
    r"source_stamp_sec=(?P<source>[0-9]+(?:\.[0-9]+)?)"
    r"(?:\s+suppressed=(?P<suppressed>[0-9]+))?"
)
CAM_FRONT_INFO_TOPIC = "/sensing/camera/CAM_FRONT/camera_info"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def signed_payload(value: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(value)
    result.pop("manifest_sha256", None)
    result["manifest_sha256"] = hashlib.sha256(canonical_json_bytes(result)).hexdigest()
    return result


def verify_signed_payload(value: Mapping[str, Any], label: str) -> None:
    observed = value.get("manifest_sha256")
    unsigned = dict(value)
    unsigned.pop("manifest_sha256", None)
    expected = hashlib.sha256(canonical_json_bytes(unsigned)).hexdigest()
    if not isinstance(observed, str) or observed != expected:
        raise CampaignError(
            f"{label} integrity mismatch: expected {expected}, observed {observed!r}"
        )


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def _absolute(path: Path) -> Path:
    return path.expanduser().absolute()


def _pointer(path: Path, role: str) -> dict[str, Any]:
    absolute = _absolute(path)
    return {
        "path": str(absolute),
        "role": role,
        "exists": absolute.exists(),
        "path_type": (
            "directory" if absolute.is_dir() else "file" if absolute.is_file() else "missing"
        ),
    }


def campaign_pointer_categories(repo_root: Path) -> dict[str, Any]:
    root = _absolute(repo_root)
    date_root = root / "artifacts/validation/2026-09-02"
    campaign = date_root / "autoware_vad_runtime_control_campaign_v1"
    matrix_current = date_root / "autoware_vad_town_matrix_30kph_camera_source_5hz_v1"
    matrix_baseline = (
        root
        / "artifacts/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v16_owned_window_final"
    )
    speed_30 = date_root / "autoware_vad_30kph_town06_long_straight_comparison_v1"
    speed_60 = (
        campaign
        / "30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3"
    )
    speed_compare = date_root / "autoware_vad_town06_same_route_30_vs_60_analysis_v1"

    def control_campaign(
        scenario: str, baseline: str, candidate: str
    ) -> list[dict[str, Any]]:
        scenario_root = campaign / "20_30kph_control_ab" / scenario
        return [
            _pointer(scenario_root / baseline, "30 kph A baseline evidence"),
            _pointer(scenario_root / candidate, "30 kph isolated B candidate evidence"),
            _pointer(
                scenario_root / "C_selected_baseline_depth1_loopback_regression_001",
                "selected depth-1 loopback regression",
            ),
            _pointer(scenario_root / "comparison", "A/B decision evidence"),
        ]

    return {
        "10_runtime_stutter": [
            _pointer(
                campaign
                / "10_runtime_stutter/"
                "town07_straight_A_baseline_health_001",
                "Town07 three-attempt pre-engagement runtime-stutter evidence",
            ),
            _pointer(date_root / "camera_cadence_ab", "camera cadence A/B raw evidence"),
            _pointer(
                speed_30 / "trial/attempt_002/runtime_load_analysis.json",
                "same-route 30 kph runtime analysis",
            ),
            _pointer(
                campaign
                / "20_30kph_control_ab/town07_straight/"
                "C_selected_baseline_depth1_loopback_regression_001/attempts/"
                "attempt_001/runtime_health.json",
                "post-fix Town07 deterministic runtime-health evidence",
            ),
        ],
        "15_all_towns_30kph": [
            _pointer(
                root / "artifacts/validation/2026-08-31/maps",
                "all-map inventory and admission inputs",
            ),
            _pointer(
                matrix_baseline / "aggregate.json",
                "pre-fix owned-window all-Town aggregate (9/9 runnable PASS)",
            ),
            _pointer(
                matrix_current / "aggregate.json",
                "camera-source-5Hz all-Town aggregate (9/9 runnable PASS)",
            ),
            _pointer(
                root / "docs/validation-2026-09-02-all-towns-camera-source-5hz.md",
                "all-Town 30 kph report",
            ),
            _pointer(
                root / "docs/assets/validation/2026-09-02-all-towns-camera-source-5hz",
                "all-Town centered full-screen and route-analysis assets",
            ),
        ],
        "20_30kph_control_ab": {
            "town07_straight": control_campaign(
                "town07_straight",
                "A_baseline_best_effort_health_001",
                "B_pid_i40_best_effort_health_001",
            ),
            "c_track_turn": control_campaign(
                "c_track_turn",
                "A_baseline_best_effort_health_001",
                "B_turn_preview_5m_best_effort_health_001",
            ),
            "town03_turn": control_campaign(
                "town03_turn",
                "A_baseline_best_effort_health_001",
                "B_turn_preview_5m_best_effort_health_001",
            ),
        },
        "30_60kph": [
            _pointer(speed_60, "selected Town06 60 kph exploratory pilot"),
            _pointer(
                campaign / "50_reports/town06_60kph_v3_speed_limit_analysis.md",
                "selected pilot speed-limit root-cause analysis",
            ),
            _pointer(speed_30, "historical Town06 same-route 30 kph trial"),
            _pointer(speed_compare, "historical strict same-route 30 versus 60 comparison"),
        ],
        "40_visuals": [
            _pointer(
                campaign / "40_visuals",
                "canonical curated A/B, selected-regression, and 60 kph visuals",
            ),
            _pointer(
                root / "docs/assets/validation/2026-09-02-all-towns-camera-source-5hz",
                "all-Town 30 kph published visuals",
            ),
            _pointer(
                root
                / "docs/assets/validation/2026-09-02-runtime-control-campaign-v1",
                "review-oriented mirror of the current campaign visuals",
            ),
        ],
        "50_reports": [
            _pointer(
                campaign / "50_reports/runtime_control_campaign_summary.md",
                "canonical deterministic runtime/control summary",
            ),
            _pointer(
                root / "docs/validation-2026-09-02-all-towns-camera-source-5hz.md",
                "all-Town report",
            ),
            _pointer(
                root / "docs/validation-2026-09-02-runtime-control-campaign.md",
                "review-oriented current campaign report",
            ),
        ],
        "90_quarantine": [
            _pointer(campaign / "90_quarantine/artifacts", "recoverable artifact quarantine"),
            _pointer(campaign / "90_quarantine/tmp", "recoverable project-temp quarantine"),
            _pointer(
                campaign / "90_quarantine/final_residue_pass",
                "recoverable final tmp-audit quarantine",
            ),
            _pointer(
                campaign / "90_quarantine/post_verification",
                "recoverable post-verification pytest quarantine",
            ),
            _pointer(
                campaign / "90_quarantine/csv_verification",
                "recoverable CSV-regression pytest quarantine",
            ),
        ],
        "99_integrity": [
            _pointer(
                campaign / "99_integrity/CLEANUP.md",
                "human-readable cleanup and restore summary",
            ),
            _pointer(
                campaign / "99_integrity/cleanup-plan.json",
                "primary hash-bound cleanup plan",
            ),
            _pointer(
                campaign / "90_quarantine/cleanup-journal.json",
                "primary applied-cleanup and restore journal",
            ),
            _pointer(
                campaign / "99_integrity/cleanup-plan-final-residue.json",
                "final tmp-audit hash-bound cleanup plan",
            ),
            _pointer(
                campaign / "90_quarantine/final_residue_pass/cleanup-journal.json",
                "final tmp-audit applied-cleanup and restore journal",
            ),
            _pointer(
                campaign / "99_integrity/cleanup-plan-post-verification.json",
                "post-verification hash-bound cleanup plan",
            ),
            _pointer(
                campaign / "90_quarantine/post_verification/cleanup-journal.json",
                "post-verification applied-cleanup and restore journal",
            ),
            _pointer(
                campaign / "99_integrity/cleanup-plan-csv-verification.json",
                "CSV-regression hash-bound cleanup plan",
            ),
            _pointer(
                campaign / "90_quarantine/csv_verification/cleanup-journal.json",
                "CSV-regression applied-cleanup and restore journal",
            ),
            _pointer(
                root
                / "docs/assets/validation/2026-09-02-all-towns-camera-source-5hz/"
                "SHA256SUMS",
                "published all-Town checksum inventory",
            ),
            _pointer(
                root
                / "docs/assets/validation/2026-09-02-runtime-control-campaign-v1/"
                "SHA256SUMS",
                "current campaign visual checksum inventory",
            ),
        ],
    }


def build_campaign_index(
    repo_root: Path,
    campaign_root: Path,
    *,
    generated_at: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": INDEX_SCHEMA_VERSION,
        "campaign_id": "autoware_vad_runtime_control_campaign_v1",
        "generated_at_utc": generated_at or utc_now(),
        "campaign_root": str(_absolute(campaign_root)),
        "repository_root": str(_absolute(repo_root)),
        "organization_mode": "canonical_campaign_plus_absolute_legacy_pointers",
        "source_mutation": "NONE",
        "source_contract": {
            "copies_created": False,
            "sources_moved": False,
            "sources_deleted": False,
            "symlinks_created": False,
            "note": (
                "This index does not mutate evidence. New campaign trials live directly "
                "under the campaign root; legacy and all-Town evidence remains at absolute "
                "paths because strict validators bind real paths and bag locations."
            ),
        },
        "category_directories": list(CATEGORY_DIRECTORIES),
        "pointers": campaign_pointer_categories(repo_root),
    }


def render_campaign_readme(index: Mapping[str, Any]) -> str:
    return "\n".join(
        (
            "# Autoware VAD runtime/control campaign v1",
            "",
            "This is the single review entry point for the current campaign. New A/B,",
            "post-fix regression, and 60 km/h evidence lives directly below this directory.",
            "`INDEX.json` also records absolute pointers to immutable all-Town and legacy",
            "evidence whose embedded path contracts prevent relocation.",
            "",
            "## Categories",
            "",
            "- `10_runtime_stutter`: camera cadence and runtime-load diagnosis",
            "- `15_all_towns_30kph`: all-map admission, straight/turn results, and visuals",
            "- `20_30kph_control_ab`: A/B trials plus selected post-fix regressions",
            "- `30_60kph`: Town06 60 km/h exploratory pilot",
            "- `40_visuals`: curated, plainly named PNG/GIF/path/control assets",
            "- `50_reports`: deterministic summaries and root-cause analysis",
            "- `90_quarantine`: recoverable failed/superseded artifacts and project temp",
            "- `99_integrity`: cleanup plans, journals, and checksum pointers",
            "",
            "## Safety contract",
            "",
            "Run `plan-cleanup` to create a dry-run manifest. `apply-cleanup` is a separate",
            "explicitly confirmed command that rechecks every tree hash, active process, open",
            "file descriptor, and project lock before same-filesystem rename operations.",
            "Ordinary `/tmp/tmp.*` paths are outside the cleanup policy.",
            "",
            f"Generated: {index['generated_at_utc']}",
            "",
        )
    )


def initialize_campaign(
    repo_root: Path,
    campaign_root: Path,
    *,
    generated_at: str | None = None,
) -> dict[str, Any]:
    campaign = _absolute(campaign_root)
    if campaign.is_symlink():
        raise CampaignError(f"campaign root must not be a symlink: {campaign}")
    for relative in CATEGORY_DIRECTORIES:
        destination = campaign / relative
        if destination.exists() and not destination.is_dir():
            raise CampaignError(f"campaign category is not a directory: {destination}")
        destination.mkdir(parents=True, exist_ok=True)
    index = build_campaign_index(repo_root, campaign, generated_at=generated_at)
    atomic_write_json(campaign / "00_index/INDEX.json", index)
    atomic_write_text(campaign / "00_index/README.md", render_campaign_readme(index))
    return index


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_path(path: Path) -> dict[str, Any]:
    """Return a no-symlink content snapshot suitable for rename preflight."""
    absolute = _absolute(path)
    root_stat = absolute.lstat()
    if stat.S_ISLNK(root_stat.st_mode):
        raise CampaignError(f"candidate is a symlink: {absolute}")
    if stat.S_ISREG(root_stat.st_mode):
        return {
            "kind": "file",
            "size_bytes": root_stat.st_size,
            "file_count": 1,
            "symlink_count": 0,
            "tree_sha256": sha256_file(absolute),
            "root_device": root_stat.st_dev,
            "root_inode": root_stat.st_ino,
        }
    if not stat.S_ISDIR(root_stat.st_mode):
        raise CampaignError(f"candidate is not a regular file or directory: {absolute}")

    digest = hashlib.sha256()
    size_bytes = 0
    file_count = 0
    symlink_count = 0
    for child in sorted(absolute.rglob("*"), key=lambda item: item.relative_to(absolute).as_posix()):
        child_stat = child.lstat()
        relative = child.relative_to(absolute).as_posix()
        if stat.S_ISLNK(child_stat.st_mode):
            target = os.readlink(child)
            digest.update(relative.encode("utf-8"))
            digest.update(b"\0SYMLINK\0")
            digest.update(os.fsencode(target))
            digest.update(b"\n")
            symlink_count += 1
            continue
        if stat.S_ISDIR(child_stat.st_mode):
            continue
        if not stat.S_ISREG(child_stat.st_mode):
            raise CampaignError(f"candidate tree contains a special file: {child}")
        file_sha256 = sha256_file(child)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(child_stat.st_size).encode("ascii"))
        digest.update(b"\0")
        digest.update(file_sha256.encode("ascii"))
        digest.update(b"\n")
        size_bytes += child_stat.st_size
        file_count += 1
    return {
        "kind": "directory",
        "size_bytes": size_bytes,
        "file_count": file_count,
        "symlink_count": symlink_count,
        "tree_sha256": digest.hexdigest(),
        "root_device": root_stat.st_dev,
        "root_inode": root_stat.st_ino,
    }


def _tmp_reason(name: str) -> tuple[str, str]:
    if name == "autoware_mission_planner_fix.UD6ymT":
        return "BUILD_STAGING", "build/install/log-only mission-planner staging directory"
    if name == "pytest-of-a":
        return "TEST_STAGING", "inactive pytest temporary tree"
    if name.startswith("launch_params_"):
        return "ROS_LAUNCH_STAGING", "orphaned generated ROS 2 launch parameter file"
    if name.endswith(".png") or name in {"repro60x", "seek60"}:
        return "VISUAL_SCRATCH", "inactive visual inspection scratch data"
    return "ANALYSIS_STAGING", "inactive validation analysis or audit staging data"


def iter_candidate_specs(repo_root: Path, tmp_root: Path) -> Iterator[tuple[CandidateSpec, Path]]:
    repo = _absolute(repo_root)
    temporary = _absolute(tmp_root)
    for spec in ARTIFACT_CANDIDATES:
        yield spec, repo / spec.relative_path
    for name in TMP_EXACT_CANDIDATES:
        observed_status, reason = _tmp_reason(name)
        spec = CandidateSpec(f"tmp_exact_{name}", "tmp", name, observed_status, reason)
        yield spec, temporary / name
    if temporary.is_dir():
        for path in sorted(temporary.iterdir(), key=lambda item: item.name):
            if not LAUNCH_PARAM_PATTERN.fullmatch(path.name):
                continue
            spec = CandidateSpec(
                f"tmp_launch_param_{path.name}",
                "tmp",
                path.name,
                "ROS_LAUNCH_STAGING",
                "orphaned generated ROS 2 launch parameter file",
            )
            yield spec, path


def _destination_relative(spec: CandidateSpec) -> str:
    if spec.scope == "artifacts":
        relative = Path(spec.relative_path)
        if relative.parts[:2] != ("artifacts", "validation"):
            raise CampaignError(f"invalid artifact candidate policy: {spec.relative_path}")
        return relative.as_posix()
    if spec.scope == "tmp":
        if Path(spec.relative_path).name != spec.relative_path:
            raise CampaignError(f"invalid tmp candidate policy: {spec.relative_path}")
        return (Path("tmp") / spec.relative_path).as_posix()
    raise CampaignError(f"unknown candidate scope: {spec.scope}")


def build_cleanup_plan(
    repo_root: Path,
    tmp_root: Path,
    *,
    generated_at: str | None = None,
) -> dict[str, Any]:
    repo = _absolute(repo_root)
    temporary = _absolute(tmp_root)
    entries: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    seen_paths: set[str] = set()
    for spec, path in iter_candidate_specs(repo, temporary):
        source = _absolute(path)
        if spec.candidate_id in seen_ids or str(source) in seen_paths:
            continue
        seen_ids.add(spec.candidate_id)
        seen_paths.add(str(source))
        entry: dict[str, Any] = {
            "candidate_id": spec.candidate_id,
            "scope": spec.scope,
            "source_path": str(source),
            "source_resolved_path": str(source.resolve(strict=False)),
            "destination_relative_path": _destination_relative(spec),
            "observed_status": spec.observed_status,
            "reason": spec.reason,
        }
        if not source.exists() and not source.is_symlink():
            entry.update(
                {
                    "status": "ABSENT_AT_PLAN",
                    "kind": None,
                    "size_bytes": None,
                    "file_count": None,
                    "symlink_count": None,
                    "tree_sha256": None,
                }
            )
        else:
            try:
                entry.update(inspect_path(source))
                entry["status"] = "READY_TO_QUARANTINE"
            except (CampaignError, FileNotFoundError, PermissionError, OSError) as error:
                entry.update(
                    {
                        "status": "UNSAFE_AT_PLAN",
                        "kind": None,
                        "size_bytes": None,
                        "file_count": None,
                        "symlink_count": None,
                        "tree_sha256": None,
                        "error": str(error),
                    }
                )
        entries.append(entry)

    ready = [entry for entry in entries if entry["status"] == "READY_TO_QUARANTINE"]
    unsafe = [entry for entry in entries if entry["status"] == "UNSAFE_AT_PLAN"]
    plan_basis = {
        "entries": entries,
        "repo_root": str(repo),
        "tmp_root": str(temporary),
    }
    plan_id = hashlib.sha256(canonical_json_bytes(plan_basis)).hexdigest()[:20]
    payload = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "kind": "autoware_e2e_validation_cleanup_plan",
        "plan_id": plan_id,
        "generated_at_utc": generated_at or utc_now(),
        "operation": "DRY_RUN_RENAME_TO_QUARANTINE",
        "applied": False,
        "repo_root": str(repo),
        "tmp_root": str(temporary),
        "hash_algorithm": TREE_HASH_ALGORITHM,
        "policy": {
            "delete_allowed": False,
            "copy_allowed": False,
            "symlink_allowed": False,
            "rename_requires_explicit_apply": True,
            "ordinary_tmp_glob_excluded": "/tmp/tmp.*",
            "launch_parameter_selector": "direct children matching ^launch_params_[A-Za-z0-9_]+$",
        },
        "protected": {
            "artifact_roots": [
                str(repo / "artifacts/validation/2026-09-02"),
                str(
                    repo
                    / "artifacts/validation/2026-09-01/"
                    "autoware_vad_town_matrix_30kph_v16_owned_window_final"
                ),
                str(repo / "docs/assets/validation"),
            ],
            "tmp_lock_patterns": list(TMP_LOCK_PATTERNS),
        },
        "summary": {
            "candidate_count": len(entries),
            "ready_count": len(ready),
            "absent_count": sum(entry["status"] == "ABSENT_AT_PLAN" for entry in entries),
            "unsafe_count": len(unsafe),
            "ready_size_bytes": sum(int(entry["size_bytes"]) for entry in ready),
            "ready_file_count": sum(int(entry["file_count"]) for entry in ready),
        },
        "entries": entries,
        "apply_contract": {
            "required_subcommand": "apply-cleanup",
            "required_confirmation": APPLY_CONFIRMATION,
            "required_arguments": ["--manifest", "--quarantine-root", "--confirm"],
            "preflight_all_entries_before_first_rename": True,
            "partial_failure_behavior": "stop immediately and persist journal",
            "restore_subcommand": "restore-cleanup",
        },
    }
    return signed_payload(payload)


def write_cleanup_plan(
    repo_root: Path,
    tmp_root: Path,
    output: Path,
    *,
    generated_at: str | None = None,
) -> dict[str, Any]:
    plan = build_cleanup_plan(repo_root, tmp_root, generated_at=generated_at)
    atomic_write_json(_absolute(output), plan)
    return plan


def load_signed_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CampaignError(f"cannot load {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise CampaignError(f"{label} must be a JSON object: {path}")
    verify_signed_payload(value, label)
    return value


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _validate_manifest_entry_policy(
    entry: Mapping[str, Any], repo_root: Path, tmp_root: Path
) -> None:
    source = Path(str(entry.get("source_path", "")))
    if not source.is_absolute():
        raise CampaignError(f"manifest source is not absolute: {source}")
    scope = entry.get("scope")
    expected: Path | None = None
    if scope == "artifacts":
        allowed = {str(repo_root / spec.relative_path) for spec in ARTIFACT_CANDIDATES}
        if str(source) not in allowed:
            raise CampaignError(f"artifact source is outside the audited exact policy: {source}")
        expected = Path("artifacts") / source.relative_to(repo_root / "artifacts")
    elif scope == "tmp":
        if source.parent != tmp_root:
            raise CampaignError(f"tmp source is not a direct child of {tmp_root}: {source}")
        allowed_name = source.name in TMP_EXACT_CANDIDATES or bool(
            LAUNCH_PARAM_PATTERN.fullmatch(source.name)
        )
        if not allowed_name or source.name.startswith("tmp."):
            raise CampaignError(f"tmp source is outside the audited exact policy: {source}")
        expected = Path("tmp") / source.name
    else:
        raise CampaignError(f"unknown manifest scope for {source}: {scope!r}")
    destination_relative = Path(str(entry.get("destination_relative_path", "")))
    if destination_relative.is_absolute() or ".." in destination_relative.parts:
        raise CampaignError(f"unsafe relative destination for {source}: {destination_relative}")
    if destination_relative != expected:
        raise CampaignError(
            f"destination policy mismatch for {source}: {destination_relative} != {expected}"
        )


def validate_cleanup_manifest(
    manifest: Mapping[str, Any], expected_repo_root: Path, expected_tmp_root: Path
) -> list[dict[str, Any]]:
    verify_signed_payload(manifest, "cleanup manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA_VERSION:
        raise CampaignError("unsupported cleanup manifest schema")
    if manifest.get("kind") != "autoware_e2e_validation_cleanup_plan":
        raise CampaignError("not an Autoware E2E cleanup plan")
    if manifest.get("operation") != "DRY_RUN_RENAME_TO_QUARANTINE":
        raise CampaignError("cleanup manifest operation is not rename-to-quarantine")
    repo_root = _absolute(expected_repo_root)
    tmp_root = _absolute(expected_tmp_root)
    if manifest.get("repo_root") != str(repo_root):
        raise CampaignError(
            f"cleanup manifest repository root mismatch: {manifest.get('repo_root')!r}"
        )
    if manifest.get("tmp_root") != str(tmp_root):
        raise CampaignError(f"cleanup manifest tmp root mismatch: {manifest.get('tmp_root')!r}")
    raw_entries = manifest.get("entries")
    if not isinstance(raw_entries, list):
        raise CampaignError("cleanup manifest entries must be a list")
    ready: list[dict[str, Any]] = []
    seen_sources: set[str] = set()
    seen_destinations: set[str] = set()
    for raw in raw_entries:
        if not isinstance(raw, dict):
            raise CampaignError("cleanup manifest entry must be an object")
        _validate_manifest_entry_policy(raw, repo_root, tmp_root)
        source = str(raw["source_path"])
        destination = str(raw["destination_relative_path"])
        if source in seen_sources or destination in seen_destinations:
            raise CampaignError(f"duplicate cleanup source or destination: {source}")
        seen_sources.add(source)
        seen_destinations.add(destination)
        if raw.get("status") == "READY_TO_QUARANTINE":
            for key in (
                "kind",
                "size_bytes",
                "file_count",
                "symlink_count",
                "tree_sha256",
            ):
                if raw.get(key) is None:
                    raise CampaignError(f"ready cleanup entry lacks {key}: {source}")
            ready.append(dict(raw))
    return ready


def _ancestor_pids(pid: int) -> set[int]:
    result = {pid}
    current = pid
    while current > 1:
        try:
            fields = (Path("/proc") / str(current) / "stat").read_text(
                encoding="utf-8"
            ).split()
            parent = int(fields[3])
        except (OSError, ValueError, IndexError):
            break
        if parent in result or parent <= 0:
            break
        result.add(parent)
        current = parent
    return result


def inspect_project_processes() -> dict[str, Any]:
    ignored = _ancestor_pids(os.getpid())
    matches: list[dict[str, Any]] = []
    errors: list[str] = []
    proc_root = Path("/proc")
    for process_dir in sorted(proc_root.glob("[0-9]*"), key=lambda path: int(path.name)):
        pid = int(process_dir.name)
        if pid in ignored:
            continue
        try:
            if process_dir.stat().st_uid != os.getuid():
                continue
            raw = (process_dir / "cmdline").read_bytes()
        except FileNotFoundError:
            continue
        except (PermissionError, OSError) as error:
            errors.append(f"pid {pid}: {error}")
            continue
        command = raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        if command and any(pattern.search(command) for pattern in PROJECT_PROCESS_PATTERNS):
            matches.append({"pid": pid, "command": command})
    return {"matches": matches, "errors": errors}


def _fd_target_matches(target: str, sources: Sequence[Path]) -> bool:
    normalized = target.removesuffix(" (deleted)")
    for source in sources:
        value = str(source)
        if normalized == value or normalized.startswith(value + os.sep):
            return True
    return False


def _read_proc_status(process_dir: Path) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in (process_dir / "status").read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if separator:
            fields[key] = value.strip()
    return fields


def _trusted_systemd_executables() -> set[Path]:
    trusted: set[Path] = set()
    for candidate in (Path("/lib/systemd/systemd"), Path("/usr/lib/systemd/systemd")):
        try:
            resolved = candidate.resolve(strict=True)
            metadata = resolved.stat()
        except OSError:
            continue
        if (
            stat.S_ISREG(metadata.st_mode)
            and metadata.st_uid == 0
            and metadata.st_mode & 0o022 == 0
        ):
            trusted.add(resolved)
    return trusted


def _is_verified_sd_pam_session_helper(
    process_dir: Path,
    process_metadata: os.stat_result,
) -> bool:
    """Recognize the fd-protected systemd PAM helper without trusting its name alone."""
    try:
        pid = int(process_dir.name)
        uid = os.getuid()
        expected_cgroup = (
            f"0::/user.slice/user-{uid}.slice/user@{uid}.service/init.scope\n"
        )
        status = _read_proc_status(process_dir)
        parent_pid = int(status["PPid"])
        parent_dir = process_dir.parent / str(parent_pid)
        parent_status = _read_proc_status(parent_dir)
        parent_executable = Path(os.readlink(parent_dir / "exe")).resolve(strict=True)
        current_metadata = process_dir.stat()
        parent_metadata = parent_dir.stat()
        process_comm = (process_dir / "comm").read_bytes()
        process_cmdline = (process_dir / "cmdline").read_bytes()
        process_cgroup = (process_dir / "cgroup").read_text(encoding="utf-8")
        parent_comm = (parent_dir / "comm").read_bytes()
        parent_cmdline = (parent_dir / "cmdline").read_bytes()
        parent_cgroup = (parent_dir / "cgroup").read_text(encoding="utf-8")
        trusted_systemd_executables = _trusted_systemd_executables()
    except (KeyError, OSError, ValueError):
        return False

    expected_uid_fields = [str(uid)] * 4
    expected_parent_commands = {
        b"/lib/systemd/systemd\0--user\0",
        b"/usr/lib/systemd/systemd\0--user\0",
    }
    return all(
        (
            current_metadata.st_dev == process_metadata.st_dev,
            current_metadata.st_ino == process_metadata.st_ino,
            current_metadata.st_ctime_ns == process_metadata.st_ctime_ns,
            status.get("Name") == "(sd-pam)",
            status.get("Pid") == str(pid),
            status.get("Tgid") == str(pid),
            status.get("Uid", "").split() == expected_uid_fields,
            process_comm == b"(sd-pam)\n",
            process_cmdline == b"(sd-pam)\0",
            process_cgroup == expected_cgroup,
            parent_pid > 1,
            parent_metadata.st_uid == uid,
            parent_status.get("Name") == "systemd",
            parent_status.get("Pid") == str(parent_pid),
            parent_status.get("Tgid") == str(parent_pid),
            parent_status.get("PPid") == "1",
            parent_status.get("Uid", "").split() == expected_uid_fields,
            parent_comm == b"systemd\n",
            parent_cmdline in expected_parent_commands,
            parent_cgroup == expected_cgroup,
            parent_executable in trusted_systemd_executables,
        )
    )


def _list_process_descriptors(process_dir: Path) -> list[Path]:
    return list((process_dir / "fd").iterdir())


def inspect_open_fds(
    sources: Sequence[Path], *, proc_root: Path = Path("/proc")
) -> dict[str, Any]:
    targets = [_absolute(path) for path in sources]
    hits: list[dict[str, Any]] = []
    errors: list[str] = []
    skipped: list[dict[str, Any]] = []
    for process_dir in sorted(proc_root.glob("[0-9]*"), key=lambda path: int(path.name)):
        pid = int(process_dir.name)
        process_metadata: os.stat_result | None = None
        try:
            process_metadata = process_dir.stat()
            if process_metadata.st_uid != os.getuid():
                continue
            descriptors = _list_process_descriptors(process_dir)
        except FileNotFoundError:
            continue
        except PermissionError as error:
            if process_metadata is not None and _is_verified_sd_pam_session_helper(
                process_dir, process_metadata
            ):
                skipped.append(
                    {
                        "pid": pid,
                        "reason": "verified systemd --user (sd-pam) session helper",
                    }
                )
                continue
            errors.append(f"pid {pid}: {error}")
            continue
        except OSError as error:
            errors.append(f"pid {pid}: {error}")
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
            except FileNotFoundError:
                continue
            except (PermissionError, OSError) as error:
                errors.append(f"pid {pid} fd {descriptor.name}: {error}")
                continue
            if _fd_target_matches(target, targets):
                hits.append({"pid": pid, "fd": int(descriptor.name), "target": target})
    return {"matches": hits, "errors": errors, "skipped": skipped}


def _lock_paths(tmp_root: Path) -> list[Path]:
    paths: set[Path] = set()
    for pattern in TMP_LOCK_PATTERNS:
        paths.update(tmp_root.glob(pattern))
    return sorted(paths)


@contextmanager
def acquire_project_lock_guards(tmp_root: Path) -> Iterator[list[TextIO]]:
    streams: list[TextIO] = []
    try:
        for path in _lock_paths(tmp_root):
            if path.is_symlink() or not path.is_file():
                raise CampaignError(f"project lock path is not a regular file: {path}")
            stream = path.open("r", encoding="utf-8")
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                stream.close()
                raise CampaignError(f"active project lock: {path}") from error
            streams.append(stream)
        yield streams
    finally:
        for stream in reversed(streams):
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
            finally:
                stream.close()


def _assert_runtime_safety(sources: Sequence[Path]) -> None:
    processes = inspect_project_processes()
    if processes["errors"]:
        raise CampaignError(
            "cannot prove project-process safety: " + "; ".join(processes["errors"][:5])
        )
    if processes["matches"]:
        details = ", ".join(
            f"pid={item['pid']} {item['command']}" for item in processes["matches"][:5]
        )
        raise CampaignError(f"active project process prevents cleanup: {details}")
    descriptors = inspect_open_fds(sources)
    if descriptors["errors"]:
        raise CampaignError(
            "cannot prove open-FD safety: " + "; ".join(descriptors["errors"][:5])
        )
    if descriptors["matches"]:
        details = ", ".join(
            f"pid={item['pid']} fd={item['fd']} {item['target']}"
            for item in descriptors["matches"][:5]
        )
        raise CampaignError(f"open file descriptor prevents cleanup: {details}")


def _nearest_existing_parent(path: Path) -> Path:
    current = path
    while not current.exists():
        if current.parent == current:
            raise CampaignError(f"no existing ancestor for destination: {path}")
        current = current.parent
    return current


def _safe_quarantine_root(
    quarantine_root: Path,
    sources: Sequence[Path],
    *,
    repo_root: Path = REPO_ROOT,
) -> Path:
    root = _absolute(quarantine_root)
    expected_repo = _absolute(repo_root)
    forbidden = {Path("/"), Path("/tmp"), expected_repo, expected_repo.parent}
    if root in forbidden:
        raise CampaignError(f"quarantine root is too broad: {root}")
    if root.is_symlink():
        raise CampaignError(f"quarantine root must not be a symlink: {root}")
    for source in sources:
        if root == source or _is_relative_to(root, source) or _is_relative_to(source, root):
            raise CampaignError(
                f"quarantine root and cleanup source overlap: {source} versus {root}"
            )
    return root


def _snapshot_matches(entry: Mapping[str, Any], snapshot: Mapping[str, Any]) -> bool:
    return all(
        entry.get(key) == snapshot.get(key)
        for key in ("kind", "size_bytes", "file_count", "symlink_count", "tree_sha256")
    )


def _journal_path(
    quarantine_root: Path, manifest: Mapping[str, Any], requested: Path | None
) -> Path:
    if requested is not None:
        return _absolute(requested)
    return quarantine_root / f"cleanup-journal-{manifest['plan_id']}.json"


def apply_cleanup(
    manifest_path: Path,
    quarantine_root: Path,
    *,
    expected_repo_root: Path = REPO_ROOT,
    expected_tmp_root: Path = DEFAULT_TMP_ROOT,
    journal_path: Path | None = None,
    confirmed: bool = False,
) -> dict[str, Any]:
    if not confirmed:
        raise CampaignError(
            f"apply-cleanup is disabled without --confirm {APPLY_CONFIRMATION}"
        )
    manifest_source = _absolute(manifest_path)
    manifest = load_signed_json(manifest_source, "cleanup manifest")
    ready = validate_cleanup_manifest(manifest, expected_repo_root, expected_tmp_root)
    if not ready:
        raise CampaignError("cleanup manifest has no ready entries")
    sources = [Path(entry["source_path"]) for entry in ready]
    quarantine = _safe_quarantine_root(
        quarantine_root, sources, repo_root=expected_repo_root
    )
    destinations = [quarantine / entry["destination_relative_path"] for entry in ready]
    if len({str(path) for path in destinations}) != len(destinations):
        raise CampaignError("cleanup manifest has duplicate destination paths")
    for destination in destinations:
        if destination.exists() or destination.is_symlink():
            raise CampaignError(f"cleanup destination already exists: {destination}")

    _assert_runtime_safety(sources)
    with acquire_project_lock_guards(_absolute(expected_tmp_root)):
        preflight: list[dict[str, Any]] = []
        for entry, source, destination in zip(ready, sources, destinations):
            if not source.exists() or source.is_symlink():
                raise CampaignError(f"cleanup source disappeared or became a symlink: {source}")
            if source.resolve(strict=True) != Path(entry["source_resolved_path"]):
                raise CampaignError(f"cleanup source realpath changed: {source}")
            snapshot = inspect_path(source)
            if not _snapshot_matches(entry, snapshot):
                raise CampaignError(
                    f"cleanup source hash/size changed since plan: {source}; "
                    f"planned={entry.get('tree_sha256')} current={snapshot.get('tree_sha256')}"
                )
            destination_device = _nearest_existing_parent(destination).stat().st_dev
            if snapshot["root_device"] != destination_device:
                raise CampaignError(
                    f"cleanup requires same-filesystem rename: {source} -> {destination}"
                )
            preflight.append({"entry": entry, "source": source, "destination": destination})

        _assert_runtime_safety(sources)
        quarantine.mkdir(parents=True, exist_ok=True)
        journal_file = _journal_path(quarantine, manifest, journal_path)
        if not _is_relative_to(journal_file, quarantine) or journal_file == quarantine:
            raise CampaignError(
                f"cleanup journal must be a file inside quarantine root: {journal_file}"
            )
        if any(
            journal_file == destination or _is_relative_to(journal_file, destination)
            for destination in destinations
        ):
            raise CampaignError(
                f"cleanup journal overlaps a quarantine destination: {journal_file}"
            )
        journal = signed_payload(
            {
                "schema_version": JOURNAL_SCHEMA_VERSION,
                "kind": "autoware_e2e_validation_cleanup_journal",
                "plan_id": manifest["plan_id"],
                "manifest_path": str(manifest_source),
                "source_manifest_sha256": manifest["manifest_sha256"],
                "quarantine_root": str(quarantine),
                "started_at_utc": utc_now(),
                "finished_at_utc": None,
                "status": "APPLYING",
                "error": None,
                "delete_performed": False,
                "copy_performed": False,
                "journal_write_strategy": "BEGIN_AND_TERMINAL_FULL_SNAPSHOTS_ONLY",
                "hard_interruption_recovery": (
                    "reconcile every entry from its exact source and quarantine locations"
                ),
                "restore_command": " ".join(
                    shlex.quote(value)
                    for value in (
                        sys.executable,
                        str(Path(__file__).resolve()),
                        "restore-cleanup",
                        "--journal",
                        str(journal_file),
                        "--confirm",
                        RESTORE_CONFIRMATION,
                    )
                ),
                "entries": [
                    {
                        "candidate_id": item["entry"]["candidate_id"],
                        "source_path": str(item["source"]),
                        "quarantine_path": str(item["destination"]),
                        "kind": item["entry"]["kind"],
                        "size_bytes": item["entry"]["size_bytes"],
                        "file_count": item["entry"]["file_count"],
                        "symlink_count": item["entry"]["symlink_count"],
                        "tree_sha256": item["entry"]["tree_sha256"],
                        "state": "PENDING",
                    }
                    for item in preflight
                ],
            }
        )
        if journal_file.exists() or journal_file.is_symlink():
            raise CampaignError(f"cleanup journal already exists: {journal_file}")
        atomic_write_json(journal_file, journal)

        try:
            for index, item in enumerate(preflight):
                destination = item["destination"]
                destination.parent.mkdir(parents=True, exist_ok=True)
                os.rename(item["source"], destination)
                journal["entries"][index]["state"] = "MOVED"
                journal["entries"][index]["moved_at_utc"] = utc_now()
        except Exception as error:
            journal["status"] = "PARTIAL_FAILED"
            journal["finished_at_utc"] = utc_now()
            journal["error"] = f"{type(error).__name__}: {error}"
            journal = signed_payload(journal)
            atomic_write_json(journal_file, journal)
            raise CampaignError(
                f"cleanup stopped after a partial failure; use: {journal['restore_command']}"
            ) from error

        journal["status"] = "APPLIED"
        journal["finished_at_utc"] = utc_now()
        journal = signed_payload(journal)
        atomic_write_json(journal_file, journal)
        return journal


def _assert_no_symlink_below(root: Path, path: Path, label: str) -> None:
    if not _is_relative_to(path, root):
        raise CampaignError(f"{label} is outside its required root: {path}")
    current = root
    if current.is_symlink():
        raise CampaignError(f"{label} uses an unsafe symlink: {current}")
    for part in path.relative_to(root).parts:
        current = current / part
        if current.is_symlink():
            raise CampaignError(f"{label} uses an unsafe symlink: {current}")


def _reconcile_restore_entry(
    entry: dict[str, Any], planned: Mapping[str, Any], quarantine_root: Path
) -> tuple[str, Path, Path, Mapping[str, Any]]:
    original = Path(entry["source_path"])
    quarantined = Path(entry["quarantine_path"])
    _assert_no_symlink_below(
        quarantine_root, quarantined, "cleanup quarantine location"
    )
    if original.is_symlink():
        raise CampaignError(f"restore source location is an unsafe symlink: {original}")
    if quarantined.is_symlink():
        raise CampaignError(
            f"quarantined source is missing or unsafe: {quarantined}"
        )
    original_exists = original.exists()
    quarantined_exists = quarantined.exists()
    if original_exists and quarantined_exists:
        raise CampaignError(
            "cleanup entry exists at both source and quarantine locations: "
            f"{original} and {quarantined}"
        )
    if not original_exists and not quarantined_exists:
        raise CampaignError(
            "cleanup entry exists at neither source nor quarantine location: "
            f"{original} and {quarantined}"
        )

    if original_exists:
        if original.resolve(strict=True) != Path(str(planned["source_resolved_path"])):
            raise CampaignError(f"restore source realpath changed: {original}")
        snapshot = inspect_path(original)
        if not _snapshot_matches(entry, snapshot):
            raise CampaignError(f"original content hash changed: {original}")
        destination_device = _nearest_existing_parent(quarantined).stat().st_dev
        if snapshot["root_device"] != destination_device:
            raise CampaignError(
                f"cleanup requires same-filesystem rename: {original} -> {quarantined}"
            )
        return "NOT_MOVED", original, quarantined, snapshot

    snapshot = inspect_path(quarantined)
    if not _snapshot_matches(entry, snapshot):
        raise CampaignError(f"quarantined content hash changed: {quarantined}")
    if snapshot["root_device"] != _nearest_existing_parent(original).stat().st_dev:
        raise CampaignError(
            f"restore requires same-filesystem rename: {quarantined} -> {original}"
        )
    return "MOVED", original, quarantined, snapshot


def restore_cleanup(
    journal_path: Path,
    *,
    expected_repo_root: Path = REPO_ROOT,
    expected_tmp_root: Path = DEFAULT_TMP_ROOT,
    confirmed: bool = False,
) -> dict[str, Any]:
    if not confirmed:
        raise CampaignError(
            f"restore-cleanup is disabled without --confirm {RESTORE_CONFIRMATION}"
        )
    journal_file = _absolute(journal_path)
    if journal_file.is_symlink():
        raise CampaignError(f"cleanup journal is an unsafe symlink: {journal_file}")
    journal = load_signed_json(journal_file, "cleanup journal")
    if journal.get("schema_version") != JOURNAL_SCHEMA_VERSION or journal.get("kind") != (
        "autoware_e2e_validation_cleanup_journal"
    ):
        raise CampaignError("unsupported cleanup journal")
    if journal.get("status") not in {
        "APPLYING",
        "APPLIED",
        "PARTIAL_FAILED",
        "RESTORE_PARTIAL_FAILED",
    }:
        raise CampaignError(f"journal is not restorable: {journal.get('status')!r}")
    if journal.get("delete_performed") is not False:
        raise CampaignError("cleanup journal does not preserve the no-delete contract")
    if journal.get("copy_performed") is not False:
        raise CampaignError("cleanup journal does not preserve the no-copy contract")
    raw_entries = journal.get("entries")
    if not isinstance(raw_entries, list) or not all(
        isinstance(entry, dict) for entry in raw_entries
    ):
        raise CampaignError("cleanup journal entries must be a list of objects")
    source_manifest_path = Path(str(journal.get("manifest_path", "")))
    if not source_manifest_path.is_absolute() or source_manifest_path.is_symlink():
        raise CampaignError("cleanup journal manifest path is missing or unsafe")
    source_manifest = load_signed_json(source_manifest_path, "cleanup manifest")
    if source_manifest.get("manifest_sha256") != journal.get("source_manifest_sha256"):
        raise CampaignError("cleanup journal is not bound to its source manifest")
    if source_manifest.get("plan_id") != journal.get("plan_id"):
        raise CampaignError("cleanup journal plan ID does not match its source manifest")
    ready = validate_cleanup_manifest(
        source_manifest, expected_repo_root, expected_tmp_root
    )
    ready_by_id = {entry["candidate_id"]: entry for entry in ready}
    entry_ids = [entry.get("candidate_id") for entry in raw_entries]
    if (
        not all(isinstance(candidate_id, str) for candidate_id in entry_ids)
        or len(raw_entries) != len(ready)
        or len(set(entry_ids)) != len(entry_ids)
        or set(entry_ids) != set(ready_by_id)
    ):
        raise CampaignError("cleanup journal entry set does not match source manifest")

    raw_quarantine_root = Path(str(journal.get("quarantine_root", "")))
    if not raw_quarantine_root.is_absolute():
        raise CampaignError("cleanup journal quarantine root is not absolute")
    original_sources = [Path(entry["source_path"]) for entry in raw_entries]
    quarantine_root = _safe_quarantine_root(
        raw_quarantine_root,
        original_sources,
        repo_root=expected_repo_root,
    )
    if journal_file == quarantine_root or not _is_relative_to(
        journal_file, quarantine_root
    ):
        raise CampaignError("cleanup journal is outside its quarantine root")
    _assert_no_symlink_below(quarantine_root, journal_file, "cleanup journal")

    allowed_entry_states = {"PENDING", "MOVED", "RESTORED"}
    for entry in raw_entries:
        planned = ready_by_id.get(entry.get("candidate_id"))
        if planned is None:
            raise CampaignError(
                f"cleanup journal entry is absent from source manifest: {entry.get('candidate_id')}"
            )
        expected_quarantine = quarantine_root / planned["destination_relative_path"]
        if (
            entry.get("source_path") != planned.get("source_path")
            or entry.get("quarantine_path") != str(expected_quarantine)
            or not _snapshot_matches(planned, entry)
            or entry.get("state") not in allowed_entry_states
        ):
            raise CampaignError(
                f"cleanup journal entry does not match source manifest: {entry.get('candidate_id')}"
            )
        if journal_file == expected_quarantine or _is_relative_to(
            journal_file, expected_quarantine
        ):
            raise CampaignError("cleanup journal overlaps a quarantine destination")

    all_locations = original_sources + [
        Path(entry["quarantine_path"]) for entry in raw_entries
    ]
    _assert_runtime_safety(all_locations)
    with acquire_project_lock_guards(_absolute(expected_tmp_root)):
        moved: list[tuple[dict[str, Any], Path, Path]] = []
        not_moved_count = 0
        for entry in reversed(raw_entries):
            planned = ready_by_id[entry["candidate_id"]]
            location_state, original, quarantined, _snapshot = (
                _reconcile_restore_entry(entry, planned, quarantine_root)
            )
            if location_state == "MOVED":
                entry["state"] = "MOVED"
                entry["restore_disposition"] = "RENAME_FROM_QUARANTINE"
                moved.append((entry, quarantined, original))
            else:
                entry["state"] = "RESTORED"
                entry["restore_disposition"] = "ALREADY_AT_SOURCE"
                not_moved_count += 1
        journal["restore_reconciliation"] = {
            "entry_count": len(raw_entries),
            "moved_at_quarantine_count": len(moved),
            "already_at_source_count": not_moved_count,
            "basis": "exact source/quarantine locations plus manifest-bound content hash",
        }
        _assert_runtime_safety(all_locations)
        try:
            for entry, quarantined, original in moved:
                original.parent.mkdir(parents=True, exist_ok=True)
                os.rename(quarantined, original)
                entry["state"] = "RESTORED"
                entry["restored_at_utc"] = utc_now()
        except Exception as error:
            journal["status"] = "RESTORE_PARTIAL_FAILED"
            journal["error"] = f"{type(error).__name__}: {error}"
            journal["finished_at_utc"] = utc_now()
            journal = signed_payload(journal)
            atomic_write_json(journal_file, journal)
            raise CampaignError(f"restore stopped after a partial failure: {journal_file}") from error
    journal["status"] = "RESTORED"
    journal["error"] = None
    journal["restored_at_utc"] = utc_now()
    journal = signed_payload(journal)
    atomic_write_json(journal_file, journal)
    return journal


def percentile(values: Sequence[float], quantile: float) -> float:
    if not values:
        raise CampaignError("cannot summarize an empty numeric sequence")
    ordered = sorted(float(value) for value in values)
    if not all(math.isfinite(value) for value in ordered):
        raise CampaignError("numeric evidence contains a non-finite value")
    position = (len(ordered) - 1) * quantile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def numeric_summary(values: Sequence[float], unit: str) -> dict[str, Any]:
    numbers = [float(value) for value in values]
    if not numbers:
        raise CampaignError(f"cannot produce an empty {unit} summary")
    return {
        "count": len(numbers),
        "unit": unit,
        "minimum": min(numbers),
        "median": statistics.median(numbers),
        "p95": percentile(numbers, 95.0),
        "maximum": max(numbers),
    }


def parse_front_image_publish_timing(stack_text: str) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for line_number, line in enumerate(stack_text.splitlines(), start=1):
        match = RUNTIME_TIMING_FRONT_IMAGE_RE.search(line)
        if match is None:
            continue
        samples.append(
            {
                "line_number": line_number,
                "duration_ms": float(match.group("duration")),
                "monotonic_ns": int(match.group("monotonic")),
                "source_stamp_sec": float(match.group("source")),
                "suppressed_count_reported": int(match.group("suppressed") or 0),
            }
        )
    return samples


def parse_env_assignments(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key and re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            values[key] = value
    return values


def _relative_path(path: Path, root: Path) -> str | None:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return None


def authoritative_file_pointer(
    path: Path,
    *,
    repo_root: Path,
    campaign_root: Path,
    source_root: Path,
    role: str,
) -> dict[str, Any]:
    absolute = _absolute(path)
    if absolute.is_symlink() or not absolute.is_file():
        raise CampaignError(f"authoritative evidence file is missing or unsafe: {absolute}")
    return {
        "role": role,
        "absolute_path": str(absolute),
        "repository_relative_path": _relative_path(absolute, repo_root),
        "campaign_relative_path": _relative_path(absolute, campaign_root),
        "source_relative_path": _relative_path(absolute, source_root),
        "size_bytes": absolute.stat().st_size,
        "sha256": sha256_file(absolute),
    }


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CampaignError(f"cannot load {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise CampaignError(f"{label} must be a JSON object: {path}")
    return value


def summarize_runtime_health_attempt(
    attempt_id: str,
    attempt_root: Path,
    *,
    repo_root: Path,
    campaign_root: Path,
    source_root: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    runtime_path = attempt_root / "runtime_health.json"
    stack_path = attempt_root / "stack.log"
    runtime_env_path = attempt_root / "runtime.env"
    runtime_pointer = authoritative_file_pointer(
        runtime_path,
        repo_root=repo_root,
        campaign_root=campaign_root,
        source_root=source_root,
        role=f"{attempt_id} pre-engagement runtime health",
    )
    stack_pointer = authoritative_file_pointer(
        stack_path,
        repo_root=repo_root,
        campaign_root=campaign_root,
        source_root=source_root,
        role=f"{attempt_id} runtime timing log",
    )
    runtime_env_pointer = authoritative_file_pointer(
        runtime_env_path,
        repo_root=repo_root,
        campaign_root=campaign_root,
        source_root=source_root,
        role=f"{attempt_id} captured runtime/gate contract",
    )
    health = _load_json_object(runtime_path, "runtime health evidence")
    if health.get("status") != "FAIL":
        raise CampaignError(f"{attempt_id} is not a failed runtime-health attempt")
    runtime = health.get("runtime")
    sequence = health.get("sequence")
    windows = health.get("windows")
    contract = health.get("contract")
    if not all(isinstance(value, dict) for value in (runtime, sequence, contract)):
        raise CampaignError(f"{attempt_id} runtime-health schema is incomplete")
    if not isinstance(windows, list) or not windows or not all(
        isinstance(window, dict) for window in windows
    ):
        raise CampaignError(f"{attempt_id} has no runtime-health windows")
    if runtime.get("vehicle_engaged") is not False:
        raise CampaignError(f"{attempt_id} does not prove vehicle_engaged=false")
    if sequence.get("status") != "FAIL" or sequence.get("timed_out") is not True:
        raise CampaignError(f"{attempt_id} does not prove a timed-out health gate")

    try:
        rtfs = [float(window["clock"]["rtf"]) for window in windows]
        minimum_rates = [
            float(window["minimum_observed_camera_wall_rate_hz"])
            for window in windows
        ]
        front_rates = [
            float(window["cameras"][CAM_FRONT_INFO_TOPIC]["wall_rate_hz"])
            for window in windows
        ]
        bundle_p95 = [
            float(window["bundles"]["receipt_span_seconds"]["p95"])
            for window in windows
        ]
        thresholds = contract["thresholds"]
        minimum_rtf = float(thresholds["minimum_rtf"])
        minimum_camera_rate = float(thresholds["minimum_camera_wall_rate_hz"])
        maximum_bundle_p95 = float(thresholds["maximum_bundle_receipt_p95_seconds"])
    except (KeyError, TypeError, ValueError) as error:
        raise CampaignError(f"{attempt_id} metric schema is incomplete: {error}") from error

    timing_samples = parse_front_image_publish_timing(
        stack_path.read_text(encoding="utf-8", errors="replace")
    )
    if not timing_samples:
        raise CampaignError(
            f"{attempt_id} stack.log has no explicit CAM_FRONT camera_image_publish timing"
        )
    timing_durations = [float(sample["duration_ms"]) for sample in timing_samples]
    stack_text = stack_path.read_text(encoding="utf-8", errors="replace")
    waiting_for_engage_lines = [
        line_number
        for line_number, line in enumerate(stack_text.splitlines(), start=1)
        if "AutowareState: Planning => WaitingForEngage" in line
    ]
    engaged_transition_lines = [
        line_number
        for line_number, line in enumerate(stack_text.splitlines(), start=1)
        if "WaitingForEngage => Driving" in line or "Route test engaged" in line
    ]
    if not waiting_for_engage_lines or engaged_transition_lines:
        raise CampaignError(
            f"{attempt_id} stack chronology does not prove waiting-without-engage"
        )
    runtime_env = parse_env_assignments(
        runtime_env_path.read_text(encoding="utf-8", errors="replace")
    )
    required_env = {
        "RUNTIME_HEALTH_GATE_PHASE": (
            "after_optional_rviz_recorder_before_rosbag_and_engagement"
        ),
        "RUNTIME_HEALTH_GATE_STATUS": "FAIL",
        "RUNTIME_HEALTH_GATE_EXIT_CODE": "1",
        "RUNTIME_HEALTH_EVIDENCE_SHA256": runtime_pointer["sha256"],
        "RUNTIME_HEALTH_EVALUATED_WINDOWS": str(len(windows)),
        "RUNTIME_HEALTH_MAXIMUM_CONSECUTIVE_PASSES": "0",
        "CAMERA_SOURCE_5HZ": "true",
        "CAMERA_SOURCE_SENSOR_TICK_SEC": "0.2",
        "CAMERA_ROS_PUBLISH_HZ": "5.0",
    }
    mismatches = {
        key: {"expected": expected, "observed": runtime_env.get(key)}
        for key, expected in required_env.items()
        if runtime_env.get(key) != expected
    }
    if mismatches:
        raise CampaignError(f"{attempt_id} runtime.env contract mismatch: {mismatches}")
    captured_probe = health.get("source")
    if (
        not isinstance(captured_probe, dict)
        or captured_probe.get("path") != runtime_env.get("RUNTIME_HEALTH_PROBE_FILE")
        or captured_probe.get("sha256") != runtime_env.get("RUNTIME_HEALTH_PROBE_SHA256")
    ):
        raise CampaignError(f"{attempt_id} captured probe provenance is inconsistent")
    configured_camera_hz = float(runtime_env["CAMERA_ROS_PUBLISH_HZ"])
    rtf_median = statistics.median(rtfs)
    observed_camera_median = statistics.median(minimum_rates)
    scaled_camera_hz = configured_camera_hz * rtf_median
    bundle_median_ms = statistics.median(bundle_p95) * 1000.0
    publish_median_ms = statistics.median(timing_durations)
    metrics = {
        "attempt_id": attempt_id,
        "status": "PRE_ENGAGEMENT_RUNTIME_HEALTH_FAIL",
        "checked_at": health.get("checked_at"),
        "finished_at": health.get("finished_at"),
        "error": health.get("error"),
        "window_count": len(windows),
        "gate_sequence": {
            "required_consecutive_passes": sequence.get("required_consecutive_passes"),
            "maximum_consecutive_passes": sequence.get("maximum_consecutive_passes"),
            "timed_out": sequence.get("timed_out"),
            "elapsed_wall_seconds": sequence.get("elapsed_wall_seconds"),
        },
        "engagement": {
            "vehicle_engaged": False,
            "blocked_before_engagement": True,
            "rosbag_started": runtime.get("rosbag_started"),
            "gate_phase": runtime_env["RUNTIME_HEALTH_GATE_PHASE"],
            "waiting_for_engage_stack_lines": waiting_for_engage_lines,
            "engaged_transition_stack_lines": engaged_transition_lines,
            "basis": [
                "runtime_health.json runtime.vehicle_engaged is false and gate timed out",
                "runtime.env places the gate before rosbag and engagement",
                "stack.log reaches WaitingForEngage and contains no Driving/engaged transition",
            ],
        },
        "captured_probe_provenance": dict(captured_probe),
        "thresholds": {
            "minimum_rtf": minimum_rtf,
            "minimum_camera_wall_rate_hz": minimum_camera_rate,
            "maximum_bundle_receipt_p95_seconds": maximum_bundle_p95,
        },
        "observed": {
            "rtf": numeric_summary(rtfs, "ratio"),
            "minimum_camera_wall_rate": numeric_summary(minimum_rates, "Hz"),
            "cam_front_camera_info_wall_rate": numeric_summary(front_rates, "Hz"),
            "bundle_receipt_window_p95": numeric_summary(bundle_p95, "seconds"),
            "runtime_timing_cam_front_image_publish": {
                **numeric_summary(timing_durations, "milliseconds"),
                "explicit_log_sample_count": len(timing_samples),
                "reported_suppressed_count_sum": sum(
                    int(sample["suppressed_count_reported"])
                    for sample in timing_samples
                ),
                "samples": timing_samples,
                "sampling_limit": (
                    "stack.log contains thresholded/rate-limited warnings; suppressed counts "
                    "are reported but are not reconstructed as synthetic duration samples"
                ),
            },
            "camera_cadence_consistency": {
                "configured_simulation_publish_hz": configured_camera_hz,
                "configured_sensor_tick_seconds": float(
                    runtime_env["CAMERA_SOURCE_SENSOR_TICK_SEC"]
                ),
                "median_rtf": rtf_median,
                "rtf_scaled_expected_wall_hz": scaled_camera_hz,
                "observed_median_wall_hz": observed_camera_median,
                "absolute_difference_hz": abs(observed_camera_median - scaled_camera_hz),
                "interpretation": (
                    "The configured 5 Hz simulation cadence scaled by the low RTF is "
                    "consistent with the approximately 1.25 Hz wall receipt cadence."
                ),
            },
            "publish_vs_bundle_receipt": {
                "cam_front_publish_median_ms": publish_median_ms,
                "bundle_receipt_window_p95_median_ms": bundle_median_ms,
                "ratio": publish_median_ms / bundle_median_ms,
                "interpretation": (
                    "The similar magnitudes make serialized CAM_FRONT publication or DDS "
                    "backpressure a strong candidate contributor; this is an inference, not "
                    "a causal proof."
                ),
            },
        },
        "all_windows_below_rtf_threshold": all(value < minimum_rtf for value in rtfs),
        "all_windows_below_camera_rate_threshold": all(
            value < minimum_camera_rate for value in minimum_rates
        ),
        "all_windows_above_bundle_receipt_p95_threshold": all(
            value > maximum_bundle_p95 for value in bundle_p95
        ),
        "authoritative_files": {
            "runtime_health": runtime_pointer,
            "stack_log": stack_pointer,
            "runtime_env": runtime_env_pointer,
        },
        "attempt_tree": {
            "absolute_path": str(attempt_root),
            "repository_relative_path": _relative_path(attempt_root, repo_root),
            "campaign_relative_path": _relative_path(attempt_root, campaign_root),
            **inspect_path(attempt_root),
        },
    }
    return metrics, [runtime_pointer, stack_pointer, runtime_env_pointer]


def build_stutter_evidence(
    repo_root: Path,
    campaign_root: Path,
    source_root: Path,
    output_root: Path,
    *,
    generated_at: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    repo = _absolute(repo_root)
    campaign = _absolute(campaign_root)
    source = _absolute(source_root)
    output = _absolute(output_root)
    if source.is_symlink() or not source.is_dir():
        raise CampaignError(f"stutter source root is missing or unsafe: {source}")
    if output == source or _is_relative_to(output, source) or _is_relative_to(source, output):
        raise CampaignError("stutter output and authoritative source must not overlap")
    owned_summary_path = source / "owned_trial_summary.json"
    owned_console_path = source / "owned_trial_console.log"
    owned_summary = _load_json_object(owned_summary_path, "owned trial summary")
    expected_attempt_ids = ["attempt_001", "attempt_002", "attempt_003"]
    attempts = owned_summary.get("attempts")
    if (
        owned_summary.get("status") != "FAIL"
        or owned_summary.get("selected_attempt") is not None
        or not isinstance(attempts, list)
        or [attempt.get("attempt_id") for attempt in attempts] != expected_attempt_ids
        or any(
            attempt.get("runtime_health_status") != "FAIL"
            or attempt.get("process_exit_status") == 0
            for attempt in attempts
        )
    ):
        raise CampaignError("owned trial summary does not prove exactly three failed attempts")

    source_snapshot = inspect_path(source)
    owned_summary_pointer = authoritative_file_pointer(
        owned_summary_path,
        repo_root=repo,
        campaign_root=campaign,
        source_root=source,
        role="three-attempt owned trial summary",
    )
    owned_console_pointer = authoritative_file_pointer(
        owned_console_path,
        repo_root=repo,
        campaign_root=campaign,
        source_root=source,
        role="owned trial fail/retry console chronology",
    )
    attempt_summaries: list[dict[str, Any]] = []
    authoritative_files = [owned_summary_pointer, owned_console_pointer]
    for attempt_id in expected_attempt_ids:
        metrics, pointers = summarize_runtime_health_attempt(
            attempt_id,
            source / "attempts" / attempt_id,
            repo_root=repo,
            campaign_root=campaign,
            source_root=source,
        )
        attempt_summaries.append(metrics)
        authoritative_files.extend(pointers)

    pointers = signed_payload(
        {
            "schema_version": 1,
            "kind": "autoware_e2e_runtime_stutter_source_pointers",
            "generated_at_utc": generated_at or utc_now(),
            "source_handling": {
                "mode": "READ_ONLY_ABSOLUTE_PATH_POINTERS",
                "source_moved": False,
                "source_deleted": False,
                "source_copied": False,
            },
            "authoritative_source_root": {
                "absolute_path": str(source),
                "repository_relative_path": _relative_path(source, repo),
                "campaign_relative_path": _relative_path(source, campaign),
                **source_snapshot,
            },
            "canonical_output_root": {
                "absolute_path": str(output),
                "repository_relative_path": _relative_path(output, repo),
                "campaign_relative_path": _relative_path(output, campaign),
            },
            "authoritative_files": authoritative_files,
        }
    )
    pointer_path = output / "source_pointers.json"
    summary = signed_payload(
        {
            "schema_version": 1,
            "kind": "autoware_e2e_pre_engagement_runtime_stutter_summary",
            "generated_at_utc": generated_at or utc_now(),
            "campaign": "autoware_vad_runtime_control_campaign_v1",
            "case_id": "town07_straight_A_baseline_health_001",
            "map": owned_summary.get("map"),
            "route": owned_summary.get("route"),
            "route_sha256": owned_summary.get("route_sha256"),
            "status": "PRE_ENGAGEMENT_RUNTIME_HEALTH_FAILED_ALL_ATTEMPTS",
            "attempt_count": 3,
            "selected_attempt": None,
            "vehicle_engaged_any_attempt": False,
            "engagement_blocked_before_motion": True,
            "interpretation": (
                "All three fresh-CARLA attempts were stopped by the pre-engagement runtime "
                "health gate. These are runtime/camera delivery failure observations, not "
                "post-engagement path-following or control-quality measurements."
            ),
            "metric_semantics": {
                "rtf": "ROS /clock simulation delta divided by monotonic wall-window duration",
                "camera_rate": "monotonic wall receipt rate for camera_info",
                "bundle_receipt_window_p95": (
                    "within each window, p95 monotonic receipt span for six camera_info messages "
                    "matched by header stamp; summary describes the 37 window-p95 values"
                ),
                "runtime_timing_cam_front_image_publish": (
                    "explicit thresholded/rate-limited stack.log warning durations only"
                ),
            },
            "source_pointer_manifest": {
                "absolute_path": str(pointer_path),
                "repository_relative_path": _relative_path(pointer_path, repo),
                "campaign_relative_path": _relative_path(pointer_path, campaign),
                "canonical_payload_sha256": pointers["manifest_sha256"],
            },
            "source_root_tree_sha256": source_snapshot["tree_sha256"],
            "attempts": attempt_summaries,
        }
    )
    readme = render_stutter_readme(summary, pointers)
    if not _snapshot_matches(source_snapshot, inspect_path(source)):
        raise CampaignError("authoritative stutter source changed while it was being summarized")
    return pointers, summary, readme


def _format_range(summary: Mapping[str, Any], scale: float = 1.0) -> str:
    return (
        f"{float(summary['median']) * scale:.3f} "
        f"[{float(summary['minimum']) * scale:.3f}–"
        f"{float(summary['maximum']) * scale:.3f}]"
    )


def render_stutter_readme(
    summary: Mapping[str, Any], pointers: Mapping[str, Any]
) -> str:
    lines = [
        "# Town07 straight baseline: pre-engagement runtime stutter",
        "",
        "Three fresh-CARLA attempts failed the runtime-health gate before engage. The vehicle",
        "was never engaged, so this evidence diagnoses runtime/camera delivery and does not",
        "measure driven path tracking or control quality.",
        "",
        "| Attempt | RTF median [min–max] | min camera rate Hz | bundle receipt window-p95 ms | CAM_FRONT image publish ms | Engage |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for attempt in summary["attempts"]:
        observed = attempt["observed"]
        lines.append(
            "| {attempt} | {rtf} | {rate} | {bundle} | {publish} | blocked |".format(
                attempt=attempt["attempt_id"],
                rtf=_format_range(observed["rtf"]),
                rate=_format_range(observed["minimum_camera_wall_rate"]),
                bundle=_format_range(observed["bundle_receipt_window_p95"], 1000.0),
                publish=_format_range(observed["runtime_timing_cam_front_image_publish"]),
            )
        )
    first = summary["attempts"][0]
    thresholds = first["thresholds"]
    lines.extend(
        [
            "",
            "Values are `median [minimum–maximum]` across 37 overlapping eight-second health",
            "windows. Required thresholds were RTF ≥ {rtf:.1f}, camera rate ≥ {rate:.1f} Hz,"
            " and bundle receipt p95 ≤ {bundle:.0f} ms. Every window in every attempt violated"
            " all three thresholds.".format(
                rtf=thresholds["minimum_rtf"],
                rate=thresholds["minimum_camera_wall_rate_hz"],
                bundle=thresholds["maximum_bundle_receipt_p95_seconds"] * 1000.0,
            ),
            "",
            "The CAM_FRONT publish durations come from explicit runtime timing warnings for",
            "`stage=camera_image_publish camera=CAM_FRONT`. They are thresholded and",
            "rate-limited; suppressed events are recorded in JSON but are not imputed.",
            "",
            "## Authoritative source",
            "",
            f"- Absolute: `{pointers['authoritative_source_root']['absolute_path']}`",
            f"- Repository-relative: `{pointers['authoritative_source_root']['repository_relative_path']}`",
            f"- Campaign-relative: `{pointers['authoritative_source_root']['campaign_relative_path']}`",
            f"- Source tree SHA-256: `{summary['source_root_tree_sha256']}`",
            f"- Pointer canonical-payload SHA-256: `{pointers['manifest_sha256']}`",
            f"- Summary canonical-payload SHA-256: `{summary['manifest_sha256']}`",
            "",
            "`source_pointers.json` lists the absolute and relative authoritative path, size,",
            "and SHA-256 for every `runtime_health.json`, `runtime.env`, and `stack.log`. The authoritative",
            "attempts remain in place; no cleanup, move, delete, copy, or symlink was performed.",
            "",
        ]
    )
    return "\n".join(lines)


def write_stutter_evidence(
    repo_root: Path,
    campaign_root: Path,
    source_root: Path,
    output_root: Path,
    *,
    generated_at: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    pointers, summary, readme = build_stutter_evidence(
        repo_root,
        campaign_root,
        source_root,
        output_root,
        generated_at=generated_at,
    )
    output = _absolute(output_root)
    output.mkdir(parents=True, exist_ok=True)
    atomic_write_json(output / "source_pointers.json", pointers)
    atomic_write_json(output / "runtime_stutter_summary.json", summary)
    atomic_write_text(output / "README.md", readme)
    return pointers, summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create a pointer-only validation campaign and plan safe quarantine moves."
    )
    subparsers = parser.add_subparsers(dest="command")

    index_parser = subparsers.add_parser(
        "index", help="create/update only the pointer index and directory skeleton"
    )
    index_parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    index_parser.add_argument("--campaign-root", type=Path, default=DEFAULT_CAMPAIGN_ROOT)

    plan_parser = subparsers.add_parser(
        "plan-cleanup", help="hash audited candidates and write a dry-run manifest"
    )
    plan_parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    plan_parser.add_argument("--tmp-root", type=Path, default=DEFAULT_TMP_ROOT)
    plan_parser.add_argument("--campaign-root", type=Path, default=DEFAULT_CAMPAIGN_ROOT)
    plan_parser.add_argument("--output", type=Path)

    stutter_parser = subparsers.add_parser(
        "summarize-stutter",
        help="write pointer-only Town07 pre-engagement stutter evidence",
    )
    stutter_parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    stutter_parser.add_argument("--campaign-root", type=Path, default=DEFAULT_CAMPAIGN_ROOT)
    stutter_parser.add_argument("--source-root", type=Path, default=DEFAULT_STUTTER_SOURCE)
    stutter_parser.add_argument("--output-root", type=Path, default=DEFAULT_STUTTER_OUTPUT)

    apply_parser = subparsers.add_parser(
        "apply-cleanup", help="explicitly rename hash-verified manifest entries to quarantine"
    )
    apply_parser.add_argument("--manifest", type=Path, required=True)
    apply_parser.add_argument("--quarantine-root", type=Path, required=True)
    apply_parser.add_argument("--journal", type=Path)
    apply_parser.add_argument("--confirm", choices=(APPLY_CONFIRMATION,), required=True)

    restore_parser = subparsers.add_parser(
        "restore-cleanup", help="restore moved entries from an apply journal"
    )
    restore_parser.add_argument("--journal", type=Path, required=True)
    restore_parser.add_argument("--confirm", choices=(RESTORE_CONFIRMATION,), required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    command = args.command or "index"
    try:
        if command == "index":
            repo_root = getattr(args, "repo_root", REPO_ROOT)
            campaign_root = getattr(args, "campaign_root", DEFAULT_CAMPAIGN_ROOT)
            index = initialize_campaign(repo_root, campaign_root)
            print(
                json.dumps(
                    {
                        "status": "POINTER_INDEX_READY",
                        "campaign_root": index["campaign_root"],
                        "index": str(_absolute(campaign_root) / "00_index/INDEX.json"),
                        "source_mutation": "NONE",
                    },
                    sort_keys=True,
                )
            )
        elif command == "plan-cleanup":
            output = args.output or args.campaign_root / "99_integrity/cleanup-plan.json"
            plan = write_cleanup_plan(args.repo_root, args.tmp_root, output)
            print(
                json.dumps(
                    {
                        "status": "DRY_RUN_PLAN_READY",
                        "manifest": str(_absolute(output)),
                        "summary": plan["summary"],
                        "applied": False,
                    },
                    sort_keys=True,
                )
            )
        elif command == "summarize-stutter":
            pointers, summary = write_stutter_evidence(
                args.repo_root,
                args.campaign_root,
                args.source_root,
                args.output_root,
            )
            print(
                json.dumps(
                    {
                        "status": summary["status"],
                        "output_root": str(_absolute(args.output_root)),
                        "attempt_count": summary["attempt_count"],
                        "vehicle_engaged_any_attempt": summary[
                            "vehicle_engaged_any_attempt"
                        ],
                        "source_pointer_manifest_sha256": pointers[
                            "manifest_sha256"
                        ],
                        "source_mutation": "NONE",
                    },
                    sort_keys=True,
                )
            )
        elif command == "apply-cleanup":
            journal = apply_cleanup(
                args.manifest,
                args.quarantine_root,
                journal_path=args.journal,
                confirmed=args.confirm == APPLY_CONFIRMATION,
            )
            print(
                json.dumps(
                    {
                        "status": journal["status"],
                        "restore_command": journal["restore_command"],
                    },
                    sort_keys=True,
                )
            )
        elif command == "restore-cleanup":
            journal = restore_cleanup(
                args.journal,
                confirmed=args.confirm == RESTORE_CONFIRMATION,
            )
            print(json.dumps({"status": journal["status"]}, sort_keys=True))
        else:
            parser.error(f"unknown command: {command}")
    except CampaignError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
