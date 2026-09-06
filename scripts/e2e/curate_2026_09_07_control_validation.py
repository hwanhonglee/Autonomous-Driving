#!/usr/bin/env python3
"""HH_260906 - Publish and verify the authoritative 30/60 km/h review set."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RAW_2026_09_06 = REPOSITORY_ROOT / "artifacts/validation/2026-09-06"
RAW_2026_09_07 = REPOSITORY_ROOT / "artifacts/validation/2026-09-07"
RAW_2026_09_02 = (
    REPOSITORY_ROOT
    / "artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs/assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1"
)
PUBLICATION_ID = "autoware-e2e.control-validation.2026-09-07.v1"

TOWN07 = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_town07_control_ab_10hz_v1_recovery"
)
CTRACK_5M = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_ctrack_control_ab_10hz_v8_5m"
)
CTRACK_5M_SCREEN = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_ctrack_5m_safety_screen_v7"
)
CTRACK_10M = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_ctrack_control_ab_10hz_v6_camera_barrier_rviz_ack"
)
CTRACK_BARRIER_V2 = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_ctrack_camera_barrier_v2"
)
TOWN03 = RAW_2026_09_06 / (
    "portable_e2e_physical_v1_30kph_town03_control_ab_10hz_v1_10m"
)
TOWN06_60 = RAW_2026_09_02 / (
    "30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3"
)
TOWN06_60_STRICT = RAW_2026_09_07 / (
    "autoware_vad_60kph_town06_strict10_transport_v1"
)

TRIAL_VISUALS = (
    ("autoware_rviz_fullscreen.png", "01_vehicle_centered_fullscreen.png"),
    ("turn_path_control.gif", "02_path_and_control.gif"),
    ("path_vs_control.png", "03_path_vs_control.png"),
    ("steering_tracking.png", "04_steering_tracking.png"),
    ("speed_profile.png", "05_speed_profile.png"),
    ("longitudinal_response.png", "06_longitudinal_response.png"),
    ("route_result.png", "07_route_result.png"),
    ("latency/e2e_latency.png", "08_e2e_latency.png"),
)
TRIAL_DRIVE = ("autoware_rviz_drive.gif", "09_autoware_drive_5fps.gif")
TRIAL_EVIDENCE = (
    ("result.json", "result.json"),
    ("diagnosis.json", "diagnosis.json"),
    ("desktop_capture.json", "desktop_capture.json"),
    ("latency/e2e_latency.json", "e2e_latency.json"),
    (
        "portable_shadow_provenance/shadow_evidence_analysis.json",
        "portable_shadow_evidence.json",
    ),
    ("runtime_health.json", "runtime_health.json"),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _image_dimensions(path: Path) -> list[int] | None:
    with path.open("rb") as stream:
        header = stream.read(24)
    if header.startswith(b"\x89PNG\r\n\x1a\n") and len(header) >= 24:
        width, height = struct.unpack(">II", header[16:24])
        return [width, height]
    if header[:6] in {b"GIF87a", b"GIF89a"} and len(header) >= 10:
        width, height = struct.unpack("<HH", header[6:10])
        return [width, height]
    return None


def _regular_source(path: Path) -> Path:
    if path.is_symlink() or not path.is_file():
        raise RuntimeError(f"missing regular source evidence: {path}")
    resolved = path.resolve(strict=True)
    if REPOSITORY_ROOT != resolved and REPOSITORY_ROOT not in resolved.parents:
        raise RuntimeError(f"source evidence escapes repository: {path}")
    if resolved.stat().st_size <= 0:
        raise RuntimeError(f"empty source evidence: {path}")
    return resolved


def _trial_specs(
    arm_root: Path,
    destination: str,
    *,
    complete_visuals: bool = True,
    include_drive: bool = False,
) -> list[tuple[Path, str]]:
    attempt = arm_root / "attempts/attempt_001"
    visuals = (
        TRIAL_VISUALS
        if complete_visuals
        else (
            TRIAL_VISUALS[0],
            TRIAL_VISUALS[1],
            TRIAL_VISUALS[2],
            TRIAL_VISUALS[6],
        )
    )
    specs = [
        (attempt / source, f"{destination}/visuals/{target}")
        for source, target in visuals
    ]
    if include_drive:
        source, target = TRIAL_DRIVE
        specs.append((attempt / source, f"{destination}/visuals/{target}"))
    evidence = TRIAL_EVIDENCE if complete_visuals else TRIAL_EVIDENCE[:4]
    specs.extend(
        (attempt / source, f"{destination}/evidence/{target}")
        for source, target in evidence
    )
    specs.append(
        (arm_root / "owned_trial_summary.json", f"{destination}/evidence/owned_trial_summary.json")
    )
    return specs


def _publication_specs() -> list[tuple[Path, str]]:
    specs: list[tuple[Path, str]] = []
    specs.extend(
        _trial_specs(
            TOWN07 / "pair_01/A_baseline",
            "01_town07_straight/A_baseline_route_fail",
        )
    )
    specs.extend(
        _trial_specs(
            TOWN07 / "pair_01/B_longitudinal_recovery_2p0",
            "01_town07_straight/B_recovery_2p0_no_go",
            include_drive=True,
        )
    )
    specs.extend(
        _trial_specs(
            CTRACK_5M / "pair_01/A_baseline",
            "02_c_track_turn/A_baseline_selected",
        )
    )
    specs.extend(
        _trial_specs(
            CTRACK_5M / "pair_01/B_turn_preview_5m",
            "02_c_track_turn/B_preview_5m_hold",
            include_drive=True,
        )
    )
    specs.extend(
        _trial_specs(
            TOWN03 / "pair_01/A_baseline",
            "03_town03_turn/A_baseline_selected",
        )
    )
    specs.extend(
        _trial_specs(
            TOWN03 / "pair_01/B_turn_preview_10m",
            "03_town03_turn/B_preview_10m_no_go",
            include_drive=True,
        )
    )
    specs.extend(
        _trial_specs(
            CTRACK_10M / "pair_04/B_turn_preview_10m",
            "04_rejected_and_runtime_diagnostics/C_track_preview_10m_geometry_fail",
            complete_visuals=False,
            include_drive=True,
        )
    )
    campaign_files = (
        (TOWN07, "01_town07_straight", "town07"),
        (CTRACK_5M, "02_c_track_turn", "c_track_5m"),
        (TOWN03, "03_town03_turn", "town03"),
        (
            CTRACK_10M,
            "04_rejected_and_runtime_diagnostics",
            "c_track_10m_rejected",
        ),
    )
    for source_root, destination, prefix in campaign_files:
        for name in (
            "campaign_plan.json",
            "campaign_closure.json",
            "control_ab_repeat_summary.json",
            "control_ab_repeat_summary.png",
        ):
            specs.append(
                (
                    source_root / name,
                    f"{destination}/comparison/{prefix}_{name}",
                )
            )
    specs.extend(
        (
            CTRACK_5M / "pair_01/comparison" / name,
            f"02_c_track_turn/comparison/{name}",
        )
        for name in (
            "A_baseline_vs_B_turn_preview_5m_decision.json",
            "A_baseline_vs_B_turn_preview_5m_decision.png",
        )
    )
    specs.extend(
        (
            TOWN03 / "pair_01/comparison" / name,
            f"03_town03_turn/comparison/{name}",
        )
        for name in (
            "A_baseline_vs_B_turn_preview_10m_decision.json",
            "A_baseline_vs_B_turn_preview_10m_decision.png",
        )
    )
    specs.extend(
        _trial_specs(
            CTRACK_5M_SCREEN / "B_turn_preview_5m",
            "02_c_track_turn/qualification/preview_5m_safety_screen_pass",
        )
    )
    specs.extend(
        [
            (
                CTRACK_5M_SCREEN / "screen_plan.json",
                "02_c_track_turn/qualification/preview_5m_screen_plan.json",
            ),
            (
                CTRACK_10M
                / "pair_04/A_baseline/attempts/attempt_001/latency/e2e_latency.json",
                "04_rejected_and_runtime_diagnostics/host_stall/e2e_latency.json",
            ),
            (
                CTRACK_10M
                / "pair_04/A_baseline/attempts/attempt_001/desktop_capture.json",
                "04_rejected_and_runtime_diagnostics/host_stall/desktop_capture.json",
            ),
            (
                CTRACK_10M
                / "pair_04/A_baseline/attempts/attempt_001/portable_shadow_provenance/shadow_evidence_analysis.json",
                "04_rejected_and_runtime_diagnostics/host_stall/portable_shadow_evidence.json",
            ),
            (
                CTRACK_10M / "pair_04/A_baseline/owned_trial_summary.json",
                "04_rejected_and_runtime_diagnostics/host_stall/owned_trial_summary.json",
            ),
            (
                CTRACK_10M
                / "pair_04/A_baseline/attempts/attempt_001/autoware_rviz_fullscreen.png",
                "04_rejected_and_runtime_diagnostics/host_stall/01_vehicle_centered_fullscreen.png",
            ),
            (
                CTRACK_10M
                / "pair_04/A_baseline/attempts/attempt_001/autoware_rviz_drive.gif",
                "04_rejected_and_runtime_diagnostics/host_stall/02_autoware_drive_5fps.gif",
            ),
            (
                CTRACK_BARRIER_V2 / "camera_stamp_audit.json",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/camera_stamp_audit.json",
            ),
            (
                CTRACK_BARRIER_V2 / "single_run_acceptance.json",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/single_run_acceptance.json",
            ),
            (
                CTRACK_BARRIER_V2 / "single_run_acceptance.png",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/single_run_acceptance.png",
            ),
            (
                CTRACK_BARRIER_V2 / "attempts/attempt_001/runtime_health.json",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/runtime_health.json",
            ),
            (
                CTRACK_BARRIER_V2 / "attempts/attempt_001/desktop_capture.json",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/desktop_capture.json",
            ),
            (
                CTRACK_BARRIER_V2
                / "attempts/attempt_001/portable_shadow_provenance/shadow_evidence_analysis.json",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/portable_shadow_evidence.json",
            ),
            (
                CTRACK_BARRIER_V2
                / "attempts/attempt_001/autoware_rviz_fullscreen.png",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/01_vehicle_centered_fullscreen.png",
            ),
            (
                CTRACK_BARRIER_V2 / "attempts/attempt_001/autoware_rviz_drive.gif",
                "04_rejected_and_runtime_diagnostics/healthy_camera_reference/02_autoware_drive_5fps.gif",
            ),
            (
                TOWN07 / "pair_01/A_baseline/attempts/attempt_001/source_route.json",
                "01_town07_straight/route/source_route.json",
            ),
            (
                CTRACK_5M / "pair_01/A_baseline/attempts/attempt_001/source_route.json",
                "02_c_track_turn/route/source_route.json",
            ),
            (
                TOWN03 / "pair_01/A_baseline/attempts/attempt_001/source_route.json",
                "03_town03_turn/route/source_route.json",
            ),
        ]
    )
    sixty_attempt = TOWN06_60 / "trial/attempt_001"
    for source, target in (
        ("autoware_rviz_fullscreen.png", "01_vehicle_centered_fullscreen.png"),
        ("autoware_rviz_drive.gif", "02_autoware_drive_5fps.gif"),
        ("path_vs_control.png", "03_path_vs_control.png"),
        ("speed_profile.png", "04_speed_profile.png"),
        ("longitudinal_response.png", "05_longitudinal_response.png"),
        ("runtime_load_analysis.png", "06_runtime_load.png"),
    ):
        specs.append(
            (
                sixty_attempt / source,
                f"05_60kph_readiness/historical_pilot/visuals/{target}",
            )
        )
    for name in (
        "result.json",
        "diagnosis.json",
        "actuation_map_coverage.json",
        "actuation_map_runtime_coverage.json",
        "camera_source_5hz_validation.json",
        "runtime_load_analysis.json",
    ):
        specs.append(
            (
                sixty_attempt / name,
                f"05_60kph_readiness/historical_pilot/evidence/{name}",
            )
        )
    specs.append(
        (
            TOWN06_60 / "pilot_run.json",
            "05_60kph_readiness/historical_pilot/evidence/pilot_run.json",
        )
    )
    strict_attempt = TOWN06_60_STRICT / "trial/attempt_001"
    for source, target in (
        ("autoware_rviz_fullscreen.png", "01_vehicle_centered_fullscreen.png"),
        ("autoware_rviz_drive.gif", "02_autoware_drive_5fps.gif"),
        ("route_result.png", "03_route_result_speed_fail.png"),
        ("speed_profile.png", "04_speed_profile.png"),
        ("longitudinal_response.png", "05_longitudinal_response.png"),
        ("runtime_load_analysis.png", "06_runtime_load.png"),
        ("latency/e2e_latency.png", "07_e2e_latency.png"),
        ("path_vs_control.png", "08_path_vs_control.png"),
        ("steering_tracking.png", "09_steering_tracking.png"),
        ("turn_path_control.gif", "10_town06_straight_path_control.gif"),
    ):
        specs.append(
            (
                strict_attempt / source,
                f"05_60kph_readiness/strict10_live_v1/visuals/{target}",
            )
        )
    for name in (
        "result.json",
        "diagnosis.json",
        "desktop_capture.json",
        "runtime_health.json",
        "camera_source_10hz_strict_validation.json",
        "pilot_acceptance_gate.json",
        "runtime_load_analysis.json",
        "speed_profile.json",
        "longitudinal_response.json",
        "actuation_map_coverage.json",
        "actuation_map_runtime_coverage.json",
        "strict_camera_runtime_parameters.json",
        "carla_preflight_health.json",
        "carla_completion_health.json",
        "carla_cleanup_health.json",
        "source_route.json",
        "aligned_route.json",
    ):
        specs.append(
            (
                strict_attempt / name,
                f"05_60kph_readiness/strict10_live_v1/evidence/{name}",
            )
        )
    specs.extend(
        [
            (
                strict_attempt / "latency/e2e_latency.json",
                "05_60kph_readiness/strict10_live_v1/evidence/e2e_latency.json",
            ),
            (
                TOWN06_60_STRICT / "pilot_run.json",
                "05_60kph_readiness/strict10_live_v1/evidence/pilot_run.json",
            ),
            (
                TOWN06_60_STRICT / "pilot_failure.json",
                "05_60kph_readiness/strict10_live_v1/evidence/pilot_failure.json",
            ),
        ]
    )
    for relative, target in (
        (
            "50_reports/town06_60kph_v3_speed_limit_analysis.json",
            "speed_limit_analysis.json",
        ),
        (
            "50_reports/town06_60kph_v3_speed_limit_analysis.md",
            "speed_limit_analysis.md",
        ),
        (
            "50_reports/town06_60kph_geometry_corridor_ab_v4.json",
            "geometry_corridor_ab.json",
        ),
        (
            "50_reports/town06_60kph_geometry_corridor_ab_v4.md",
            "geometry_corridor_ab.md",
        ),
        (
            "50_reports/town06_60kph_geometry_corridor_ab_v4.png",
            "geometry_corridor_ab.png",
        ),
        (
            "50_reports/town06_60kph_smoothing_offline_screen_v1/summary.json",
            "smoothing_offline_summary.json",
        ),
        (
            "50_reports/town06_60kph_smoothing_offline_screen_v1/comparison.png",
            "smoothing_offline_comparison.png",
        ),
        (
            "50_reports/town06_60kph_smoothing_production_preflight_v1/summary.json",
            "smoothing_production_summary.json",
        ),
        (
            "50_reports/town06_60kph_smoothing_production_preflight_v1/comparison.png",
            "smoothing_production_comparison.png",
        ),
        (
            "50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/summary.json",
            "endpoint_tapered_c1_summary.json",
        ),
        (
            "50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/comparison.png",
            "endpoint_tapered_c1_comparison.png",
        ),
    ):
        specs.append(
            (
                RAW_2026_09_02 / relative,
                f"05_60kph_readiness/analysis/{target}",
            )
        )
    return specs


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"JSON evidence is not an object: {path}")
    return payload


def _arm_summary(arm_root: Path, arm_id: str, selected: bool) -> dict[str, Any]:
    attempt = arm_root / "attempts/attempt_001"
    result = _read_json(attempt / "result.json")
    owner = _read_json(arm_root / "owned_trial_summary.json")
    shadow = _read_json(
        attempt / "portable_shadow_provenance/shadow_evidence_analysis.json"
    )
    latency = _read_json(attempt / "latency/e2e_latency.json")
    metrics = result.get("metrics", {})
    ten_hz = shadow.get("claims", {}).get("ten_hz", {})
    requirements = ten_hz.get("requirements", {})
    continuous = requirements.get("continuous_source_anchor_period", {})
    inference = requirements.get("latency_deadline", {})
    camera = latency.get("camera_bundle", {})
    return {
        "arm_id": arm_id,
        "retained_default_setting": selected,
        "setting_disposition": (
            "RETAINED_DEFAULT" if selected else "CANDIDATE_NOT_PROMOTED"
        ),
        "arm_run_qualified": owner.get("status") == "PASS" and result.get("success") is True,
        "owner_status": owner.get("status"),
        "route_success": result.get("success"),
        "route_reason": result.get("reason"),
        "maximum_speed_mps": metrics.get("maximum_observed_speed_mps"),
        "maximum_cte_m": metrics.get("maximum_absolute_cte_m"),
        "maximum_trajectory_correction_m": metrics.get(
            "maximum_trajectory_correction_m"
        ),
        "maximum_lateral_acceleration_mps2": metrics.get(
            "maximum_lateral_acceleration_mps2"
        ),
        "sim_elapsed_sec": metrics.get("sim_elapsed_sec"),
        "camera_bundle_coverage_percent": camera.get("bundle_coverage_percent"),
        "portable_shadow_analysis_status": shadow.get("analysis_status"),
        "portable_shadow_10hz_status": ten_hz.get("status"),
        "portable_shadow_accepted_count": shadow.get("health", {}).get(
            "accepted_count"
        ),
        "source_period_minimum_ns": continuous.get("observed_minimum_period_ns"),
        "source_period_maximum_ns": continuous.get("observed_maximum_period_ns"),
        "source_period_violation_count": continuous.get(
            "observed_violation_count"
        ),
        "inference_p99_ms": inference.get("observed_p99_ms"),
        "source_result_sha256": _sha256(attempt / "result.json"),
    }


def _result_matrix() -> dict[str, Any]:
    sixty_attempt = TOWN06_60 / "trial/attempt_001"
    sixty_result = _read_json(sixty_attempt / "result.json")
    sixty_coverage = _read_json(sixty_attempt / "actuation_map_coverage.json")
    sixty_camera = _read_json(sixty_attempt / "camera_source_5hz_validation.json")
    sixty_geometry = _read_json(
        RAW_2026_09_02 / "50_reports/town06_60kph_geometry_corridor_ab_v4.json"
    )
    sixty_production = _read_json(
        RAW_2026_09_02
        / "50_reports/town06_60kph_smoothing_production_preflight_v1/summary.json"
    )
    sixty_endpoint = _read_json(
        RAW_2026_09_02
        / "50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/summary.json"
    )
    sixty_metrics = sixty_result.get("metrics", {})
    strict_attempt = TOWN06_60_STRICT / "trial/attempt_001"
    strict_pilot = _read_json(TOWN06_60_STRICT / "pilot_run.json")
    strict_gate = _read_json(strict_attempt / "pilot_acceptance_gate.json")
    strict_camera = _read_json(
        strict_attempt / "camera_source_10hz_strict_validation.json"
    )
    strict_runtime = _read_json(strict_attempt / "runtime_load_analysis.json")
    strict_result = _read_json(strict_attempt / "result.json")
    strict_integrity = strict_runtime.get("camera_delivery", {}).get(
        "source_stamp_integrity", {}
    )
    strict_topic_counts = {
        topic: item.get("record_count")
        for topic, item in strict_integrity.get("topics", {}).items()
    }
    strict_receipt = (
        strict_runtime.get("camera_delivery", {})
        .get("phases", {})
        .get("full_run", {})
        .get("receipt_span_ms", {})
    )
    return {
        "schema_id": "autoware-e2e.control-validation-result-matrix.v1",
        "publication_id": PUBLICATION_ID,
        "control_owner": "autoware_vad",
        "portable_e2e_role": "shadow_only",
        "real_vehicle_ready": False,
        "final_retained_30kph_defaults": {
            "town07_straight": {
                "setting": "baseline",
                "basis": "candidate_not_promoted_latest_baseline_arm_failed_speed_exposure",
            },
            "c_track_turn": {
                "setting": "baseline_turn_preview_3m",
                "basis": "candidate_not_promoted",
            },
            "town03_turn": {
                "setting": "baseline_turn_preview_3m",
                "basis": "candidate_not_promoted",
            },
        },
        "arms": [
            _arm_summary(
                TOWN07 / "pair_01/A_baseline", "town07_baseline", True
            ),
            _arm_summary(
                TOWN07 / "pair_01/B_longitudinal_recovery_2p0",
                "town07_recovery_2p0",
                False,
            ),
            _arm_summary(
                CTRACK_5M / "pair_01/A_baseline", "c_track_baseline", True
            ),
            _arm_summary(
                CTRACK_5M / "pair_01/B_turn_preview_5m",
                "c_track_preview_5m",
                False,
            ),
            _arm_summary(
                TOWN03 / "pair_01/A_baseline", "town03_baseline", True
            ),
            _arm_summary(
                TOWN03 / "pair_01/B_turn_preview_10m",
                "town03_preview_10m",
                False,
            ),
        ],
        "campaign_decisions": {
            "town07_recovery_2p0": "NO_GO",
            "c_track_preview_5m": "NO_GO_HOLD",
            "c_track_preview_10m": "NO_GO_GEOMETRY_FAIL",
            "town03_preview_10m": "NO_GO_HOLD",
            "60kph": "NO_GO_STRICT10_LIVE_V1",
        },
        "historical_60kph_readiness": {
            "map": "Town06",
            "scenario": "straight",
            "target_speed_mps": sixty_coverage.get("target_speed_mps"),
            "maximum_observed_speed_mps": sixty_metrics.get(
                "maximum_observed_speed_mps"
            ),
            "minimum_15mps_sustained_duration_sec": sixty_metrics.get(
                "maximum_sustained_speed_duration_sec"
            ),
            "goal_reached": sixty_result.get("final", {}).get("goal_reached"),
            "speed_exposure_status": sixty_result.get("speed_exposure", {}).get(
                "status"
            ),
            "camera_source_frequency_hz": sixty_camera.get("contract", {}).get(
                "source_frequency_hz"
            ),
            "camera_bundle_coverage_percent": sixty_camera.get(
                "bundle_coverage_percent"
            ),
            "actuation_map_axis_maximum_mps": sixty_coverage.get(
                "map_velocity_axis_maximum_mps"
            ),
            "target_within_actuation_map_axis": sixty_coverage.get(
                "target_within_map_velocity_axis"
            ),
            "corridor_candidate_decision": sixty_geometry.get(
                "geometry_outcome", {}
            ).get("decision"),
            "production_smoothing_preflight_status": sixty_production.get(
                "status"
            ),
            "endpoint_tapered_c1_preflight_status": sixty_endpoint.get("status"),
            "rerun_authorized": False,
            "blockers": [
                "strict_six_camera_10hz_transport_runtime_qualification",
                "geometry_preflight_399_of_399_pass",
                "actuation_map_calibrated_beyond_16p667_mps",
            ],
        },
        "strict10_live_60kph_result": {
            "map": "Town06",
            "scenario": "straight",
            "pilot_status": strict_pilot.get("status"),
            "evidence_integrity_status": strict_gate.get(
                "evidence_integrity_status"
            ),
            "pre_engagement_runtime_health_status": strict_gate.get(
                "summary", {}
            ).get("pre_engagement_runtime_health_status"),
            "standalone_post_run_camera_status": strict_camera.get("status"),
            "final_camera_transport_qualification_status": strict_gate.get(
                "camera_transport_qualification_status"
            ),
            "physical_goal_completion_status": strict_gate.get(
                "physical_goal_completion_status"
            ),
            "speed_exposure_contract_status": strict_gate.get(
                "speed_exposure_contract_status"
            ),
            "simulation_pilot_acceptance_status": strict_gate.get(
                "simulation_pilot_acceptance_status"
            ),
            "real_vehicle_readiness_status": strict_gate.get(
                "real_vehicle_readiness_status"
            ),
            "goal_reached": strict_result.get("final", {}).get("route_status")
            == "goal_reached",
            "traveled_distance_m": strict_result.get("metrics", {}).get(
                "traveled_distance_m"
            ),
            "maximum_observed_speed_mps": strict_result.get("metrics", {}).get(
                "maximum_observed_speed_mps"
            ),
            "maximum_observed_speed_kph": strict_result.get("metrics", {}).get(
                "maximum_observed_speed_mps"
            )
            * 3.6,
            "maximum_sustained_15mps_duration_sec": strict_result.get(
                "metrics", {}
            ).get("maximum_sustained_speed_duration_sec"),
            "route_real_time_factor": strict_gate.get("summary", {}).get(
                "route_real_time_factor"
            ),
            "camera_source_minimum_rate_hz": strict_camera.get(
                "minimum_camera_stamp_rate_hz"
            ),
            "camera_source_maximum_rate_hz": strict_camera.get(
                "maximum_camera_stamp_rate_hz"
            ),
            "camera_bundle_coverage_percent": strict_camera.get(
                "bundle_coverage_percent"
            ),
            "camera_bundle_receipt_p95_ms": strict_receipt.get("p95"),
            "camera_bundle_receipt_limit_ms": 40.0,
            "matched_camera_bundle_count": strict_runtime.get(
                "camera_delivery", {}
            ).get("matched_bundle_count"),
            "camera_record_counts": strict_topic_counts,
            "full_record_set_source_stamp_integrity_status": strict_integrity.get(
                "status"
            ),
            "camera_failures": strict_gate.get(
                "camera_transport_qualification", {}
            ).get("failures"),
            "interpretation": (
                "The standalone cadence check passed, but the final full-run camera "
                "qualification failed. Five topics recorded 865 CameraInfo messages and "
                "CAM_FRONT_LEFT recorded 864; raw-bag inspection localized the mismatch "
                "to the first recorder-boundary stamp. That boundary finding does not "
                "erase the independent 45.715 ms p95 receipt-span failure."
            ),
            "source_hashes": {
                "pilot_run": _sha256(TOWN06_60_STRICT / "pilot_run.json"),
                "acceptance_gate": _sha256(
                    strict_attempt / "pilot_acceptance_gate.json"
                ),
                "runtime_load_analysis": _sha256(
                    strict_attempt / "runtime_load_analysis.json"
                ),
                "result": _sha256(strict_attempt / "result.json"),
            },
        },
    }


def _host_stall_summary() -> dict[str, Any]:
    attempt = CTRACK_10M / "pair_04/A_baseline/attempts/attempt_001"
    desktop_path = attempt / "desktop_capture.json"
    latency_path = attempt / "latency/e2e_latency.json"
    telemetry_path = (
        CTRACK_10M
        / "pair_04/A_baseline/host_telemetry/attempt_001_vmstat.log"
    )
    desktop = _read_json(desktop_path)
    latency = _read_json(latency_path)
    route_start_utc = datetime.fromisoformat(
        desktop["route_evaluation_started_at"].replace("Z", "+00:00")
    )
    route_finish_utc = datetime.fromisoformat(
        desktop["route_evaluation_finished_at"].replace("Z", "+00:00")
    )
    kst = timezone(timedelta(hours=9))
    route_start_kst = route_start_utc.astimezone(kst).replace(tzinfo=None)
    route_finish_kst = route_finish_utc.astimezone(kst).replace(tzinfo=None)
    samples = []
    for line in telemetry_path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) != 19 or len(fields[-2]) != 10 or fields[-2][4] != "-":
            continue
        observed_at = datetime.strptime(
            " ".join(fields[-2:]), "%Y-%m-%d %H:%M:%S"
        )
        if route_start_kst <= observed_at <= route_finish_kst:
            samples.append(
                {
                    "observed_at_kst": observed_at.isoformat(),
                    "run_queue": int(fields[0]),
                    "blocked_processes": int(fields[1]),
                    "cpu_user_percent": int(fields[12]),
                    "cpu_system_percent": int(fields[13]),
                    "cpu_idle_percent": int(fields[14]),
                    "cpu_iowait_percent": int(fields[15]),
                    "cpu_stolen_percent": int(fields[16]),
                }
            )
    if not samples:
        raise RuntimeError("host-stall route window has no vmstat samples")
    peak = max(item["run_queue"] for item in samples)
    camera_maxima = {
        topic: event["receipt_period_sec"]["max"]
        for topic, event in latency["event_rates"].items()
        if "/sensing/camera/" in topic and topic.endswith("/camera_info")
    }
    if len(camera_maxima) != 6:
        raise RuntimeError("host-stall evidence does not contain six camera rates")
    return {
        "schema_id": "autoware-e2e.host-runtime-stall-summary.v1",
        "status": "EVIDENCE_VALID",
        "route_evaluation": {
            "started_at_utc": route_start_utc.isoformat(),
            "finished_at_utc": route_finish_utc.isoformat(),
            "vmstat_sample_count": len(samples),
            "run_queue_peak": peak,
            "run_queue_peak_timestamps_kst": [
                item["observed_at_kst"]
                for item in samples
                if item["run_queue"] == peak
            ],
            "vmstat_samples": samples,
        },
        "six_camera_receipt_period_maximum_sec": {
            "by_topic": camera_maxima,
            "minimum_across_cameras": min(camera_maxima.values()),
            "maximum_across_cameras": max(camera_maxima.values()),
        },
        "sources": {
            "desktop_capture": {
                "path": desktop_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "sha256": _sha256(desktop_path),
            },
            "e2e_latency": {
                "path": latency_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "sha256": _sha256(latency_path),
            },
            "vmstat": {
                "path": telemetry_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "sha256": _sha256(telemetry_path),
            },
        },
    }


def _readme() -> str:
    return """# 2026-09-07 control validation review set

이 폴더는 30 km/h strict six-camera 10 Hz A/B 결과, 과거 60 km/h readiness 근거,
Town06 strict 10 Hz 60 km/h 1차 실행 결과를 한곳에 모은 발행본이다. 원시 rosbag,
MKV, 실행 로그와 임시 파일은 포함하지 않았다.

- `00_summary`: 기계 판독 결과표, 파일 manifest와 SHA-256
- `01_town07_straight`: baseline과 longitudinal recovery 2.0 후보
- `02_c_track_turn`: 선택 baseline, 5 m 후보, 독립 safety screen과 비교
- `03_town03_turn`: 선택 baseline과 10 m 후보 비교
- `04_rejected_and_runtime_diagnostics`: C-track 10 m geometry fail과 host stall 근거
- `05_60kph_readiness`: 5 Hz historical pilot, strict 10 Hz live v1과 blocker 분석

차량 제어 주체는 모든 최신 30 km/h 실행에서 Autoware VAD였다. Portable E2E는
10 Hz shadow-only였고 actuator를 제어하지 않았다. 후보는 어느 것도 승격되지 않았으며
strict 10 Hz live v1은 Town06 경로를 완주했지만 최고 36.44 km/h였고, 전체 구간
카메라 p95 지연과 첫 녹화 경계의 1프레임 비대칭 때문에 최종 `NO_GO`다.

manifest의 source-relative 경로와 JSON 내부 `/home/a/...` 값은 실행 당시 원본
provenance를 보존한 기록이며 clone 후 실행 경로가 아니다. 발행본 검증은 상대 경로의 `SHA256SUMS`와
`publication_manifest.json`만 사용한다. rosbag, MKV, stack/runtime 로그는 제외했고,
host stall 판정에 필요한 route 구간 vmstat 표본만 파생 JSON으로 보존했다.
"""


def _transform_published_file(path: Path, destination: str) -> bool:
    if destination != "05_60kph_readiness/analysis/speed_limit_analysis.md":
        return False
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "- 기계 판독 보고서: [`town06_60kph_v3_speed_limit_analysis.json`](town06_60kph_v3_speed_limit_analysis.json)",
        "- 발행용 기계 판독 보고서: [`speed_limit_analysis.json`](speed_limit_analysis.json)",
    )
    text = text.replace(
        "- 원본 root: [`../30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001`](../30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001)",
        "- 발행한 historical pilot 근거: [`historical_pilot/evidence`](../historical_pilot/evidence/)",
    )
    text = text.replace(
        "## 다음 단계\n\n1. 먼저 기존 `--tight-corridor`만 켠 geometry 단일 A/B를 수행한다.",
        "## 당시 제안한 다음 단계 (후속 결과 반영)\n\n이 목록은 2026-09-02 분석 당시 제안이다. 1번 corridor A/B는 이후 실행되어 HOLD였고 [후속 결과](geometry_corridor_ab.md)를 함께 봐야 한다.\n\n1. 먼저 기존 `--tight-corridor`만 켠 geometry 단일 A/B를 수행한다.",
    )
    text = text.replace(
        "- 핵심 원본: `result.json`, `speed_profile.json`, `longitudinal_response.json`, `diagnosis.json`, `actuation_map_runtime_coverage.json`, `aligned_route.json`, `bag/bag_0.db3`, `stack.log`",
        "- 발행한 핵심 근거: `result.json`, `diagnosis.json`, `actuation_map_coverage.json`, `actuation_map_runtime_coverage.json`, `camera_source_5hz_validation.json`, `runtime_load_analysis.json`",
    )
    text = text.replace(
        "- 설정/코드 provenance: `actuation_config_provenance/`, `speed_profile_provenance/`, `trajectory_code_provenance/`, `controller.params.yaml`, `vad_route_manager.params.yaml`",
        "- 원시 rosbag, stack log와 대용량 설정 스냅샷은 이 경량 발행본에서 제외했으며 원본 SHA-256은 JSON 보고서에 보존했다.",
    )
    # HH_260906 - Keep generated Markdown free of invisible hard-break whitespace.
    trailing_newline = "\n" if text.endswith("\n") else ""
    text = "\n".join(line.rstrip() for line in text.splitlines()) + trailing_newline
    path.write_text(text, encoding="utf-8")
    return True


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _publish(output: Path, *, replace: bool = False) -> None:
    backup: Path | None = None
    if output.exists() or output.is_symlink():
        if not replace:
            raise RuntimeError(f"publication output already exists: {output}")
        if output.is_symlink() or not output.is_dir():
            raise RuntimeError(f"publication output is not a regular directory: {output}")
        existing = _read_json(output / "00_summary/publication_manifest.json")
        if existing.get("publication_id") != PUBLICATION_ID:
            raise RuntimeError("refusing to replace a different publication")
        backup = output.parent / f".{output.name}.previous"
        if backup.exists() or backup.is_symlink():
            raise RuntimeError(f"publication backup already exists: {backup}")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    records: list[dict[str, Any]] = []
    try:
        destinations: set[str] = set()
        for source, destination in _publication_specs():
            if destination in destinations:
                raise RuntimeError(f"duplicate publication destination: {destination}")
            destinations.add(destination)
            resolved = _regular_source(source)
            target = stage / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(resolved, target)
            source_sha256 = _sha256(resolved)
            transformed = _transform_published_file(target, destination)
            dimensions = _image_dimensions(target)
            if target.name == "01_vehicle_centered_fullscreen.png" and dimensions != [1920, 1080]:
                raise RuntimeError(
                    f"fullscreen must be exactly 1920x1080: {resolved}: {dimensions}"
                )
            records.append(
                {
                    "destination": destination,
                    "source": resolved.relative_to(REPOSITORY_ROOT).as_posix(),
                    "source_sha256": source_sha256,
                    "transformed": transformed,
                    "sha256": _sha256(target),
                    "size_bytes": target.stat().st_size,
                    "dimensions": dimensions,
                }
            )
        readme = stage / "README.md"
        readme.write_text(_readme(), encoding="utf-8")
        matrix = stage / "00_summary/result_matrix.json"
        _write_json(matrix, _result_matrix())
        host_stall = (
            stage
            / "04_rejected_and_runtime_diagnostics/host_stall/host_runtime_stall_summary.json"
        )
        _write_json(host_stall, _host_stall_summary())
        for derived, kind in (
            (readme, "readme"),
            (matrix, "result_matrix"),
            (host_stall, "host_runtime_stall_summary"),
        ):
            records.append(
                {
                    "destination": derived.relative_to(stage).as_posix(),
                    "source": None,
                    "derived_kind": kind,
                    "sha256": _sha256(derived),
                    "size_bytes": derived.stat().st_size,
                    "dimensions": None,
                }
            )
        records.sort(key=lambda item: item["destination"])
        checksums = stage / "00_summary/SHA256SUMS"
        checksums.write_text(
            "".join(
                f"{item['sha256']}  {item['destination']}\n" for item in records
            ),
            encoding="utf-8",
        )
        records.append(
            {
                "destination": "00_summary/SHA256SUMS",
                "source": None,
                "derived_kind": "checksums",
                "sha256": _sha256(checksums),
                "size_bytes": checksums.stat().st_size,
                "dimensions": None,
            }
        )
        records.sort(key=lambda item: item["destination"])
        _write_json(
            stage / "00_summary/publication_manifest.json",
            {
                "schema_id": "autoware-e2e.curated-publication-manifest.v1",
                "publication_id": PUBLICATION_ID,
                "managed_file_count": len(records),
                "files": records,
            },
        )
        if backup is not None:
            os.replace(output, backup)
        try:
            os.replace(stage, output)
        except Exception:
            if backup is not None and backup.exists() and not output.exists():
                os.replace(backup, output)
            raise
        if backup is not None:
            shutil.rmtree(backup)
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def _verify(output: Path) -> None:
    manifest_path = output / "00_summary/publication_manifest.json"
    manifest = _read_json(manifest_path)
    if manifest.get("schema_id") != "autoware-e2e.curated-publication-manifest.v1":
        raise RuntimeError("publication manifest schema mismatch")
    if manifest.get("publication_id") != PUBLICATION_ID:
        raise RuntimeError("publication identity mismatch")
    files = manifest.get("files")
    if not isinstance(files, list) or manifest.get("managed_file_count") != len(files):
        raise RuntimeError("publication manifest count mismatch")
    expected = {"00_summary/publication_manifest.json"}
    destinations: set[str] = set()
    content_hashes: set[str] = set()
    for record in files:
        if not isinstance(record, dict):
            raise RuntimeError("publication manifest record is not an object")
        relative = record.get("destination")
        if not isinstance(relative, str) or not relative:
            raise RuntimeError("publication manifest destination is invalid")
        if relative in destinations:
            raise RuntimeError(f"duplicate publication destination: {relative}")
        destinations.add(relative)
        candidate = Path(relative)
        if candidate.is_absolute() or ".." in candidate.parts:
            raise RuntimeError(f"unsafe publication path: {relative}")
        path = output / candidate
        if path.is_symlink() or not path.is_file():
            raise RuntimeError(f"missing published file: {relative}")
        digest = record.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise RuntimeError(f"published SHA-256 is invalid: {relative}")
        if digest in content_hashes:
            raise RuntimeError(f"duplicate published content SHA-256: {relative}")
        content_hashes.add(digest)
        if _sha256(path) != digest:
            raise RuntimeError(f"published SHA-256 mismatch: {relative}")
        if path.stat().st_size != record.get("size_bytes"):
            raise RuntimeError(f"published size mismatch: {relative}")
        dimensions = _image_dimensions(path)
        if dimensions != record.get("dimensions"):
            raise RuntimeError(f"published image dimensions mismatch: {relative}")
        if path.name == "01_vehicle_centered_fullscreen.png" and dimensions != [1920, 1080]:
            raise RuntimeError(f"published fullscreen dimensions mismatch: {relative}")
        source = record.get("source")
        if source is not None:
            source_path = Path(source)
            if source_path.is_absolute() or ".." in source_path.parts:
                raise RuntimeError(f"unsafe source provenance path: {relative}")
            source_digest = record.get("source_sha256")
            if not isinstance(source_digest, str) or len(source_digest) != 64:
                raise RuntimeError(f"source provenance SHA-256 is invalid: {relative}")
            if record.get("transformed") not in {True, False}:
                raise RuntimeError(f"source transform flag is invalid: {relative}")
        expected.add(relative)
    actual = {
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_file()
    }
    if actual != expected:
        raise RuntimeError(
            f"publication file set mismatch: extra={sorted(actual - expected)}, "
            f"missing={sorted(expected - actual)}"
        )
    matrix = _read_json(output / "00_summary/result_matrix.json")
    if matrix.get("publication_id") != PUBLICATION_ID:
        raise RuntimeError("result matrix publication identity mismatch")
    checksum_records = [
        record for record in files if record["destination"] != "00_summary/SHA256SUMS"
    ]
    expected_checksums = "".join(
        f"{record['sha256']}  {record['destination']}\n"
        for record in sorted(checksum_records, key=lambda item: item["destination"])
    )
    if (output / "00_summary/SHA256SUMS").read_text(encoding="utf-8") != expected_checksums:
        raise RuntimeError("publication SHA256SUMS content mismatch")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if args.verify_only:
        if args.replace:
            parser.error("--replace cannot be combined with --verify-only")
        _verify(output)
    else:
        _publish(output, replace=args.replace)
        _verify(output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
