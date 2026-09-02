from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts/e2e/summarize_runtime_control_campaign.py"
)
SPEC = importlib.util.spec_from_file_location(
    "summarize_runtime_control_campaign", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _trial_fixture(
    campaign: Path,
    spec: dict[str, object],
    *,
    role: str,
) -> tuple[Path, dict[str, object]]:
    directory_name = spec["baseline"] if role == "baseline" else spec["candidate"]
    owner_root = (
        campaign
        / "20_30kph_control_ab"
        / str(spec["id"])
        / str(directory_name)
    )
    trial = owner_root / "attempts/attempt_001"
    trial.mkdir(parents=True)
    route_scenario = tuple(spec["route_scenarios"])[0]
    _write_json(
        trial / "source_route.json",
        {"town": spec["town"], "scenario": route_scenario},
    )
    route_sha = hashlib.sha256((trial / "source_route.json").read_bytes()).hexdigest()
    _write_json(
        trial / "result.json",
        {
            "success": True,
            "reason": "goal reached",
            "metrics": {
                "sim_elapsed_sec": 40.0,
                "wall_elapsed_sec": 40.1,
                "maximum_observed_speed_mps": 7.8,
                "maximum_sustained_speed_duration_sec": 3.6,
                "maximum_absolute_cte_m": 0.5,
                "maximum_lateral_acceleration_mps2": 0.8,
                "maximum_trajectory_correction_m": 8.0,
            },
        },
    )
    health = {
        "schema_version": 1,
        "status": "PASS",
        "sequence": {
            "status": "PASS",
            "timed_out": False,
            "winning_window_indexes": [0, 1, 2],
        },
        "windows": [
            {
                "index": index,
                "status": "PASS",
                "failures": [],
                "clock": {"rtf": 0.99},
                "minimum_observed_camera_wall_rate_hz": 5.0,
                "bundles": {"receipt_span_seconds": {"p95": 0.01}},
            }
            for index in range(3)
        ],
    }
    _write_json(trial / "runtime_health.json", health)
    health_sha = hashlib.sha256(
        (trial / "runtime_health.json").read_bytes()
    ).hexdigest()
    _write_json(trial / "speed_profile.json", {"quality": {"problems": []}})
    _write_json(
        trial / "longitudinal_response.json",
        {
            "quality": {"problems": []},
            "tracking": {
                "gated_target_minus_actual_speed_mps": {"rmse": 1.8},
                "raw_minus_gated_target_speed_mps": {"rmse": 1.3},
            },
            "robust_measured_acceleration": {
                "robust_summary_mps2": {"p95": 1.0}
            },
            "saturation_and_duty": {
                "gated_positive_acceleration_limit": {
                    "time_fraction_percent": 40.0
                }
            },
        },
    )
    _write_json(
        trial / "diagnosis.json",
        {
            "metrics": {
                "tracking": {"actual_to_route_cte_m": {"p95_abs": 0.4}}
            }
        },
    )
    _write_json(
        trial / "latency/e2e_latency.json",
        {
            "camera_bundle": {
                "bundle_coverage_percent": 100.0,
                "receipt_span_sec": {"p95": 0.01},
            }
        },
    )
    _write_json(
        trial / "desktop_capture.json",
        {
            "schema_version": 1,
            "candidate_observed": True,
            "capture_started_after_candidate": True,
            "png_file": "autoware_rviz_fullscreen.png",
            "gif_file": "autoware_rviz_drive.gif",
            "capture_source": {
                "root_capture": False,
                "shell_surfaces_excluded": True,
            },
            "desktop_overlay_check": {"passed": True},
            "rviz_view_contract": {
                "vehicle_centered": True,
                "visible_path_topics": [
                    "/planning/trajectory",
                    "/planning/vad_route/actual_path",
                    "/planning/vad_route/reference_path",
                ],
            },
        },
    )
    candidate_id = "baseline" if role == "baseline" else str(spec["candidate_id"])
    (trial / "runtime.env").write_text(
        "\n".join(
            (
                "SPEED_PROFILE_ID=carla_vad_30kph_v2",
                f"CONTROL_AB_CANDIDATE_ID={candidate_id}",
                f"CONTROL_AB_PID_I40={'true' if candidate_id == 'pid_i40' else 'false'}",
                f"CONTROL_AB_TURN_PREVIEW_5M={'true' if candidate_id == 'turn_preview_5m' else 'false'}",
                "CONTROL_AB_ISOLATED_SINGLE_KNOB=true",
                "RUNTIME_HEALTH_GATE_ENABLED=true",
                "RUNTIME_HEALTH_GATE_STATUS=PASS",
                f"RUNTIME_HEALTH_EVIDENCE_SHA256={health_sha}",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    for _, filename, _ in module.TRIAL_VISUALS:
        (trial / filename).write_bytes(b"synthetic visual evidence\n")
    _write_json(
        owner_root / "owned_trial_summary.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "route_sha256": route_sha,
            "selected_attempt": "attempt_001",
            "attempts": [
                {
                    "attempt_id": "attempt_001",
                    "path": str(trial.resolve()),
                    "runtime_health_status": "PASS",
                    "process_exit_status": 0,
                }
            ],
        },
    )
    loaded = module._load_comparator_trial(owner_root, role)
    return owner_root, loaded


def _campaign_fixture(tmp_path: Path) -> Path:
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    (campaign / "30_60kph").mkdir()
    for spec in module.SCENARIOS:
        _, baseline = _trial_fixture(campaign, spec, role="baseline")
        _, candidate = _trial_fixture(campaign, spec, role="candidate")
        comparison_dir = (
            campaign / "20_30kph_control_ab" / spec["id"] / "comparison"
        )
        _write_json(
            comparison_dir / f"{spec['comparison']}.json",
            {
                "schema_version": 1,
                "scenario": spec["id"],
                "candidate_id": spec["candidate_id"],
                "baseline": baseline,
                "candidate": candidate,
                "checks": {
                    "synthetic_contract": {
                        "status": "PASS",
                        "actual": True,
                        "requirement": "synthetic fixture",
                    }
                },
                "decision": "ACCEPT",
                "real_vehicle_ready": False,
            },
        )
        (comparison_dir / f"{spec['comparison']}.png").write_bytes(
            b"synthetic comparison visual\n"
        )
    return campaign


def test_deterministic_summary_and_reference_only_manifest(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)

    first_summary, first_manifest = module.collect_campaign(campaign)
    second_summary, second_manifest = module.collect_campaign(campaign)

    assert first_summary == second_summary
    assert first_manifest == second_manifest
    assert first_summary["overall_control_decision"] == "ACCEPT"
    assert first_summary["60kph_pilot"]["status"] == "ABSENT"
    assert first_manifest["copy_policy"] == "reference_only_no_asset_duplication"
    assert first_manifest["asset_count"] == 33
    assert all(
        not Path(asset["campaign_relative_path"]).is_absolute()
        for asset in first_manifest["assets"]
    )


def test_comparison_drift_fails_closed(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)
    spec = module.SCENARIOS[0]
    comparison = (
        campaign
        / "20_30kph_control_ab"
        / spec["id"]
        / "comparison"
        / f"{spec['comparison']}.json"
    )
    payload = json.loads(comparison.read_text(encoding="utf-8"))
    payload["candidate"]["metrics"]["maximum_speed_mps"] = 123.0
    _write_json(comparison, payload)

    with pytest.raises(
        module.CampaignSummaryError, match="no longer matches source evidence"
    ):
        module.collect_campaign(campaign)


def test_in_progress_60kph_does_not_consume_partial_result(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)
    pilot = campaign / "30_60kph" / module.PILOT_DIRECTORY_V2
    (pilot / "trial/attempt_001").mkdir(parents=True)
    (pilot / "trial/attempt_001/result.json").write_text("{partial", encoding="utf-8")

    summary, _ = module.collect_campaign(campaign)

    assert summary["60kph_pilot"]["status"] == "IN_PROGRESS"
    assert "terminal" in summary["60kph_pilot"]["reason"]


def test_ambiguous_60kph_directories_fail_closed(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)
    (campaign / "30_60kph" / module.PILOT_DIRECTORY_V1).mkdir()
    (campaign / "30_60kph" / module.PILOT_DIRECTORY_V2).mkdir()

    with pytest.raises(module.CampaignSummaryError, match="ambiguous"):
        module.collect_campaign(campaign)


def test_explicit_v2_selection_resolves_two_active_pilots(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)
    (campaign / "30_60kph" / module.PILOT_DIRECTORY_V1).mkdir()
    (campaign / "30_60kph" / module.PILOT_DIRECTORY_V2).mkdir()

    summary, _ = module.collect_campaign(
        campaign, pilot_directory=module.PILOT_DIRECTORY_V2
    )

    pilot = summary["60kph_pilot"]
    assert pilot["status"] == "IN_PROGRESS"
    assert pilot["selection"] == {
        "mode": "explicit_argument",
        "selected_directory": module.PILOT_DIRECTORY_V2,
        "other_directory_count": 1,
    }
    assert pilot["pilot_directory"].endswith(module.PILOT_DIRECTORY_V2)


def test_duplicate_json_keys_fail_closed(tmp_path: Path) -> None:
    campaign = _campaign_fixture(tmp_path)
    spec = module.SCENARIOS[0]
    owner = (
        campaign
        / "20_30kph_control_ab"
        / spec["id"]
        / spec["baseline"]
        / "owned_trial_summary.json"
    )
    owner.write_text('{"schema_version": 1, "schema_version": 1}\n', encoding="utf-8")

    with pytest.raises(module.CampaignSummaryError, match="duplicate JSON key"):
        module.collect_campaign(campaign)
