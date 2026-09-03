from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts/e2e/curate_runtime_control_campaign_assets.py"
)
SPEC = importlib.util.spec_from_file_location(
    "curate_runtime_control_campaign_assets", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


@pytest.fixture(autouse=True)
def _isolate_publication_locks(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Keep per-test lock identities out of the workstation-wide /tmp pool."""
    lock_tmp = tmp_path / "publication-lock-tmp"
    lock_tmp.mkdir()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(lock_tmp))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_digest(payload: object) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_media(path: Path, mime_type: str, label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    prefix = b"\x89PNG\r\n\x1a\n" if mime_type == "image/png" else b"GIF89a"
    path.write_bytes(prefix + label.encode("utf-8") + b"\n")


def _reference(campaign: Path, path: Path) -> dict[str, object]:
    return {
        "campaign_relative_path": path.relative_to(campaign).as_posix(),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
    }


def _desktop(campaign: Path, trial: Path) -> dict[str, object]:
    path = trial / "desktop_capture.json"
    _write_json(
        path,
        {
            "schema_version": 1,
            "candidate_observed": True,
            "capture_started_after_candidate": True,
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
    return _reference(campaign, path)


def _trial(
    campaign: Path,
    scenario: str,
    variant: str,
    owner: str,
    manifest_assets: list[dict[str, object]],
    *,
    speed_class: str = "30kph",
    pilot: bool = False,
) -> dict[str, object]:
    if pilot:
        trial = campaign / "30_60kph" / owner / "trial/attempt_001"
    else:
        trial = (
            campaign
            / "20_30kph_control_ab"
            / scenario
            / owner
            / "attempts/attempt_001"
        )
    trial.mkdir(parents=True, exist_ok=True)
    definitions = (
        ("fullscreen", "autoware_rviz_fullscreen.png", "image/png"),
        ("drive", "autoware_rviz_drive.gif", "image/gif"),
        ("path_control", "path_vs_control.png", "image/png"),
    )
    for kind, filename, mime_type in definitions:
        path = trial / filename
        _write_media(path, mime_type, f"{speed_class}-{scenario}-{variant}-{kind}")
        manifest_assets.append(
            {
                **_reference(campaign, path),
                "id": f"{speed_class}.{scenario}.{variant}.{kind}",
                "speed_class": speed_class,
                "scenario": scenario,
                "variant": variant,
                "kind": kind,
                "mime_type": mime_type,
            }
        )
    _write_media(
        trial / "steering_tracking.png",
        "image/png",
        f"{speed_class}-{scenario}-{variant}-steering",
    )
    _write_media(
        trial / "speed_profile.png",
        "image/png",
        f"{speed_class}-{scenario}-{variant}-speed",
    )
    return {
        "candidate_id": "baseline" if variant == "baseline" else "test_candidate",
        "evidence": {"desktop_capture.json": _desktop(campaign, trial)},
    }


def _selected_regression(campaign: Path, scenario: str, map_name: str) -> None:
    owner = (
        campaign
        / "20_30kph_control_ab"
        / scenario
        / module.SELECTED_30KPH_REGRESSION_DIRECTORY
    )
    trial = owner / "attempts" / module.SELECTED_30KPH_REGRESSION_ATTEMPT
    trial.mkdir(parents=True, exist_ok=True)
    for filename, mime_type in (
        ("autoware_rviz_fullscreen.png", "image/png"),
        ("autoware_rviz_drive.gif", "image/gif"),
        ("path_vs_control.png", "image/png"),
        ("steering_tracking.png", "image/png"),
        ("speed_profile.png", "image/png"),
    ):
        _write_media(
            trial / filename,
            mime_type,
            f"30kph-{scenario}-selected-regression-{filename}",
        )
    _desktop(campaign, trial)
    _write_json(trial / "result.json", {"schema_version": 1, "success": True})
    _write_json(
        trial / "runtime_health.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "contract": {
                "camera_transport": {
                    "profile_id": module.SELECTED_30KPH_TRANSPORT_PROFILE,
                    "camera_image_endpoint_depth": 1,
                    "camera_image_publisher_reliability": "best_effort",
                    "vad_image_subscription_reliability": "best_effort",
                    "rviz_image_subscription_reliability": "best_effort",
                    "cyclonedds_loopback_interface_required": True,
                    "ros_localhost_only_expected": "0",
                }
            },
            "camera_image_graph": {"status": "PASS"},
        },
    )
    _write_json(
        owner / "owned_trial_summary.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "map": map_name,
            "selected_attempt": module.SELECTED_30KPH_REGRESSION_ATTEMPT,
            "attempts": [
                {
                    "attempt_id": module.SELECTED_30KPH_REGRESSION_ATTEMPT,
                    "path": str(trial),
                    "process_exit_status": 0,
                    "runtime_health_status": "PASS",
                }
            ],
            "trial_options": [
                "--speed-30kph",
                "--camera-source-5hz",
                "--visualize",
                "--capture-desktop",
            ],
        },
    )


def _geometry_60kph_candidate(campaign: Path) -> Path:
    owner = campaign / "30_60kph" / module.GEOMETRY_60KPH_DIRECTORY
    trial = campaign / module.GEOMETRY_60KPH_TRIAL_RELATIVE_PATH
    trial.mkdir(parents=True, exist_ok=True)

    source_payload = {
        "schema_version": 1,
        "town": "Town06",
        "scenario": "straight",
        "route": [{"x": 0.0, "y": 0.0}, {"x": 445.0, "y": 0.0}],
    }
    catalog_source = owner / "catalog/routes/town06/straight/route.json"
    copied_source = trial / "source_route.json"
    _write_json(catalog_source, source_payload)
    _write_json(copied_source, source_payload)
    source_sha = _sha256(copied_source)

    aligned_path = trial / "aligned_route.json"
    _write_json(
        aligned_path,
        {
            "schema_version": 1,
            "town": "Town06",
            "scenario": "straight",
            "route_length_m": 445.0,
            "route": source_payload["route"],
            "coordinate_alignment": {
                "source_route": str(catalog_source),
                "source_route_sha256": source_sha,
            },
        },
    )
    aligned_sha = _sha256(aligned_path)
    _write_json(
        trial / "route_alignment.json",
        {
            "status": "PASS",
            "source_route": str(catalog_source),
            "source_route_sha256": source_sha,
            "aligned_route": str(aligned_path),
            "aligned_route_sha256": aligned_sha,
        },
    )
    result_path = trial / "result.json"
    _write_json(
        result_path,
        {
            "schema_version": 1,
            "route_file": str(aligned_path),
            "success": False,
            "speed_exposure": {"status": "FAIL"},
        },
    )
    _write_json(
        trial / "runtime_health.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "camera_image_graph": {"status": "PASS"},
        },
    )
    bag = trial / "bag"
    bag.mkdir()
    (bag / "bag_0.db3").write_bytes(b"fixture sqlite bag\n")
    (bag / "metadata.yaml").write_text("fixture: true\n", encoding="utf-8")
    bag_files = [
        {
            "path": path.name,
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(bag.iterdir())
    ]
    bag_manifest = {
        "schema_version": 1,
        "root": str(bag),
        "files": bag_files,
        "sha256": _canonical_digest(
            {"schema_version": 1, "files": bag_files}
        ),
    }
    source_identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(aligned_path),
            "sha256": aligned_sha,
            "town": "Town06",
            "scenario": "straight",
        },
        "route_result": {
            "path": str(result_path),
            "sha256": _sha256(result_path),
            "success": False,
        },
        "rosbag": bag_manifest,
    }
    source_identity["sha256"] = _canonical_digest(source_identity)
    _write_json(
        trial / "speed_profile.json",
        {
            "schema_version": 1,
            "inputs": {
                "bag": str(bag),
                "profile_id": "carla_vad_60kph_straight_pilot_v1",
            },
            "source_identity": source_identity,
        },
    )
    _write_json(
        trial / "diagnosis.json",
        {
            "schema_version": 1,
            "inputs": {
                "bag": str(bag),
                "route_file": str(aligned_path),
                "town": "Town06",
                "scenario": "straight",
            },
        },
    )
    _write_json(trial / "camera_source_5hz_validation.json", {"status": "PASS"})
    _write_json(trial / "latency/e2e_latency.json", {"status": "PASS"})
    provenance_path = trial / "trajectory_code_provenance/vad_route_manager.py"
    provenance_path.parent.mkdir(parents=True)
    provenance_path.write_text("# fixture provenance\n", encoding="utf-8")
    provenance = {
        "trajectory_code_provenance/vad_route_manager.py": _sha256(
            provenance_path
        )
    }

    for filename, mime_type in (
        ("autoware_rviz_fullscreen.png", "image/png"),
        ("autoware_rviz_drive.gif", "image/gif"),
        ("path_vs_control.png", "image/png"),
        ("steering_tracking.png", "image/png"),
        ("speed_profile.png", "image/png"),
        ("route_result.png", "image/png"),
        ("runtime_load_analysis.png", "image/png"),
        ("longitudinal_response.png", "image/png"),
    ):
        _write_media(trial / filename, mime_type, f"geometry-v4-{filename}")
    _desktop(campaign, trial)

    evidence = {
        relative: _sha256(trial / relative)
        for relative in module.GEOMETRY_60KPH_EVIDENCE_FILES
    }
    geometry_identity = "c" * 64
    report = {
        "schema_version": 1,
        "analysis": "town06_60kph_geometry_only_corridor_ab",
        "baseline": {
            "trial_directory": str(
                campaign
                / "30_60kph"
                / module.GEOMETRY_60KPH_BASELINE_DIRECTORY
                / "trial/attempt_001"
            )
        },
        "candidate": {
            "role": "candidate",
            "trial_directory": str(trial),
            "evidence_sha256": evidence,
            "provenance_sha256": provenance,
            "bag_manifest": bag_manifest,
            "geometry_metadata": {
                "candidate_id": "route_corridor_0p2",
                "route_corridor_0p2": "true",
            },
            "health": {
                "status": "PASS",
                "runtime_health_status": "PASS",
                "camera_validation_status": "PASS",
            },
            "outcomes": {
                "goal_reached": True,
                "route_result_success": False,
                "speed_exposure_status": "FAIL",
            },
            "environment": {
                "GEOMETRY_AB_CANDIDATE_ID": "route_corridor_0p2",
                "GEOMETRY_AB_ROUTE_CORRIDOR_0P2": "true",
                "REAL_VEHICLE_READY": "false",
            },
            "route": {
                "town": "Town06",
                "scenario": "straight",
                "source_route_sha256": source_sha,
                "aligned_route_file_sha256": aligned_sha,
                "canonical_geometry_identity_sha256": geometry_identity,
            },
        },
        "pair_contract": {
            "status": "PASS",
            "effective_route_identity": {
                "source_route_sha256": source_sha,
                "candidate_aligned_file_sha256": aligned_sha,
                "canonical_geometry_identity_sha256": geometry_identity,
            },
            "fixed_provenance_sha256": provenance,
        },
        "geometry_outcome": {
            "decision": "HOLD",
            "diagnostic_effect": "PARTIAL_IMPROVEMENT_INSUFFICIENT",
        },
        "speed_contract": {
            "decision": "FAIL",
            "independent_from_geometry_acceptance": True,
        },
        "real_vehicle_ready": False,
    }
    _write_json(campaign / module.GEOMETRY_60KPH_REPORT_JSON_RELATIVE_PATH, report)
    _write_media(
        campaign / module.GEOMETRY_60KPH_REPORT_PNG_RELATIVE_PATH,
        "image/png",
        "geometry-v4-comparison",
    )
    decoy = campaign / "30_60kph/town06_straight_60kph_geometry_v999"
    decoy.mkdir()
    (decoy / "DO_NOT_COPY.png").write_bytes(b"not selected\n")
    return trial


def _build_campaign(
    tmp_path: Path,
    *,
    include_60kph: bool = False,
    include_60kph_geometry_ab: bool = False,
    sixty_selection_mode: str = "explicit_argument",
) -> Path:
    campaign = tmp_path / "campaign_v1"
    campaign.mkdir(parents=True)
    manifest_assets: list[dict[str, object]] = []
    scenarios: list[dict[str, object]] = []
    owners = {
        "town07_straight": ("A_baseline", "B_pid_i40"),
        "c_track_turn": ("A_baseline", "B_turn_preview"),
        "town03_turn": ("A_baseline", "B_turn_preview"),
    }
    for scenario in module.SCENARIO_ORDER:
        baseline_owner, candidate_owner = owners[scenario]
        baseline = _trial(
            campaign,
            scenario,
            "baseline",
            baseline_owner,
            manifest_assets,
        )
        candidate = _trial(
            campaign,
            scenario,
            "candidate",
            candidate_owner,
            manifest_assets,
        )
        comparison_path = (
            campaign
            / "20_30kph_control_ab"
            / scenario
            / "comparison/A_vs_B_decision.png"
        )
        _write_media(comparison_path, "image/png", f"{scenario}-comparison")
        manifest_assets.append(
            {
                **_reference(campaign, comparison_path),
                "id": f"30kph.{scenario}.comparison",
                "speed_class": "30kph",
                "scenario": scenario,
                "variant": "comparison",
                "kind": "comparison",
                "mime_type": "image/png",
            }
        )
        scenarios.append(
            {
                "scenario": scenario,
                "baseline": baseline,
                "candidate": candidate,
                "comparison": {"decision": "HOLD"},
            }
        )
        _selected_regression(campaign, scenario, module.SCENARIO_MAPS[scenario])

    top_evidence: list[dict[str, object]] = []
    if include_60kph:
        pilot_name = (
            module.GEOMETRY_60KPH_BASELINE_DIRECTORY
            if include_60kph_geometry_ab
            else "town06_straight_60kph_pilot_explicit_v3"
        )
        pilot = _trial(
            campaign,
            "town06_straight",
            "pilot",
            pilot_name,
            manifest_assets,
            speed_class="60kph",
            pilot=True,
        )
        top_evidence.append(
            {
                **pilot["evidence"]["desktop_capture.json"],
                "kind": "desktop_capture.json",
                "scenario": "town06_straight_60kph",
            }
        )
        sixty: dict[str, object] = {
            "status": "COMPLETE_PASS",
            "pilot_directory": f"30_60kph/{pilot_name}",
            "selection": {
                "mode": sixty_selection_mode,
                "selected_directory": pilot_name,
                "other_directory_count": 1,
            },
            "result": {"present": True, "success": True},
        }
        # A decoy proves the curator never scans or chooses a newer-looking run.
        decoy = campaign / "30_60kph/town06_straight_60kph_pilot_v999"
        decoy.mkdir(parents=True)
        (decoy / "DO_NOT_COPY.png").write_bytes(b"not selected\n")
        if include_60kph_geometry_ab:
            _geometry_60kph_candidate(campaign)
    else:
        (campaign / "30_60kph").mkdir()
        sixty = {"status": "ABSENT", "reason": "fixture has no pilot"}

    snapshot = "a" * 64
    visual_manifest = {
        "schema_version": 1,
        "kind": "runtime_control_campaign_visual_manifest",
        "campaign_id": campaign.name,
        "copy_policy": "reference_only_no_asset_duplication",
        "source_snapshot_sha256": snapshot,
        "asset_count": len(manifest_assets),
        "assets": manifest_assets,
    }
    summary = {
        "schema_version": 1,
        "kind": "autoware_vad_runtime_control_campaign_summary",
        "campaign_id": campaign.name,
        "source_snapshot_sha256": snapshot,
        "campaign_status": "COMPLETE_FIXTURE",
        "overall_control_decision": "HOLD",
        "visual_manifest": "40_visuals/visual_manifest.json",
        "30kph_control_ab": {
            "status": "COMPLETE",
            "scenario_count": 3,
            "scenarios": scenarios,
        },
        "60kph_pilot": sixty,
        "evidence": top_evidence,
    }
    _write_json(campaign / "40_visuals/visual_manifest.json", visual_manifest)
    _write_json(
        campaign / "50_reports/runtime_control_campaign_summary.json", summary
    )
    return campaign


def _load_manifest(root: Path) -> dict[str, object]:
    return json.loads((root / "publication_manifest.json").read_text(encoding="utf-8"))


def test_curates_30kph_and_docs_mirror_idempotently(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path)
    docs = tmp_path / "docs-assets"

    first = module.curate(campaign, docs_root=docs)

    assert [row["status"] for row in first] == ["UPDATED", "UPDATED"]
    campaign_output = campaign / "40_visuals"
    for output in (campaign_output, docs):
        manifest = _load_manifest(output)
        assert manifest["asset_count"] == 48
        assert len(manifest["assets"]) == 48
        assert manifest["manifest_payload_sha256"] == module._manifest_signature(
            manifest
        )
        assert (output / "README.md").is_file()
        assert (output / "SHA256SUMS").is_file()
        assert (
            output
            / "30kph/town07_straight/A_baseline/01_autoware_vehicle_centered_fullscreen.png"
        ).is_file()
        assert (
            output
            / "30kph/town07_straight/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png"
        ).is_file()
        assert len(manifest["selected_30kph_regressions"]) == 3
        assert {
            row["scenario"] for row in manifest["selected_30kph_regressions"]
        } == set(module.SCENARIO_ORDER)
        assert all(
            Path(asset["published_relative_path"]).suffix.lower()
            in {".png", ".gif"}
            for asset in manifest["assets"]
        )
        for asset in manifest["assets"]:
            published = output / asset["published_relative_path"]
            assert _sha256(published) == asset["published_sha256"]

    tracked = campaign_output / "30kph/town03_turn/B_candidate/05_speed_profile.png"
    inode_before = tracked.stat().st_ino
    second = module.curate(campaign, docs_root=docs)
    assert [row["status"] for row in second] == ["UNCHANGED", "UNCHANGED"]
    assert tracked.stat().st_ino == inode_before


def test_missing_and_reference_hash_drift_fail_before_publication(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(tmp_path)
    missing = next(
        campaign.glob(
            "20_30kph_control_ab/town07_straight/A_baseline/attempts/*/steering_tracking.png"
        )
    )
    missing.unlink()
    with pytest.raises(module.CurationError, match="missing .*Steering tracking"):
        module.curate(campaign)
    assert not (campaign / "40_visuals/publication_manifest.json").exists()

    campaign = _build_campaign(tmp_path / "second")
    manifest_path = campaign / "40_visuals/visual_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    source = campaign / manifest["assets"][0]["campaign_relative_path"]
    payload = bytearray(source.read_bytes())
    payload[-1] ^= 1
    source.write_bytes(payload)
    with pytest.raises(module.CurationError, match="hash drift"):
        module.curate(campaign)
    assert not (campaign / "40_visuals/publication_manifest.json").exists()


def test_centered_capture_contract_is_enforced(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path)
    desktop = next(
        campaign.glob(
            "20_30kph_control_ab/c_track_turn/A_baseline/attempts/*/desktop_capture.json"
        )
    )
    payload = json.loads(desktop.read_text(encoding="utf-8"))
    payload["rviz_view_contract"]["vehicle_centered"] = False
    _write_json(desktop, payload)
    summary_path = campaign / "50_reports/runtime_control_campaign_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    row = next(
        row
        for row in summary["30kph_control_ab"]["scenarios"]
        if row["scenario"] == "c_track_turn"
    )
    row["baseline"]["evidence"]["desktop_capture.json"] = _reference(
        campaign, desktop
    )
    _write_json(summary_path, summary)

    with pytest.raises(module.CurationError, match="vehicle-centered"):
        module.curate(campaign)


def test_selected_regression_is_exact_and_depth1_health_is_enforced(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(tmp_path)
    decoy = (
        campaign
        / "20_30kph_control_ab/town07_straight"
        / "C_selected_baseline_depth1_loopback_regression_999"
    )
    decoy.mkdir()
    (decoy / "DO_NOT_COPY.png").write_bytes(b"not selected\n")

    result = module.curate(campaign)

    assert result[0]["asset_count"] == 48
    manifest = _load_manifest(campaign / "40_visuals")
    selected = [
        asset
        for asset in manifest["assets"]
        if asset["variant"] == module.SELECTED_30KPH_REGRESSION_VARIANT
    ]
    assert len(selected) == 15
    assert {asset["kind"] for asset in selected} == {
        "fullscreen",
        "drive",
        "path_vs_control",
        "steering_tracking",
        "speed_profile",
    }
    assert all(
        module.SELECTED_30KPH_REGRESSION_DIRECTORY
        in asset["source_campaign_relative_path"]
        for asset in selected
    )
    assert not any(
        path.name == "DO_NOT_COPY.png"
        for path in campaign.glob("40_visuals/**/*")
    )

    campaign = _build_campaign(tmp_path / "bad-health")
    health_path = (
        campaign
        / "20_30kph_control_ab/c_track_turn"
        / module.SELECTED_30KPH_REGRESSION_DIRECTORY
        / "attempts/attempt_001/runtime_health.json"
    )
    health = json.loads(health_path.read_text(encoding="utf-8"))
    health["contract"]["camera_transport"]["camera_image_endpoint_depth"] = 10
    _write_json(health_path, health)
    with pytest.raises(module.CurationError, match="depth-1 loopback health"):
        module.curate(campaign)
    assert not (campaign / "40_visuals/publication_manifest.json").exists()


def test_completed_60kph_rejects_selection_that_contradicts_exact_path(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(
        tmp_path, include_60kph=True, sixty_selection_mode="sole_active_directory"
    )
    summary_path = campaign / "50_reports/runtime_control_campaign_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["60kph_pilot"]["selection"]["selected_directory"] = "newest_v999"
    _write_json(summary_path, summary)

    with pytest.raises(module.CurationError, match="contradicts pilot_directory"):
        module.curate(campaign)


def test_explicit_60kph_adds_only_selected_pilot_assets(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path, include_60kph=True)

    result = module.curate(campaign)

    assert result[0]["asset_count"] == 53
    output = campaign / "40_visuals"
    manifest = _load_manifest(output)
    sixty_assets = [
        asset for asset in manifest["assets"] if asset["speed_class"] == "60kph"
    ]
    assert len(sixty_assets) == 5
    assert {
        asset["kind"] for asset in sixty_assets
    } == {"fullscreen", "drive", "path_vs_control", "steering_tracking", "speed_profile"}
    assert all(
        "town06_straight_60kph_pilot_explicit_v3"
        in asset["source_campaign_relative_path"]
        for asset in sixty_assets
    )
    assert not any(path.name == "DO_NOT_COPY.png" for path in output.rglob("*"))


def test_geometry_60kph_v4_is_published_without_replacing_selected_v3(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(
        tmp_path,
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    docs = tmp_path / "docs-assets"

    first = module.curate(campaign, docs_root=docs)

    assert [row["asset_count"] for row in first] == [62, 62]
    for output in (campaign / "40_visuals", docs):
        manifest = _load_manifest(output)
        assert manifest["asset_count"] == 62
        assert manifest["selected_60kph_pilot"][
            "campaign_relative_directory"
        ] == f"30_60kph/{module.GEOMETRY_60KPH_BASELINE_DIRECTORY}"
        geometry = manifest["60kph_geometry_ab"]
        assert geometry["status"] == "HOLD"
        assert geometry["speed_contract"] == "FAIL"
        assert geometry["real_vehicle_ready"] is False
        candidate = [
            asset
            for asset in manifest["assets"]
            if asset["variant"] == module.GEOMETRY_60KPH_VARIANT
        ]
        assert len(candidate) == 8
        assert {asset["kind"] for asset in candidate} == {
            "fullscreen",
            "drive",
            "path_vs_control",
            "steering_tracking",
            "speed_profile",
            "route_result",
            "runtime_load",
            "longitudinal_response",
        }
        comparison = [
            asset
            for asset in manifest["assets"]
            if asset["variant"] == "geometry_comparison"
        ]
        assert len(comparison) == 1
        assert comparison[0]["source_campaign_relative_path"] == (
            module.GEOMETRY_60KPH_REPORT_PNG_RELATIVE_PATH.as_posix()
        )
        assert all(
            Path(asset["source_campaign_relative_path"]).suffix.lower()
            not in module.BANNED_SOURCE_SUFFIXES
            for asset in manifest["assets"]
        )
        assert (
            output
            / module.GEOMETRY_60KPH_DESTINATION
            / "01_autoware_vehicle_centered_fullscreen.png"
        ).is_file()
        assert (
            output
            / module.GEOMETRY_60KPH_COMPARISON_DESTINATION
            / "09_A_selected_pilot_v3_vs_B_geometry_corridor_0p2_HOLD.png"
        ).is_file()
        readme = (output / "README.md").read_text(encoding="utf-8")
        assert "does not replace the selected pilot v3" in readme
        assert "Independent speed contract: `FAIL`" in readme
        assert "Real-vehicle ready: `false`" in readme
        assert not any(path.name == "DO_NOT_COPY.png" for path in output.rglob("*"))

    second = module.curate(campaign, docs_root=docs)
    assert [row["status"] for row in second] == ["UNCHANGED", "UNCHANGED"]


@pytest.mark.parametrize(
    ("field", "bad_value"),
    (
        ("pair_contract", "FAIL"),
        ("geometry_outcome", "ACCEPT"),
        ("speed_contract", "PASS"),
        ("real_vehicle_ready", True),
    ),
)
def test_geometry_60kph_decision_contract_fails_closed(
    tmp_path: Path, field: str, bad_value: object
) -> None:
    campaign = _build_campaign(
        tmp_path,
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    report_path = campaign / module.GEOMETRY_60KPH_REPORT_JSON_RELATIVE_PATH
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if field == "pair_contract":
        report[field]["status"] = bad_value
    elif field in {"geometry_outcome", "speed_contract"}:
        report[field]["decision"] = bad_value
    else:
        report[field] = bad_value
    _write_json(report_path, report)

    with pytest.raises(module.CurationError, match="decision contract"):
        module.curate(campaign)
    assert not (campaign / "40_visuals/publication_manifest.json").exists()


def test_geometry_60kph_candidate_sha_and_centered_capture_fail_closed(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(
        tmp_path / "sha-drift",
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    speed = campaign / module.GEOMETRY_60KPH_TRIAL_RELATIVE_PATH / "speed_profile.json"
    speed.write_bytes(speed.read_bytes() + b"drift\n")
    with pytest.raises(module.CurationError, match="evidence hash drift"):
        module.curate(campaign)

    campaign = _build_campaign(
        tmp_path / "not-centered",
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    desktop_path = (
        campaign
        / module.GEOMETRY_60KPH_TRIAL_RELATIVE_PATH
        / "desktop_capture.json"
    )
    desktop = json.loads(desktop_path.read_text(encoding="utf-8"))
    desktop["rviz_view_contract"]["vehicle_centered"] = False
    _write_json(desktop_path, desktop)
    with pytest.raises(module.CurationError, match="vehicle-centered"):
        module.curate(campaign)


def test_geometry_60kph_partial_exact_evidence_fails_closed(tmp_path: Path) -> None:
    campaign = _build_campaign(
        tmp_path,
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    (campaign / module.GEOMETRY_60KPH_REPORT_PNG_RELATIVE_PATH).unlink()

    with pytest.raises(module.CurationError, match="exact v4 evidence set is incomplete"):
        module.curate(campaign)


def test_geometry_route_alignment_rejects_external_source(tmp_path: Path) -> None:
    campaign = _build_campaign(
        tmp_path,
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    trial = campaign / module.GEOMETRY_60KPH_TRIAL_RELATIVE_PATH
    alignment_path = trial / "route_alignment.json"
    alignment = json.loads(alignment_path.read_text(encoding="utf-8"))
    external_source = tmp_path / "outside-campaign-route.json"
    external_source.write_bytes(Path(alignment["source_route"]).read_bytes())
    alignment["source_route"] = str(external_source)
    _write_json(alignment_path, alignment)

    if "route_alignment.json" in module.GEOMETRY_60KPH_EVIDENCE_FILES:
        report_path = campaign / module.GEOMETRY_60KPH_REPORT_JSON_RELATIVE_PATH
        report = json.loads(report_path.read_text(encoding="utf-8"))
        report["candidate"]["evidence_sha256"]["route_alignment.json"] = _sha256(
            alignment_path
        )
        _write_json(report_path, report)

    with pytest.raises(module.CurationError, match="canonical catalog source"):
        module.curate(campaign)
    assert not (campaign / "40_visuals/publication_manifest.json").exists()


def test_geometry_route_alignment_mutation_while_staged_fails_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    campaign = _build_campaign(
        tmp_path,
        include_60kph=True,
        include_60kph_geometry_ab=True,
    )
    alignment_path = (
        campaign
        / module.GEOMETRY_60KPH_TRIAL_RELATIVE_PATH
        / "route_alignment.json"
    )
    original_stage = module._stage_publication
    mutated = False

    def stage_then_mutate(*args: object, **kwargs: object) -> object:
        nonlocal mutated
        staged = original_stage(*args, **kwargs)
        if not mutated:
            alignment_path.write_bytes(alignment_path.read_bytes() + b"\n")
            mutated = True
        return staged

    monkeypatch.setattr(module, "_stage_publication", stage_then_mutate)

    with pytest.raises(module.CurationError, match="geometry evidence changed while staged"):
        module.curate(campaign)
    assert mutated is True
    assert not (campaign / "40_visuals/publication_manifest.json").exists()


def test_prior_output_drift_fails_and_unmanaged_file_is_preserved(
    tmp_path: Path,
) -> None:
    campaign = _build_campaign(tmp_path)
    source = next(
        campaign.glob(
            "20_30kph_control_ab/town07_straight/A_baseline/attempts/*/speed_profile.png"
        )
    )
    source_digest = _sha256(source)
    module.curate(campaign)
    output = campaign / "40_visuals"
    unmanaged = output / "notes_from_reviewer.txt"
    unmanaged.write_text("keep me\n", encoding="utf-8")

    assert module.curate(campaign)[0]["status"] == "UNCHANGED"
    assert unmanaged.read_text(encoding="utf-8") == "keep me\n"

    managed = output / "30kph/town07_straight/A_baseline/05_speed_profile.png"
    managed.write_bytes(managed.read_bytes() + b"tamper")
    with pytest.raises(module.CurationError, match="prior managed output drift"):
        module.curate(campaign)
    assert unmanaged.read_text(encoding="utf-8") == "keep me\n"
    assert _sha256(source) == source_digest


def test_prior_manifest_signature_tamper_fails_closed(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path)
    module.curate(campaign)
    manifest_path = campaign / "40_visuals/publication_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["managed_paths"].append("30kph/unowned.png")
    _write_json(manifest_path, manifest)

    with pytest.raises(module.CurationError, match="signature mismatch"):
        module.curate(campaign)


def test_resigned_prior_manifest_cannot_claim_unmanaged_file(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path)
    module.curate(campaign)
    output = campaign / "40_visuals"
    unmanaged = output / "60kph/reviewer_unmanaged.txt"
    unmanaged.parent.mkdir(parents=True, exist_ok=True)
    unmanaged.write_text("must survive\n", encoding="utf-8")

    manifest_path = output / "publication_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    relative = unmanaged.relative_to(output).as_posix()
    manifest["managed_paths"].append(relative)
    manifest["managed_paths"].sort()
    manifest["managed_files"].append(
        {
            "relative_path": relative,
            "role": "visual_asset",
            "sha256": _sha256(unmanaged),
            "size_bytes": unmanaged.stat().st_size,
        }
    )
    manifest["managed_files"].sort(key=lambda row: row["relative_path"])
    manifest["manifest_payload_sha256"] = module._manifest_signature(manifest)
    _write_json(manifest_path, manifest)

    with pytest.raises(module.CurationError, match="asset-derived ownership"):
        module.curate(campaign)
    assert unmanaged.read_text(encoding="utf-8") == "must survive\n"


def test_cleanup_removes_only_previously_managed_stale_files(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path, include_60kph=True)
    module.curate(campaign)
    output = campaign / "40_visuals"
    stale = (
        output
        / "60kph/town06_straight/selected_pilot/02_autoware_drive.gif"
    )
    assert stale.is_file()
    unmanaged = output / "60kph/reviewer_note.txt"
    unmanaged.write_text("preserve this unmanaged file\n", encoding="utf-8")

    reference_path = campaign / "40_visuals/visual_manifest.json"
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    reference["assets"] = [
        asset
        for asset in reference["assets"]
        if asset["speed_class"] != "60kph"
    ]
    reference["asset_count"] = len(reference["assets"])
    reference["source_snapshot_sha256"] = "b" * 64
    _write_json(reference_path, reference)

    summary_path = campaign / "50_reports/runtime_control_campaign_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["source_snapshot_sha256"] = "b" * 64
    summary["60kph_pilot"] = {"status": "ABSENT", "reason": "superseded fixture"}
    summary["evidence"] = []
    _write_json(summary_path, summary)

    result = module.curate(campaign)

    assert result[0]["status"] == "UPDATED"
    assert result[0]["asset_count"] == 48
    assert not stale.exists()
    assert unmanaged.read_text(encoding="utf-8") == "preserve this unmanaged file\n"
    assert (
        campaign
        / "30_60kph/town06_straight_60kph_pilot_explicit_v3/trial/attempt_001/autoware_rviz_drive.gif"
    ).is_file()


def test_cleanup_parent_symlink_swap_cannot_delete_external_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    campaign = _build_campaign(tmp_path, include_60kph=True)
    module.curate(campaign)
    output = campaign / "40_visuals"

    reference_path = campaign / "40_visuals/visual_manifest.json"
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    reference["assets"] = [
        asset for asset in reference["assets"] if asset["speed_class"] != "60kph"
    ]
    reference["asset_count"] = len(reference["assets"])
    reference["source_snapshot_sha256"] = "b" * 64
    _write_json(reference_path, reference)

    summary_path = campaign / "50_reports/runtime_control_campaign_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["source_snapshot_sha256"] = "b" * 64
    summary["60kph_pilot"] = {"status": "ABSENT", "reason": "superseded fixture"}
    summary["evidence"] = []
    _write_json(summary_path, summary)

    selected_parent = output / "60kph/town06_straight/selected_pilot"
    moved_parent = tmp_path / "moved-selected-pilot"
    external_parent = tmp_path / "outside-publication"
    external_parent.mkdir()
    external_victim = external_parent / "02_autoware_drive.gif"
    external_victim.write_bytes(b"outside victim\n")
    original_replace = module.os.replace
    swapped = False

    def replace_then_swap(
        source: object, destination: object, *args: object, **kwargs: object
    ) -> None:
        nonlocal swapped
        original_replace(source, destination, *args, **kwargs)
        if Path(destination).name == module.PUBLICATION_MANIFEST and not swapped:
            selected_parent.rename(moved_parent)
            selected_parent.symlink_to(external_parent, target_is_directory=True)
            swapped = True

    monkeypatch.setattr(module.os, "replace", replace_then_swap)

    with pytest.raises(module.CurationError, match="managed output directory"):
        module.curate(campaign)
    assert swapped is True
    assert external_victim.read_bytes() == b"outside victim\n"


def test_publication_lock_rejects_concurrent_curator(tmp_path: Path) -> None:
    campaign = _build_campaign(tmp_path)
    target = (campaign / module.PUBLICATION_ROOT_RELATIVE_PATH, "campaign")

    with module._publication_locks([target]):
        with pytest.raises(module.CurationError, match="publication target is locked"):
            module.curate(campaign)
