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


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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


def _build_campaign(
    tmp_path: Path,
    *,
    include_60kph: bool = False,
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
        pilot_name = "town06_straight_60kph_pilot_explicit_v3"
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
