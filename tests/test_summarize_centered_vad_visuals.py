from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image
import pytest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts"
    / "e2e"
    / "summarize_centered_vad_visuals.py"
)
SPEC = importlib.util.spec_from_file_location(
    "summarize_centered_vad_visuals", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
summary_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary_tool)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _result(success: bool) -> dict:
    return {
        "success": success,
        "execution_mode": "full_stack",
        "assessment": {
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "PASS" if success else "FAIL",
        },
        "final": {
            "goal_reached": True,
            "route_status": "goal_reached",
        },
    }


def _complete_attempt(path: Path, route: dict, *, success: bool = True) -> None:
    path.mkdir(parents=True, exist_ok=True)
    _write_json(path / "result.json", _result(success))
    _write_json(path / "source_route.json", route)
    if not success:
        return

    representative = Image.new("RGB", (120, 80), "navy")
    candidate = Image.new("RGB", (120, 80), "black")
    representative.save(path / "autoware_rviz_fullscreen.png")
    candidate.save(path / "autoware_rviz_candidate.png")
    gif_frames = [Image.new("RGB", (960, 64), color) for color in ("navy", "teal")]
    gif_frames[0].save(
        path / "autoware_rviz_drive.gif",
        save_all=True,
        append_images=gif_frames[1:],
        duration=100,
        loop=0,
    )
    (path / "autoware_rviz_capture.mkv").write_bytes(b"fixture-video")

    provenance = path / "rviz_capture_provenance"
    provenance.mkdir()
    config = provenance / "autoware_vad_carla.rviz"
    config.write_text("fixture centered config\n", encoding="utf-8")
    config_sha256 = _sha256(config)
    (provenance / "SHA256SUMS").write_text(
        f"{config_sha256}  autoware_vad_carla.rviz\n", encoding="utf-8"
    )

    capture = {
        "schema_version": 1,
        "candidate_observed": True,
        "candidate_topic": "/planning/vad/candidate_trajectories",
        "capture_started_after_candidate": True,
        "candidate_observed_at": "2026-08-31T00:00:00+00:00",
        "candidate_still_captured_at": "2026-08-31T00:00:00.100000+00:00",
        "recording_started_at": "2026-08-31T00:00:00.200000+00:00",
        "route_evaluation_started_at": "2026-08-31T00:00:01+00:00",
        "route_evaluation_finished_at": "2026-08-31T00:00:05+00:00",
        "captured_at": "2026-08-31T00:00:03+00:00",
        "source_dimensions": [120, 80],
        "png_dimensions": [120, 80],
        "candidate_png_dimensions": [120, 80],
        "gif_dimensions": [960, 64],
        "png_file": "autoware_rviz_fullscreen.png",
        "candidate_png_file": "autoware_rviz_candidate.png",
        "gif_file": "autoware_rviz_drive.gif",
        "recording_file": "autoware_rviz_capture.mkv",
        "representative_frame": {
            "source": "autoware_rviz_capture.mkv",
            "selection": "route_evaluation_midpoint",
            "offset_sec": 2.8,
            "recording_duration_sec": 6.0,
            "captured_at": "2026-08-31T00:00:03+00:00",
        },
        "rviz_view_contract": {
            "vehicle_centered": True,
            "controller": "rviz_default_plugins/TopDownOrtho",
            "target_frame": "base_link",
            "center_xy_m": [0.0, 0.0],
            "angle_rad": 0.0,
            "scale": 10.0,
            "visible_path_topics": sorted(summary_tool.REQUIRED_PATH_TOPICS),
            "config_file": "rviz_capture_provenance/autoware_vad_carla.rviz",
            "config_sha256": config_sha256,
        },
    }
    _write_json(path / "desktop_capture.json", capture)
    (path / "runtime.env").write_text(
        "RECOMMENDED=true\n"
        "VISUALIZE=true\n"
        "CAPTURE_DESKTOP=true\n"
        f"RVIZ_CAPTURE_CONFIG={config.resolve()}\n"
        f"RVIZ_CAPTURE_CONFIG_SHA256={config_sha256}\n",
        encoding="utf-8",
    )


def _fixture(tmp_path: Path) -> tuple[Path, Path]:
    centered_root = tmp_path / "centered"
    original_root = tmp_path / "original"
    publications = []
    for trial_id in summary_tool.TRIAL_IDS:
        route = {
            "schema_version": 1,
            "town": "TownFixture",
            "scenario": "straight" if trial_id == "straight" else "left",
            "route": [[0.0, 0.0], [1.0, 1.0]],
        }
        original = original_root / trial_id
        _write_json(original / "source_route.json", route)
        trial_root = centered_root / "maps/town_fixture/trials" / trial_id
        _complete_attempt(trial_root / "attempt_001", route)
        publications.append(
            {
                "map_id": "town_fixture",
                "trial_id": trial_id,
                "trial_directory": str(original),
                "source": "fixture_selected_original",
            }
        )
    _complete_attempt(
        centered_root / "maps/town_fixture/trials/straight/attempt_002",
        {"schema_version": 1, "town": "TownFixture", "scenario": "straight"},
        success=False,
    )
    manifest = tmp_path / "publication_manifest.json"
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "autoware_vad_publications": publications,
        },
    )
    return centered_root, manifest


def test_audit_selects_latest_strict_pass_and_matches_original_route(
    tmp_path: Path,
) -> None:
    centered_root, manifest = _fixture(tmp_path)

    records = summary_tool.audit_trials(
        centered_root,
        manifest,
        expected_map_count=1,
        verify_frames=False,
    )

    assert len(records) == 2
    straight = next(record for record in records if record["trial_id"] == "straight")
    assert straight["selected_attempt"] == "attempt_001"
    assert straight["attempt_history"][-1]["full_stack_result_pass"] is False
    assert straight["source_route"]["status"] == "EXACT_MATCH"
    assert straight["capture"]["representative_frame"]["selection"] == (
        "route_evaluation_midpoint"
    )
    assert straight["capture"]["representative_frame"]["frame_verification"][
        "status"
    ] == "SKIPPED"


def test_audit_rejects_route_drift_from_publication_selected_original(
    tmp_path: Path,
) -> None:
    centered_root, manifest = _fixture(tmp_path)
    route_path = (
        centered_root
        / "maps/town_fixture/trials/turn/attempt_001/source_route.json"
    )
    route = json.loads(route_path.read_text(encoding="utf-8"))
    route["route"][1] = [9.0, 9.0]
    _write_json(route_path, route)

    with pytest.raises(summary_tool.AuditError, match="source_route.json differs"):
        summary_tool.audit_trials(
            centered_root,
            manifest,
            expected_map_count=1,
            verify_frames=False,
        )


def test_audit_rejects_non_centered_view_contract(tmp_path: Path) -> None:
    centered_root, manifest = _fixture(tmp_path)
    capture_path = (
        centered_root
        / "maps/town_fixture/trials/turn/attempt_001/desktop_capture.json"
    )
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["rviz_view_contract"]["center_xy_m"] = [2.0, 0.0]
    _write_json(capture_path, capture)

    with pytest.raises(summary_tool.AuditError, match="does not center base_link"):
        summary_tool.audit_trials(
            centered_root,
            manifest,
            expected_map_count=1,
            verify_frames=False,
        )


def test_cli_writes_pending_summary_and_contact_sheet(tmp_path: Path) -> None:
    centered_root, manifest = _fixture(tmp_path)
    completed = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--centered-root",
            str(centered_root),
            "--publication-manifest",
            str(manifest),
            "--expected-map-count",
            "1",
            "--skip-frame-reextract",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    summary = json.loads((centered_root / "summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "MECHANICAL_PASS_VISUAL_PENDING"
    assert summary["schema_version"] == 2
    assert "publication_manifest" not in summary
    selection = summary["publication_selection"]
    assert selection["scope"] == summary_tool.PUBLICATION_SELECTION_SCOPE
    assert selection["canonicalization"] == (
        summary_tool.PUBLICATION_SELECTION_CANONICALIZATION
    )
    assert selection["record_count"] == 2
    assert selection["records"] == sorted(
        selection["records"], key=lambda record: (record["map_id"], record["trial_id"])
    )
    assert summary["counts"] == {
        "maps": 1,
        "total": 2,
        "mechanical_pass": 2,
        "visual_pass": 0,
        "visual_flag": 0,
        "visual_pending": 2,
    }
    assert (centered_root / "SUMMARY.md").is_file()
    with Image.open(centered_root / "contact_sheet.png") as sheet:
        assert sheet.size == (1920, 402)


def test_publication_selection_digest_ignores_non_selection_manifest_fields(
    tmp_path: Path,
) -> None:
    _, manifest = _fixture(tmp_path)
    original = summary_tool._publication_selection(manifest, 1)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["owned_window_visual_audit"] = {
        "status": "PASS",
        "files": ["audit.json"],
    }
    _write_json(manifest, payload)

    unrelated_update = summary_tool._publication_selection(manifest, 1)

    assert unrelated_update == original
    payload["autoware_vad_publications"][0]["source"] = "changed-selection"
    _write_json(manifest, payload)
    changed_selection = summary_tool._publication_selection(manifest, 1)
    assert changed_selection["sha256"] != original["sha256"]


def test_visual_review_is_bound_to_representative_png_hashes(tmp_path: Path) -> None:
    centered_root, manifest = _fixture(tmp_path)
    records = summary_tool.audit_trials(
        centered_root,
        manifest,
        expected_map_count=1,
        verify_frames=False,
    )
    scenes = []
    for record in records:
        scenes.append(
            {
                "map_id": record["map_id"],
                "trial_id": record["trial_id"],
                "status": "PASS",
                "vehicle_visible": True,
                "reference_route_visible": True,
                "final_trajectory_visible": True,
                "vad_trajectories_visible": True,
                "viewport_centered": True,
                "representative_png_sha256": record["capture"][
                    "representative_png"
                ]["sha256"],
                "notes": "fixture review",
            }
        )
    review = tmp_path / "visual_review.json"
    _write_json(
        review,
        {
            "schema_version": 1,
            "reviewer": "pytest",
            "reviewed_at": datetime.now(timezone.utc).isoformat(),
            "scenes": scenes,
        },
    )

    summary = summary_tool.build_outputs(
        centered_root,
        manifest,
        output_json=centered_root / "summary.json",
        output_markdown=centered_root / "SUMMARY.md",
        contact_sheet=centered_root / "contact_sheet.png",
        visual_review_path=review,
        expected_map_count=1,
        verify_frames=False,
    )

    assert summary["status"] == "PASS"
    assert summary["counts"]["visual_pass"] == 2

    stale_review = json.loads(review.read_text(encoding="utf-8"))
    stale_review["scenes"][0]["representative_png_sha256"] = "0" * 64
    _write_json(review, stale_review)
    with pytest.raises(summary_tool.AuditError, match="is stale"):
        summary_tool.build_outputs(
            centered_root,
            manifest,
            output_json=centered_root / "summary.json",
            output_markdown=centered_root / "SUMMARY.md",
            contact_sheet=centered_root / "contact_sheet.png",
            visual_review_path=review,
            expected_map_count=1,
            verify_frames=False,
        )
