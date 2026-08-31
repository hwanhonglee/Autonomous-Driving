from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess

from PIL import Image
import pytest

from scripts.e2e import carla_basicagent_sweep_report as sweep_report
from scripts.e2e import publish_validation_assets as publisher


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_complete_smoke(artifact_root: Path) -> None:
    job_root = artifact_root / "maps/town01/smoke/job"
    episode = job_root / "episode"
    export = job_root / "export"
    preview = job_root / "preview"
    for directory in (episode, export, preview):
        directory.mkdir(parents=True, exist_ok=True)

    samples = []
    states = []
    for index in range(2):
        frame = 100 + index
        timestamp_ns = index * 1_000_000_000
        states.append(
            {
                "timestamp_ns": timestamp_ns,
                "x": float(index),
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
            }
        )
        cameras = {}
        for camera_index, camera_name in enumerate(publisher.CAMERA_NAMES):
            relative = f"images/{camera_name}/{frame}.jpg"
            image_path = episode / relative
            image_path.parent.mkdir(parents=True, exist_ok=True)
            Image.new(
                "RGB",
                (96, 54),
                (30 + camera_index * 25, 60 + index * 30, 120),
            ).save(image_path)
            cameras[camera_name] = {"path": relative}
        samples.append(
            {
                "frame": frame,
                "timestamp_ns": timestamp_ns,
                "town": "Town01",
                "weather": "ClearNoon",
                "vad_command_0based": 3,
                "route_progress_m": float(index),
                "route_cte_m": 0.05,
                "pose_map_xyz_yaw": [float(index), 0.0, 0.0, 0.0],
                "future_expert": {
                    "positions_xy": [[0.5, 0.0], [1.0, 0.0], [1.5, 0.0]]
                },
                "cameras": cameras,
            }
        )
    _write_json(
        episode / "route.json",
        {
            "town": "Town01",
            "weather": "ClearNoon",
            "route": [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}],
        },
    )
    (episode / "states.jsonl").write_text(
        "".join(json.dumps(item) + "\n" for item in states), encoding="utf-8"
    )
    (export / "samples.jsonl").write_text(
        "".join(json.dumps(item) + "\n" for item in samples), encoding="utf-8"
    )
    _write_json(
        episode / "manifest.json",
        {
            "status": "complete",
            "runtime": {
                "client_map_loading_allowed": False,
                "client_map_loading_performed": False,
            },
            "result": {
                "goal_reached": True,
                "collision_event_count": 0,
                "lane_invasion_event_count": 0,
            },
        },
    )
    _write_json(
        export / "manifest.json",
        {
            "status": "validated",
            "sample_count": 2,
            "maximum_route_cte_m": 0.05,
            "collision_event_count": 0,
            "lane_invasion_event_count": 0,
        },
    )
    Image.new("RGB", (32, 18), "navy").save(preview / "overview.png")
    Image.new("RGB", (32, 18), "navy").save(
        preview / "drive.gif", save_all=True
    )
    _write_json(
        artifact_root / "maps/town01/smoke/collection_plan.json",
        {
            "status": "COMPLETE",
            "jobs": [
                {
                    "status": "COMPLETE",
                    "map_id": "town01",
                    "scenario": "lane_follow",
                    "weather": "ClearNoon",
                    "seed": 0,
                    "route_id": "route_a",
                    "paths": {
                        "episode": str(episode),
                        "export": str(export),
                        "preview_png": str(preview / "overview.png"),
                        "preview_gif": str(preview / "drive.gif"),
                    },
                }
            ],
            "server": {
                "map_load_allowed": False,
                "map_lifecycle_managed": False,
            },
        },
    )
    _write_json(
        artifact_root / "maps/town01/catalog/route_catalog.json",
        {
            "status": "complete",
            "map_id": "town01",
            "server": {
                "map_load_allowed": False,
                "map_load_performed": False,
            },
            "generation": {"seeds": [0]},
            "routes": [
                {"id": "route_a", "scenario": "lane_follow", "seed": 0}
            ],
        },
    )


def _write_labeled_vad_trial(
    directory: Path,
    *,
    scenario: str,
    start_spawn_index: int,
    goal_spawn_index: int,
    town: str = "Town01",
    captured_at: str = "2026-08-31T12:00:00+09:00",
) -> None:
    route = directory / "aligned_route.json"
    route_payload = {
        "town": town,
        "scenario": scenario,
        "route_length_m": 42.5,
        "start_spawn_index": start_spawn_index,
        "goal_spawn_index": goal_spawn_index,
    }
    _write_json(route, route_payload)
    _write_json(directory / "source_route.json", route_payload)
    _write_json(
        directory / "result.json",
        {
            "success": True,
            "execution_mode": "full_stack",
            "route_file": str(route),
            "assessment": {
                "planning_architecture": "vad_route_manager_hybrid",
                "route_completion": "PASS",
            },
            "final": {"goal_reached": True, "route_status": "goal_reached"},
        },
    )
    for name in (
        "route_result.png",
        "path_vs_control.png",
        "steering_tracking.png",
    ):
        Image.new("RGB", (320, 180), "green").save(directory / name)
    Image.new("RGB", (320, 180), "green").save(
        directory / "turn_path_control.gif", save_all=True
    )
    Image.new("RGB", (1280, 720), "navy").save(
        directory / "autoware_rviz_fullscreen.png"
    )
    Image.new("RGB", (960, 540), "navy").save(
        directory / "autoware_rviz_drive.gif", save_all=True
    )
    _write_json(
        directory / "desktop_capture.json",
        {
            "schema_version": 1,
            "candidate_observed": True,
            "candidate_topic": "/planning/vad/candidate_trajectories",
            "capture_started_after_candidate": True,
            "display": ":99.0",
            "source_dimensions": [1280, 720],
            "png_dimensions": [1280, 720],
            "gif_dimensions": [960, 540],
            "png_file": "autoware_rviz_fullscreen.png",
            "gif_file": "autoware_rviz_drive.gif",
            "captured_at": captured_at,
        },
    )
    _write_json(directory / "diagnosis.json", {"status": "complete"})
    _write_json(
        directory / "latency/e2e_latency.json", {"status": "complete"}
    )


def _write_centered_vad_visual_refresh(
    directory: Path, selected_directory: Path
) -> None:
    selected_route = json.loads(
        (selected_directory / "source_route.json").read_text(encoding="utf-8")
    )
    _write_labeled_vad_trial(
        directory,
        scenario=selected_route["scenario"],
        start_spawn_index=selected_route["start_spawn_index"],
        goal_spawn_index=selected_route["goal_spawn_index"],
        town=selected_route["town"],
        captured_at="2026-08-31T05:00:30+00:00",
    )
    shutil.copy2(
        selected_directory / "source_route.json", directory / "source_route.json"
    )
    Image.new("RGB", (1280, 720), "yellow").save(
        directory / "autoware_rviz_candidate.png"
    )
    (directory / "autoware_rviz_capture.mkv").write_bytes(b"fixture-recording")
    provenance = directory / "rviz_capture_provenance"
    provenance.mkdir(parents=True, exist_ok=True)
    config = provenance / "autoware_vad_carla.rviz"
    config.write_text("Panels: []\n", encoding="utf-8")
    config_sha256 = publisher._sha256(config)
    (provenance / "SHA256SUMS").write_text(
        f"{config_sha256}  autoware_vad_carla.rviz\n", encoding="utf-8"
    )
    (directory / "runtime.env").write_text(
        "\n".join(
            [
                "RECOMMENDED=true",
                "VISUALIZE=true",
                "CAPTURE_DESKTOP=true",
                f"RVIZ_CAPTURE_CONFIG={config.resolve()}",
                f"RVIZ_CAPTURE_CONFIG_SHA256={config_sha256}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    capture_path = directory / "desktop_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture.update(
        {
            "candidate_observed_at": "2026-08-31T04:59:55+00:00",
            "candidate_png_dimensions": [1280, 720],
            "candidate_png_file": "autoware_rviz_candidate.png",
            "candidate_still_captured_at": "2026-08-31T04:59:57+00:00",
            "recording_file": "autoware_rviz_capture.mkv",
            "recording_started_at": "2026-08-31T04:59:58+00:00",
            "route_evaluation_started_at": "2026-08-31T05:00:00+00:00",
            "route_evaluation_finished_at": "2026-08-31T05:01:00+00:00",
            "captured_at": "2026-08-31T05:00:30+00:00",
            "representative_frame": {
                "captured_at": "2026-08-31T05:00:30+00:00",
                "offset_sec": 32.0,
                "recording_duration_sec": 64.0,
                "selection": "route_evaluation_midpoint",
                "source": "autoware_rviz_capture.mkv",
            },
            "rviz_view_contract": {
                "angle_rad": 0.0,
                "center_xy_m": [0.0, 0.0],
                "config_file": (
                    "rviz_capture_provenance/autoware_vad_carla.rviz"
                ),
                "config_sha256": config_sha256,
                "controller": "rviz_default_plugins/TopDownOrtho",
                "scale": 10.0,
                "target_frame": "base_link",
                "vehicle_centered": True,
                "visible_path_topics": [
                    "/planning/trajectory",
                    "/planning/vad/candidate_trajectories",
                    "/planning/vad_route/actual_path",
                    "/planning/vad_route/reference_path",
                    "/planning/vad_route/selected_raw_trajectory",
                ],
                "visual_clarity": {
                    "odometry_display": "Kinematic State",
                    "odometry_keep": 1,
                    "odometry_covariance": False,
                    "odometry_orientation": False,
                    "odometry_position": False,
                    "candidate_path_alpha": 0.22,
                    "candidate_path_width": 0.04,
                },
            },
        }
    )
    _write_json(capture_path, capture)


def _write_terminal_vad_matrix(
    artifact_root: Path, snapshot, *, aggregate_status: str = "COMPLETE"
) -> Path:
    matrix_root = artifact_root / "autoware_vad_town_matrix"
    matrix_id = "fixture_straight_turn"
    runtime_profile = {
        "id": "fixture_recommended_visualized",
        "wrapper_options": [
            "--recommended",
            "--visualize",
            "--capture-desktop",
        ],
        "client_map_loading_allowed": False,
    }
    route_contract = {
        "trials": [
            {"id": "straight", "catalog_scenarios": ["straight"]},
            {"id": "turn", "catalog_scenarios": ["left", "right"]},
        ]
    }
    plan_maps = []
    statuses = []
    for entry in snapshot["maps"]:
        map_id = entry["map_id"]
        canonical_name = entry["canonical_name"]
        runnable = map_id == "town01"
        plan_maps.append(
            {
                "map_id": map_id,
                "canonical_name": canonical_name,
                "runnable": runnable,
            }
        )
        trials = {}
        for trial_id, scenario in (("straight", "straight"), ("turn", "left")):
            if runnable:
                attempt = (
                    matrix_root
                    / "maps"
                    / map_id
                    / "trials"
                    / trial_id
                    / "attempt_001"
                )
                _write_labeled_vad_trial(
                    attempt,
                    scenario=scenario,
                    start_spawn_index=1,
                    goal_spawn_index=2,
                    town=canonical_name,
                )
                validation = {
                    "schema_version": 1,
                    "status": "PASS",
                    "matrix_id": matrix_id,
                    "map_id": map_id,
                    "trial_id": trial_id,
                    "catalog_scenario": scenario,
                    "trial_directory": str(attempt.resolve()),
                    "runtime_profile": runtime_profile,
                    "result": {
                        "success": True,
                        "execution_mode": "full_stack",
                        "planning_architecture": "vad_route_manager_hybrid",
                        "route_completion": "PASS",
                        "goal_reached": True,
                    },
                }
                validation_path = attempt / "matrix_validation.json"
                _write_json(validation_path, validation)
                trials[trial_id] = {
                    "status": "PASS",
                    "reason": "fixture passed",
                    "attempt_directory": str(attempt.resolve()),
                    "validation": str(validation_path.resolve()),
                }
            else:
                trials[trial_id] = {
                    "status": "BLOCKED",
                    "reason": "fixture has no admitted bundle",
                    "attempt_directory": None,
                    "validation": None,
                }
        status = {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "map_id": map_id,
            "canonical_name": canonical_name,
            "runnable": runnable,
            "status": "PASS" if runnable else "BLOCKED",
            "stage": "complete" if runnable else "admission",
            "reason": "fixture terminal status",
            "block_code": None if runnable else "fixture_blocked",
            "updated_at": "2026-08-31T03:30:00+00:00",
            "trials": trials,
        }
        _write_json(matrix_root / "maps" / map_id / "status.json", status)
        statuses.append(status)
    _write_json(
        matrix_root / "matrix_plan.json",
        {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "generated_at": "2026-08-31T02:00:00+00:00",
            "canonical_map_count": len(plan_maps),
            "runnable_map_count": 1,
            "runtime_profile": runtime_profile,
            "route_contract": route_contract,
            "maps": plan_maps,
        },
    )
    _write_json(
        matrix_root / "aggregate.json",
        {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "generated_at": "2026-08-31T04:00:00+00:00",
            "status": aggregate_status,
            "canonical_map_count": len(statuses),
            "runnable_map_count": 1,
            "runnable_pass_count": 1,
            "blocked_map_count": len(statuses) - 1,
            "status_counts": {"BLOCKED": len(statuses) - 1, "PASS": 1},
            "runtime_profile": runtime_profile,
            "route_contract": route_contract,
            "maps": statuses,
        },
    )
    return matrix_root


def _mark_terminal_matrix_straight_failed(matrix_root: Path) -> None:
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    failed_attempt = matrix_root / "maps/town01/trials/straight/attempt_002"
    _write_labeled_vad_trial(
        failed_attempt,
        scenario="straight",
        start_spawn_index=30,
        goal_spawn_index=40,
    )
    failed_result_path = failed_attempt / "result.json"
    failed_result = json.loads(failed_result_path.read_text(encoding="utf-8"))
    failed_result["success"] = False
    failed_result["assessment"]["route_completion"] = "FAIL"
    failed_result["final"] = {"goal_reached": False, "route_status": "ready"}
    _write_json(failed_result_path, failed_result)
    status.update(
        {
            "status": "FAILED",
            "stage": "straight_failed",
            "reason": "The original catalog straight trial failed strict validation.",
        }
    )
    status["trials"]["straight"] = {
        "status": "FAILED",
        "reason": "Original straight attempt failed; retain this matrix result.",
        "attempt_directory": str(failed_attempt.resolve()),
        "validation": None,
    }
    _write_json(status_path, status)

    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate.update(
        {
            "status": "FAILED",
            "runnable_pass_count": 0,
            "status_counts": {"BLOCKED": 18, "FAILED": 1},
        }
    )
    aggregate["maps"] = [
        status if entry["map_id"] == "town01" else entry
        for entry in aggregate["maps"]
    ]
    _write_json(aggregate_path, aggregate)


def _snapshot(artifact_root: Path, complete: bool = False):
    manifest, _ = sweep_report.load_manifest(MANIFEST)
    sweep_report.initialize_statuses(
        artifact_root, manifest, None, ("town01",), "127.0.0.1", 2100
    )
    if complete:
        _write_complete_smoke(artifact_root)
        sweep_report.update_status(
            artifact_root,
            "town01",
            "PASS",
            "complete",
            "BasicAgent evidence validated; not Autoware VAD evidence.",
            None,
        )
    else:
        sweep_report.update_status(
            artifact_root,
            "town01",
            "FAILED",
            "failed",
            "Synthetic terminal status without publishable evidence.",
            1,
        )
    aggregate, markdown = sweep_report.build_summary(artifact_root, manifest)
    _write_json(artifact_root / "aggregate.json", aggregate)
    (artifact_root / "SUMMARY.md").write_text(markdown, encoding="utf-8")
    return publisher.load_sweep_snapshot(
        artifact_root, expected_selected_map_count=1
    )


def test_dashboard_uses_all_19_matching_aggregate_status_records(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "artifacts")
    output = tmp_path / "dashboard.png"

    publisher.render_status_dashboard(snapshot, output)

    with Image.open(output) as image:
        assert image.size == (1920, 1080)
    assert len(snapshot["maps"]) == 19
    assert publisher.collect_basicagent_runs(snapshot) == []


def test_stale_aggregate_is_rejected(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root)
    status_path = artifact_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["stage"] = "changed_after_summary"
    _write_json(status_path, status)

    with pytest.raises(publisher.PublicationError, match="aggregate/status mismatch"):
        publisher.load_sweep_snapshot(
            artifact_root, expected_selected_map_count=1
        )


def test_verified_source_archive_fails_closed_on_digest_mismatch(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.json"
    destination = tmp_path / "archive.json"
    source.write_text('{"status":"changed"}\n', encoding="utf-8")

    with pytest.raises(publisher.PublicationError, match="changed during publication"):
        publisher._copy_verified_source(
            source,
            destination,
            "0" * 64,
            "fixture aggregate",
        )


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg is required")
def test_publish_renders_only_verified_basicagent_run_and_separates_scope(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root, complete=True)
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    stale = docs_root / "town03/expert_overview_1920x1080.png"
    stale.parent.mkdir(parents=True)
    stale.write_bytes(b"stale")
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        gif_fps=2.0,
        gif_max_frames=2,
    )

    assert len(payload["basicagent_publications"]) == 1
    assert payload["autoware_vad_publications"] == []
    archived_basicagent = docs_root / publisher.BASICAGENT_AGGREGATE_NAME
    assert archived_basicagent.read_bytes() == (artifact_root / "aggregate.json").read_bytes()
    assert payload["published_source_aggregate_file"] == (
        publisher.BASICAGENT_AGGREGATE_NAME
    )
    assert payload["source_aggregate_sha256"] == publisher._sha256(
        archived_basicagent
    )
    with Image.open(docs_root / "town01/expert_overview_1920x1080.png") as image:
        assert image.size == (1920, 1080)
    with Image.open(docs_root / "town01/expert_drive.gif") as image:
        assert image.size == (960, 540)
    assert not stale.exists()
    readme = (docs_root / "README.md").read_text(encoding="utf-8")
    assert "not Autoware VAD inference" in readme
    assert "No Autoware VAD trial was supplied" in readme
    assert "expert_overview_1920x1080.png" in readme
    assert publisher.BASICAGENT_AGGREGATE_NAME in readme
    assert "../../../../artifacts/" not in readme
    assert "all_maps_basicagent_status_1920x1080.png" in report.read_text(
        encoding="utf-8"
    )
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_vad_publication_requires_successful_full_stack_result(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    trial = artifact_root / "vad_trial"
    _write_json(
        trial / "result.json",
        {
            "success": True,
            "execution_mode": "minimal",
            "assessment": {
                "planning_architecture": "vad_route_manager_hybrid",
                "route_completion": "PASS",
            },
            "final": {"goal_reached": True, "route_status": "goal_reached"},
        },
    )
    Image.new("RGB", (32, 18), "green").save(trial / "route_result.png")
    Image.new("RGB", (32, 18), "green").save(
        trial / "turn_path_control.gif", save_all=True
    )

    with pytest.raises(publisher.PublicationError, match="not a successful full-stack"):
        publisher.collect_vad_trials(snapshot, {"town01": trial})

    result = json.loads((trial / "result.json").read_text(encoding="utf-8"))
    result["execution_mode"] = "full_stack"
    _write_json(trial / "result.json", result)
    trials = publisher.collect_vad_trials(snapshot, {"town01": trial})
    assert len(trials) == 1
    assert trials[0].map_id == "town01"


def test_vad_trial_spec_supports_two_labeled_routes_for_one_map() -> None:
    specs = publisher._parse_vad_trial_specs(
        [
            "town01:straight=/tmp/straight",
            "town01:turn_left=/tmp/turn_left",
            "town02=/tmp/legacy",
        ]
    )

    assert [(item.map_id, item.trial_id) for item in specs] == [
        ("town01", "straight"),
        ("town01", "turn_left"),
        ("town02", None),
    ]
    with pytest.raises(publisher.PublicationError, match="duplicate --vad-trial id"):
        publisher._parse_vad_trial_specs(
            ["town01:straight=/tmp/one", "town01:straight=/tmp/two"]
        )


def test_visual_refresh_spec_requires_a_labeled_trial() -> None:
    specs = publisher._parse_vad_visual_refresh_specs(
        ["town01:straight=/tmp/centered"]
    )

    assert len(specs) == 1
    assert specs[0].source == "same_route_vehicle_centered_visual_refresh"
    with pytest.raises(
        publisher.PublicationError,
        match="must use MAP_ID:TRIAL_ID",
    ):
        publisher._parse_vad_visual_refresh_specs(
            ["town01=/tmp/unlabeled"]
        )


def test_same_route_centered_refresh_overlays_only_visual_media_and_is_archived(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    refresh_specs = publisher._parse_vad_visual_refresh_specs(
        [f"town01:straight={refresh}"]
    )
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_matrix_root=matrix_root,
        vad_visual_refresh_paths=refresh_specs,
    )

    assert payload["autoware_vad_visual_refresh_count"] == 1
    record = next(
        item
        for item in payload["autoware_vad_publications"]
        if item["map_id"] == "town01" and item["trial_id"] == "straight"
    )
    refresh_record = record["visual_refresh"]
    assert refresh_record["status"] == "PASS"
    assert refresh_record["route_match"] == "exact_payload_and_sha256"
    assert refresh_record["source_route_sha256"] == (
        refresh_record["validation_source_route_sha256"]
    )
    assert refresh_record["rviz_view_contract"]["vehicle_centered"] is True
    assert record["validation_desktop_capture"]["captured_at"] == (
        "2026-08-31T12:00:00+09:00"
    )
    assert record["desktop_capture"]["captured_at"] == (
        "2026-08-31T05:00:30+00:00"
    )
    published = docs_root / "town01/autoware_vad/straight"
    assert (published / "autoware_vad_result.json").read_bytes() == (
        selected / "result.json"
    ).read_bytes()
    assert (published / "autoware_rviz_fullscreen.png").read_bytes() == (
        refresh / "autoware_rviz_fullscreen.png"
    ).read_bytes()
    assert (published / "autoware_rviz_candidate.png").is_file()
    assert (published / "visual_refresh/repeat_result.json").read_bytes() == (
        refresh / "result.json"
    ).read_bytes()
    assert (published / "visual_refresh/source_route.json").is_file()
    assert (published / "visual_refresh/runtime.env").is_file()
    assert (
        published
        / "visual_refresh/rviz_capture_provenance/autoware_vad_carla.rviz"
    ).is_file()
    assert (
        published / "visual_refresh/original_validation_desktop_capture.json"
    ).read_bytes() == (selected / "desktop_capture.json").read_bytes()
    report_text = report.read_text(encoding="utf-8")
    assert "same-route vehicle-centered repeated PASS" in report_text
    assert "original validation provenance preserved" in report_text
    assert f"--vad-visual-refresh town01:straight={refresh.resolve()}" in report_text
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_centered_refresh_rejects_a_different_source_route(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    specs, _ = publisher.resolve_vad_trial_specs(snapshot, (), matrix_root)
    selected_trials = publisher.collect_vad_trials(snapshot, specs)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    source_route = json.loads(
        (refresh / "source_route.json").read_text(encoding="utf-8")
    )
    source_route["route_length_m"] = 43.0
    _write_json(refresh / "source_route.json", source_route)

    with pytest.raises(
        publisher.PublicationError,
        match="does not exactly match",
    ):
        publisher.collect_vad_visual_refreshes(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", refresh)],
            selected_trials,
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("not_centered", "RViz view must center base_link"),
        ("not_midpoint", "representative frame must be"),
        ("not_recommended", "did not use the recommended"),
        ("odometry_trail", "visual clarity must disable odometry trails"),
    ],
)
def test_centered_refresh_fails_closed_on_capture_contract_changes(
    tmp_path: Path, mutation: str, message: str
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    specs, _ = publisher.resolve_vad_trial_specs(snapshot, (), matrix_root)
    selected_trials = publisher.collect_vad_trials(snapshot, specs)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    if mutation == "not_recommended":
        runtime_path = refresh / "runtime.env"
        runtime_path.write_text(
            runtime_path.read_text(encoding="utf-8").replace(
                "RECOMMENDED=true", "RECOMMENDED=false"
            ),
            encoding="utf-8",
        )
    else:
        capture_path = refresh / "desktop_capture.json"
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        if mutation == "not_centered":
            capture["rviz_view_contract"]["vehicle_centered"] = False
        elif mutation == "odometry_trail":
            capture["rviz_view_contract"]["visual_clarity"][
                "odometry_keep"
            ] = 250
        else:
            capture["representative_frame"]["selection"] = "first_frame"
        _write_json(capture_path, capture)

    with pytest.raises(publisher.PublicationError, match=message):
        publisher.collect_vad_visual_refreshes(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", refresh)],
            selected_trials,
        )


def test_labeled_vad_trials_publish_separately_with_desktop_and_analysis(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root)
    straight = artifact_root / "maps/town01/trials/straight"
    turn_left = artifact_root / "maps/town01/trials/turn_left"
    _write_labeled_vad_trial(
        straight,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
    )
    _write_labeled_vad_trial(
        turn_left,
        scenario="left",
        start_spawn_index=369,
        goal_spawn_index=425,
    )
    specs = publisher._parse_vad_trial_specs(
        [
            f"town01:straight={straight}",
            f"town01:turn_left={turn_left}",
        ]
    )
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=specs,
    )

    records = payload["autoware_vad_publications"]
    assert [record["trial_id"] for record in records] == ["straight", "turn_left"]
    assert records[0]["route"]["start_spawn_index"] == 394
    assert records[1]["route"]["goal_spawn_index"] == 425
    for trial_id in ("straight", "turn_left"):
        published = docs_root / f"town01/autoware_vad/{trial_id}"
        assert (published / "autoware_vad_result.json").is_file()
        assert (published / "autoware_vad_route.json").is_file()
        assert (published / "autoware_rviz_fullscreen.png").is_file()
        assert (published / "autoware_rviz_drive.gif").is_file()
        assert (published / "diagnosis.json").is_file()
        assert (published / "latency/e2e_latency.json").is_file()
    report_text = report.read_text(encoding="utf-8")
    assert "`straight`" in report_text
    assert "`turn_left`" in report_text
    assert "`394→290`" in report_text
    assert "`369→425`" in report_text
    assert f"--vad-trial town01:straight={straight.resolve()}" in report_text
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_labeled_vad_trial_rejects_capture_without_candidate(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    trial = artifact_root / "maps/town01/trials/straight"
    _write_labeled_vad_trial(
        trial,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
    )
    capture_path = trial / "desktop_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["candidate_observed"] = False
    _write_json(capture_path, capture)

    with pytest.raises(publisher.PublicationError, match="did not observe a VAD candidate"):
        publisher.collect_vad_trials(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", trial)],
        )


def test_terminal_matrix_auto_discovers_pass_trials_and_keeps_explicit_lane_follow(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    lane_follow = artifact_root / "vad_reference/c_track_lane_follow"
    _write_labeled_vad_trial(
        lane_follow,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
        town="C_track_1_0_7",
    )
    explicit = [
        publisher.VadTrialSpec(
            "c_track_1_0_7", "lane_follow", lane_follow
        )
    ]

    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, explicit, matrix_root
    )

    assert matrix_record is not None
    assert matrix_record["status"] == "COMPLETE"
    assert matrix_record["discovered_pass_trial_count"] == 2
    assert {
        (spec.map_id, spec.trial_id, spec.source) for spec in specs
    } == {
        ("town01", "straight", "matrix_pass_auto_discovery"),
        ("town01", "turn", "matrix_pass_auto_discovery"),
        ("c_track_1_0_7", "lane_follow", "explicit"),
    }

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    records = payload["autoware_vad_publications"]
    assert len(records) == 3
    assert payload["autoware_vad_matrix_source"][
        "discovered_pass_trial_count"
    ] == 2
    matrix_source = payload["autoware_vad_matrix_source"]
    assert len(matrix_source["maps"]) == 19
    assert matrix_source["published_plan_file"] == publisher.VAD_MATRIX_PLAN_NAME
    assert matrix_source["published_aggregate_file"] == (
        publisher.VAD_MATRIX_AGGREGATE_NAME
    )
    archived_plan = docs_root / publisher.VAD_MATRIX_PLAN_NAME
    archived_aggregate = docs_root / publisher.VAD_MATRIX_AGGREGATE_NAME
    assert archived_plan.read_bytes() == (matrix_root / "matrix_plan.json").read_bytes()
    assert archived_aggregate.read_bytes() == (matrix_root / "aggregate.json").read_bytes()
    assert publisher._sha256(archived_plan) == matrix_source["plan_sha256"]
    assert publisher._sha256(archived_aggregate) == matrix_source["aggregate_sha256"]
    assert (
        docs_root
        / "c_track_1_0_7/autoware_vad/lane_follow/autoware_rviz_fullscreen.png"
    ).is_file()
    report_text = report.read_text(encoding="utf-8")
    assert f"--vad-matrix-root {matrix_root.resolve()}" in report_text
    assert f"--vad-trial c_track_1_0_7:lane_follow={lane_follow.resolve()}" in report_text
    assert "matrix PASS" in report_text
    assert "explicit supplement for matrix-BLOCKED map; block preserved" in report_text
    assert "CARLA BasicAgent" in report_text
    assert "Original terminal matrix: all 19 maps" in report_text
    matrix_table = report_text.split(
        "### Original terminal matrix: all 19 maps", 1
    )[1].split("### Final paired coverage", 1)[0]
    for matrix_map in matrix_source["maps"]:
        assert (
            f"`{matrix_map['map_id']}` ({matrix_map['canonical_name']})"
            in matrix_table
        )
    assert "`town02` (Town02) | no | **BLOCKED**" in report_text
    assert "`town05` (Town05) remains distinct from `town05_opt`" in report_text
    assert "`town10hd` (Town10HD) remains distinct" in report_text
    assert "Final paired coverage across executable variants" in report_text
    assert "`town01` (Town01) | PASS (matrix) | PASS (matrix) | **PASS/PASS**" in report_text
    assert "lane_follow` row is supplemental" in report_text
    assert "route-assisted HYBRID" in report_text


def test_matrix_auto_discovery_rejects_an_incomplete_campaign(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        artifact_root, snapshot, aggregate_status="INCOMPLETE"
    )

    with pytest.raises(publisher.PublicationError, match="matrix is not terminal"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_matrix_auto_discovery_requires_offset_aware_provenance_time(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    plan_path = matrix_root / "matrix_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["generated_at"] = "2026-08-31T02:00:00"
    _write_json(plan_path, plan)

    with pytest.raises(publisher.PublicationError, match="must include a UTC offset"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_post_terminal_supplement_preserves_blocked_optimized_map_status(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    supplement = artifact_root / "supplemental_vad/town10hd_opt/straight/attempt_001"
    _write_labeled_vad_trial(
        supplement,
        scenario="straight",
        start_spawn_index=130,
        goal_spawn_index=137,
        town="Town10HD_Opt",
        captured_at="2026-08-31T05:00:00+00:00",
    )
    explicit = [
        publisher.VadTrialSpec("town10hd_opt", "straight", supplement)
    ]
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    record = next(
        item
        for item in payload["autoware_vad_publications"]
        if item["map_id"] == "town10hd_opt"
    )
    context = record["matrix_context"]
    assert context["relation"] == "explicit_supplement_for_blocked_map"
    assert context["timing"] == "after_terminal_matrix"
    assert context["matrix_map_status"] == "BLOCKED"
    assert context["matrix_trial_status"] == "BLOCKED"
    report_text = report.read_text(encoding="utf-8")
    assert "`town10hd` (Town10HD) remains distinct" in report_text
    assert "`town10hd_opt` (Town10HD_Opt)" in report_text
    assert "explicit supplement for matrix-BLOCKED map; block preserved" in report_text
    assert "after terminal matrix" in report_text


def test_terminal_failed_matrix_publishes_only_pass_and_preserves_failure(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    _mark_terminal_matrix_straight_failed(matrix_root)
    recovery = artifact_root / "supplemental_vad/town01/straight/attempt_001"
    _write_labeled_vad_trial(
        recovery,
        scenario="straight",
        start_spawn_index=10,
        goal_spawn_index=20,
    )
    explicit = [publisher.VadTrialSpec("town01", "straight", recovery)]

    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, explicit, matrix_root
    )

    assert matrix_record is not None
    assert matrix_record["status"] == "FAILED"
    assert matrix_record["status_counts"] == {"BLOCKED": 18, "FAILED": 1}
    assert matrix_record["runnable_pass_count"] == 0
    assert matrix_record["runnable_map_count"] == 1
    assert matrix_record["discovered_pass_trial_count"] == 1
    assert matrix_record["failed_map_count"] == 1
    assert matrix_record["failed_trial_count"] == 1
    assert matrix_record["failed_maps"][0]["map_id"] == "town01"
    assert {
        (spec.map_id, spec.trial_id, spec.source) for spec in specs
    } == {
        ("town01", "turn", "matrix_pass_auto_discovery"),
        ("town01", "straight", "explicit_recovery_for_matrix_failure"),
    }

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    archived_aggregate = docs_root / publisher.VAD_MATRIX_AGGREGATE_NAME
    archived = json.loads(archived_aggregate.read_text(encoding="utf-8"))
    assert archived["status"] == "FAILED"
    assert archived["maps"][0]["trials"]["straight"]["status"] == "FAILED"
    assert payload["autoware_vad_matrix_source"][
        "published_aggregate_file"
    ] == publisher.VAD_MATRIX_AGGREGATE_NAME
    records = payload["autoware_vad_publications"]
    assert {(record["trial_id"], record["source"]) for record in records} == {
        ("straight", "explicit_recovery_for_matrix_failure"),
        ("turn", "matrix_pass_auto_discovery"),
    }
    straight_record = next(
        record for record in records if record["trial_id"] == "straight"
    )
    assert straight_record["matrix_context"]["relation"] == (
        "explicit_recovery_alternate_route"
    )
    assert straight_record["matrix_context"]["timing"] == (
        "during_matrix_window_outside_matrix"
    )
    assert straight_record["matrix_context"]["matrix_failed_route"][
        "start_spawn_index"
    ] == 30
    report_text = report.read_text(encoding="utf-8")
    assert "Matrix campaign status: **FAILED**" in report_text
    assert "Original terminal matrix failures" in report_text
    assert "Original straight attempt failed" in report_text
    assert "explicit alternate-route supplement; matrix failure preserved" in report_text
    assert "failed matrix route 30→40, straight" in report_text
    assert "during matrix window (external)" in report_text
    assert (
        "`town01` (Town01) | PASS (alternate-route supplement; matrix failure "
        "preserved) | PASS (matrix) | **PASS/PASS**"
    ) in report_text
    assert publisher.VAD_MATRIX_AGGREGATE_NAME in report_text
    assert publisher.VAD_MATRIX_PLAN_NAME in report_text


def test_matrix_failed_status_does_not_hide_a_still_running_map(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["status"] = "RUNNING"
    status["trials"]["turn"]["status"] = "RUNNING"
    _write_json(status_path, status)
    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate["status"] = "FAILED"
    aggregate["runnable_pass_count"] = 0
    aggregate["status_counts"] = {"BLOCKED": 18, "RUNNING": 1}
    aggregate["maps"] = [
        status if entry["map_id"] == "town01" else entry
        for entry in aggregate["maps"]
    ]
    _write_json(aggregate_path, aggregate)

    with pytest.raises(publisher.PublicationError, match="non-terminal runnable"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_failed_matrix_report_discloses_failures_without_pass_trials(
    tmp_path: Path,
) -> None:
    """A matrix failure remains visible even when it yields no VAD rows."""
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status.update({"status": "FAILED", "stage": "turn_failed"})
    for trial_id in ("straight", "turn"):
        status["trials"][trial_id] = {
            "status": "FAILED",
            "reason": f"Original {trial_id} attempt failed.",
            "attempt_directory": None,
            "validation": None,
        }
    _write_json(status_path, status)
    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate.update(
        {
            "status": "FAILED",
            "runnable_pass_count": 0,
            "status_counts": {"BLOCKED": 18, "FAILED": 1},
            "maps": [
                status if entry["map_id"] == "town01" else entry
                for entry in aggregate["maps"]
            ],
        }
    )
    _write_json(aggregate_path, aggregate)

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_matrix_root=matrix_root,
    )

    assert payload["autoware_vad_publications"] == []
    report_text = report.read_text(encoding="utf-8")
    assert "Matrix campaign status: **FAILED**" in report_text
    assert "Original terminal matrix failures" in report_text
    assert "Original straight attempt failed" in report_text
    assert "Original turn attempt failed" in report_text
