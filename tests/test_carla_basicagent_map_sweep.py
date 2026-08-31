from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
REPORTER = ROOT / "scripts/e2e/carla_basicagent_sweep_report.py"
SWEEP = ROOT / "scripts/e2e/run_packaged_map_evidence_sweep.sh"
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"

SAFE_MAPS = (
    "town01",
    "town02_opt",
    "town03",
    "town04",
    "town05_opt",
    "town06",
    "town07",
    "town10hd_opt",
    "c_track_1_0_7",
)


def load_module():
    spec = importlib.util.spec_from_file_location("carla_basicagent_sweep_report", REPORTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_initialization_covers_all_19_maps_with_explicit_non_run_statuses(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    inventory = {
        "static": {
            "maps": [
                {
                    "id": entry["id"],
                    "assets_complete": entry["id"] != "woraksan_1_0_3",
                }
                for entry in manifest["maps"]
            ]
        }
    }

    paths = module.initialize_statuses(
        tmp_path, manifest, inventory, SAFE_MAPS, "127.0.0.1", 2100
    )

    assert len(paths) == 19
    assert all((path.parent / "server.log").is_file() for path in paths)
    assert all((path.parent / "catalog.log").is_file() for path in paths)
    assert all((path.parent / "suite.log").is_file() for path in paths)
    statuses = {
        path.parent.name: json.loads(path.read_text(encoding="utf-8"))
        for path in paths
    }
    assert all(statuses[map_id]["status"] == "PENDING" for map_id in SAFE_MAPS)
    assert all(statuses[map_id]["selected"] for map_id in SAFE_MAPS)
    assert {
        map_id
        for map_id, value in statuses.items()
        if value["status"] == "EXCLUDED"
    } == {"town02", "town05", "town08", "town09", "town10hd"}
    assert {
        map_id
        for map_id, value in statuses.items()
        if value["status"] == "SOURCE_EDITOR_REQUIRED"
    } == {"town11", "town12", "town13", "town15"}
    assert statuses["woraksan_1_0_3"]["status"] == "BLOCKED"
    assert "missing packaged assets" in statuses["woraksan_1_0_3"]["reason"]
    assert all(
        value["configuration"]["client_map_loading_allowed"] is False
        for value in statuses.values()
    )


def _write_valid_smoke(root: Path, map_id: str) -> None:
    job_root = root / "maps" / map_id / "smoke" / "job"
    episode = job_root / "episode"
    export = job_root / "export"
    preview = job_root / "preview"
    episode.mkdir(parents=True)
    export.mkdir()
    preview.mkdir()
    (episode / "manifest.json").write_text(
        json.dumps(
            {
                "status": "complete",
                "result": {
                    "goal_reached": True,
                    "collision_event_count": 0,
                    "lane_invasion_event_count": 0,
                },
                "runtime": {
                    "client_map_loading_allowed": False,
                    "client_map_loading_performed": False,
                },
            }
        ),
        encoding="utf-8",
    )
    (export / "manifest.json").write_text(
        json.dumps(
            {
                "status": "validated",
                "sample_count": 10,
                "maximum_route_cte_m": 0.2,
                "collision_event_count": 0,
                "lane_invasion_event_count": 0,
            }
        ),
        encoding="utf-8",
    )
    (preview / "overview.png").write_bytes(b"\x89PNG\r\n\x1a\npreview")
    (preview / "drive.gif").write_bytes(b"GIF89apreview")
    plan = {
        "status": "COMPLETE",
        "server": {
            "map_load_allowed": False,
            "map_lifecycle_managed": False,
        },
        "jobs": [
            {
                "status": "COMPLETE",
                "map_id": map_id,
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
    }
    plan_path = root / "maps" / map_id / "smoke/collection_plan.json"
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(json.dumps(plan), encoding="utf-8")
    catalog_path = root / "maps" / map_id / "catalog/route_catalog.json"
    catalog_path.parent.mkdir(parents=True, exist_ok=True)
    catalog_path.write_text(
        json.dumps(
            {
                "status": "complete",
                "map_id": map_id,
                "server": {
                    "map_load_allowed": False,
                    "map_load_performed": False,
                },
                "generation": {"seeds": [0]},
                "routes": [
                    {
                        "id": "route_a",
                        "status": "ready",
                        "scenario": "lane_follow",
                        "seed": 0,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )


def test_summary_labels_basicagent_and_links_valid_png_gif(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    module.initialize_statuses(tmp_path, manifest, None, ("town01",), "127.0.0.1", 2100)
    _write_valid_smoke(tmp_path, "town01")
    module.update_status(tmp_path, "town01", "PASS", "complete", "ok", None)

    summary, markdown = module.build_summary(tmp_path, manifest)

    assert summary["canonical_map_count"] == 19
    assert summary["selected_map_count"] == 1
    assert summary["selected_success_count"] == 1
    assert summary["status"] == "COMPLETE"
    assert summary["invocation_map_count"] == 1
    assert "not Autoware VAD" in summary["evidence_disclaimer"]
    town01 = next(entry for entry in summary["maps"] if entry["map_id"] == "town01")
    assert town01["smoke"]["scenario"] == "lane_follow"
    assert town01["smoke"]["artifacts"]["png"]["sha256"]
    assert "[PNG](maps/town01/smoke/job/preview/overview.png)" in markdown
    assert "[GIF](maps/town01/smoke/job/preview/drive.gif)" in markdown
    assert "Autoware VAD 추론/폐루프 제어 결과가 아닙니다" in markdown


def test_subset_resume_preserves_prior_campaign_results(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    module.initialize_statuses(
        tmp_path, manifest, None, SAFE_MAPS, "127.0.0.1", 2100
    )
    _write_valid_smoke(tmp_path, "town03")
    module.update_status(tmp_path, "town03", "PASS", "complete", "ok", None)

    module.initialize_statuses(
        tmp_path,
        manifest,
        None,
        SAFE_MAPS,
        "127.0.0.1",
        2100,
        ("town02_opt",),
    )
    summary, _ = module.build_summary(tmp_path, manifest)
    by_id = {entry["map_id"]: entry for entry in summary["maps"]}

    assert summary["selected_map_count"] == 9
    assert summary["invocation_map_count"] == 1
    assert by_id["town03"]["selected"] is True
    assert by_id["town03"]["selected_this_invocation"] is False
    assert by_id["town03"]["status"] == "SKIP_RESUME_VALIDATED"
    assert by_id["town02_opt"]["status"] == "PENDING"


def test_subset_resume_preserves_a_prior_failed_campaign_plan(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    module.initialize_statuses(
        tmp_path, manifest, None, SAFE_MAPS, "127.0.0.1", 2100
    )
    plan_path = tmp_path / "maps/town01/smoke/collection_plan.json"
    plan_path.parent.mkdir(parents=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "FAILED",
                "jobs": [
                    {
                        "status": "FAILED",
                        "reason": "strict exporter rejected one lane invasion",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    module.initialize_statuses(
        tmp_path,
        manifest,
        None,
        SAFE_MAPS,
        "127.0.0.1",
        2100,
        ("town02_opt",),
    )
    town01 = json.loads(
        (tmp_path / "maps/town01/status.json").read_text(encoding="utf-8")
    )

    assert town01["selected"] is True
    assert town01["selected_this_invocation"] is False
    assert town01["status"] == "FAILED"
    assert town01["exit_code"] == 1
    assert "strict exporter" in town01["reason"]


def test_validate_job_rejects_missing_visual_artifact(tmp_path):
    module = load_module()
    _write_valid_smoke(tmp_path, "town01")
    (tmp_path / "maps/town01/smoke/job/preview/drive.gif").unlink()

    _, error = module.validated_job(tmp_path.resolve(), "town01")

    assert error == "valid in-root PNG and GIF previews are both required"


def test_validate_job_rejects_cross_map_or_external_artifact_paths(tmp_path):
    module = load_module()
    _write_valid_smoke(tmp_path, "town01")
    plan_path = tmp_path / "maps/town01/smoke/collection_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    external = tmp_path / "outside.png"
    external.write_bytes(b"\x89PNG\r\n\x1a\npreview")
    plan["jobs"][0]["paths"]["preview_png"] = str(external)
    plan_path.write_text(json.dumps(plan), encoding="utf-8")

    _, error = module.validated_job(tmp_path.resolve(), "town01")

    assert error == "valid in-root PNG and GIF previews are both required"


def test_validate_job_rejects_a_different_requested_route_seed(tmp_path):
    module = load_module()
    _write_valid_smoke(tmp_path, "town01")

    _, error = module.validated_job(tmp_path.resolve(), "town01", 1)

    assert error == "route seed mismatch: expected 1, actual 0"


def test_sweep_script_has_safe_cold_start_and_cleanup_contract():
    source = SWEEP.read_text(encoding="utf-8")

    assert all(map_id in source for map_id in SAFE_MAPS)
    assert "--quality Epic" in source
    assert "--seeds 0" in source
    assert "--pairs-per-seed 1" in source
    assert "--route-seed ID=SEED" in source
    assert '--seeds "${route_seed}"' in source
    assert "--scenarios lane_follow" in source
    assert "setsid --wait" in source
    assert ".packaged_map_evidence_sweep.lock" in source
    assert 'flock -n "${sweep_lock_fd}"' in source
    assert "process_group_cleanup.sh" in source
    assert "e2e_stop_owned_process_group" in source
    assert "client.load_world" not in source
    assert "--allow-map-load" not in source
    assert "pkill" not in source
    assert "killall" not in source


def test_route_seed_override_is_recorded_in_status(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    module.initialize_statuses(
        tmp_path, manifest, None, ("town01",), "127.0.0.1", 2100
    )

    path = module.update_status(
        tmp_path, "town01", "RUNNING", "server_starting", "seed override", None, 7
    )
    status = json.loads(path.read_text(encoding="utf-8"))

    assert status["configuration"]["route_seed"] == 7


def test_suite_forwards_explicit_map_load_opt_in_only_when_requested(tmp_path):
    script = ROOT / "scripts/e2e/run_carla_expert_collection_suite.py"
    spec = importlib.util.spec_from_file_location("collection_suite_for_sweep", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = SimpleNamespace(
        collector=ROOT / "scripts/e2e/collect_carla_vad_expert.py",
        exporter=ROOT / "scripts/e2e/export_carla_vad_expert.py",
        renderer=ROOT / "scripts/e2e/render_carla_vad_expert.py",
        host="127.0.0.1",
        port=2100,
        physics_hz=20.0,
        capture_hz=5.0,
        target_speed_kmh=9.0,
        goal_tolerance_m=3.0,
        max_duration_sec=180.0,
        allow_map_load=False,
    )
    job = {
        "route_path": str(tmp_path / "route.json"),
        "seed": 0,
        "weather": "ClearNoon",
        "paths": {
            "episode": str(tmp_path / "episode"),
            "export": str(tmp_path / "export"),
            "preview_png": str(tmp_path / "overview.png"),
            "preview_gif": str(tmp_path / "drive.gif"),
        },
    }

    command = module._commands_for_job(args, job)["collector"]
    assert "--goal-tolerance-m" in command
    assert "--allow-map-load" not in command

    args.allow_map_load = True
    opted_in = module._commands_for_job(args, job)["collector"]
    assert opted_in[-1] == "--allow-map-load"


def test_collector_defaults_to_requiring_the_cold_started_route_map():
    script = ROOT / "scripts/e2e/collect_carla_vad_expert.py"
    spec = importlib.util.spec_from_file_location("collector_for_sweep", script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    args = module.parse_args(["episode", "route.json"])

    assert args.allow_map_load is False
