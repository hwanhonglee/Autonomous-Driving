from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/run_carla_expert_collection_suite.py"
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("run_carla_expert_collection_suite", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_catalog(
    root: Path,
    map_id="town01",
    status="complete",
    route_exists=True,
    scenarios=("right",),
):
    root.mkdir(parents=True)
    routes = []
    for index, scenario in enumerate(scenarios):
        route_id = f"route_{chr(ord('a') + index)}"
        route = root / "routes" / f"{route_id}.json"
        if route_exists:
            route.parent.mkdir(parents=True, exist_ok=True)
            route.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "coordinate_reference": "base_link",
                        "town": "Town01",
                        "route": [{"x": 0}, {"x": 1}],
                    }
                ),
                encoding="utf-8",
            )
        digest = hashlib.sha256(route.read_bytes()).hexdigest() if route_exists else None
        routes.append(
            {
                "id": route_id,
                "status": "ready",
                "scenario": scenario,
                "path": f"routes/{route_id}.json",
                "sha256": digest,
            }
        )
    profile = "packaged_0915" if map_id == "town01" else "source_editor_0915_4ws"
    payload = {
        "schema_version": 1,
        "status": status,
        "map_id": map_id,
        "server_profile": profile,
        "routes": routes,
    }
    catalog = root / "route_catalog.json"
    catalog.write_text(json.dumps(payload), encoding="utf-8")
    return catalog


def arguments(module, tmp_path, catalog, execute=False):
    values = [
        "--catalog", str(catalog), "--output-root", str(tmp_path / "output"),
        "--weathers", "ClearNoon,WetNoon", "--seeds", "3,4",
        "--max-duration-sec", "20", "--estimated-jpeg-kib", "40",
    ]
    if execute:
        values.append("--execute")
    return module.parse_args(values)


def write_resume_artifacts(
    module,
    args,
    route_path: Path,
    job_root: Path,
    *,
    weather: str = "ClearNoon",
    seed: int = 3,
    include_export: bool = True,
    include_preview: bool = True,
):
    episode = job_root / "episode"
    episode.mkdir(parents=True)
    (episode / "route.json").write_bytes(route_path.read_bytes())
    (episode / "states.jsonl").write_text("{}\n", encoding="utf-8")
    (episode / "camera_frames.jsonl").write_text("{}\n", encoding="utf-8")
    route_hash = module._sha256_file(route_path)
    episode_manifest = {
        "status": "complete",
        "capture_contract": {
            "physics_hz": args.physics_hz,
            "camera_hz": args.capture_hz,
            "target_speed_kmh": args.target_speed_kmh,
            "goal_tolerance_m": args.goal_tolerance_m,
            "spawn_z_offset_m": args.spawn_z_offset_m,
            "maximum_total_duration_sec": args.max_duration_sec,
            "seed": seed,
            "capture_phase_field": "capture_phase",
            "capture_phase_schedule": {
                "stationary_warmup": {
                    "requested_duration_sec": args.stationary_warmup_sec,
                },
                "stationary_tail": {
                    "requested_duration_sec": args.stationary_tail_sec,
                },
            },
        },
        "runtime": {"weather": weather},
        "provenance": {
            "route_sha256": route_hash,
            "collector_sha256": module._sha256_file(args.collector.resolve()),
        },
    }
    (episode / "manifest.json").write_text(
        json.dumps(episode_manifest), encoding="utf-8"
    )
    if include_export:
        export = job_root / "export"
        export.mkdir()
        source_hashes = {
            name: module._sha256_file(episode / name)
            for name in (
                "manifest.json",
                "route.json",
                "states.jsonl",
                "camera_frames.jsonl",
            )
        }
        (export / "manifest.json").write_text(
            json.dumps(
                {
                    "status": "validated",
                    "source_hashes": source_hashes,
                    "future_horizons_s": (
                        list(args.export_horizons)
                        if args.export_horizons is not None
                        else [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
                    ),
                }
            ),
            encoding="utf-8",
        )
    if include_preview:
        preview = job_root / "preview"
        preview.mkdir()
        (preview / "overview.png").write_bytes(b"png")
        (preview / "drive.gif").write_bytes(b"gif")
    return episode_manifest


def test_dry_plan_builds_route_weather_seed_matrix_and_storage(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    catalog = module.load_catalog(write_catalog(tmp_path / "catalog"), manifest)
    args = arguments(module, tmp_path, tmp_path / "catalog" / "route_catalog.json")

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["mode"] == "dry-run"
    assert plan["execution_performed"] is False
    assert plan["status"] == "READY"
    assert len(plan["jobs"]) == 4
    assert {job["weather"] for job in plan["jobs"]} == {"ClearNoon", "WetNoon"}
    assert {job["seed"] for job in plan["jobs"]} == {3, 4}
    assert plan["matrix"]["scenarios"] == [
        "lane_follow", "straight", "left", "right"
    ]
    assert plan["route_selection"] == {
        "selected_scenarios": ["lane_follow", "straight", "left", "right"],
        "requested_route_ids": None,
        "catalog_route_count": 1,
        "selected_route_count": 1,
        "filtered_route_count": 0,
    }
    assert plan["estimated_storage"]["pending_bytes"] > 0
    assert plan["server"]["server_lifecycle_managed"] is False
    assert plan["server"]["map_lifecycle_managed"] is False
    assert plan["server"]["map_load_allowed"] is False
    assert plan["server"]["map_load_warning"] is None
    assert all(job["commands"]["collector"][1].endswith("collect_carla_vad_expert.py") for job in plan["jobs"])


def test_scenario_filter_runs_only_matching_catalog_routes(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(
        tmp_path / "catalog", scenarios=("right", "lane_follow", "left")
    )
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.scenarios = ("lane_follow",)

    plan = module.build_plan(args, manifest, [catalog])

    assert len(plan["jobs"]) == 4
    assert {job["scenario"] for job in plan["jobs"]} == {"lane_follow"}
    assert {job["route_id"] for job in plan["jobs"]} == {"route_b"}
    assert plan["route_selection"] == {
        "selected_scenarios": ["lane_follow"],
        "requested_route_ids": None,
        "catalog_route_count": 3,
        "selected_route_count": 1,
        "filtered_route_count": 2,
    }
    assert plan["catalogs"][0]["route_count"] == 3
    assert plan["catalogs"][0]["selected_route_count"] == 1
    assert plan["catalogs"][0]["filtered_route_count"] == 2


def test_scenario_filter_fails_when_no_catalog_route_matches(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog", scenarios=("right",))
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.scenarios = ("lane_follow",)

    with pytest.raises(module.SuiteError, match="no catalog route matches"):
        module.build_plan(args, manifest, [catalog])


def test_exact_route_id_filter_selects_only_requested_catalog_routes(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(
        tmp_path / "catalog", scenarios=("right", "lane_follow", "left")
    )
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.route_ids = ("route_b",)

    plan = module.build_plan(args, manifest, [catalog])

    assert {job["route_id"] for job in plan["jobs"]} == {"route_b"}
    assert plan["route_selection"] == {
        "selected_scenarios": ["lane_follow", "straight", "left", "right"],
        "requested_route_ids": ["route_b"],
        "catalog_route_count": 3,
        "selected_route_count": 1,
        "filtered_route_count": 2,
    }


def test_exact_route_id_filter_fails_closed_for_missing_excluded_or_ambiguous(
    tmp_path,
):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog", scenarios=("right", "lane_follow"))
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)

    args.route_ids = ("missing",)
    with pytest.raises(module.SuiteError, match="route ids are missing"):
        module.build_plan(args, manifest, [catalog])

    args.route_ids = ("route_a",)
    args.scenarios = ("lane_follow",)
    with pytest.raises(module.SuiteError, match="excluded by --scenarios"):
        module.build_plan(args, manifest, [catalog])

    second_path = write_catalog(tmp_path / "second")
    second = module.load_catalog(second_path, manifest)
    args.route_ids = ("route_a",)
    args.scenarios = ("lane_follow", "straight", "left", "right")
    with pytest.raises(module.SuiteError, match="ambiguous across catalogs"):
        module.build_plan(args, manifest, [catalog, second])


def test_exact_route_filter_ignores_unselected_catalog_server_profile(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    selected_path = write_catalog(tmp_path / "selected", map_id="town01")
    unrelated_path = write_catalog(tmp_path / "unrelated", map_id="town12")
    catalogs = [
        module.load_catalog(selected_path, manifest),
        module.load_catalog(unrelated_path, manifest),
    ]
    catalogs[1]["routes"][0]["id"] = "unrelated_route"
    args = arguments(module, tmp_path, selected_path)
    args.route_ids = ("route_a",)

    plan = module.build_plan(args, manifest, catalogs)

    assert plan["status"] == "READY"
    assert {job["map_id"] for job in plan["jobs"]} == {"town01"}
    assert plan["server"]["server_profiles"] == ["packaged_0915"]
    assert plan["catalogs"][1]["status"] == "FILTERED"


def test_source_editor_catalog_is_blocked(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog", map_id="town12")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["status"] == "BLOCKED"
    assert all(job["status"] == "BLOCKED" for job in plan["jobs"])
    assert "source_editor_required" in plan["jobs"][0]["reason"]


def test_source_editor_catalog_is_ready_with_matching_active_profile(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog", map_id="town12")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.active_server_profile = "source_editor_0915_4ws"

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["status"] == "READY"
    assert all(job["status"] == "PENDING" for job in plan["jobs"])
    assert plan["server"]["active_server_profile"] == "source_editor_0915_4ws"


def test_matching_declaration_does_not_allow_mixed_server_profiles(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    packaged_path = write_catalog(tmp_path / "packaged", map_id="town01")
    source_path = write_catalog(tmp_path / "source", map_id="town12")
    catalogs = [
        module.load_catalog(packaged_path, manifest),
        module.load_catalog(source_path, manifest),
    ]
    args = arguments(module, tmp_path, packaged_path)
    args.active_server_profile = "source_editor_0915_4ws"

    plan = module.build_plan(args, manifest, catalogs)

    assert plan["status"] == "BLOCKED"
    assert all(job["status"] == "BLOCKED" for job in plan["jobs"])
    assert all("different server profiles" in job["reason"] for job in plan["jobs"])


def test_missing_route_is_explicit_skip(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog", route_exists=False)
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)

    plan = module.build_plan(args, manifest, [catalog])

    assert all(job["status"] == "SKIP" for job in plan["jobs"])
    assert all(job["reason"] == "route file is missing" for job in plan["jobs"])
    assert plan["status"] == "BLOCKED"


def test_resume_skips_complete_and_validated_job(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.weathers = ("ClearNoon",)
    args.seeds = (3,)
    job_root = tmp_path / "output" / "town01" / "route_a" / "ClearNoon" / "seed_0003"
    route_path = Path(catalog["routes"][0]["resolved_path"])
    write_resume_artifacts(module, args, route_path, job_root)

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["jobs"][0]["status"] == "SKIP_RESUME_VALIDATED"
    assert plan["status"] == "COMPLETE"


def test_resume_rejects_legacy_artifacts_for_explicit_common10_contract(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.weathers = ("ClearNoon",)
    args.seeds = (3,)
    args.stationary_warmup_sec = 1.0
    args.stationary_tail_sec = 6.4
    args.export_horizons = module.parse_export_horizons("0.1:0.1:6.4")
    job_root = tmp_path / "output" / "town01" / "route_a" / "ClearNoon" / "seed_0003"
    (job_root / "episode").mkdir(parents=True)
    (job_root / "export").mkdir()
    (job_root / "preview").mkdir()
    (job_root / "episode" / "manifest.json").write_text(
        json.dumps({"status": "complete"}), encoding="utf-8"
    )
    (job_root / "export" / "manifest.json").write_text(
        json.dumps({"status": "validated", "future_horizons_s": [0.5, 1.0]}),
        encoding="utf-8",
    )
    (job_root / "preview" / "overview.png").write_bytes(b"png")
    (job_root / "preview" / "drive.gif").write_bytes(b"gif")

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["jobs"][0]["status"] == "SKIP"
    assert "do not match requested contract" in plan["jobs"][0]["reason"]
    assert plan["status"] == "BLOCKED"


def test_resume_rejects_default_collection_contract_mismatch_even_without_preview(
    tmp_path,
):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.weathers = ("ClearNoon",)
    args.seeds = (3,)
    job_root = tmp_path / "output" / "town01" / "route_a" / "ClearNoon" / "seed_0003"
    route_path = Path(catalog["routes"][0]["resolved_path"])
    episode_manifest = write_resume_artifacts(
        module,
        args,
        route_path,
        job_root,
        include_preview=False,
    )
    episode_manifest["capture_contract"]["target_speed_kmh"] = 30.0
    (job_root / "episode" / "manifest.json").write_text(
        json.dumps(episode_manifest), encoding="utf-8"
    )

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["jobs"][0]["status"] == "SKIP"
    assert "target_speed_kmh differs" in plan["jobs"][0]["reason"]
    assert plan["status"] == "BLOCKED"


def test_resume_rejects_route_collector_and_export_source_hash_drift(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)

    def make_case(name):
        case_root = tmp_path / name
        path = write_catalog(case_root / "catalog")
        catalog = module.load_catalog(path, manifest)
        args = arguments(module, case_root, path)
        args.weathers = ("ClearNoon",)
        args.seeds = (3,)
        collector = case_root / "collector.py"
        collector.write_text("# collector v1\n", encoding="utf-8")
        args.collector = collector
        job_root = (
            case_root
            / "output"
            / "town01"
            / "route_a"
            / "ClearNoon"
            / "seed_0003"
        )
        route_path = Path(catalog["routes"][0]["resolved_path"])
        write_resume_artifacts(module, args, route_path, job_root)
        return catalog, args, job_root, route_path

    catalog, args, _job_root, route_path = make_case("route-drift")
    route_path.write_text('{"changed":true}', encoding="utf-8")
    catalog["routes"][0]["sha256"] = module._sha256_file(route_path)
    plan = module.build_plan(args, manifest, [catalog])
    assert plan["jobs"][0]["status"] == "SKIP"
    assert "route SHA-256 differs" in plan["jobs"][0]["reason"]

    catalog, args, _job_root, _route_path = make_case("collector-drift")
    args.collector.write_text("# collector v2\n", encoding="utf-8")
    plan = module.build_plan(args, manifest, [catalog])
    assert plan["jobs"][0]["status"] == "SKIP"
    assert "collector SHA-256 differs" in plan["jobs"][0]["reason"]

    catalog, args, job_root, _route_path = make_case("source-drift")
    (job_root / "episode" / "states.jsonl").write_text(
        '{"changed":true}\n', encoding="utf-8"
    )
    plan = module.build_plan(args, manifest, [catalog])
    assert plan["jobs"][0]["status"] == "SKIP"
    assert "source hash differs for states.jsonl" in plan["jobs"][0]["reason"]


def test_resume_reruns_only_missing_renderer_output(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.weathers = ("ClearNoon",)
    args.seeds = (3,)
    job_root = tmp_path / "output" / "town01" / "route_a" / "ClearNoon" / "seed_0003"
    route_path = Path(catalog["routes"][0]["resolved_path"])
    write_resume_artifacts(
        module,
        args,
        route_path,
        job_root,
        include_preview=False,
    )

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["jobs"][0]["status"] == "PENDING"
    assert plan["status"] == "READY"
    calls = []

    def fake_run(command, _log):
        stage = Path(command[1]).stem
        calls.append(stage)
        assert stage == "render_carla_vad_expert"
        preview = job_root / "preview"
        preview.mkdir()
        (preview / "overview.png").write_bytes(b"png")
        (preview / "drive.gif").write_bytes(b"gif")
        return 0

    module.execute_job(plan["jobs"][0], run_logged=fake_run)

    assert calls == ["render_carla_vad_expert"]
    assert plan["jobs"][0]["status"] == "COMPLETE"


def test_resume_reuses_compatible_episode_when_export_is_missing(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)
    args.weathers = ("ClearNoon",)
    args.seeds = (3,)
    job_root = tmp_path / "output" / "town01" / "route_a" / "ClearNoon" / "seed_0003"
    route_path = Path(catalog["routes"][0]["resolved_path"])
    write_resume_artifacts(
        module,
        args,
        route_path,
        job_root,
        include_export=False,
        include_preview=False,
    )

    plan = module.build_plan(args, manifest, [catalog])

    assert plan["jobs"][0]["status"] == "PENDING"
    assert plan["jobs"][0]["reason"] is None
    assert plan["status"] == "READY"


def test_execute_job_calls_collector_exporter_renderer_in_order(tmp_path):
    module = load_module()
    root = tmp_path / "job"
    job = {
        "paths": {
            "root": str(root),
            "episode": str(root / "episode"),
            "export": str(root / "export"),
            "preview_png": str(root / "preview" / "overview.png"),
            "preview_gif": str(root / "preview" / "drive.gif"),
        },
        "commands": {
            "collector": ["python", "collector.py"],
            "exporter": ["python", "exporter.py"],
            "renderer": ["python", "renderer.py"],
        },
        "status": "RUNNING",
        "reason": None,
    }
    calls = []

    def fake_run(command, log):
        stage = Path(command[1]).stem
        calls.append(stage)
        if stage == "collector":
            (root / "episode").mkdir(parents=True)
            (root / "episode" / "manifest.json").write_text(
                json.dumps({"status": "complete"}), encoding="utf-8"
            )
        elif stage == "exporter":
            (root / "export").mkdir()
            (root / "export" / "manifest.json").write_text(
                json.dumps({"status": "validated"}), encoding="utf-8"
            )
        else:
            (root / "preview").mkdir()
            (root / "preview" / "overview.png").write_bytes(b"png")
            (root / "preview" / "drive.gif").write_bytes(b"gif")
        return 0

    module.execute_job(job, run_logged=fake_run)

    assert calls == ["collector", "exporter", "renderer"]
    assert job["status"] == "COMPLETE"


def test_execute_job_refreshes_preview_after_export_is_rebuilt(tmp_path):
    module = load_module()
    root = tmp_path / "job"
    episode = root / "episode"
    preview = root / "preview"
    episode.mkdir(parents=True)
    preview.mkdir()
    (episode / "manifest.json").write_text(
        json.dumps({"status": "complete"}), encoding="utf-8"
    )
    (preview / "overview.png").write_bytes(b"old-png")
    (preview / "drive.gif").write_bytes(b"old-gif")
    job = {
        "paths": {
            "root": str(root),
            "episode": str(episode),
            "export": str(root / "export"),
            "preview_png": str(preview / "overview.png"),
            "preview_gif": str(preview / "drive.gif"),
        },
        "commands": {
            "collector": ["python", "collector.py"],
            "exporter": ["python", "exporter.py"],
            "renderer": ["python", "renderer.py"],
        },
        "status": "RUNNING",
        "reason": None,
    }
    calls = []

    def fake_run(command, _log):
        stage = Path(command[1]).stem
        calls.append(stage)
        if stage == "exporter":
            (root / "export").mkdir()
            (root / "export" / "manifest.json").write_text(
                json.dumps({"status": "validated"}), encoding="utf-8"
            )
        elif stage == "renderer":
            (preview / "overview.png").write_bytes(b"new-png")
            (preview / "drive.gif").write_bytes(b"new-gif")
        else:
            pytest.fail("valid episode must not be recollected")
        return 0

    module.execute_job(job, run_logged=fake_run)

    assert calls == ["exporter", "renderer"]
    assert (preview / "overview.png").read_bytes() == b"new-png"


def test_default_mode_is_dry_run_and_output_root_is_required(tmp_path):
    module = load_module()
    catalog = write_catalog(tmp_path / "catalog")
    args = module.parse_args(["--catalog", str(catalog), "--output-root", str(tmp_path / "out")])
    assert args.execute is False
    assert args.map_load_settle_sec == pytest.approx(10.0)
    assert args.active_server_profile is None
    assert args.allow_map_load is False
    assert args.capture_hz == pytest.approx(10.0)
    assert args.scenarios == ("lane_follow", "straight", "left", "right")
    assert args.route_ids is None
    assert args.spawn_z_offset_m == pytest.approx(0.0)
    assert args.stationary_warmup_sec == pytest.approx(0.0)
    assert args.stationary_tail_sec == pytest.approx(0.0)
    assert args.export_horizons is None

    opted_in = module.parse_args(
        [
            "--catalog",
            str(catalog),
            "--output-root",
            str(tmp_path / "unsafe-out"),
            "--allow-map-load",
        ]
    )
    assert opted_in.allow_map_load is True

    lane_follow = module.parse_args(
        [
            "--catalog",
            str(catalog),
            "--output-root",
            str(tmp_path / "lane-follow-out"),
            "--scenarios",
            "lane_follow",
        ]
    )
    assert lane_follow.scenarios == ("lane_follow",)

    exact_routes = module.parse_args(
        [
            "--catalog",
            str(catalog),
            "--output-root",
            str(tmp_path / "exact-routes-out"),
            "--route-ids",
            "route_a,route_b",
        ]
    )
    assert exact_routes.route_ids == ("route_a", "route_b")

    for invalid in ("lane_follow,lane_follow", "curve", "left,unsafe/value"):
        with pytest.raises(SystemExit):
            module.parse_args(
                [
                    "--catalog",
                    str(catalog),
                    "--output-root",
                    str(tmp_path / "invalid-out"),
                    "--scenarios",
                    invalid,
                ]
            )

    for invalid_route_ids in ("route_a,route_a", "unsafe/value", "route_a,"):
        with pytest.raises(SystemExit):
            module.parse_args(
                [
                    "--catalog",
                    str(catalog),
                    "--output-root",
                    str(tmp_path / "invalid-route-out"),
                    "--route-ids",
                    invalid_route_ids,
                ]
            )

    with pytest.raises(SystemExit):
        module.parse_args(["--catalog", str(catalog)])


def test_common10_horizon_grid_and_capture_controls_pass_through(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = module.parse_args(
        [
            "--catalog",
            str(path),
            "--output-root",
            str(tmp_path / "output"),
            "--weathers",
            "ClearNoon",
            "--seeds",
            "0",
            "--max-duration-sec",
            "30",
            "--spawn-z-offset-m",
            "1.4",
            "--stationary-warmup-sec",
            "1.0",
            "--stationary-tail-sec",
            "6.4",
            "--export-horizons",
            "0.1:0.1:6.4",
        ]
    )

    assert len(args.export_horizons) == 64
    assert args.export_horizons[0] == pytest.approx(0.1)
    assert args.export_horizons[-1] == pytest.approx(6.4)
    plan = module.build_plan(args, manifest, [catalog])
    commands = plan["jobs"][0]["commands"]

    collector = commands["collector"]
    assert collector[collector.index("--spawn-z-offset-m") + 1] == "1.4"
    assert collector[collector.index("--stationary-warmup-sec") + 1] == "1.0"
    assert collector[collector.index("--stationary-tail-sec") + 1] == "6.4"
    exporter = commands["exporter"]
    horizons = exporter[exporter.index("--horizons") + 1].split(",")
    assert len(horizons) == 64
    assert horizons[0] == "0.1"
    assert horizons[-1] == "6.4"
    assert plan["matrix"]["export_horizons_s"][-1] == pytest.approx(6.4)

    close_values = (1.000000000000001, 1.000000000000002)
    serialized = module.format_export_horizons(close_values)
    assert tuple(float(value) for value in serialized.split(",")) == close_values


@pytest.mark.parametrize(
    "value",
    (
        "0.1:0.2:6.4",
        "0.1:0:6.4",
        "0.2:0.1:0.1",
        "0.1,0.1",
        "0.1,nan",
    ),
)
def test_export_horizons_fail_closed(value, tmp_path):
    module = load_module()
    catalog = write_catalog(tmp_path / "catalog")
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "--catalog",
                str(catalog),
                "--output-root",
                str(tmp_path / "out"),
                "--export-horizons",
                value,
            ]
        )


def test_stationary_reservation_must_leave_a_driving_tick(tmp_path):
    module = load_module()
    catalog = write_catalog(tmp_path / "catalog")
    with pytest.raises(SystemExit):
        module.parse_args(
            [
                "--catalog",
                str(catalog),
                "--output-root",
                str(tmp_path / "out"),
                "--physics-hz",
                "20",
                "--max-duration-sec",
                "1",
                "--stationary-warmup-sec",
                "0.5",
                "--stationary-tail-sec",
                "0.5",
            ]
        )


def test_duplicate_catalog_jobs_are_rejected(tmp_path):
    module = load_module()
    manifest, _ = module.load_manifest(MANIFEST)
    path = write_catalog(tmp_path / "catalog")
    catalog = module.load_catalog(path, manifest)
    args = arguments(module, tmp_path, path)

    with pytest.raises(module.SuiteError, match="duplicate job id"):
        module.build_plan(args, manifest, [catalog, catalog])


def test_prepare_server_never_loads_mismatched_map_by_default(tmp_path):
    module = load_module()

    class Actors:
        @staticmethod
        def filter(_pattern):
            return []

    class World:
        def __init__(self, name):
            self.name = name

        def get_map(self):
            return SimpleNamespace(name=self.name)

        @staticmethod
        def get_actors():
            return Actors()

    current = World("Town01")
    target = World("/Game/Carla/Maps/Town12/Town12")
    loads = []

    class Client:
        @staticmethod
        def set_timeout(value):
            assert value == 7.0

        @staticmethod
        def get_world():
            return current

        @staticmethod
        def load_world(name):
            loads.append(name)
            return target

    carla = SimpleNamespace(Client=lambda host, port: Client())
    args = SimpleNamespace(
        host="127.0.0.1",
        port=2100,
        server_timeout_sec=7.0,
        map_load_settle_sec=10.0,
        allow_map_load=False,
    )
    job = {
        "map_load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_map_name": "Town12",
    }
    sleeps = []

    with pytest.raises(module.SuiteError, match="automatic map loading is disabled"):
        module.prepare_server_for_job(
            job, args, carla_module=carla, sleep=sleeps.append
        )

    assert loads == []
    assert sleeps == [10.0]
    assert job["server_map_before"] == "Town01"
    assert job["server_map_required"].endswith("Town12/Town12")


def test_prepare_server_accepts_matching_external_map_without_loading():
    module = load_module()

    class Actors:
        @staticmethod
        def filter(_pattern):
            return []

    world = SimpleNamespace(
        get_map=lambda: SimpleNamespace(name="/Game/Carla/Maps/Town12/Town12"),
        get_actors=lambda: Actors(),
    )
    client = SimpleNamespace(
        set_timeout=lambda _value: None,
        get_world=lambda: world,
        load_world=lambda _name: pytest.fail("matching map must never be reloaded"),
    )
    args = SimpleNamespace(
        host="127.0.0.1",
        port=2100,
        server_timeout_sec=7.0,
        map_load_settle_sec=10.0,
        allow_map_load=False,
    )
    job = {
        "map_load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_map_name": "Town12",
    }
    sleeps = []

    loaded = module.prepare_server_for_job(
        job,
        args,
        carla_module=SimpleNamespace(Client=lambda _host, _port: client),
        sleep=sleeps.append,
    )

    assert loaded is False
    assert sleeps == [10.0]
    assert job["server_map_load_performed"] is False
    assert job["server_map_load_settle_sec"] == 0.0
    assert job["server_pre_read_settle_sec"] == 10.0
    assert job["server_map_load_allowed"] is False
    assert job["server_map_before"].endswith("Town12/Town12")
    assert job["server_map_after"].endswith("Town12/Town12")


def test_prepare_server_loads_only_with_explicit_unsafe_opt_in():
    module = load_module()

    class Actors:
        @staticmethod
        def filter(_pattern):
            return []

    class World:
        def __init__(self, name):
            self.name = name

        def get_map(self):
            return SimpleNamespace(name=self.name)

        @staticmethod
        def get_actors():
            return Actors()

    current = World("Town01")
    target = World("/Game/Carla/Maps/Town12/Town12")
    loads = []
    client = SimpleNamespace(
        set_timeout=lambda _value: None,
        get_world=lambda: current,
        load_world=lambda name: loads.append(name) or target,
    )
    carla = SimpleNamespace(Client=lambda _host, _port: client)
    args = SimpleNamespace(
        host="127.0.0.1",
        port=2100,
        server_timeout_sec=7.0,
        map_load_settle_sec=10.0,
        allow_map_load=True,
    )
    job = {
        "map_load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_map_name": "Town12",
    }
    sleeps = []

    loaded = module.prepare_server_for_job(
        job, args, carla_module=carla, sleep=sleeps.append
    )

    assert loaded is True
    assert loads == [job["map_load_name"]]
    assert sleeps == [10.0, 10.0]
    assert job["server_map_load_performed"] is True
    assert job["server_map_load_settle_sec"] == 10.0
    assert job["server_pre_read_settle_sec"] == 10.0
    assert job["server_map_load_allowed"] is True
    assert job["server_map_before"] == "Town01"
    assert job["server_map_after"].endswith("Town12/Town12")


def test_prepare_server_refuses_to_reload_a_world_with_dynamic_actors():
    module = load_module()
    actor = SimpleNamespace(
        id=5, type_id="vehicle.test", attributes={"role_name": "someone_else"}
    )

    class Actors:
        @staticmethod
        def filter(pattern):
            return [actor] if pattern == "vehicle.*" else []

    world = SimpleNamespace(
        get_map=lambda: SimpleNamespace(name="Town01"),
        get_actors=lambda: Actors(),
    )
    client = SimpleNamespace(
        set_timeout=lambda _value: None,
        get_world=lambda: world,
        load_world=lambda _name: pytest.fail("occupied world must not be reloaded"),
    )
    carla = SimpleNamespace(Client=lambda _host, _port: client)
    args = SimpleNamespace(
        host="127.0.0.1",
        port=2100,
        server_timeout_sec=7.0,
        map_load_settle_sec=10.0,
        allow_map_load=True,
    )
    job = {
        "map_load_name": "/Game/Carla/Maps/Town12/Town12",
        "canonical_map_name": "Town12",
    }

    with pytest.raises(module.SuiteError, match="already in use"):
        module.prepare_server_for_job(job, args, carla_module=carla, sleep=lambda _: None)
