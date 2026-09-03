from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "e2e" / "bootstrap_preflight.sh"
GUIDE = ROOT / "docs" / "BEGINNER_QUICKSTART_KO.md"
DOCTOR = ROOT / "scripts" / "e2e" / "doctor.sh"


def _isolated_preflight_env() -> dict[str, str]:
    environment = os.environ.copy()
    for name in (
        "CARLA_ROOT",
        "AUTOWARE_E2E_DATA_PATH",
        "AUTOWARE_E2E_CUDA_ROOT",
        "AUTOWARE_E2E_TENSORRT_ROOT",
        "AUTOWARE_E2E_SPCONV_ROOT",
        "AUTOWARE_E2E_ACADOS_ROOT",
        "AUTOWARE_E2E_PREFLIGHT_MAP_PATH",
    ):
        environment.pop(name, None)
    return environment


def _fresh_clone_fixture(tmp_path: Path) -> Path:
    root = tmp_path / "Autonomous-Driving"
    script = root / "scripts" / "e2e" / SCRIPT.name
    script.parent.mkdir(parents=True)
    shutil.copy2(SCRIPT, script)
    (root / "docs").mkdir()
    (root / "autoware.repos").write_text(
        """repositories:
  core/example_one:
    type: git
    url: https://example.invalid/one.git
    version: v1
  universe/example_two:
    type: git
    url: https://example.invalid/two.git
    version: v2
""",
        encoding="utf-8",
    )
    return root


def _snapshot(root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        result[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def test_bootstrap_preflight_has_valid_bash_syntax_and_is_executable() -> None:
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)
    assert os.access(SCRIPT, os.X_OK)


def test_default_mode_reports_every_fresh_clone_gap_without_mutation(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    before = _snapshot(root)

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "source.layout" in completed.stdout
    assert "0/2" in completed.stdout
    assert "carla.runtime" in completed.stdout
    assert "model.vad_v0.1" in completed.stdout
    assert "map.full_bundle" in completed.stdout
    assert "vendor.cuda" in completed.stdout
    assert "build.install_space" in completed.stdout
    assert "status: NOT_READY" in completed.stdout
    assert "mutation: NONE" in completed.stdout
    assert "vcs import src < autoware.repos" in completed.stdout
    assert _snapshot(root) == before


def test_source_revision_check_rejects_a_head_not_named_by_manifest(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    for relative_path, tag in (
        ("core/example_one", "v1"),
        ("universe/example_two", "wrong-tag"),
    ):
        checkout = root / "src" / relative_path
        checkout.mkdir(parents=True)
        subprocess.run(["git", "init", "-q", str(checkout)], check=True)
        subprocess.run(
            ["git", "-C", str(checkout), "config", "user.name", "Test"], check=True
        )
        subprocess.run(
            ["git", "-C", str(checkout), "config", "user.email", "test@example.com"],
            check=True,
        )
        (checkout / "README").write_text("fixture\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(checkout), "add", "README"], check=True)
        subprocess.run(
            ["git", "-C", str(checkout), "commit", "-qm", "fixture"], check=True
        )
        subprocess.run(["git", "-C", str(checkout), "tag", tag], check=True)

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "source.layout" in completed.stdout
    assert "2/2" in completed.stdout
    assert "source.revision" in completed.stdout
    assert "HEAD 1/2" in completed.stdout
    assert "universe/example_two" in completed.stdout


def test_dirty_checkout_cannot_claim_commit_reproducibility(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "config", "user.name", "Test"], check=True)
    subprocess.run(
        ["git", "-C", str(root), "config", "user.email", "test@example.com"], check=True
    )
    subprocess.run(["git", "-C", str(root), "add", "."], check=True)
    subprocess.run(["git", "-C", str(root), "commit", "-qm", "fixture"], check=True)
    current_branch = subprocess.run(
        ["git", "-C", str(root), "branch", "--show-current"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    (root / "untracked-drift.txt").write_text("drift\n", encoding="utf-8")
    environment = _isolated_preflight_env()
    environment["AUTOWARE_E2E_PREFLIGHT_EXPECTED_BRANCH"] = current_branch

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0
    worktree_line = next(
        line for line in completed.stdout.splitlines() if "git.worktree " in line
    )
    assert worktree_line.startswith("[BLOCK]")
    assert "untracked-drift.txt" in completed.stdout


def test_git_status_probe_failure_is_a_blocker(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "config", "user.name", "Test"], check=True)
    subprocess.run(
        ["git", "-C", str(root), "config", "user.email", "test@example.com"], check=True
    )
    subprocess.run(["git", "-C", str(root), "add", "."], check=True)
    subprocess.run(["git", "-C", str(root), "commit", "-qm", "fixture"], check=True)
    current_branch = subprocess.run(
        ["git", "-C", str(root), "branch", "--show-current"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        "#!/usr/bin/env bash\n"
        "for argument in \"$@\"; do\n"
        "  [[ \"$argument\" == status ]] && exit 42\n"
        "done\n"
        "exec /usr/bin/git \"$@\"\n",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    environment = _isolated_preflight_env()
    environment["AUTOWARE_E2E_PREFLIGHT_EXPECTED_BRANCH"] = current_branch
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0
    worktree_line = next(
        line for line in completed.stdout.splitlines() if "git.worktree " in line
    )
    assert worktree_line.startswith("[BLOCK]")
    assert "status 검사 실패(exit=42)" in worktree_line


def test_model_presence_cannot_hide_a_checksum_mismatch(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    model_root = root / "data/ml_models/vad/v0.1"
    model_root.mkdir(parents=True)
    for name in (
        "vad-carla-tiny_backbone.onnx",
        "vad-carla-tiny_head.onnx",
        "vad-carla-tiny_head_no_prev.onnx",
        "vad-carla-tiny.param.json",
    ):
        (model_root / name).write_bytes(b"not-the-pinned-model\n")

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "model.vad_v0.1" in completed.stdout
    assert "4/4 파일 확인" in completed.stdout
    assert "model.integrity" in completed.stdout
    assert "4/4 SHA-256 mismatch" in completed.stdout


def test_map_bundle_assets_are_checked_against_recorded_hashes(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    map_root = root / "external-map"
    map_root.mkdir()
    assets = {
        "lanelet2_map.osm": b"lanelet fixture\n",
        "pointcloud_map.pcd": b"pointcloud fixture\n",
    }
    for name, content in assets.items():
        (map_root / name).write_bytes(content)
    (map_root / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    (map_root / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "fixture",
                "canonical_carla_map": "/Game/Carla/Maps/TownFixture",
                "status": "fixture_only",
                "projector_type": "Local",
                "carla_to_map_transform": {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "bundle_sources": {
                    "lanelet2_map": {
                        "sha256": hashlib.sha256(assets["lanelet2_map.osm"]).hexdigest(),
                        "size_bytes": len(assets["lanelet2_map.osm"]),
                    },
                    "pointcloud_map": {
                        "sha256": hashlib.sha256(assets["pointcloud_map.pcd"]).hexdigest(),
                        "size_bytes": len(assets["pointcloud_map.pcd"]),
                    },
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            "bash",
            str(root / "scripts/e2e/bootstrap_preflight.sh"),
            "--map",
            str(map_root),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "map.full_bundle" in completed.stdout
    assert "4/4 파일 확인" in completed.stdout
    assert "map.integrity" in completed.stdout
    assert "profile=fixture status=fixture_only assets=2/2" in completed.stdout
    assert "map.admission" in completed.stdout
    assert "알 수 없는 status=fixture_only" in completed.stdout


def test_reference_only_map_cannot_be_admitted_for_runtime(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    map_root = root / "reference-map"
    map_root.mkdir()
    assets = {
        "lanelet2_map.osm": b"reference lanelet\n",
        "pointcloud_map.pcd": b"reference cloud\n",
    }
    for name, content in assets.items():
        (map_root / name).write_bytes(content)
    (map_root / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    (map_root / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "reference",
                "canonical_carla_map": "/Game/Carla/Maps/Reference",
                "status": "reference_only_xy_unvalidated",
                "projector_type": "Local",
                "carla_to_map_transform": {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "bundle_sources": {
                    "lanelet2_map": {
                        "sha256": hashlib.sha256(assets["lanelet2_map.osm"]).hexdigest(),
                        "size_bytes": len(assets["lanelet2_map.osm"]),
                    },
                    "pointcloud_map": {
                        "sha256": hashlib.sha256(assets["pointcloud_map.pcd"]).hexdigest(),
                        "size_bytes": len(assets["pointcloud_map.pcd"]),
                    },
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            "bash",
            str(root / "scripts/e2e/bootstrap_preflight.sh"),
            "--map",
            str(map_root),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "map.integrity" in completed.stdout
    assert "map.admission" in completed.stdout
    assert "참고용 bundle은 runtime map으로 사용 금지" in completed.stdout


def test_custom_game_level_passes_integrity_but_single_anchor_stays_blocked(
    tmp_path: Path,
) -> None:
    root = _fresh_clone_fixture(tmp_path)
    map_root = root / "custom-map"
    map_root.mkdir()
    assets = {
        "lanelet2_map.osm": b"custom lanelet\n",
        "pointcloud_map.pcd": b"custom cloud\n",
    }
    for name, content in assets.items():
        (map_root / name).write_bytes(content)
    (map_root / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    (map_root / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "woraksan_simulation_current",
                "canonical_carla_map": "/Game/map_package/Maps/Woraksan/Level",
                "status": "ready_with_single_anchor_alignment",
                "projector_type": "Local",
                "carla_to_map_transform": {
                    "x_m": 1.0,
                    "y_m": 2.0,
                    "z_m": -33.0,
                    "yaw_rad": -0.04,
                },
                "bundle_sources": {
                    "lanelet2_map": {
                        "sha256": hashlib.sha256(assets["lanelet2_map.osm"]).hexdigest(),
                        "size_bytes": len(assets["lanelet2_map.osm"]),
                    },
                    "pointcloud_map": {
                        "sha256": hashlib.sha256(assets["pointcloud_map.pcd"]).hexdigest(),
                        "size_bytes": len(assets["pointcloud_map.pcd"]),
                    },
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh"), "--map", str(map_root)],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    integrity_line = next(
        line for line in completed.stdout.splitlines() if "map.integrity" in line
    )
    admission_line = next(
        line for line in completed.stdout.splitlines() if "map.admission" in line
    )
    assert integrity_line.startswith("[PASS]")
    assert admission_line.startswith("[BLOCK]")
    assert "single-anchor custom map" in admission_line


def test_zero_byte_engine_files_are_not_accepted_as_cache(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    model_root = root / "data/ml_models/vad/v0.1"
    model_root.mkdir(parents=True)
    for name in (
        "vad-carla-tiny_backbone.engine",
        "vad-carla-tiny_head.engine",
        "vad-carla-tiny_head_no_prev.engine",
    ):
        (model_root / name).touch()

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "model.engine_cache" in completed.stdout
    assert "0/3 non-empty" in completed.stdout


def test_strict_mode_fails_after_printing_all_blockers(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh"), "--strict"],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 1
    assert "[BLOCK]" in completed.stdout
    assert "mutation: NONE" in completed.stdout
    assert "scripts/e2e/doctor.sh" in completed.stdout


def test_preflight_detects_unmaterialized_latest_visual_lfs_pointers(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    png = (
        root
        / "docs/assets/validation/2026-09-02-runtime-control-campaign-v1"
        / "example/screen.png"
    )
    gif = png.with_name("drive.gif")
    pointer = (
        "version https://git-lfs.github.com/spec/v1\n"
        "oid sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\n"
        "size 12345\n"
    )
    png.parent.mkdir(parents=True)
    png.write_text(pointer, encoding="utf-8")
    gif.write_text(pointer, encoding="utf-8")
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "add", "docs"], check=True)

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh")],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "대표 PNG/GIF가 pointer임" in completed.stdout
    assert "2026-09-02-runtime-control-campaign-v1" in completed.stdout


def test_missing_option_value_is_usage_error() -> None:
    completed = subprocess.run(
        ["bash", str(SCRIPT), "--carla-root"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "경로가 필요" in completed.stderr


def test_another_option_cannot_be_consumed_as_a_path_value() -> None:
    completed = subprocess.run(
        ["bash", str(SCRIPT), "--map", "--strict"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "--map에는 경로가 필요" in completed.stderr


def test_strict_mode_blocks_a_non_reproduction_branch() -> None:
    environment = _isolated_preflight_env()
    environment["AUTOWARE_E2E_PREFLIGHT_EXPECTED_BRANCH"] = "not-this-branch"
    completed = subprocess.run(
        ["bash", str(SCRIPT), "--strict"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 1
    branch_line = next(
        line for line in completed.stdout.splitlines() if "git.branch" in line
    )
    assert branch_line.startswith("[BLOCK]")


def test_map_without_runtime_transform_fails_integrity(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    map_root = root / "no-transform-map"
    map_root.mkdir()
    assets = {
        "lanelet2_map.osm": b"lanelet fixture\n",
        "pointcloud_map.pcd": b"pointcloud fixture\n",
    }
    for name, content in assets.items():
        (map_root / name).write_bytes(content)
    (map_root / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    (map_root / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "town01",
                "canonical_carla_map": "/Game/Carla/Maps/Town01",
                "status": "full_map_structurally_ready_not_vad_validated",
                "projector_type": "Local",
                "bundle_sources": {
                    "lanelet2_map": {
                        "sha256": hashlib.sha256(assets["lanelet2_map.osm"]).hexdigest(),
                        "size_bytes": len(assets["lanelet2_map.osm"]),
                    },
                    "pointcloud_map": {
                        "sha256": hashlib.sha256(assets["pointcloud_map.pcd"]).hexdigest(),
                        "size_bytes": len(assets["pointcloud_map.pcd"]),
                    },
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh"), "--map", str(map_root)],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "map.integrity" in completed.stdout
    assert "carla_to_map_transform is missing" in completed.stdout


def test_game_root_alone_is_not_a_canonical_level_identity(tmp_path: Path) -> None:
    root = _fresh_clone_fixture(tmp_path)
    map_root = root / "root-only-map"
    map_root.mkdir()
    assets = {
        "lanelet2_map.osm": b"lanelet fixture\n",
        "pointcloud_map.pcd": b"pointcloud fixture\n",
    }
    for name, content in assets.items():
        (map_root / name).write_bytes(content)
    (map_root / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    (map_root / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "town01",
                "canonical_carla_map": "/Game/",
                "status": "full_map_structurally_ready_not_vad_validated",
                "projector_type": "Local",
                "carla_to_map_transform": {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "bundle_sources": {
                    "lanelet2_map": {
                        "sha256": hashlib.sha256(assets["lanelet2_map.osm"]).hexdigest(),
                        "size_bytes": len(assets["lanelet2_map.osm"]),
                    },
                    "pointcloud_map": {
                        "sha256": hashlib.sha256(assets["pointcloud_map.pcd"]).hexdigest(),
                        "size_bytes": len(assets["pointcloud_map.pcd"]),
                    },
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        ["bash", str(root / "scripts/e2e/bootstrap_preflight.sh"), "--map", str(map_root)],
        check=False,
        capture_output=True,
        text=True,
        env=_isolated_preflight_env(),
    )

    assert completed.returncode == 0
    assert "map.integrity" in completed.stdout
    assert "must name a non-empty /Game/ level" in completed.stdout


def test_preflight_is_not_a_hidden_installer() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "READ-ONLY" in source
    assert "mutation: NONE" in source
    assert "source scripts/e2e/env.sh" not in source
    assert "setup-dev-env.sh를 바로 실행하지 마세요" in source
    assert "2026-09-02-runtime-control-campaign-v1/**" in source
    assert "rm -rf" not in source
    assert "pkill" not in source
    assert "timeout 10s nvidia-smi" in source
    assert "source.revision" in source
    assert "model.integrity" in source
    assert "map.integrity" in source
    assert "map.admission" in source
    assert '[[ -s "${engine}" ]]' in source
    assert "importlib.util.find_spec" in source


def test_beginner_guide_states_portability_and_asset_boundaries() -> None:
    guide = GUIDE.read_text(encoding="utf-8")

    assert "어떤 PC에서든 clone 한 번으로 즉시 자율주행" in guide
    assert "native Ubuntu 22.04 x86_64" in guide
    assert "GIT_LFS_SKIP_SMUDGE=1" in guide
    assert "2026-09-02-runtime-control-campaign-v1/**" in guide
    assert "vcs import src < autoware.repos" in guide
    assert "source.revision" in guide
    assert "10718787ba6e28f038a0cb29ff99cc627b5abfd2" in guide
    assert 'export PATH="$HOME/.local/bin:$PATH"' in guide
    assert "-e cuda_install_drivers=false" in guide
    assert "fresh-host 절차에는 사용하지 않는다" in guide
    assert "setup-dev-env.sh" in guide
    assert "CARLA 0.9.15" in guide
    assert "carla-0.9.15-py3.10-linux-x86_64.egg" in guide
    assert "scripts/e2e/download_vad_models.sh" in guide
    assert "prepare_packaged_town_full_maps.py --map town01" in guide
    assert "map.full_bundle 3/4" in guide
    assert "python3-open3d" in guide
    assert "pcl-tools" in guide
    assert "ffmpeg x11-utils" in guide
    assert "AUTOWARE_E2E_TOWN_LANELET_ROOT" in guide
    assert "scripts/e2e/build_full.sh" in guide
    assert "python3 -m pytest -q tests" in guide
    assert "python3 -m pytest -q autoware_e2e_vad_launch/test" in guide
    assert "python3 -m pytest -q\n" not in guide
    assert "--scenario straight" in guide
    assert "--scenario left" in guide
    assert "--min-distance 170" in guide
    assert "--preferred-distance 210" in guide
    assert guide.count("--max-traces 20000") >= 2
    assert "--speed-30kph" in guide
    assert guide.count("--ready-timeout 600") >= 2
    assert "1280×720 이상, 1920×1080 이하" in guide
    assert "real_vehicle_ready=false" in guide
    assert guide.count("timeout 10s nvidia-smi") >= 2


def test_runtime_doctor_bounds_the_driver_probe() -> None:
    source = DOCTOR.read_text(encoding="utf-8")

    assert "timeout 10s nvidia-smi" in source
    assert "gpu_status} -eq 124" in source
