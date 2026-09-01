from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = (
    ROOT / "patches/autoware_mission_planner_lane_only_no_area.manifest.json"
)
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_mission_planner_lane_only_patch.sh"
BUILD_PROVENANCE_SCRIPT = ROOT / "scripts/e2e/mission_planner_build_provenance.py"
REPOSITORY_CONTRACTS = {
    "autoware_lanelet2_extension": {
        "source": ROOT / "src/core/autoware_lanelet2_extension",
        "workspace": "src/core/autoware_lanelet2_extension",
        "tag": "1.2.0",
        "commit": "4a3420d8cc19906e7739618f8a1686400f79b4ac",
        "patch": ROOT
        / "patches/autoware_lanelet2_extension_zero_length_resample.patch",
        "files": (
            "autoware_lanelet2_extension/lib/deprecated.cpp",
            "autoware_lanelet2_extension/lib/utilities.cpp",
            "autoware_lanelet2_extension/test/src/test_utilities.cpp",
        ),
    },
    "autoware_core": {
        "source": ROOT / "src/core/autoware_core",
        "workspace": "src/core/autoware_core",
        "tag": "1.9.0",
        "commit": "f25f83c632c1984ec276c894c41857d4abc0dad8",
        "patch": ROOT / "patches/autoware_route_handler_finite_path.patch",
        "files": (
            "planning/autoware_route_handler/src/route_handler.cpp",
            "planning/autoware_route_handler/test/test_route_handler.cpp",
        ),
    },
    "autoware_universe": {
        "source": ROOT / "src/universe/autoware_universe",
        "workspace": "src/universe/autoware_universe",
        "tag": "0.52.0",
        "commit": "6e477c645efec33f7909095eea684474e97f5e3d",
        "patch": ROOT
        / "patches/autoware_mission_planner_lane_only_no_area.patch",
        "files": (
            "planning/autoware_mission_planner_universe/"
            "src/lanelet2_plugins/default_planner.cpp",
            "planning/autoware_mission_planner_universe/"
            "test/test_lanelet2_plugins_default_planner.cpp",
        ),
    },
}
EXPECTED_NAMES = tuple(REPOSITORY_CONTRACTS)


def run(*args: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(args),
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )


def clone_repositories(tmp_path: Path) -> dict[str, Path]:
    checkouts: dict[str, Path] = {}
    for name, contract in REPOSITORY_CONTRACTS.items():
        checkout = tmp_path / name
        subprocess.run(
            [
                "git",
                "clone",
                "--shared",
                "--no-checkout",
                "-q",
                str(contract["source"]),
                str(checkout),
            ],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(checkout),
                "checkout",
                "-q",
                "--detach",
                str(contract["commit"]),
            ],
            check=True,
        )
        checkouts[name] = checkout
    return checkouts


def patch_environment(checkouts: dict[str, Path]) -> dict[str, str]:
    return {
        **os.environ,
        "AUTOWARE_E2E_LANELET2_EXTENSION_REPOSITORY": str(
            checkouts["autoware_lanelet2_extension"]
        ),
        "AUTOWARE_E2E_CORE_REPOSITORY": str(checkouts["autoware_core"]),
        "AUTOWARE_E2E_UNIVERSE_REPOSITORY": str(
            checkouts["autoware_universe"]
        ),
        "AUTOWARE_E2E_LANELET2_EXTENSION_PATCH_FILE": str(
            REPOSITORY_CONTRACTS["autoware_lanelet2_extension"]["patch"]
        ),
        "AUTOWARE_E2E_CORE_ROUTE_PATCH_FILE": str(
            REPOSITORY_CONTRACTS["autoware_core"]["patch"]
        ),
        "AUTOWARE_E2E_MISSION_PATCH_FILE": str(
            REPOSITORY_CONTRACTS["autoware_universe"]["patch"]
        ),
        "AUTOWARE_E2E_MISSION_PATCH_MANIFEST": str(MANIFEST),
    }


def test_three_repository_patch_scope_and_town05_contract_are_explicit() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 2
    repositories = manifest["repositories"]
    assert [item["name"] for item in repositories] == list(EXPECTED_NAMES)
    town05 = manifest["town05_regression_contract"]
    assert town05["map"] == "Town05_Opt"
    assert town05["lanelet2_map_sha256"] == (
        "45cdcfd7eca052969e84c645becdfcbda5b0d9f63002da49d6f22b553fab6ad0"
    )
    assert town05["lanelet_path"] == [22775, 28467, 1850]
    assert town05["duplicate_boundary"] == {
        "lanelet_id": 28467,
        "side": "left",
        "node_ids": [22667, 28413],
        "local_xyz": [-49.6552200317, 74.6567993164, 0.0554504394531],
    }
    assert town05["route_start_ros_pose"]["x"] == -47.711585998535156
    assert town05["route_goal_ros_pose"]["x"] == -80.08187103271484
    assert town05["failure_mode"] == (
        "duplicate consecutive left-boundary coordinate caused non-finite "
        "centerline and route cost"
    )
    workspace_map = ROOT / "data/maps/Town05_Opt_full/lanelet2_map.osm"
    if workspace_map.is_file():
        assert hashlib.sha256(workspace_map.read_bytes()).hexdigest() == (
            town05["lanelet2_map_sha256"]
        )

    for item in repositories:
        contract = REPOSITORY_CONTRACTS[item["name"]]
        assert item["workspace_path"] == contract["workspace"]
        assert item["git_tag"] == contract["tag"]
        assert item["git_commit"] == contract["commit"]
        assert ROOT / item["patch"] == contract["patch"]
        assert [source["path"] for source in item["files"]] == list(
            contract["files"]
        )
        for source in item["files"]:
            assert re.fullmatch(r"[0-9a-f]{64}", source["base_sha256"])
            assert re.fullmatch(r"[0-9a-f]{64}", source["patched_sha256"])

        patch = contract["patch"].read_text(encoding="utf-8")
        headers = [
            line for line in patch.splitlines() if line.startswith("diff --git a/")
        ]
        assert headers == [
            f"diff --git a/{path} b/{path}" for path in contract["files"]
        ]
        assert "GIT binary patch" not in patch
        assert "new file mode " not in patch
        assert "deleted file mode " not in patch

    extension_patch = REPOSITORY_CONTRACTS["autoware_lanelet2_extension"][
        "patch"
    ].read_text(encoding="utf-8")
    assert "accumulated_lengths.at(i) > accumulated_lengths.at(i - 1)" in (
        extension_patch
    )
    assert "Town05DuplicateBoundaryCenterlineIsFinite" in extension_patch
    assert "town05_turn_lanelet_id = 28467" in extension_patch

    core_patch = REPOSITORY_CONTRACTS["autoware_core"]["patch"].read_text(
        encoding="utf-8"
    )
    assert "has_finite_lanelet_path" in core_patch
    assert "if (path_lanelets.empty())" in core_patch
    assert "nonFiniteTown05RouteCostFailsClosed" in core_patch
    assert "path_lanelets->clear();" in core_patch

    universe_patch = REPOSITORY_CONTRACTS["autoware_universe"][
        "patch"
    ].read_text(encoding="utf-8")
    assert "param_.allow_area && !lanelet_map_ptr->areaLayer.empty()" in (
        universe_patch
    )
    assert "has_finite_lanelet_geometry" in universe_patch
    assert "planTown05TurnWithoutAreas" in universe_patch
    assert "rejectNonFiniteTown05RouteWithoutCrash" in universe_patch
    assert "autoware_vad_town_matrix.py" not in universe_patch

    runtime = manifest["runtime_source_contract"]
    source = ROOT / REPOSITORY_CONTRACTS["autoware_universe"]["workspace"]
    source /= runtime["path"]
    source_bytes = source.read_bytes()
    matches = re.findall(rb"source-contract=([0-9a-f]{64})", source_bytes)
    assert len(matches) == 1
    normalized = hashlib.sha256(
        source_bytes.replace(matches[0], b"0" * 64)
    ).hexdigest()
    assert matches[0].decode("ascii") == normalized
    assert runtime["normalized_sha256"] == normalized


def test_patch_set_replays_from_all_pinned_repositories_and_is_idempotent(
    tmp_path: Path,
) -> None:
    checkouts = clone_repositories(tmp_path)
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest_by_name = {item["name"]: item for item in manifest["repositories"]}

    for name, contract in REPOSITORY_CONTRACTS.items():
        forward = run(
            "git",
            "-C",
            str(checkouts[name]),
            "apply",
            "--check",
            str(contract["patch"]),
        )
        assert forward.returncode == 0, forward.stderr

    first = subprocess.run(
        ["bash", str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=patch_environment(checkouts),
    )
    assert first.returncode == 0, first.stderr
    assert "Applied Autoware mission planner lane-only fallback" in first.stdout

    for name, contract in REPOSITORY_CONTRACTS.items():
        changed = run("git", "-C", str(checkouts[name]), "diff", "--name-only")
        assert changed.returncode == 0, changed.stderr
        assert changed.stdout.splitlines() == list(contract["files"])
        reverse = run(
            "git",
            "-C",
            str(checkouts[name]),
            "apply",
            "--reverse",
            "--check",
            str(contract["patch"]),
        )
        assert reverse.returncode == 0, reverse.stderr
        expected_hashes = {
            item["path"]: item["patched_sha256"]
            for item in manifest_by_name[name]["files"]
        }
        for relative in contract["files"]:
            digest = hashlib.sha256((checkouts[name] / relative).read_bytes())
            assert digest.hexdigest() == expected_hashes[relative]

    second = subprocess.run(
        ["bash", str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=patch_environment(checkouts),
    )
    assert second.returncode == 0, second.stderr
    assert "patch set is already applied (APPLIED)" in second.stdout


def test_apply_script_rejects_patch_file_set_outside_manifest(
    tmp_path: Path,
) -> None:
    checkouts = clone_repositories(tmp_path)
    universe_patch = REPOSITORY_CONTRACTS["autoware_universe"]["patch"]
    adversarial_patch = tmp_path / "extra-file.patch"
    adversarial_patch.write_text(
        universe_patch.read_text(encoding="utf-8")
        + "\ndiff --git a/README.md b/README.md\n"
        + "--- a/README.md\n+++ b/README.md\n@@ -1 +1 @@\n-old\n+new\n",
        encoding="utf-8",
    )
    environment = patch_environment(checkouts)
    environment["AUTOWARE_E2E_MISSION_PATCH_FILE"] = str(adversarial_patch)
    completed = subprocess.run(
        ["bash", str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert completed.returncode != 0
    assert "autoware_universe patch file set differs" in completed.stderr
    for name, contract in REPOSITORY_CONTRACTS.items():
        changed = run("git", "-C", str(checkouts[name]), "diff", "--name-only")
        assert changed.stdout == "", (name, changed.stdout, contract)


def test_build_entrypoints_apply_and_build_all_patched_packages() -> None:
    marker = "scripts/e2e/apply_mission_planner_lane_only_patch.sh"
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert source.count(marker) == 1
        temporal = source.index("apply_vad_temporal_head_mode_patch.sh")
        mission = source.index("apply_mission_planner_lane_only_patch.sh")
        tensorrt = source.index("apply_tensorrt_system_headers_patch.sh")
        build = source.index("colcon build")
        provenance = source.index("mission_planner_build_provenance.py capture")
        assert temporal < mission < tensorrt < build < provenance
        assert "e2e_acquire_workspace_runtime_lock" in source

    minimal = (ROOT / "scripts/e2e/build.sh").read_text(encoding="utf-8")
    for package in (
        "autoware_lanelet2_extension",
        "autoware_route_handler",
        "autoware_mission_planner_universe",
    ):
        assert f"  {package}\n" in minimal
    full = (ROOT / "scripts/e2e/build_full.sh").read_text(encoding="utf-8")
    assert 'if [[ "${AUTOWARE_E2E_FULL_BUILD_RESUME:-0}" == "1" ]]' in full
    for package in (
        "autoware_lanelet2_extension",
        "autoware_route_handler",
        "autoware_mission_planner_universe",
    ):
        assert package in full


def test_apply_script_and_runtime_provenance_contracts_are_explicit() -> None:
    apply_source = APPLY_SCRIPT.read_text(encoding="utf-8")
    assert "schema_version must be 2" in apply_source
    assert "EXPECTED_NAMES" in apply_source
    assert "patch source SHA256 drift" in apply_source
    assert "patch file set differs from the exact manifest" in apply_source
    assert "forbidden binary/path operation" in apply_source
    assert '"apply", "--reverse", "--check"' in apply_source
    assert "e2e_acquire_workspace_runtime_lock" in apply_source
    syntax = run("bash", "-n", str(APPLY_SCRIPT))
    assert syntax.returncode == 0, syntax.stderr

    provenance = BUILD_PROVENANCE_SCRIPT.read_text(encoding="utf-8")
    assert "libautoware_lanelet2_extension_lib.so" in provenance
    assert "libautoware_route_handler.so" in provenance
    assert "libautoware_mission_planner_universe_lanelet2_plugins.so" in provenance
    assert "liblanelet2_map_loader_node.so" in provenance
    assert "runtime library predates its patched sources" in provenance
    assert "mission_planner_to_route_handler" in provenance
    assert "route_handler_to_lanelet2_extension" in provenance
    assert "map_loader_to_lanelet2_extension" in provenance
    assert "computed_normalized_sha" in provenance
    assert '["ros2", "pkg", "prefix", package]' in provenance
    assert '["ldd", str(library)]' in provenance
    assert "acquire_workspace_lock" in provenance

    matrix_runner = (
        ROOT / "scripts/e2e/run_autoware_vad_town_matrix.sh"
    ).read_text(encoding="utf-8")
    apply_index = matrix_runner.index("apply_mission_planner_lane_only_patch.sh")
    verify_index = matrix_runner.index("mission_planner_build_provenance.py verify")
    prepare_index = matrix_runner.index('prepare_args=(\n  prepare')
    assert apply_index < verify_index < prepare_index
    assert "e2e_acquire_workspace_runtime_lock" in matrix_runner
