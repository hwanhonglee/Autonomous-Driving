# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())


def test_launch_only_rebuild_does_not_discover_full_source_graph() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        if relative_path.endswith("build_smart_mpc.sh"):
            marker = "# Keep the data-only project launch package"
        elif relative_path.endswith("build.sh"):
            marker = "# This package is launch/config data only"
        else:
            marker = "# Reconfigure only the project package"
        launch_build = source[source.index(marker) :]

        assert "--base-paths autonomy/autoware_e2e_vad_launch" in launch_build
        assert "--base-paths autonomy/src autonomy/autoware_e2e_vad_launch" not in launch_build
        assert launch_build.index("colcon --log-base autonomy/log build --build-base autonomy/build --install-base autonomy/install") < launch_build.index(
            "unset AUTOWARE_E2E_SKIP_INSTALL"
        )


def test_build_scripts_reload_completed_install_after_launch_package() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        assert source.rindex("unset AUTOWARE_E2E_SKIP_INSTALL") < source.rindex(
            "source tools/e2e/env.sh"
        )


def test_memory_heavy_full_build_limits_cmake_workers() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        assert 'MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL} ' in source
        assert "CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer" in source
