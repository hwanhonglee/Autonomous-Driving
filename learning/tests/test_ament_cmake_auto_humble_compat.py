# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
COMPAT = (
    ROOT
    / "autonomy/cmake"
    / "ament_cmake_auto_humble_compat"
    / "share"
    / "ament_cmake_auto"
    / "cmake"
)


def test_compat_macro_consumes_scoped_header_option() -> None:
    macro = (COMPAT / "ament_auto_package.cmake").read_text(encoding="utf-8")

    assert '"INSTALL_TO_PATH;USE_SCOPED_HEADER_INSTALL_DIR"' in macro
    assert "_ARG_AMENT_AUTO_PACKAGE_USE_SCOPED_HEADER_INSTALL_DIR" in macro
    assert 'set(_include_destination "include/${PROJECT_NAME}")' in macro


def test_compat_config_uses_humble_macros_except_patched_package_macro() -> None:
    extras = (COMPAT / "ament_cmake_auto-extras.cmake").read_text(encoding="utf-8")

    assert "ament_auto_add_library.cmake" in extras
    assert 'include("${CMAKE_CURRENT_LIST_DIR}/ament_auto_package.cmake")' in extras


def test_environment_enables_compat_only_for_an_old_system_macro() -> None:
    environment = (_hh_source_path(ROOT, "scripts") / "e2e" / "env.sh").read_text(encoding="utf-8")

    assert '! grep -q "USE_SCOPED_HEADER_INSTALL_DIR"' in environment
    assert "cmake/ament_cmake_auto_humble_compat" in environment
    assert "AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR" in environment


def test_build_scripts_override_any_stale_cmake_cache() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        assert '-Dament_cmake_auto_DIR="${AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR}"' in source
