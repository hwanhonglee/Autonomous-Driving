# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())


def test_patch_marks_only_tensorrt_vendor_headers_as_system() -> None:
    patch = (
        ROOT / "autonomy/patches" / "autoware_tensorrt_common_system_headers.patch"
    ).read_text(encoding="utf-8")

    assert "target_include_directories(${PROJECT_NAME} SYSTEM" in patch
    assert "${TENSORRT_INCLUDE_DIRS}" in patch
    assert "-Wno-error" not in patch
    assert "-Werror" in patch


def test_apply_script_is_idempotent_and_checks_patch() -> None:
    source = (
        _hh_source_path(ROOT, "scripts") / "e2e" / "apply_tensorrt_system_headers_patch.sh"
    ).read_text(encoding="utf-8")

    assert "git -C \"${repository}\" apply --check" in source
    assert "target_include_directories(${PROJECT_NAME} SYSTEM" in source


def test_all_build_entrypoints_apply_vendor_header_patch() -> None:
    for relative_path in (
        "tools/e2e/build.sh",
        "tools/e2e/build_full.sh",
        "tools/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        assert "tools/e2e/apply_tensorrt_system_headers_patch.sh" in source
