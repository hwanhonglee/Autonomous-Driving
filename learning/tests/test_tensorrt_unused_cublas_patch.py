# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())


def test_patch_removes_only_the_unused_cublas_include() -> None:
    patch = (
        ROOT / "autonomy/patches" / "autoware_tensorrt_plugins_remove_unused_cublas.patch"
    ).read_text(encoding="utf-8")

    assert "-#include <cublas_v2.h>" in patch
    assert " #include <cuda_runtime.h>" in patch
    assert "+#include" not in patch


def test_apply_script_is_idempotent() -> None:
    source = (
        _hh_source_path(ROOT, "scripts") / "e2e" / "apply_tensorrt_unused_cublas_patch.sh"
    ).read_text(encoding="utf-8")

    assert "git -C \"${repository}\" apply --check" in source
    assert "if ! grep -q '#include <cublas_v2.h>'" in source


def test_build_entrypoints_apply_unused_cublas_patch() -> None:
    for relative_path in (
        "tools/e2e/build.sh",
        "tools/e2e/build_full.sh",
        "tools/e2e/build_smart_mpc.sh",
    ):
        source = (_hh_source_path(ROOT, relative_path)).read_text(encoding="utf-8")
        assert "tools/e2e/apply_tensorrt_unused_cublas_patch.sh" in source
