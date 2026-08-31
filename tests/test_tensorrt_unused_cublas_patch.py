from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_patch_removes_only_the_unused_cublas_include() -> None:
    patch = (
        ROOT / "patches" / "autoware_tensorrt_plugins_remove_unused_cublas.patch"
    ).read_text(encoding="utf-8")

    assert "-#include <cublas_v2.h>" in patch
    assert " #include <cuda_runtime.h>" in patch
    assert "+#include" not in patch


def test_apply_script_is_idempotent() -> None:
    source = (
        ROOT / "scripts" / "e2e" / "apply_tensorrt_unused_cublas_patch.sh"
    ).read_text(encoding="utf-8")

    assert "git -C \"${repository}\" apply --check" in source
    assert "if ! grep -q '#include <cublas_v2.h>'" in source


def test_build_entrypoints_apply_unused_cublas_patch() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "scripts/e2e/apply_tensorrt_unused_cublas_patch.sh" in source
