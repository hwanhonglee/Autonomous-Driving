from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_patch_marks_only_tensorrt_vendor_headers_as_system() -> None:
    patch = (
        ROOT / "patches" / "autoware_tensorrt_common_system_headers.patch"
    ).read_text(encoding="utf-8")

    assert "target_include_directories(${PROJECT_NAME} SYSTEM" in patch
    assert "${TENSORRT_INCLUDE_DIRS}" in patch
    assert "-Wno-error" not in patch
    assert "-Werror" in patch


def test_apply_script_is_idempotent_and_checks_patch() -> None:
    source = (
        ROOT / "scripts" / "e2e" / "apply_tensorrt_system_headers_patch.sh"
    ).read_text(encoding="utf-8")

    assert "git -C \"${repository}\" apply --check" in source
    assert "target_include_directories(${PROJECT_NAME} SYSTEM" in source


def test_all_build_entrypoints_apply_vendor_header_patch() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "scripts/e2e/apply_tensorrt_system_headers_patch.sh" in source
