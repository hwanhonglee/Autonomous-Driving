from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_patch_prefers_the_pinned_tensorrt_headers_as_system_headers() -> None:
    patch = (
        ROOT / "patches" / "autoware_tensorrt_plugins_local_sdk_headers.patch"
    ).read_text(encoding="utf-8")

    assert "+  include_directories(SYSTEM ${TENSORRT_INCLUDE_DIRS})" in patch


def test_apply_script_checks_and_is_idempotent() -> None:
    source = (
        ROOT / "scripts" / "e2e" / "apply_tensorrt_local_sdk_headers_patch.sh"
    ).read_text(encoding="utf-8")

    assert "git -C \"${repository}\" apply --check" in source
    assert "include_directories(SYSTEM ${TENSORRT_INCLUDE_DIRS})" in source


def test_build_entrypoints_apply_local_sdk_header_patch() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "scripts/e2e/apply_tensorrt_local_sdk_headers_patch.sh" in source
