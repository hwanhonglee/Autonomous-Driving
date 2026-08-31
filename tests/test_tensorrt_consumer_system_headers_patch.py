from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_patch_marks_all_direct_tensorrt_consumer_includes_system() -> None:
    patch = (
        ROOT / "patches" / "autoware_tensorrt_consumers_system_headers.patch"
    ).read_text(encoding="utf-8")

    assert "autoware_tensorrt_classifier/CMakeLists.txt" in patch
    assert "autoware_simpl_prediction/CMakeLists.txt" in patch
    assert "target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE" in patch
    assert patch.count("SYSTEM PUBLIC") >= 2


def test_every_runtime_build_applies_consumer_patch() -> None:
    for relative_path in (
        "scripts/e2e/build.sh",
        "scripts/e2e/build_full.sh",
        "scripts/e2e/build_smart_mpc.sh",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "apply_tensorrt_consumer_system_headers_patch.sh" in source
