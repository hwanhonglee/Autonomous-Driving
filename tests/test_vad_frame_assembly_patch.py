from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_frame_patch_is_applied_after_precision_prerequisite() -> None:
    precision = "scripts/e2e/apply_vad_fp16_layer_norm_patch.sh"
    frame = "scripts/e2e/apply_vad_frame_assembly_patch.sh"
    for name in ("build.sh", "build_full.sh"):
        text = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert precision in text
        assert frame in text
        assert text.index(precision) < text.index(frame)


def test_frame_patch_apply_script_has_stable_applied_markers() -> None:
    text = (
        ROOT / "scripts/e2e/apply_vad_frame_assembly_patch.sh"
    ).read_text(encoding="utf-8")
    assert "class SynchronizedFrameBuffer" in text
    assert "find_nearest_image" in text
    assert "find_newest_exact_frame" in text
    assert "class VadRuntimeStartup" in text
    assert "parse_image_reliability" in text
    assert "image_queue_depth: 8" in text
    assert 'image_reliability: "best_effort"' in text
    assert "test_frame_assembly.cpp" in text
    assert "CombinesJitteredCamerasAroundFrontAnchor" in text
    assert "ToleranceOneExactPathPreservesFramesAcrossInterleavedFutureCallbacks" in text
    assert "test_runtime_startup.cpp" in text
    assert "ParsesSupportedImageReliabilityValues" in text


def test_vad_runtime_starts_model_before_subscribers_and_worker() -> None:
    source = (
        ROOT
        / "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/src/vad_node.cpp"
    ).read_text(encoding="utf-8")
    startup = source.index("VadRuntimeStartup::run")
    model = source.index("initialize_vad_model();", startup)
    image_subscribers = source.index("create_camera_image_subscribers(image_qos);", startup)
    state_subscriber = source.index(
        '"~/input/kinematic_state", reliable_qos', startup
    )
    worker = source.index("inference_worker_ = std::thread", startup)
    assert startup < model < image_subscribers < state_subscriber < worker


def test_vad_image_reliability_parameter_controls_image_qos() -> None:
    source = (
        ROOT
        / "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/src/vad_node.cpp"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter<std::string>("sync_params.image_reliability", "best_effort")' in source
    assert "parse_image_reliability(image_reliability_name)" in source
    assert "image_reliability == ImageReliability::RELIABLE" in source
    assert "rclcpp::ReliabilityPolicy::Reliable" in source
    assert "rclcpp::ReliabilityPolicy::BestEffort" in source


def test_frame_patch_carries_runtime_startup_regressions() -> None:
    patch = (
        ROOT / "patches/autoware_tensorrt_vad_frame_assembly.patch"
    ).read_text(encoding="utf-8")
    assert "src/runtime_startup.hpp" in patch
    assert "test/test_runtime_startup.cpp" in patch
    assert "DoesNotStartWorkerWhenSubscriberCreationFails" in patch
    assert "find_nearest_image" in patch
    assert "find_newest_exact_frame" in patch
    assert "CombinesJitteredCamerasAroundFrontAnchor" in patch
    assert "ToleranceOneExactPathPreservesFramesAcrossInterleavedFutureCallbacks" in patch
    assert "src/runtime_configuration.hpp" in patch
    assert "ParsesSupportedImageReliabilityValues" in patch
    assert 'image_reliability: "best_effort"' in patch
