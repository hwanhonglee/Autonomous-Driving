from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
CARLA_INTERFACE_SRC = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface/src"
)


@pytest.fixture()
def qos_modules(monkeypatch):
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]

    from autoware_carla_interface.modules.ros_publisher_manager import (
        ROSPublisherManager,
    )
    from autoware_carla_interface.modules.sensor_manager import SensorConfig

    return ROSPublisherManager, SensorConfig


def test_camera_image_and_info_publishers_use_independent_qos(qos_modules) -> None:
    manager_type, config_type = qos_modules
    created = []

    class FakeNode:
        def create_publisher(self, message_type, topic, qos):
            publisher = SimpleNamespace(topic=topic, qos=qos)
            created.append((message_type, topic, qos))
            return publisher

    manager = manager_type(
        FakeNode(), logger=SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)
    )
    config = config_type(
        sensor_id="CAM_FRONT",
        sensor_type="sensor.camera.rgb",
        carla_type="sensor.camera.rgb",
        frame_id="CAM_FRONT/camera_optical_link",
        topic_image="/sensing/camera/CAM_FRONT/image_raw",
        topic_info="/sensing/camera/CAM_FRONT/camera_info",
        qos_profile="reliable",
        image_qos_profile="best_effort",
        camera_info_qos_profile="reliable",
    )

    assert manager._create_camera_publishers(config)
    assert len(created) == 2
    assert created[0][2].reliability == manager.qos_profiles["best_effort"].reliability
    assert created[1][2].reliability == manager.qos_profiles["reliable"].reliability
    assert config.publisher.topic.endswith("/image_raw")
    assert config.publisher_info.topic.endswith("/camera_info")


def test_best_effort_depth_one_profile_is_bounded_and_keeps_legacy_depth(qos_modules) -> None:
    manager_type, _ = qos_modules
    manager = manager_type(SimpleNamespace())

    assert manager.qos_profiles["best_effort"].depth == 10
    assert manager.qos_profiles["best_effort_depth_1"].depth == 1
    assert (
        manager.qos_profiles["best_effort_depth_1"].reliability
        == manager.qos_profiles["best_effort"].reliability
    )


def test_legacy_camera_mapping_still_uses_one_shared_qos(qos_modules) -> None:
    manager_type, config_type = qos_modules
    created = []
    node = SimpleNamespace(
        create_publisher=lambda message_type, topic, qos: created.append(qos)
        or SimpleNamespace()
    )
    manager = manager_type(
        node, logger=SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)
    )
    config = config_type(
        sensor_id="CAM_FRONT",
        sensor_type="sensor.camera.rgb",
        carla_type="sensor.camera.rgb",
        frame_id="front",
        topic_image="/image_raw",
        topic_info="/camera_info",
        qos_profile="reliable",
    )

    assert manager._create_camera_publishers(config)
    assert created[0].reliability == created[1].reliability
    assert created[0].reliability == manager.qos_profiles["reliable"].reliability


def test_camera_qos_split_patch_is_persisted_after_runtime_timing_patch() -> None:
    patch = ROOT / "patches/autoware_carla_interface_camera_qos_split.patch"
    apply_script = ROOT / "scripts/e2e/apply_carla_camera_qos_split_patch.sh"
    patch_text = patch.read_text(encoding="utf-8")
    apply_text = apply_script.read_text(encoding="utf-8")
    assert "image_qos_profile" in patch_text
    assert '"best_effort_depth_1": self._create_best_effort_qos(depth=1)' in patch_text
    assert "simulator/autoware_carla_interface/README.md" in patch_text
    assert "falls back to the shared" in patch_text
    assert "git -C \"${repository}\" apply --check" in apply_text
    assert "falls back to the shared" in apply_text
    assert '"best_effort_depth_1": self._create_best_effort_qos(depth=1)' in apply_text
    for build_name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / build_name).read_text(encoding="utf-8")
        assert source.index("apply_carla_runtime_timing_patch.sh") < source.index(
            "apply_carla_camera_qos_split_patch.sh"
        )


def test_camera_qos_split_readme_documents_legacy_fallback() -> None:
    readme = (
        ROOT
        / "src/universe/autoware_universe/simulator/"
        "autoware_carla_interface/README.md"
    ).read_text(encoding="utf-8")

    assert "image_qos_profile: best_effort" in readme
    assert "camera_info_qos_profile: reliable" in readme
    assert "falls back to the shared" in readme
    assert "`qos_profile`, preserving existing sensor mappings" in readme
    assert "`best_effort_depth_1`" in readme
    assert "existing `best_effort` profile retains depth 10" in readme
