from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPAT = ROOT / "include" / "ros_humble_tf2_compat" / "tf2_ros"


def test_humble_forwarding_headers_cover_autoware_hpp_spellings() -> None:
    expected = {
        "buffer.hpp": "buffer.h",
        "create_timer_ros.hpp": "create_timer_ros.h",
        "static_transform_broadcaster.hpp": "static_transform_broadcaster.h",
        "transform_broadcaster.hpp": "transform_broadcaster.h",
        "transform_listener.hpp": "transform_listener.h",
    }

    for wrapper_name, humble_name in expected.items():
        wrapper = (COMPAT / wrapper_name).read_text(encoding="utf-8")
        assert f"#include <tf2_ros/{humble_name}>" in wrapper


def test_environment_enables_tf2_compat_only_for_humble_h_layout() -> None:
    environment = (ROOT / "scripts" / "e2e" / "env.sh").read_text(
        encoding="utf-8"
    )

    assert '[[ ! -f "${_tf2_ros_include_root}/tf2_ros/buffer.hpp" ]]' in environment
    assert '[[ -f "${_tf2_ros_include_root}/tf2_ros/buffer.h" ]]' in environment
    assert "include/ros_humble_tf2_compat" in environment
    assert "AUTOWARE_E2E_TF2_HEADER_COMPAT_DIR" in environment


def test_environment_exposes_local_tensorrt_runtime_libraries() -> None:
    environment = (ROOT / "scripts" / "e2e" / "env.sh").read_text(
        encoding="utf-8"
    )

    assert '[[ -d "${TENSORRT_ROOT}/lib64" ]]' in environment
    assert 'LD_LIBRARY_PATH="${TENSORRT_ROOT}/lib64' in environment
