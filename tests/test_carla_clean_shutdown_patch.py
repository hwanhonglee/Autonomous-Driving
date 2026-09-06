from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUTOWARE_UNIVERSE = ROOT / "src/universe/autoware_universe"
CARLA_INTERFACE_SRC = (
    AUTOWARE_UNIVERSE / "simulator/autoware_carla_interface/src"
)
PATCH = ROOT / "patches/autoware_carla_interface_clean_shutdown.patch"
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_carla_clean_shutdown_patch.sh"
PREREQUISITE_PATCHES = (
    "autoware_carla_interface_camera_fast_options.patch",
    "autoware_carla_interface_ros_sensor_frame.patch",
    "autoware_carla_interface_vehicle_status_contract.patch",
    "autoware_carla_interface_base_link_pose.patch",
    "autoware_carla_interface_base_link_route_contract.patch",
    "autoware_carla_interface_imu_source_timestamp.patch",
    "autoware_carla_interface_runtime_timing.patch",
    "autoware_carla_interface_camera_qos_split.patch",
)


@pytest.fixture()
def carla_ros_module(monkeypatch):
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]
    import autoware_carla_interface.carla_ros as module

    return module


def make_interface(module, events, warnings):
    interface = module.carla_ros2_interface.__new__(module.carla_ros2_interface)
    interface._publish_workers = {
        "camera": SimpleNamespace(stop=lambda: events.append("worker_stop"))
    }
    interface.ros_publisher_manager = SimpleNamespace(
        destroy_all_publishers=lambda: events.append("publishers_destroy")
    )
    interface.logger = SimpleNamespace(
        warning=warnings.append,
        debug=lambda message: events.append(("debug", message)),
    )
    return interface


def test_shutdown_stops_context_before_join_and_destroys_node_last(
    carla_ros_module, monkeypatch
) -> None:
    events = []
    warnings = []
    context = {"ok": True}

    class SpinThread:
        alive = True

        def is_alive(self):
            return self.alive

        def join(self, timeout):
            events.append(("join", timeout))
            assert not context["ok"]
            self.alive = False

    interface = make_interface(carla_ros_module, events, warnings)
    interface.spin_thread = SpinThread()
    interface.ros2_node = SimpleNamespace(
        destroy_node=lambda: events.append("node_destroy")
    )

    monkeypatch.setattr(carla_ros_module.rclpy, "ok", lambda: context["ok"])

    def shutdown():
        events.append("context_shutdown")
        context["ok"] = False

    monkeypatch.setattr(carla_ros_module.rclpy, "shutdown", shutdown)

    interface.shutdown()

    assert events == [
        "worker_stop",
        "publishers_destroy",
        "context_shutdown",
        ("join", 2.0),
        "node_destroy",
    ]
    assert interface._publish_workers == {}
    assert warnings == []


def test_shutdown_joins_an_executor_after_an_external_context_stop(
    carla_ros_module, monkeypatch
) -> None:
    events = []
    warnings = []

    class SpinThread:
        alive = True

        def is_alive(self):
            return self.alive

        def join(self, timeout):
            events.append(("join", timeout))
            self.alive = False

    interface = make_interface(carla_ros_module, events, warnings)
    interface.spin_thread = SpinThread()
    interface.ros2_node = SimpleNamespace(
        destroy_node=lambda: events.append("node_destroy")
    )
    monkeypatch.setattr(carla_ros_module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        carla_ros_module.rclpy,
        "shutdown",
        lambda: pytest.fail("shutdown called for an inactive context"),
    )

    interface.shutdown()

    assert events == [
        "worker_stop",
        "publishers_destroy",
        ("join", 2.0),
        "node_destroy",
    ]
    assert warnings == []


def test_patch_encodes_executor_shutdown_order() -> None:
    patch = PATCH.read_text(encoding="utf-8")
    postimage = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if (line.startswith("+") and not line.startswith("+++"))
        or line.startswith(" ")
    )

    # HH_260906 - Bind the portable patch to the executor release order that prevents timeout.
    context_stop = postimage.index("rclpy.shutdown()")
    spin_join = postimage.index("self.spin_thread.join(timeout=2.0)")
    node_destroy = postimage.index("self.ros2_node.destroy_node()")
    assert context_stop < spin_join < node_destroy
    assert "HH_260906 - Stop the ROS context first" in postimage
    assert "HH_260906 - Join only after shutdown" in postimage
    assert "HH_260906 - Destroy the node only after" in postimage


def test_apply_script_replays_after_the_pinned_carla_patch_chain(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    checkout = workspace / "src/universe/autoware_universe"
    checkout.parent.mkdir(parents=True)
    subprocess.run(
        [
            "git",
            "clone",
            "--quiet",
            "--no-hardlinks",
            str(AUTOWARE_UNIVERSE),
            str(checkout),
        ],
        check=True,
        capture_output=True,
    )

    for patch_name in PREREQUISITE_PATCHES:
        subprocess.run(
            ["git", "-C", str(checkout), "apply", str(ROOT / "patches" / patch_name)],
            check=True,
            capture_output=True,
        )

    copied_patch = workspace / "patches" / PATCH.name
    copied_patch.parent.mkdir(parents=True)
    copied_patch.write_bytes(PATCH.read_bytes())
    copied_script = workspace / "scripts/e2e" / APPLY_SCRIPT.name
    copied_script.parent.mkdir(parents=True)
    shutil.copy2(APPLY_SCRIPT, copied_script)

    first = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "Applied CARLA clean shutdown patch." in first.stdout

    second = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "already applied" in second.stdout
    subprocess.run(
        ["git", "-C", str(checkout), "diff", "--check"],
        check=True,
        capture_output=True,
    )


def test_build_entrypoints_apply_clean_shutdown_patch_last() -> None:
    marker = "scripts/e2e/apply_carla_clean_shutdown_patch.sh"
    predecessor = "scripts/e2e/apply_carla_camera_qos_split_patch.sh"
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert source.count(marker) == 1
        assert source.index(predecessor) < source.index(marker)
        assert "autoware_carla_interface" in source
