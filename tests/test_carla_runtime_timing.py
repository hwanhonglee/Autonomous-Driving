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
def runtime_modules(monkeypatch):
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]

    import autoware_carla_interface.carla_autoware as carla_autoware
    from autoware_carla_interface.modules.runtime_timing import RuntimeTimingMonitor

    return carla_autoware, RuntimeTimingMonitor


class MutableClock:
    def __init__(self):
        self.now_ns = 0

    def __call__(self):
        return self.now_ns


class RecordingTimingMonitor:
    def __init__(self):
        self.next_token = 0
        self.calls = []

    def start(self):
        self.next_token += 1
        return self.next_token

    def warn_if_slow(self, stage, started_ns, **fields):
        self.calls.append((stage, started_ns, fields))


def test_runtime_timing_warns_only_above_50_ms(runtime_modules) -> None:
    _, monitor_type = runtime_modules
    clock = MutableClock()
    warnings = []
    monitor = monitor_type(
        SimpleNamespace(warning=warnings.append),
        threshold_sec=0.050,
        clock_ns=clock,
    )

    clock.now_ns = 50_000_000
    assert monitor.warn_if_slow("world_tick", 0) == 50_000_000
    assert warnings == []

    clock.now_ns = 50_000_001
    assert monitor.warn_if_slow("world_tick", 0) == 50_000_001
    assert len(warnings) == 1
    assert "runtime_timing stage=world_tick" in warnings[0]
    assert "duration_ms=50.000" in warnings[0]
    assert "monotonic_ns=50000001" in warnings[0]


def test_runtime_timing_rate_limits_each_camera_independently(runtime_modules) -> None:
    _, monitor_type = runtime_modules
    clock = MutableClock()
    warnings = []
    monitor = monitor_type(
        SimpleNamespace(warning=warnings.append),
        threshold_sec=0.050,
        log_interval_sec=5.0,
        clock_ns=clock,
    )

    clock.now_ns = 60_000_000
    monitor.warn_if_slow("camera_image_publish", 0, camera="CAM_FRONT")
    clock.now_ns = 120_000_000
    monitor.warn_if_slow(
        "camera_image_publish", 60_000_000, camera="CAM_FRONT"
    )
    clock.now_ns = 180_000_000
    monitor.warn_if_slow(
        "camera_image_publish", 120_000_000, camera="CAM_BACK"
    )
    clock.now_ns = 5_180_000_001
    monitor.warn_if_slow(
        "camera_image_publish", 5_120_000_000, camera="CAM_FRONT"
    )

    assert len(warnings) == 3
    assert "camera=CAM_FRONT" in warnings[0]
    assert "camera=CAM_BACK" in warnings[1]
    assert "camera=CAM_FRONT" in warnings[2]
    assert "suppressed=1" in warnings[2]


def test_runtime_timing_logging_failure_does_not_escape(runtime_modules) -> None:
    _, monitor_type = runtime_modules
    clock = MutableClock()

    def fail(_message):
        raise RuntimeError("logger unavailable")

    monitor = monitor_type(
        SimpleNamespace(warning=fail), threshold_sec=0.050, clock_ns=clock
    )
    clock.now_ns = 60_000_000

    assert monitor.warn_if_slow("world_tick", 0) == 60_000_000


def test_sensor_loop_times_sensor_wait_and_world_tick_without_reordering(
    runtime_modules, monkeypatch
) -> None:
    carla_autoware, _ = runtime_modules
    monitor = RecordingTimingMonitor()
    events = []
    control = object()
    world = SimpleNamespace(tick=lambda: events.append("world_tick"))
    loop = carla_autoware.SensorLoop(timing_monitor=monitor)
    loop.sensor = lambda: events.append("sensor") or control
    loop.ego_actor = SimpleNamespace(
        apply_control=lambda value: events.append(("apply_control", value))
    )
    loop.running = True

    monkeypatch.setattr(
        carla_autoware.GameTime,
        "on_carla_tick",
        lambda timestamp: events.append(("game_time", timestamp.elapsed_seconds)),
    )
    monkeypatch.setattr(
        carla_autoware.CarlaDataProvider,
        "on_carla_tick",
        lambda: events.append("data_provider"),
    )
    monkeypatch.setattr(
        carla_autoware.CarlaDataProvider, "get_world", lambda: world
    )

    timestamp = SimpleNamespace(elapsed_seconds=1.25)
    loop._tick_sensor(timestamp)

    assert events == [
        ("game_time", 1.25),
        "data_provider",
        "sensor",
        ("apply_control", control),
        "world_tick",
    ]
    assert [call[0] for call in monitor.calls] == ["sensor_wait", "world_tick"]
    assert all(call[2] == {"sim_time_sec": 1.25} for call in monitor.calls)


def test_runtime_patch_is_bound_to_both_build_entrypoints() -> None:
    for build_name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / build_name).read_text(encoding="utf-8")
        imu_patch = source.index("apply_carla_imu_source_timestamp_patch.sh")
        timing_patch = source.index("apply_carla_runtime_timing_patch.sh")
        assert imu_patch < timing_patch


def test_runtime_patch_has_complete_idempotence_markers() -> None:
    apply_source = (
        ROOT / "scripts/e2e/apply_carla_runtime_timing_patch.sh"
    ).read_text(encoding="utf-8")
    patch_source = (
        ROOT / "patches/autoware_carla_interface_runtime_timing.patch"
    ).read_text(encoding="utf-8")

    for marker in (
        '"sensor_wait"',
        '"world_tick"',
        '"camera_bundle_total"',
        '"camera_image_publish"',
        '"camera_bundle_queue_wait"',
        "class RuntimeTimingMonitor",
    ):
        assert marker in apply_source
        assert marker in patch_source
    assert 'git -C "${repository}" apply --check "${patch_file}"' in apply_source
