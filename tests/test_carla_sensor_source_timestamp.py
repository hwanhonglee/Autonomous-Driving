import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
CARLA_INTERFACE_SRC = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface/src"
)


@pytest.fixture()
def bridge(monkeypatch):
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]
    from autoware_carla_interface.carla_ros import carla_ros2_interface

    instance = carla_ros2_interface.__new__(carla_ros2_interface)
    instance.timestamp = 9.5
    module = sys.modules[carla_ros2_interface.__module__]
    monkeypatch.setattr(module.GameTime, "get_carla_time", staticmethod(lambda: 109.5))
    return instance


def test_measurement_timestamp_prefers_carla_acquisition_time(bridge) -> None:
    assert bridge._measurement_timestamp(SimpleNamespace(timestamp=102.25)) == 2.25
    assert bridge._measurement_timestamp(SimpleNamespace(timestamp=100.0)) == 0.0


def test_camera_timestamp_prefers_capture_frame(bridge, monkeypatch) -> None:
    module = sys.modules[bridge.__class__.__module__]
    monkeypatch.setattr(module.GameTime, "get_frame", staticmethod(lambda: 200))
    bridge.param_values = {"fixed_delta_seconds": 0.05}

    first = SimpleNamespace(frame=198, timestamp=102.25)
    delayed_header = SimpleNamespace(frame=198, timestamp=102.30)

    assert bridge._measurement_timestamp(first, prefer_frame=True) == pytest.approx(9.4)
    assert bridge._measurement_timestamp(delayed_header, prefer_frame=True) == pytest.approx(
        9.4
    )


def test_camera_timestamp_falls_back_for_future_frame(bridge, monkeypatch) -> None:
    module = sys.modules[bridge.__class__.__module__]
    monkeypatch.setattr(module.GameTime, "get_frame", staticmethod(lambda: 200))
    bridge.param_values = {"fixed_delta_seconds": 0.05}

    measurement = SimpleNamespace(frame=201, timestamp=102.25)

    assert bridge._measurement_timestamp(measurement, prefer_frame=True) == pytest.approx(2.25)


@pytest.mark.parametrize("bad_timestamp", [None, "bad", math.nan, math.inf, -1.0])
def test_measurement_timestamp_falls_back_for_legacy_or_invalid_data(
    bridge, bad_timestamp
) -> None:
    measurement = SimpleNamespace()
    if bad_timestamp is not None:
        measurement.timestamp = bad_timestamp
    assert bridge._measurement_timestamp(measurement) == 9.5


def test_source_cadence_is_used_only_for_positive_sensor_tick(bridge) -> None:
    configs = {
        "carla_camera": SimpleNamespace(
            parameters={"sensor_tick": 0.2}, frequency_hz=5.0, carla_type="sensor.camera.rgb"
        ),
        "fast_source": SimpleNamespace(
            parameters={"sensor_tick": 0.1}, frequency_hz=5.0, carla_type="sensor.camera.rgb"
        ),
        "lidar": SimpleNamespace(
            parameters={"sensor_tick": 0.2}, frequency_hz=5.0, carla_type="sensor.lidar.ray_cast"
        ),
        "legacy_camera": SimpleNamespace(
            parameters={}, frequency_hz=5.0, carla_type="sensor.camera.rgb"
        ),
        "invalid_camera": SimpleNamespace(
            parameters={"sensor_tick": "bad"},
            frequency_hz=5.0,
            carla_type="sensor.camera.rgb",
        ),
    }
    bridge.sensor_registry = SimpleNamespace(get_sensor=configs.get)

    assert bridge._source_controls_cadence("carla_camera")
    assert not bridge._source_controls_cadence("fast_source")
    assert not bridge._source_controls_cadence("lidar")
    assert not bridge._source_controls_cadence("legacy_camera")
    assert not bridge._source_controls_cadence("invalid_camera")
    assert not bridge._source_controls_cadence("missing")


def test_frequency_gate_uses_explicit_measurement_timestamp(bridge) -> None:
    calls = []
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda _: object(),
        should_publish=lambda sensor, timestamp: calls.append((sensor, timestamp)) or True,
    )

    assert bridge.checkFrequency("CAM_FRONT", timestamp=2.25) is False
    assert calls == [("CAM_FRONT", 2.25)]


def configure_run_step(bridge, sensor_type, config):
    updates = []
    submissions = []
    bridge._state_lock = threading.Lock()
    bridge.clock_publisher = SimpleNamespace(publish=lambda _: None)
    bridge.logger = SimpleNamespace(warning=lambda *_: None)
    bridge.id_to_sensor_type_map = {"sensor": sensor_type}
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda _: config,
        update_sensor_timestamp=lambda sensor, stamp: updates.append((sensor, stamp)),
    )
    bridge._submit_to_publish_worker = lambda *args: submissions.append(args)
    bridge.apply_light_state = lambda: None
    bridge.ego_status = lambda: None
    bridge.current_control = object()
    return updates, submissions


def test_camera_with_sensor_tick_bypasses_duplicate_bridge_gate(bridge) -> None:
    config = SimpleNamespace(
        parameters={"sensor_tick": 0.2},
        frequency_hz=5.0,
        carla_type="sensor.camera.rgb",
        last_publish_time=None,
    )
    updates, submissions = configure_run_step(bridge, "sensor.camera.rgb", config)
    bridge.checkFrequency = lambda *_: pytest.fail("duplicate frequency gate was called")
    measurement = SimpleNamespace(timestamp=102.25)

    bridge.run_step({"sensor": (123, measurement)}, timestamp=9.5)

    assert updates == [("sensor", 2.25)]
    assert submissions == [
        (
            "camera_bundle",
            bridge._publish_camera_bundle,
            ((measurement, "sensor", 2.25),),
        )
    ]


def test_camera_cadence_drop_is_not_reported_as_a_bad_bundle(bridge) -> None:
    config = SimpleNamespace(
        parameters={"sensor_tick": 0.0},
        frequency_hz=5.0,
        carla_type="sensor.camera.rgb",
        last_publish_time=None,
    )
    updates, submissions = configure_run_step(bridge, "sensor.camera.rgb", config)
    bridge.checkFrequency = lambda *_: True
    measurement = SimpleNamespace(timestamp=102.25)

    bridge.run_step({"sensor": (123, measurement)}, timestamp=9.5)

    assert updates == []
    assert submissions == []
    assert not hasattr(bridge, "_rejected_camera_bundles")


def test_lidar_gate_and_worker_receive_measurement_timestamp(bridge) -> None:
    config = SimpleNamespace(
        parameters={},
        frequency_hz=10.0,
        carla_type="sensor.lidar.ray_cast",
        last_publish_time=None,
    )
    updates, submissions = configure_run_step(bridge, "sensor.lidar.ray_cast", config)
    gate_calls = []
    bridge.checkFrequency = lambda sensor, stamp: gate_calls.append((sensor, stamp)) or False
    measurement = SimpleNamespace(timestamp=103.75)

    bridge.run_step({"sensor": (321, measurement)}, timestamp=9.5)

    assert gate_calls == [("sensor", 3.75)]
    assert updates == [("sensor", 3.75)]
    assert submissions[0][1:] == (bridge.lidar, measurement, "sensor", 3.75)


def test_imu_dispatch_uses_capture_frame_timestamp(bridge, monkeypatch) -> None:
    module = sys.modules[bridge.__class__.__module__]
    monkeypatch.setattr(module.GameTime, "get_frame", staticmethod(lambda: 200))
    bridge.param_values = {"fixed_delta_seconds": 0.05}
    config = SimpleNamespace(
        parameters={},
        frequency_hz=20.0,
        carla_type="sensor.other.imu",
        last_publish_time=None,
    )
    configure_run_step(bridge, "sensor.other.imu", config)
    calls = []
    bridge.imu = lambda *args: calls.append(args)
    measurement = SimpleNamespace(frame=198, timestamp=102.25)

    bridge.run_step({"sensor": (123, measurement)}, timestamp=9.5)

    assert calls == [(measurement, pytest.approx(9.4))]


def test_imu_uses_one_source_timestamp_for_gate_header_and_registry(bridge) -> None:
    gate_calls = []
    updates = []
    published = []
    config = SimpleNamespace(frame_id="imu_link")
    bridge.checkFrequency = (
        lambda sensor, stamp: gate_calls.append((sensor, stamp)) or False
    )
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda _: config,
        update_sensor_timestamp=lambda sensor, stamp: updates.append((sensor, stamp)),
    )
    bridge.pub_imu = SimpleNamespace(publish=published.append)
    bridge.logger = SimpleNamespace(warning=lambda *_: None)
    measurement = SimpleNamespace(
        gyroscope=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        accelerometer=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        transform=SimpleNamespace(
            rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
        ),
    )

    bridge.imu(measurement, 2.25)

    assert gate_calls == [("imu", 2.25)]
    assert updates == [("imu", 2.25)]
    assert len(published) == 1
    assert published[0].header.stamp.sec == 2
    assert published[0].header.stamp.nanosec == 250_000_000


@pytest.mark.parametrize("measurement_stamp", [104.0, 103.5])
def test_stale_or_duplicate_measurement_is_not_published(bridge, measurement_stamp) -> None:
    config = SimpleNamespace(
        parameters={"sensor_tick": 0.2},
        frequency_hz=5.0,
        carla_type="sensor.camera.rgb",
        last_publish_time=4.0,
    )
    updates, submissions = configure_run_step(bridge, "sensor.camera.rgb", config)
    bridge.checkFrequency = lambda *_: pytest.fail("stale measurement reached the cadence gate")

    bridge.run_step(
        {"sensor": (123, SimpleNamespace(timestamp=measurement_stamp))}, timestamp=9.5
    )

    assert updates == []
    assert submissions == []


def test_camera_bundle_worker_publishes_every_camera_serially(bridge) -> None:
    calls = []
    bridge.camera = lambda *args: calls.append(args)
    measurements = ((object(), "front", 1.0), (object(), "back", 1.0))

    bridge._publish_camera_bundle(measurements)

    assert calls == list(measurements)


def test_run_step_submits_six_cameras_as_one_all_or_none_worker_item(bridge) -> None:
    config = SimpleNamespace(
        parameters={"sensor_tick": 0.2},
        frequency_hz=5.0,
        carla_type="sensor.camera.rgb",
        last_publish_time=None,
    )
    updates, submissions = configure_run_step(bridge, "sensor.camera.rgb", config)
    camera_tags = [f"camera_{index}" for index in range(6)]
    bridge.id_to_sensor_type_map = {tag: "sensor.camera.rgb" for tag in camera_tags}
    measurements = {
        tag: (200, SimpleNamespace(timestamp=102.25, camera_id=tag))
        for tag in camera_tags
    }

    bridge.run_step(measurements, timestamp=9.5)

    assert len(submissions) == 1
    worker_key, callback, bundle = submissions[0]
    assert worker_key == "camera_bundle"
    assert callback == bridge._publish_camera_bundle
    assert len(bundle) == 6
    assert [camera_id for _, camera_id, _ in bundle] == camera_tags
    assert {stamp for _, _, stamp in bundle} == {2.25}
    assert updates == [(tag, 2.25) for tag in camera_tags]


def test_run_step_canonicalizes_delayed_camera_headers(bridge, monkeypatch) -> None:
    module = sys.modules[bridge.__class__.__module__]
    monkeypatch.setattr(module.GameTime, "get_frame", staticmethod(lambda: 202))
    bridge.param_values = {"fixed_delta_seconds": 0.05}
    config = SimpleNamespace(
        parameters={"sensor_tick": 0.2},
        frequency_hz=5.0,
        carla_type="sensor.camera.rgb",
        last_publish_time=None,
    )
    updates, submissions = configure_run_step(bridge, "sensor.camera.rgb", config)
    camera_tags = [f"camera_{index}" for index in range(6)]
    bridge.id_to_sensor_type_map = {tag: "sensor.camera.rgb" for tag in camera_tags}
    measurements = {
        tag: (
            200,
            SimpleNamespace(frame=200, timestamp=102.25 + 0.05 * (index % 2)),
        )
        for index, tag in enumerate(camera_tags)
    }

    bridge.run_step(measurements, timestamp=9.5)

    assert len(submissions) == 1
    bundle = submissions[0][2]
    assert {stamp for _, _, stamp in bundle} == {9.4}
    assert updates == [(tag, 9.4) for tag in camera_tags]


@pytest.mark.parametrize(
    "bundle",
    [
        ((object(), "front", -0.05), (object(), "back", -0.05)),
        ((object(), "front", 1.0), (object(), "back", 1.001)),
    ],
)
def test_invalid_camera_bundle_is_rejected_before_registry_update(bridge, bundle) -> None:
    updates = []
    submissions = []
    warnings = []
    bridge.sensor_registry = SimpleNamespace(
        update_sensor_timestamp=lambda camera, stamp: updates.append((camera, stamp))
    )
    bridge._submit_to_publish_worker = lambda *args: submissions.append(args)
    bridge.logger = SimpleNamespace(warning=lambda message: warnings.append(message))

    assert not bridge._queue_camera_bundle(bundle, expected_count=2)
    assert updates == []
    assert submissions == []
    assert warnings == [
        "Rejected camera bundle 1: received=2 expected=2 stamp_span_ms="
        + ("0.000" if bundle[0][2] < 0.0 else "1.000")
    ]


def test_camera_bundle_worker_drops_a_whole_stale_bundle_when_slow(bridge) -> None:
    module = sys.modules[bridge.__class__.__module__]
    started = threading.Event()
    release = threading.Event()
    completed = threading.Event()
    published = []
    first = (("front-1", "front", 1.0), ("back-1", "back", 1.0))
    stale = (("front-2", "front", 2.0), ("back-2", "back", 2.0))
    latest = (("front-3", "front", 3.0), ("back-3", "back", 3.0))

    def publish(bundle):
        published.append(bundle)
        if bundle == first:
            started.set()
            assert release.wait(timeout=2.0)
        else:
            completed.set()

    logger = SimpleNamespace(debug=lambda *_: None, error=lambda *_: None)
    worker = module.SensorPublishWorker("camera_bundle_test", logger, queue_size=1)
    worker.submit(publish, (first,))
    assert started.wait(timeout=2.0)
    worker.submit(publish, (stale,))
    worker.submit(publish, (latest,))
    release.set()
    assert completed.wait(timeout=2.0)
    worker.stop()

    assert published == [first, latest]


def make_sensor_interface(bridge):
    module = sys.modules[bridge.__class__.__module__]
    interface = module.SensorInterface()
    camera_tags = [f"camera_{index}" for index in range(6)]
    for tag in camera_tags:
        interface.register_sensor(tag, SimpleNamespace(type_id="sensor.camera.rgb"))
    return interface, camera_tags


def test_sensor_interface_holds_partial_camera_frame(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    for tag in camera_tags[:-1]:
        interface.update_sensor(tag, f"{tag}-100", 100)

    assert interface.get_data() == {}

    interface.update_sensor(camera_tags[-1], "last-100", 100)
    data = interface.get_data()
    assert set(data) == set(camera_tags)
    assert {frame for frame, _ in data.values()} == {100}


def test_sensor_interface_returns_latest_complete_frame_and_discards_stale_partial(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    for tag in camera_tags[:3]:
        interface.update_sensor(tag, f"{tag}-100", 100)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-101", 101)

    data = interface.get_data()
    assert {frame for frame, _ in data.values()} == {101}

    for tag in camera_tags[3:]:
        interface.update_sensor(tag, f"{tag}-100", 100)
    assert interface.get_data() == {}


def test_sensor_interface_keeps_non_camera_data_immediate(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    interface.register_sensor("gnss", SimpleNamespace(type_id="sensor.other.gnss"))
    interface.update_sensor(camera_tags[0], "partial-camera", 100)
    interface.update_sensor("gnss", "pose", 102)

    assert interface.get_data() == {"gnss": (102, "pose")}
