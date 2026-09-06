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


class RecordingTimingMonitor:
    def __init__(self):
        self.next_token = 0
        self.slow_calls = []
        self.events = []

    def start(self):
        self.next_token += 1
        return self.next_token

    def warn_if_slow(self, stage, started_ns, **fields):
        self.slow_calls.append((stage, started_ns, fields))

    def warn_event(self, stage, **fields):
        self.events.append((stage, fields))


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
    timing = RecordingTimingMonitor()
    bridge._runtime_timing = timing
    bridge.camera = lambda *args: calls.append(args)
    measurements = ((object(), "front", 1.0), (object(), "back", 1.0))

    bridge._publish_camera_bundle(measurements)

    assert calls == list(measurements)
    assert timing.slow_calls == [
        (
            "camera_bundle_total",
            1,
            {"camera_count": 2, "source_stamp_sec": 1.0},
        )
    ]


def test_camera_delivery_contract_uses_sensor_tick_frame_stride(bridge) -> None:
    calls = []
    bridge.param_values = {
        "sync_mode": True,
        "fixed_delta_seconds": 0.05,
        "camera_frame_barrier_enabled": True,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    bridge.sensor_interface = SimpleNamespace(
        configure_camera_contract=lambda **kwargs: calls.append(kwargs)
    )
    configs = [
        SimpleNamespace(
            carla_type="sensor.camera.rgb", parameters={"sensor_tick": 0.1}
        )
        for _ in range(6)
    ] + [
        SimpleNamespace(carla_type="sensor.other.imu", parameters={}),
    ]

    bridge._configure_camera_delivery_contract(configs)

    assert calls == [
        {
            "frame_stride": 2,
            "wait_timeout_sec": 0.25,
            "pending_frame_limit": 8,
            "expected_camera_count": 6,
        }
    ]


def test_camera_delivery_contract_is_opt_in_for_generic_carla(bridge) -> None:
    calls = []
    bridge.param_values = {
        "camera_frame_barrier_enabled": False,
        "fixed_delta_seconds": 0.05,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    bridge.sensor_interface = SimpleNamespace(
        configure_camera_contract=lambda **kwargs: calls.append(kwargs)
    )

    bridge._configure_camera_delivery_contract(
        [
            SimpleNamespace(
                carla_type="sensor.camera.rgb", parameters={"sensor_tick": 0.075}
            )
        ]
    )

    assert calls == []


@pytest.mark.parametrize(
    ("sensor_ticks", "message"),
    [
        ((0.1, 0.1, 0.1, 0.1, 0.1, 0.15), "same source frame stride"),
        ((0.075,) * 6, "integer multiple"),
    ],
)
def test_camera_delivery_contract_rejects_incompatible_source_cadence(
    bridge, sensor_ticks, message
) -> None:
    bridge.param_values = {
        "sync_mode": True,
        "camera_frame_barrier_enabled": True,
        "fixed_delta_seconds": 0.05,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    bridge.sensor_interface = SimpleNamespace(configure_camera_contract=lambda **_: None)
    configs = [
        SimpleNamespace(
            carla_type="sensor.camera.rgb", parameters={"sensor_tick": sensor_tick}
        )
        for sensor_tick in sensor_ticks
    ]

    with pytest.raises(ValueError, match=message):
        bridge._configure_camera_delivery_contract(configs)


def test_camera_delivery_contract_rejects_asynchronous_carla_mode(bridge) -> None:
    bridge.param_values = {
        "sync_mode": False,
        "camera_frame_barrier_enabled": True,
        "fixed_delta_seconds": 0.05,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    bridge.sensor_interface = SimpleNamespace(configure_camera_contract=lambda **_: None)
    configs = [
        SimpleNamespace(
            carla_type="sensor.camera.rgb", parameters={"sensor_tick": 0.1}
        )
        for _ in range(6)
    ]

    with pytest.raises(ValueError, match="requires synchronous CARLA mode"):
        bridge._configure_camera_delivery_contract(configs)


@pytest.mark.parametrize("camera_count", [5, 7])
def test_camera_delivery_contract_rejects_non_six_camera_config(
    bridge, camera_count
) -> None:
    bridge.param_values = {
        "sync_mode": True,
        "camera_frame_barrier_enabled": True,
        "fixed_delta_seconds": 0.05,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    bridge.sensor_interface = SimpleNamespace(configure_camera_contract=lambda **_: None)
    configs = [
        SimpleNamespace(
            carla_type="sensor.camera.rgb", parameters={"sensor_tick": 0.1}
        )
        for _ in range(camera_count)
    ]

    with pytest.raises(ValueError, match="requires exactly six physical RGB cameras"):
        bridge._configure_camera_delivery_contract(configs)


def test_camera_publish_worker_is_fail_closed_but_lidar_remains_latest_wins(
    bridge, monkeypatch
) -> None:
    module = sys.modules[bridge.__class__.__module__]
    created = []

    class Worker:
        def __init__(self, name, logger, **kwargs):
            created.append((name, kwargs))

        def submit(self, fn, args):
            pass

    bridge._publish_workers = {}
    bridge.logger = SimpleNamespace()
    bridge._runtime_timing = RecordingTimingMonitor()
    bridge.param_values = {
        "camera_frame_barrier_enabled": True,
        "camera_publish_deadline_sec": 0.25,
    }
    monkeypatch.setattr(module, "SensorPublishWorker", Worker)

    bridge._submit_to_publish_worker("camera_bundle", lambda: None)
    bridge._submit_to_publish_worker("lidar", lambda: None)

    assert created[0][0] == "camera_bundle"
    assert created[0][1]["overflow_policy"] == "fail_closed"
    assert created[0][1]["max_task_duration_sec"] == 0.25
    assert created[1][0] == "lidar"
    assert created[1][1]["overflow_policy"] == "latest_wins"
    assert created[1][1]["max_task_duration_sec"] is None

    bridge._publish_workers = {}
    bridge.param_values["camera_frame_barrier_enabled"] = False
    bridge._submit_to_publish_worker("camera_bundle", lambda: None)
    assert created[2][1]["overflow_policy"] == "latest_wins"
    assert created[2][1]["max_task_duration_sec"] is None


def test_camera_timing_wraps_each_publish_stage_without_reordering(bridge) -> None:
    actions = []
    timing = RecordingTimingMonitor()
    header = object()
    camera_info = SimpleNamespace(header=None)
    image_message = SimpleNamespace(header=None)
    image_publisher = SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=lambda message: actions.append(("image_publish", message)),
    )
    info_publisher = SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=lambda message: actions.append(("info_publish", message)),
    )
    bridge._runtime_timing = timing
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda _: SimpleNamespace(frame_id="front_optical")
    )
    bridge.pub_camera = {"front": image_publisher}
    bridge.pub_camera_info = {"front": info_publisher}
    bridge.camera_info_cache = {"front": camera_info}
    bridge.get_msg_header = lambda **_: header
    bridge.cv_bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda image, encoding: actions.append(
            ("cv_bridge", image.shape, encoding)
        )
        or image_message
    )
    measurement = SimpleNamespace(height=1, width=1, raw_data=bytes(4))

    bridge.camera(measurement, "front", timestamp=2.25)

    assert actions == [
        ("info_publish", camera_info),
        ("cv_bridge", (1, 1, 4), "bgra8"),
        ("image_publish", image_message),
    ]
    assert camera_info.header is header
    assert image_message.header is header
    assert [call[0] for call in timing.slow_calls] == [
        "camera_info_publish",
        "camera_cv_bridge",
        "camera_image_publish",
    ]
    assert all(
        call[2] == {"camera": "front", "source_stamp_sec": 2.25}
        for call in timing.slow_calls
    )


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

    logger = SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    timing = RecordingTimingMonitor()
    worker = module.SensorPublishWorker(
        "camera_bundle", logger, queue_size=1, timing_monitor=timing
    )
    worker.submit(publish, (first,))
    assert started.wait(timeout=2.0)
    worker.submit(publish, (stale,))
    worker.submit(publish, (latest,))
    release.set()
    assert completed.wait(timeout=2.0)
    worker.stop()

    assert published == [first, latest]
    assert [call[0] for call in timing.slow_calls] == [
        "camera_bundle_queue_wait",
        "camera_bundle_queue_wait",
    ]
    assert all(
        call[2] == {"worker": "camera_bundle"}
        for call in timing.slow_calls
    )
    assert timing.events == [
        (
            "sensor_publish_drop",
            {
                "worker": "camera_bundle",
                "dropped_this_submit": 1,
                "dropped_total": 1,
            },
        )
    ]


def test_camera_bundle_worker_fails_closed_without_replacing_source_frame(bridge) -> None:
    module = sys.modules[bridge.__class__.__module__]
    worker_module = sys.modules[
        "autoware_carla_interface.modules.sensor_publish_worker"
    ]
    started = threading.Event()
    release = threading.Event()
    completed = threading.Event()
    published = []

    def publish(value):
        published.append(value)
        if value == "first":
            started.set()
            assert release.wait(timeout=2.0)
        else:
            completed.set()

    timing = RecordingTimingMonitor()
    logger = SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    worker = module.SensorPublishWorker(
        "camera_bundle",
        logger,
        queue_size=1,
        timing_monitor=timing,
        overflow_policy="fail_closed",
    )
    worker.submit(publish, ("first",))
    assert started.wait(timeout=2.0)
    worker.submit(publish, ("second",))

    with pytest.raises(worker_module.SensorPublishFailure, match="refusing to skip"):
        worker.submit(publish, ("third",))

    release.set()
    worker._thread.join(timeout=2.0)
    worker.stop()
    assert not completed.is_set()
    assert published == ["first"]
    with pytest.raises(worker_module.SensorPublishFailure, match="refusing to skip"):
        worker.raise_if_failed()
    assert timing.events == [
        (
            "sensor_publish_overrun",
            {"worker": "camera_bundle", "queue_size": 1},
        )
    ]


def test_fail_closed_worker_propagates_async_publish_error(bridge) -> None:
    module = sys.modules[bridge.__class__.__module__]
    worker_module = sys.modules[
        "autoware_carla_interface.modules.sensor_publish_worker"
    ]
    error_logged = threading.Event()

    def fail_publish():
        raise ValueError("dds failure")

    logger = SimpleNamespace(
        warning=lambda *_: None, error=lambda *_: error_logged.set()
    )
    worker = module.SensorPublishWorker(
        "camera_bundle", logger, overflow_policy="fail_closed"
    )
    worker.submit(fail_publish, ())
    assert error_logged.wait(timeout=2.0)

    with pytest.raises(worker_module.SensorPublishFailure, match="dds failure"):
        worker.raise_if_failed()
    worker.stop()


def test_fail_closed_worker_propagates_publish_deadline_miss(
    bridge, monkeypatch
) -> None:
    module = sys.modules[bridge.__class__.__module__]
    worker_module = sys.modules[
        "autoware_carla_interface.modules.sensor_publish_worker"
    ]
    clock_values = iter((0, 2_000_000))
    monkeypatch.setattr(
        worker_module.time, "monotonic_ns", lambda: next(clock_values)
    )
    logger = SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    worker = module.SensorPublishWorker(
        "camera_bundle",
        logger,
        timing_monitor=RecordingTimingMonitor(),
        overflow_policy="fail_closed",
        max_task_duration_sec=0.001,
    )
    worker.submit(lambda: None, ())
    worker._thread.join(timeout=2.0)

    with pytest.raises(worker_module.SensorPublishFailure, match="task deadline"):
        worker.raise_if_failed()
    worker.stop()


def test_fail_closed_worker_stop_joins_active_failed_worker(bridge) -> None:
    worker_module = sys.modules[
        "autoware_carla_interface.modules.sensor_publish_worker"
    ]
    join_timeouts = []

    def raise_full(_):
        raise worker_module.queue.Full

    worker = worker_module.SensorPublishWorker.__new__(
        worker_module.SensorPublishWorker
    )
    worker._overflow_policy = "fail_closed"
    worker._failure = "terminal camera publish failure"
    worker._failure_lock = threading.Lock()
    worker._queue = SimpleNamespace(put_nowait=raise_full)
    worker._thread = SimpleNamespace(
        is_alive=lambda: True,
        join=lambda timeout: join_timeouts.append(timeout),
    )

    worker.stop(timeout=0.125)

    assert join_timeouts == [0.125]


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


def test_sensor_interface_returns_complete_frame_and_discards_stale_partial(bridge) -> None:
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


def test_sensor_interface_drains_complete_camera_frames_in_source_order(bridge) -> None:
    # HH_260906 - Prevent a callback burst from silently coalescing two valid camera frames.
    interface, camera_tags = make_sensor_interface(bridge)
    for frame in (100, 101):
        for tag in camera_tags:
            interface.update_sensor(tag, f"{tag}-{frame}", frame)

    first = interface.get_data()
    second = interface.get_data()

    assert {frame for frame, _ in first.values()} == {100}
    assert {frame for frame, _ in second.values()} == {101}
    assert interface.get_data() == {}


def test_sensor_interface_waits_for_due_frame_without_advancing_world(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.25)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-100", 100)
    assert {frame for frame, _ in interface.get_data(current_frame=100).values()} == {100}

    def publish_delayed_frame():
        for tag in camera_tags:
            interface.update_sensor(tag, f"{tag}-102", 102)

    timer = threading.Timer(0.01, publish_delayed_frame)
    timer.start()
    data = interface.get_data(current_frame=102)
    timer.join(timeout=1.0)

    assert {frame for frame, _ in data.values()} == {102}


def test_sensor_interface_bootstrap_without_callback_does_not_wait(
    bridge, monkeypatch
) -> None:
    interface, _ = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.25)
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    monkeypatch.setattr(
        wrapper_module.time,
        "monotonic",
        lambda: pytest.fail("camera bootstrap entered wait without a callback"),
    )

    assert interface.get_data(current_frame=100) == {}


def test_sensor_interface_waits_for_first_partial_frame_to_complete(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.25)
    interface.update_sensor(camera_tags[0], f"{camera_tags[0]}-100", 100)

    def complete_first_frame():
        for tag in camera_tags[1:]:
            interface.update_sensor(tag, f"{tag}-100", 100)

    timer = threading.Timer(0.01, complete_first_frame)
    timer.start()
    data = interface.get_data(current_frame=100)
    timer.join(timeout=1.0)

    assert {frame for frame, _ in data.values()} == {100}


def test_sensor_interface_does_not_skip_initial_partial_for_newer_complete_frame(
    bridge
) -> None:
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.01)
    interface.update_sensor(camera_tags[0], f"{camera_tags[0]}-100", 100)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-102", 102)

    with pytest.raises(wrapper_module.SensorReceivedNoData, match="expected=initial"):
        interface.get_data(current_frame=102)


def test_sensor_interface_does_not_wait_on_alternate_physics_frame(
    bridge, monkeypatch
) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.25)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-100", 100)
    interface.get_data(current_frame=100)
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    monkeypatch.setattr(
        wrapper_module.time,
        "monotonic",
        lambda: pytest.fail("alternate frame entered the camera wait path"),
    )

    assert interface.get_data(current_frame=101) == {}


def test_sensor_interface_refuses_to_skip_missing_expected_frame(bridge) -> None:
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(frame_stride=2, wait_timeout_sec=0.01)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-100", 100)
    interface.get_data(current_frame=100)
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-104", 104)

    with pytest.raises(wrapper_module.SensorReceivedNoData, match="expected=102"):
        interface.get_data(current_frame=104)


def test_sensor_interface_fails_closed_on_pending_frame_overflow(bridge) -> None:
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    interface, camera_tags = make_sensor_interface(bridge)
    interface.configure_camera_contract(
        frame_stride=2, wait_timeout_sec=0.25, pending_frame_limit=2
    )
    for tag in camera_tags:
        interface.update_sensor(tag, f"{tag}-100", 100)
    interface.get_data(current_frame=100)
    for frame in (102, 104, 106):
        for tag in camera_tags:
            interface.update_sensor(tag, f"{tag}-{frame}", frame)

    with pytest.raises(wrapper_module.SensorReceivedNoData, match="bounded limit"):
        interface.get_data(current_frame=106)


def test_sensor_interface_callback_queue_is_bounded_and_fail_closed(bridge) -> None:
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    interface = wrapper_module.SensorInterface()
    interface.register_sensor("imu", SimpleNamespace(type_id="sensor.other.imu"))
    interface.register_sensor(
        "camera", SimpleNamespace(type_id="sensor.camera.rgb")
    )
    interface.configure_camera_contract(
        frame_stride=2, wait_timeout_sec=0.25, expected_camera_count=1
    )
    for frame in range(interface.MAX_PENDING_SENSOR_MESSAGES + 1):
        interface.update_sensor("imu", frame, frame)

    with pytest.raises(wrapper_module.SensorReceivedNoData, match="bounded capacity"):
        interface.get_data()


def test_sensor_interface_fails_when_registered_rgb_count_differs_from_contract(
    bridge
) -> None:
    wrapper_module = sys.modules[
        "autoware_carla_interface.modules.carla_wrapper"
    ]
    interface = wrapper_module.SensorInterface()
    for index in range(5):
        interface.register_sensor(
            f"camera_{index}", SimpleNamespace(type_id="sensor.camera.rgb")
        )
    interface.configure_camera_contract(
        frame_stride=2, wait_timeout_sec=0.25, expected_camera_count=6
    )

    with pytest.raises(
        wrapper_module.SensorReceivedNoData, match="expected=6 actual=5"
    ):
        interface.get_data(current_frame=100)


def test_fast_sensor_patch_and_apply_guard_require_source_order_dispatch() -> None:
    patch = (
        ROOT / "patches/autoware_carla_interface_camera_fast_options.patch"
    ).read_text(encoding="utf-8")
    apply_helper = (ROOT / "scripts/e2e/apply_carla_fast_sensor_patch.sh").read_text(
        encoding="utf-8"
    )

    assert "+            oldest_complete = min(complete_frames)" in patch
    assert "latest_complete = max(complete_frames)" not in patch
    assert "return min\\(complete_frames\\)" in apply_helper


def test_camera_delivery_contract_patch_is_persisted_after_runtime_patches() -> None:
    # HH_260906 - Keep every ignored Universe camera-contract change reproducible.
    patch = (
        ROOT / "patches/autoware_carla_interface_camera_delivery_contract.patch"
    ).read_text(encoding="utf-8")
    apply_helper = (
        ROOT / "scripts/e2e/apply_carla_camera_delivery_contract_patch.sh"
    ).read_text(encoding="utf-8")

    for marker in (
        "camera_frame_barrier_enabled",
        "_configure_camera_delivery_contract",
        "configure_camera_contract",
        "class SensorPublishFailure",
        "refusing to skip a source frame",
        "camera frame barrier requires synchronous CARLA mode",
        "camera frame barrier requires exactly six physical RGB cameras",
        "expected_camera_count=len(rgb_configs)",
        "first_observed_frame = min(self._pending_camera_frames)",
        "RGB camera registration count violates the delivery contract",
        "Join an active failed publisher before ROS entities are destroyed",
    ):
        assert marker in patch
    reverse_check = (
        'git -C "${repository}" apply --check --reverse "${patch_file}"'
    )
    forward_check = 'git -C "${repository}" apply --check "${patch_file}"'
    assert reverse_check in apply_helper
    assert forward_check in apply_helper
    assert apply_helper.index(reverse_check) < apply_helper.index(forward_check)
    assert "partially applied or conflicts" in apply_helper
    for build_name in ("build.sh", "build_full.sh"):
        build = (ROOT / "scripts/e2e" / build_name).read_text(encoding="utf-8")
        assert build.index("apply_carla_clean_shutdown_patch.sh") < build.index(
            "apply_carla_camera_delivery_contract_patch.sh"
        )


def test_camera_frame_barrier_is_disabled_by_default_through_launch_chain() -> None:
    # HH_260906 - Preserve legacy behavior unless the Common10 wrapper opts in.
    generic_launch = (
        CARLA_INTERFACE_SRC.parent / "launch/autoware_carla_interface.launch.xml"
    ).read_text(encoding="utf-8")
    assert 'name="camera_frame_barrier_enabled"' in generic_launch
    assert 'default="False"' in generic_launch

    for launch_name in ("carla_vad.launch.xml", "carla_vad_full.launch.xml"):
        launch = (
            ROOT / "autoware_e2e_vad_launch/launch" / launch_name
        ).read_text(encoding="utf-8")
        assert '<arg name="camera_frame_barrier_enabled" default="false"/>' in launch
        assert 'value="$(var camera_frame_barrier_enabled)"' in launch
        for name, default in (
            ("camera_frame_wait_timeout_sec", "0.25"),
            ("camera_publish_deadline_sec", "0.25"),
            ("camera_pending_frame_limit", "8"),
        ):
            assert f'<arg name="{name}" default="{default}"/>' in launch
            assert f'name="{name}"' in launch
            assert f'value="$(var {name})"' in launch


def test_sensor_interface_keeps_non_camera_data_immediate(bridge) -> None:
    interface, camera_tags = make_sensor_interface(bridge)
    interface.register_sensor("gnss", SimpleNamespace(type_id="sensor.other.gnss"))
    interface.update_sensor(camera_tags[0], "partial-camera", 100)
    interface.update_sensor("gnss", "pose", 102)

    assert interface.get_data() == {"gnss": (102, "pose")}


def test_sensor_interface_does_not_register_non_rgb_camera_for_bundle(bridge) -> None:
    module = sys.modules[bridge.__class__.__module__]
    interface = module.SensorInterface()
    interface.register_sensor(
        "depth", SimpleNamespace(type_id="sensor.camera.depth")
    )
    interface.update_sensor("depth", "depth-frame", 100)

    assert interface.get_data() == {"depth": (100, "depth-frame")}
