"""HH_261004 - Verify bounded encode scheduling, exact source math and the actual copied recorder seam.

These are synthetic, CPU-only lifecycle tests, not a CARLA drive or throughput
measurement.  The later actual901x6 JPEG replay must retain its own denominator
and must not claim that decoded JPEG pixels are original sensor BGRA bytes.
"""

import ast
import hashlib
import inspect
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from scripts.e2e import parallel_exact_jpeg_after_ack_v1_20261004 as h


def image(ordinal=0, frame=2):
    return SimpleNamespace(frame=frame, timestamp=frame * .05, width=640, height=360,
                           raw_data=bytes([ordinal]) * (640 * 360 * 4))


def target(output, ordinal, frame=2):
    path = output / "images" / h.CAMERA_ORDER[ordinal] / f"{frame:08d}.jpg"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def enqueue(recorder, output, *, frame=2, count=6):
    sources = [image(ordinal, frame) for ordinal in range(count)]
    for ordinal, source in enumerate(sources):
        recorder.enqueue(source, target(output, ordinal, frame), 95)
    return sources


def synthetic_encoder(source, path, quality):
    return b"synthetic-encoded-not-a-real-JPEG:" + bytes([source.raw_data[0], quality])


@pytest.fixture
def make_recorder(monkeypatch):
    # HH_261004 - Inject only numerical work in synthetic tests; never stub transport acceptance or alias checks.
    recorders = []
    def make(output, encoder=synthetic_encoder):
        monkeypatch.setattr(h, "exact_encoder", lambda original: encoder)
        recorder = h.ParallelExactJPEGAfterACK(lambda *args: None, output)
        recorders.append(recorder)
        return recorder
    yield make
    for recorder in recorders:
        recorder.close()


def test_all_original_source_pins_and_encoder_inverse_AST():
    h.verify_frozen()
    original, changed = h.encoder_AST()
    assert ast.unparse(original.body[-1]) == "output.write_bytes(encoded.tobytes())"
    assert ast.unparse(changed.body[-1]) == "return encoded.tobytes()"
    changed.body[-1] = original.body[-1]
    assert ast.dump(changed) == ast.dump(original)


def test_no_Torch_CARLA_or_executor_activated_at_module_import():
    tree = ast.parse(Path(h.__file__).read_bytes())
    names = [node.module or "" for node in tree.body if isinstance(node, ast.ImportFrom)]
    names += [alias.name for node in tree.body if isinstance(node, ast.Import) for alias in node.names]
    assert not any(name.startswith(("torch", "carla", "portable_e2e")) for name in names)
    assert h._active is False
    assert h.WORKERS == 3


@pytest.mark.parametrize("quality", [35, 90, 95])
def test_actual_frozen_encoder_bytes_equal_synthetic_source_not_actual_camera(tmp_path, quality):
    # HH_261004 - Compare genuine cv2 operations on one explicitly synthetic image, not prerecorded CARLA payloads.
    definition, _ = h.encoder_AST()
    namespace = dict(Any=object, Path=Path, CollectionError=RuntimeError)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[])),
                 str(h.ROOT / "tools/e2e/collect_carla_vad_expert.py"), "exec"), namespace)
    output = tmp_path / "original.jpg"
    source = image(7)
    namespace["_save_jpeg"](source, output, quality)
    expected = output.read_bytes()
    assert h.exact_encoder(namespace["_save_jpeg"])(source, output, quality) == expected
    assert h._encode_image(source, output, quality) == expected
    assert source.raw_data == bytes([7]) * (640 * 360 * 4)


def test_three_workers_all_join_then_original_role_ordered_writes(tmp_path, make_recorder, monkeypatch):
    barrier = threading.Barrier(3, timeout=5)
    lock = threading.Lock()
    calls = []
    active = dict(value=0, maximum=0)
    def encode(source, path, quality):
        with lock:
            active["value"] += 1
            active["maximum"] = max(active["maximum"], active["value"])
            calls.append(("encode_start", source.raw_data[0]))
        barrier.wait()
        with lock:
            calls.append(("encode_end", source.raw_data[0]))
            active["value"] -= 1
        return synthetic_encoder(source, path, quality)
    real_write = h._write_create_only
    def write(path, payload):
        assert len([row for row in calls if row[0] == "encode_end"]) == 6
        calls.append(("write", h.CAMERA_ORDER.index(path.parent.name)))
        real_write(path, payload)
    monkeypatch.setattr(h, "_write_create_only", write)
    recorder = make_recorder(tmp_path, encode)
    sources = enqueue(recorder, tmp_path)
    recorder.flush(acknowledged=True, receipt={"sequence": 3})
    assert active == dict(value=0, maximum=3)
    assert [row[1] for row in calls if row[0] == "write"] == list(range(6))
    assert recorder.pending == []
    receipt = recorder.events[-1]
    detail = receipt["parallel_encoding_research"]
    assert receipt["schema"] == h.original.SCHEMA
    assert receipt["ACK_receipt_sequence"] == 3 and receipt["acknowledged_before_recording"] is True
    assert receipt["synchronous_flush_before_next_tick"] is True
    assert detail["submitted_encode_count"] == 6 and detail["all_submitted_encodes_joined"] is True
    assert detail["background_work_after_flush"] is False and detail["actual_speed_gain_proven"] is False
    assert len({row["worker_thread_id"] for row in detail["encode_rows"]}) <= 3
    for ordinal, (source, row) in enumerate(zip(sources, detail["encode_rows"])):
        assert row["image_object_id"] == id(source) and row["native_frame"] == 2
        assert row["source_image_timestamp_simulation_ns"] == 100_000_000
        assert row["source_BGRA_encoding_prefixed_sha256"] == hashlib.sha256(b"bgra8" + source.raw_data).hexdigest()
        assert row["encoding_finished_monotonic_ns"] <= detail["all_encodes_joined_monotonic_ns"] <= row["write_started_monotonic_ns"]
        assert row["encoded_JPEG_sha256"] == hashlib.sha256(target(tmp_path, ordinal).read_bytes()).hexdigest()


@pytest.mark.parametrize("bad", ["seventh", "new_frame", "duplicate", "existing"])
def test_exact_original_enqueue_guards_unchanged(tmp_path, make_recorder, bad):
    recorder = make_recorder(tmp_path)
    if bad == "existing":
        path = target(tmp_path, 0)
        path.touch()
        with pytest.raises(RuntimeError, match="already exists"):
            recorder.enqueue(image(), path, 95)
        return
    recorder.enqueue(image(), target(tmp_path, 0), 95)
    if bad == "seventh":
        for ordinal in range(1, 6):
            recorder.enqueue(image(ordinal), target(tmp_path, ordinal), 95)
        value, path = image(), tmp_path / "seventh.jpg"
    elif bad == "new_frame":
        value, path = image(frame=3), target(tmp_path, 1, frame=3)
    else:
        value, path = image(), target(tmp_path, 0)
    with pytest.raises(RuntimeError):
        recorder.enqueue(value, path, 95)
    assert h.ParallelExactJPEGAfterACK.enqueue is h.original.ExactJPEGAfterACK.enqueue


def test_encode_errors_keep_all_rows_and_exact_original_first_exception_prefix(tmp_path, make_recorder):
    first, later = ValueError("role2 failed"), OSError("role4 failed")
    def encode(source, path, quality):
        if source.raw_data[0] == 2:
            raise first
        if source.raw_data[0] == 4:
            raise later
        return synthetic_encoder(source, path, quality)
    recorder = make_recorder(tmp_path, encode)
    sources = enqueue(recorder, tmp_path)
    with pytest.raises(ValueError) as caught:
        recorder.flush(acknowledged=True, receipt={"sequence": 7})
    assert caught.value is first
    assert [item[0] for item in recorder.pending] == sources[2:]
    assert [target(tmp_path, i).exists() for i in range(6)] == [True, True, False, False, False, False]
    event = recorder.events[-1]
    assert event["written_camera_count"] == 2 and event["all_queued_images_written"] is False
    assert event["failure"] == "ValueError: role2 failed"
    encodings = event["parallel_encoding_research"]["encode_rows"]
    assert len(encodings) == 6 and encodings[2]["encoding_failure"] == "ValueError: role2 failed"
    assert encodings[4]["encoding_failure"] == "OSError: role4 failed"
    with pytest.raises(RuntimeError, match="cannot be retried"):
        recorder.flush(acknowledged=False)


def test_write_error_preserves_prefix_pending_and_all_worker_records(tmp_path, make_recorder, monkeypatch):
    failed = OSError("disk write failed at role3")
    original = h._write_create_only
    def write(path, payload):
        if path.parent.name == h.CAMERA_ORDER[3]:
            raise failed
        original(path, payload)
    monkeypatch.setattr(h, "_write_create_only", write)
    recorder = make_recorder(tmp_path)
    sources = enqueue(recorder, tmp_path)
    with pytest.raises(OSError) as caught:
        recorder.flush(acknowledged=True, receipt={"sequence": 3})
    assert caught.value is failed and [row[0] for row in recorder.pending] == sources[3:]
    assert recorder.events[-1]["written_camera_count"] == 3
    assert len(recorder.events[-1]["parallel_encoding_research"]["encode_rows"]) == 6


def test_final_create_only_race_never_overwrites_foreign_bytes(tmp_path, make_recorder, monkeypatch):
    original = h._write_create_only
    def write(path, payload):
        path.write_bytes(b"foreign-preexisting-content")
        original(path, payload)
    monkeypatch.setattr(h, "_write_create_only", write)
    recorder = make_recorder(tmp_path)
    sources = enqueue(recorder, tmp_path)
    with pytest.raises(FileExistsError):
        recorder.flush(acknowledged=True, receipt={"sequence": 3})
    assert target(tmp_path, 0).read_bytes() == b"foreign-preexisting-content"
    assert [row[0] for row in recorder.pending] == sources
    assert recorder.events[-1]["written_camera_count"] == 0


@pytest.mark.parametrize("change", ["role_order", "frame_name", "relative", "parent_symlink", "mutable", "shape", "timestamp"])
def test_unsafe_source_or_target_fails_closed_and_retains_queued_object(tmp_path, make_recorder, change):
    recorder = make_recorder(tmp_path)
    source, path = image(), target(tmp_path, 0)
    if change == "role_order":
        path = target(tmp_path, 1)
    elif change == "frame_name":
        path = target(tmp_path, 0, frame=3)
    elif change == "relative":
        path = Path("images/CAM_FRONT/00000002.jpg")
    elif change == "parent_symlink":
        outside = tmp_path / "outside"
        outside.mkdir()
        path.parent.rmdir()
        path.parent.symlink_to(outside, target_is_directory=True)
    elif change == "mutable":
        source.raw_data = bytearray(source.raw_data)
    elif change == "shape":
        source.width = 639
    else:
        source.timestamp = float("nan")
    recorder.enqueue(source, path, 95)
    with pytest.raises((RuntimeError, ValueError)):
        recorder.flush(acknowledged=False)
    assert recorder.pending[0][0] is source and recorder.events[-1]["all_queued_images_written"] is False
    assert recorder.events[-1]["parallel_encoding_research"]["all_submitted_encodes_joined"] is True


def test_existing_receipt_and_closed_reentrant_guards(tmp_path, make_recorder):
    recorder = make_recorder(tmp_path)
    recorder._flushing = True
    with pytest.raises(RuntimeError, match="reentrant"):
        recorder.flush(acknowledged=True)
    recorder._flushing = False
    recorder.close()
    with pytest.raises(RuntimeError, match="closed"):
        recorder.flush(acknowledged=True)
    (tmp_path / "recording_after_ack.jsonl").touch()
    with pytest.raises(RuntimeError, match="fresh owner-scoped"):
        make_recorder(tmp_path)


def modules():
    return (SimpleNamespace(__file__=str(h.ROOT / "tools/e2e/collect_temporal_route_recovery_fast45_20261001.py"),
                            deferred_recording=h.original.deferred_recording, _active=False),
            SimpleNamespace(__file__=str(h.ROOT / "tools/e2e/collect_temporal_route_joint_town03_straight_v2_20261004.py"),
                            ExactJPEGAfterACK=h.original.ExactJPEGAfterACK, _active=False))


def frozen_recorder_check(collector, recorder_type):
    # HH_261004 - Execute the real frozen type/closure checker AST, not a success stub or a fake recorder.
    path = h.ROOT / "tools/e2e/collect_temporal_route_joint_town03_straight_v2_20261004.py"
    definition = next(node for node in ast.parse(path.read_bytes()).body
                      if isinstance(node, ast.FunctionDef) and node.name == "_recorder")
    namespace = dict(collector=collector, frozen=SimpleNamespace(_require=h.require),
                     ExactJPEGAfterACK=recorder_type, inspect=inspect)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[])),
                 str(path) + ":synthetic_actual_recorder_check", "exec"), namespace)
    return namespace["_recorder"]()


def test_actual_runtime_alias_and_ACK_closure_reach_real_frozen_factory_checker(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(h, "exact_encoder", lambda _: synthetic_encoder)
    class Transport:
        def send(self, control, reason, expected_before_frame=None):
            calls.append(("ACK", expected_before_frame))
            return {"server_accepted": True, "sequence": 3}
    original_save = lambda *args: None
    collector = SimpleNamespace(_save_jpeg=original_save, AcknowledgedControlTransport=Transport)
    recovery, straight = modules()
    with h.installed_parallel_recording(recovery, straight, collector=collector):
        assert recovery.deferred_recording is h.parallel_deferred_recording
        assert straight.ExactJPEGAfterACK is h.ParallelExactJPEGAfterACK
        with recovery.deferred_recording(collector, tmp_path) as recorder:
            assert h.assert_runtime_recorder(collector) is recorder
            assert frozen_recorder_check(collector, straight.ExactJPEGAfterACK) is recorder
            for i, source in enumerate([image(i) for i in range(6)]):
                collector._save_jpeg(source, target(tmp_path, i), 95)
            receipt = collector.AcknowledgedControlTransport().send("unchanged-control", "next_driving_control", 2)
            assert receipt == {"server_accepted": True, "sequence": 3}
            assert not recorder.pending and len(recorder.events) == 1
            assert calls == [("ACK", 2)]
            assert h.assert_runtime_recorder(collector) is recorder
        assert recorder.closed is True
    assert collector._save_jpeg is original_save and collector.AcknowledgedControlTransport is Transport
    assert recovery.deferred_recording is h.original.deferred_recording
    assert straight.ExactJPEGAfterACK is h.original.ExactJPEGAfterACK
    assert not h._active and not hasattr(collector, "_exact_jpeg_after_ACK_active")


@pytest.mark.parametrize("kind", ["nested", "foreign_alias", "foreign_type", "foreign_module", "already_active"])
def test_scope_denies_unknown_or_active_aliases(kind):
    collector = SimpleNamespace()
    recovery, straight = modules()
    if kind == "foreign_alias":
        recovery.deferred_recording = lambda *args: None
    elif kind == "foreign_type":
        straight.ExactJPEGAfterACK = object
    elif kind == "foreign_module":
        recovery.__file__ = str(h.ROOT / "tools/e2e/foreign.py")
    elif kind == "already_active":
        collector._exact_jpeg_after_ACK_active = True
    if kind == "nested":
        with h.installed_parallel_recording(recovery, straight, collector=collector):
            with pytest.raises(RuntimeError, match="cannot nest"):
                with h.installed_parallel_recording(recovery, straight, collector=collector):
                    pass
        return
    saved = recovery.deferred_recording, straight.ExactJPEGAfterACK
    with pytest.raises(RuntimeError):
        with h.installed_parallel_recording(recovery, straight, collector=collector):
            pass
    assert (recovery.deferred_recording, straight.ExactJPEGAfterACK) == saved
    assert h._active is False


@pytest.mark.parametrize("mode", ["inner_error", "send_error", "unaccepted_ACK"])
def test_original_exception_false_ACK_and_all_alias_restoration(tmp_path, monkeypatch, mode):
    monkeypatch.setattr(h, "exact_encoder", lambda _: synthetic_encoder)
    error = ValueError("original error identity")
    class Transport:
        def send(self, *args, **kwargs):
            if mode == "send_error":
                raise error
            return {"server_accepted": False, "sequence": 3}
    save = lambda *args: None
    collector = SimpleNamespace(_save_jpeg=save, AcknowledgedControlTransport=Transport)
    recovery, straight = modules()
    def run():
        with h.installed_parallel_recording(recovery, straight, collector=collector):
            with recovery.deferred_recording(collector, tmp_path) as recorder:
                for ordinal in range(6):
                    collector._save_jpeg(image(ordinal), target(tmp_path, ordinal), 95)
                if mode == "inner_error":
                    raise error
                collector.AcknowledgedControlTransport().send("control")
            return recorder
    if mode == "unaccepted_ACK":
        recorder = run()
        assert recorder.closed and recorder.events[0]["acknowledged_before_recording"] is False
    else:
        with pytest.raises(ValueError) as caught:
            run()
        assert caught.value is error
    assert all(target(tmp_path, ordinal).is_file() for ordinal in range(6))
    assert collector._save_jpeg is save and collector.AcknowledgedControlTransport is Transport
    assert recovery.deferred_recording is h.original.deferred_recording
    assert straight.ExactJPEGAfterACK is h.original.ExactJPEGAfterACK and not h._active


def test_private_delegate_bytecode_and_global_alias_do_not_mutate_original():
    delegated = inspect.unwrap(h._delegate_recording())
    original = inspect.unwrap(h.original.deferred_recording)
    assert delegated.__code__ is original.__code__
    assert delegated.__globals__["ExactJPEGAfterACK"] is h.ParallelExactJPEGAfterACK
    assert original.__globals__["ExactJPEGAfterACK"] is h.original.ExactJPEGAfterACK


def test_unknown_transport_or_missing_copied_provider_is_never_accepted(tmp_path, make_recorder):
    recorder = make_recorder(tmp_path)
    collector = SimpleNamespace(_save_jpeg=recorder.enqueue, AcknowledgedControlTransport=object,
                                _exact_jpeg_after_ACK_active=True)
    with pytest.raises(RuntimeError, match="closure"):
        h.assert_runtime_recorder(collector)
    collector._save_jpeg = lambda *args: None
    with pytest.raises(RuntimeError, match="provider"):
        h.assert_runtime_recorder(collector)


@pytest.mark.parametrize("validation_failure", [False, True])
def test_unexpected_future_error_joins_every_submitted_task_and_preserves_primary_error(tmp_path, make_recorder, validation_failure):
    recorder = make_recorder(tmp_path)
    recorder.executor.shutdown(wait=True)
    calls = []
    error = OSError("unexpected executor future error")
    class Future:
        def __init__(self, ordinal, value):
            self.ordinal, self.value = ordinal, value
        def result(self):
            calls.append(self.ordinal)
            if self.ordinal == 1:
                raise error
            return self.value
    class Executor:
        def __init__(self):
            self.submitted = 0
        def submit(self, function, *args):
            ordinal = self.submitted
            self.submitted += 1
            return Future(ordinal, None if ordinal == 1 else function(*args))
        def shutdown(self, **kwargs):
            assert kwargs == dict(wait=True, cancel_futures=False)
    recorder.executor = Executor()
    sources = enqueue(recorder, tmp_path)
    if validation_failure:
        source, _, quality = recorder.pending[2]
        recorder.pending[2] = (source, target(tmp_path, 3), quality)
    with pytest.raises((RuntimeError, OSError)) as caught:
        recorder.flush(acknowledged=False)
    event = recorder.events[-1]
    detail = event["parallel_encoding_research"]
    if validation_failure:
        assert "role/order/frame" in str(caught.value) and caught.value is not error
        assert calls == [0, 1] and event["written_camera_count"] == 0
        assert [item[0] for item in recorder.pending] == sources
    else:
        assert caught.value is error and calls == list(range(6))
        assert event["written_camera_count"] == 1 and [item[0] for item in recorder.pending] == sources[1:]
    assert detail["all_submitted_encodes_joined"] is True
    unexpected = detail["encode_rows"][1]
    assert unexpected["unexpected_future_failure"] is True
    assert unexpected["worker_execution_or_start_time_known"] is False
    assert unexpected["encoding_failure"] == "OSError: unexpected executor future error"
    assert "encoding_started_monotonic_ns" not in unexpected
