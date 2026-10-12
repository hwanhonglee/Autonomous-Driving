"""HH_261001 - Check physical ACK precedes all unchanged JPEG writes and bound all failure paths."""

from types import SimpleNamespace

import pytest

from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import ExactJPEGAfterACK, deferred_recording


def image(frame): return SimpleNamespace(frame=frame, raw_data=bytes([frame]))


def test_real_ACK_precedes_all_same_exact_image_writes_and_restore(tmp_path):
    calls = []
    sources = [image(2) for _ in range(6)]
    def save(source, target, quality): calls.append(("write", source, target, quality))
    class Transport:
        def send(self, control):
            calls.append(("ACK", control))
            return {"server_accepted": True, "sequence": 3}
    collector = SimpleNamespace(_save_jpeg=save, AcknowledgedControlTransport=Transport)
    with deferred_recording(collector, tmp_path) as recorder:
        for i, source in enumerate(sources): collector._save_jpeg(source, tmp_path/(str(i)+".jpg"), 95)
        assert calls == []
        receipt = collector.AcknowledgedControlTransport().send("learned")
        assert receipt["sequence"] == 3
        assert calls[0] == ("ACK", "learned")
        for i, call in enumerate(calls[1:]): assert call == ("write", sources[i], tmp_path/(str(i)+".jpg"), 95)
        assert not recorder.pending
        assert recorder.events[0]["acknowledged_before_recording"] is True
        with pytest.raises(RuntimeError):
            with deferred_recording(collector, tmp_path): pass
    assert collector._save_jpeg is save and collector.AcknowledgedControlTransport is Transport


@pytest.mark.parametrize("bad", ["seventh", "new_frame", "duplicate", "existing"])
def test_bounded_queue_no_overwrite_or_frame_leak(tmp_path, bad):
    recorder = ExactJPEGAfterACK(lambda *args: None, tmp_path)
    if bad == "existing":
        (tmp_path/"0.jpg").touch()
        with pytest.raises(RuntimeError): recorder.enqueue(image(1), tmp_path/"0.jpg", 95)
        return
    recorder.enqueue(image(1), tmp_path/"0.jpg", 95)
    if bad == "seventh":
        for i in range(1, 6): recorder.enqueue(image(1), tmp_path/(str(i)+".jpg"), 95)
        target, value = tmp_path/"6.jpg", image(1)
    elif bad == "new_frame": target, value = tmp_path/"1.jpg", image(2)
    else: target, value = tmp_path/"0.jpg", image(1)
    with pytest.raises(RuntimeError): recorder.enqueue(value, target, 95)


def test_ACK_failure_still_records_exact_camera_and_restores(tmp_path):
    calls = []
    class Transport:
        def send(self, *args): raise ValueError("RPC failed")
    collector = SimpleNamespace(_save_jpeg=lambda *args: calls.append(args), AcknowledgedControlTransport=Transport)
    with deferred_recording(collector, tmp_path) as recorder:
        source = image(2)
        collector._save_jpeg(source, tmp_path/"image.jpg", 95)
        with pytest.raises(ValueError, match="RPC failed"): collector.AcknowledgedControlTransport().send("control")
        assert calls[0][0] is source and recorder.events[0]["acknowledged_before_recording"] is False
    assert collector.AcknowledgedControlTransport is Transport


def test_write_failure_remains_failed_and_keeps_unwritten_sources(tmp_path):
    def save(*args): raise OSError("disk problem")
    recorder = ExactJPEGAfterACK(save, tmp_path)
    source = image(2)
    recorder.enqueue(source, tmp_path/"camera.jpg", 95)
    with pytest.raises(OSError): recorder.flush(acknowledged=True, receipt={"sequence": 1})
    assert recorder.pending[0][0] is source
    assert recorder.events[0]["all_queued_images_written"] is False
    assert recorder.events[0]["written_camera_count"] == 0


def test_unflushed_data_fails_scope_and_restores(tmp_path):
    class Transport: pass
    save = lambda *args: None
    collector = SimpleNamespace(_save_jpeg=save, AcknowledgedControlTransport=Transport)
    with pytest.raises(RuntimeError, match="left without"):
        with deferred_recording(collector, tmp_path): collector._save_jpeg(image(2), tmp_path/"image.jpg", 95)
    assert collector._save_jpeg is save and collector.AcknowledgedControlTransport is Transport


def test_pre_send_exception_preserves_sources_without_ACK(tmp_path):
    calls = []
    collector = SimpleNamespace(_save_jpeg=lambda *args: calls.append(args), AcknowledgedControlTransport=type("T", (), {}))
    source = image(4)
    with pytest.raises(ValueError, match="before send"):
        with deferred_recording(collector, tmp_path) as recorder:
            collector._save_jpeg(source, tmp_path/"image.jpg", 95)
            raise ValueError("before send")
    assert calls[0][0] is source and not recorder.pending
    assert recorder.events[0]["acknowledged_before_recording"] is False


def test_pre_send_and_write_errors_are_both_preserved_and_bindings_restored(tmp_path):
    def save(*args): raise OSError("disk problem")
    original = type("T", (), {})
    collector = SimpleNamespace(_save_jpeg=save, AcknowledgedControlTransport=original)
    with pytest.raises(RuntimeError, match="ValueError.*before send.*OSError.*disk problem") as error:
        with deferred_recording(collector, tmp_path):
            collector._save_jpeg(image(4), tmp_path/"image.jpg", 95)
            raise ValueError("before send")
    assert isinstance(error.value.__cause__, ValueError)
    assert collector._save_jpeg is save and collector.AcknowledgedControlTransport is original


def test_preexisting_receipt_is_never_appended(tmp_path):
    (tmp_path/"recording_after_ack.jsonl").touch()
    with pytest.raises(RuntimeError, match="fresh owner-scoped"):
        ExactJPEGAfterACK(lambda *args: None, tmp_path)
