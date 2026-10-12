"""HH_261001 - Record every exact camera image after its real control ACK and before the next synchronous tick."""

from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import time


SCHEMA = "portable_e2e.exact_camera_recording_after_ACK.20261001.v1"


class ExactJPEGAfterACK:
    """HH_261001 - Bound the queue to one six-camera bundle; never discard, retimestamp or transform images."""

    def __init__(self, save_jpeg, output):
        self.save_jpeg, self.output = save_jpeg, Path(output)
        self.pending = []
        self._flushing = False
        self.events = []
        self._receipt_created = False
        if (self.output/"recording_after_ack.jsonl").exists() or (self.output/"recording_after_ack.jsonl").is_symlink():
            raise RuntimeError("fresh owner-scoped recording receipt required")

    def enqueue(self, image, output, quality):
        if self._flushing or len(self.pending) >= 6:
            raise RuntimeError("exact JPEG recorder exceeded its single-bundle bound or reentered")
        target = Path(output)
        if target.exists() or target.is_symlink():
            raise RuntimeError("exact JPEG target already exists")
        if self.pending and int(image.frame) != int(self.pending[0][0].frame):
            raise RuntimeError("next camera frame arrived before the prior ACK recorder flush")
        if any(target == previous[1] for previous in self.pending):
            raise RuntimeError("duplicate exact JPEG target")
        self.pending.append((image, target, quality))

    def flush(self, *, acknowledged, receipt=None):
        if self._flushing:
            raise RuntimeError("exact JPEG recorder flush is not reentrant")
        if not self.pending:
            return
        self._flushing = True
        started = time.perf_counter_ns()
        items = tuple(self.pending)
        written = []
        error = None
        try:
            for image, target, quality in items:
                self.save_jpeg(image, target, quality)
                written.append(str(target))
        except BaseException as exc:
            error = type(exc).__name__ + ": " + str(exc)
            raise
        finally:
            finished = time.perf_counter_ns()
            # HH_261001 - Keep remaining exact sources on a failed write; never label a partial bundle complete.
            self.pending = list(items[len(written):])
            row = dict(schema=SCHEMA, frame=int(items[0][0].frame), enqueued_camera_count=len(items),
                       written_camera_count=len(written), written_files=written,
                       recording_started_monotonic_ns=started, recording_finished_monotonic_ns=finished,
                       recording_seconds=(finished-started)/1.e9, acknowledged_before_recording=bool(acknowledged),
                       ACK_receipt_sequence=receipt.get("sequence") if isinstance(receipt, dict) else None,
                       failure=error, all_queued_images_written=not self.pending,
                       no_source_image_dropped=True, source_timestamp_changed=False,
                       source_pixels_changed=False, synchronous_flush_before_next_tick=True,
                       legacy_jpeg_encode_write_timing_stage_measures_enqueue_only=True,
                       actual_encoding_timing_is_this_receipt=True,
                       full_tick_10Hz_wall_rate_claimed=False)
            self.events.append(row)
            self._flushing = False
            # HH_261001 - On-disk receipts are separate from the frozen collector's actual ACK timestamps.
            self.output.mkdir(parents=True, exist_ok=True)
            mode = "a" if self._receipt_created else "x"
            with (self.output/"recording_after_ack.jsonl").open(mode) as stream:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
            self._receipt_created = True


@contextmanager
def deferred_recording(collector, output):
    """HH_261001 - Enter after the declared control adapter activates; restore both original bindings on all exits."""
    if getattr(collector, "_exact_jpeg_after_ACK_active", False):
        raise RuntimeError("deferred camera recording scope cannot nest")
    original_save, original_transport = collector._save_jpeg, collector.AcknowledgedControlTransport
    recorder = ExactJPEGAfterACK(original_save, output)

    class RecordingAfterACKTransport(original_transport):
        def send(self, *args, **kwargs):
            receipt = None
            try:
                receipt = super().send(*args, **kwargs)
                return receipt
            finally:
                # HH_261001 - The real inherited RPC has returned (and timed its ACK) before any JPEG is encoded.
                recorder.flush(acknowledged=isinstance(receipt, dict) and receipt.get("server_accepted") is True,
                               receipt=receipt)

    collector._exact_jpeg_after_ACK_active = True
    collector._save_jpeg = recorder.enqueue
    collector.AcknowledgedControlTransport = RecordingAfterACKTransport
    try:
        yield recorder
        if recorder.pending:
            raise RuntimeError("exact camera bundle left without an ACK or failure flush")
    except BaseException as original_error:
        # HH_261001 - A pre-send exception must still preserve every delivered source, with no fabricated ACK.
        if recorder.pending:
            try:
                recorder.flush(acknowledged=False)
            except BaseException as write_error:
                raise RuntimeError("capture failed before recording completed: " + repr(original_error)
                                   + "; exact JPEG preservation also failed: " + repr(write_error)) from original_error
        raise
    finally:
        collector._save_jpeg, collector.AcknowledgedControlTransport = original_save, original_transport
        del collector._exact_jpeg_after_ACK_active
