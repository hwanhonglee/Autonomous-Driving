"""HH_261012 - Bound serial input metadata and separate optimizer return from journaled ACK.

This is an owner-thread lifecycle library, not an NN trainer or a TRAIN grant.
The external caller must authenticate the iterator, sources, packages, providers,
model/optimizer, GPU lease, journal implementation and every numeric callback.
The outer object retains at most eight declaration rows and one active batch.
The frozen adapter owns its separate eight-row copy. A supplied iterator or
callback may retain additional data; these are not whole-process memory bounds.
Default public-API wrappers are lazy and require the caller's sealed package
loader; no removed filesystem alias is recreated. Mock callbacks prove only
chronology. No dependency is installed, file opened, model constructed or GPU
initialized by importing this module.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import os
import threading

SCHEMA = "portable_e2e.map_streaming_training_boundary.20261012.v1"
MODES = ("ORIGINAL_CROP_CONTROL", "SIGN_PRESERVING_CANDIDATE")
CHUNK_STEPS = 8
MAX_METADATA_BYTES = 64 * 1024
PHASES = ("CUDA_CHECKED", "FORWARD_CHECKED", "LOSS_CHECKED", "BACKWARD_CHECKED",
          "OPTIMIZER_ATTEMPTED", "OPTIMIZER_RETURNED")
BASELINE_FIELDS = {"step", "physical_slot_indices", "shared_crop_boxes",
                   "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"}
SCOPE = dict(TRAIN_admitted=False, data_or_provider_authenticated=False,
    numeric_callbacks_independently_authenticated=False, journal_storage_authenticated=False,
    NN_trainer_implemented=False, GPU_ownership_acquired=False, model_approved=False,
    whole_process_memory_contained=False, callback_buffers_retained_by_library=False,
    outer_declaration_rows_retained_at_most=8, adapter_declaration_rows_at_most=8,
    pending_batches_at_most=1,
    workers=0, prefetch=0, resume=False, retry=False, legacy_filesystem_aliases_created=False)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def encoded(value):
    """HH_261012 - Deny foreign serialization/copy hooks before retaining bounded builtin metadata."""
    count = 0
    def check(item, depth=0):
        nonlocal count
        count += 1
        require(count <= 8192 and depth <= 24, "bounded metadata structure required")
        if type(item) is dict:
            require(len(item) <= 128 and all(type(k) is str and len(k) <= 128 for k in item),
                    "bounded builtin dictionary required")
            for child in item.values():
                check(child, depth + 1)
        elif type(item) in (list, tuple):
            require(len(item) <= 128, "bounded builtin metadata list required")
            for child in item:
                check(child, depth + 1)
        else:
            require(type(item) in (str, int, float, bool, type(None)), "builtin metadata scalar required")
            require(type(item) is not str or len(item) <= MAX_METADATA_BYTES, "metadata text too large")
            require(type(item) is not int or item.bit_length() <= 128, "metadata integer too large")
            require(type(item) is not float or math.isfinite(item), "finite metadata number required")
    check(value)
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    require(len(raw) <= MAX_METADATA_BYTES, "metadata byte cap exceeded")
    return raw


def metadata_sha(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def safe_error(error):
    args = BaseException.args.__get__(error, type(error))
    message = args[0] if type(args) is tuple and args and type(args[0]) is str else "non-string exception message"
    return dict(type=type(error).__name__[:64], message=message[:2048], truncated=len(message) > 2048)


def validate_row(row, expected):
    """HH_261012 - Check stream metadata only; the frozen adapter still checks actual input bytes."""
    encoded(row)
    require(type(row) is dict and set(row) == BASELINE_FIELDS, "exact six-field baseline required")
    require(type(row["step"]) is int and row["step"] == expected, "global one-based chronology differs")
    slots, boxes, bits = (row[k] for k in ("physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits"))
    require(type(slots) is list and len(slots) == 8 and all(type(v) is int and 0 <= v < 1209 for v in slots),
            "eight exact physical TRAIN slots required")
    require(type(bits) is list and len(bits) == 8 and all(type(v) is int and v == 0 for v in bits),
            "eight integer FALSE bits required")
    require(type(boxes) is list and len(boxes) == 8, "eight original crop proposals required")
    for box in boxes:
        require(type(box) is list and len(box) == 4 and all(type(v) is int for v in box), "integer crop box required")
        top, left, height, width = box
        require(0 <= top < 360 and 0 <= left < 640 and 0 < height <= 360 - top and 0 < width <= 640 - left,
                "native crop box outside image")
    require(sha(row["RGB_sha256"]) and sha(row["target_sha256"]), "baseline hashes required")


def make_serial_adapter(rows, reader, crop, *, mode, checkpoint):
    from .map_serial_training_input_adapter_20261008_v1 import SerialTrainingInputAdapter
    return SerialTrainingInputAdapter(rows, reader, crop, mode=mode, checkpoint=checkpoint)


def build_cpu_tensors(source, *, baseline, mode, checkpoint):
    from .map_torch_cpu_input_bridge_20261012_v1 import build_cpu_tensor_batch
    return build_cpu_tensor_batch(source, expected_step=baseline["step"], expected_mode=mode,
        expected_baseline=baseline, owner_pid=os.getpid(), owner_thread_ident=threading.get_ident(),
        checkpoint=checkpoint)


def verify_cpu_tensors(bundle, *, checkpoint):
    from .map_torch_cpu_input_bridge_20261012_v1 import verify_cpu_tensor_batch
    return verify_cpu_tensor_batch(bundle, checkpoint=checkpoint)


class NonOwningReader:
    """HH_261012 - A chunk closes its facade, never the outer reader or retained decoder cache."""
    def __init__(self, reader, owner_check):
        self._reader, self._owner_check, self._closed = reader, owner_check, False
        self.close_calls, self.closing_errors = 0, []

    def read(self, slot):
        self._owner_check()
        require(not self._closed, "chunk reader facade is closed")
        return self._reader.read(slot)

    def close(self):
        self._owner_check()
        if not self._closed:
            self._closed = True
            self.close_calls += 1


class StreamingTrainingBoundary:
    """HH_261012 - Stream one declared arm with bounded chunks and a create-only external journal sink.

    consume(bundle, mark) must mark PHASES in order at actual callback boundaries
    and return step/optimizer_updates plus bounded metadata. An attempted but
    non-returning optimizer is UNKNOWN_PARTIAL, not known unchanged. A returned
    update is never undone or retried when subsequent journal/ACK work fails.
    journal.append(record) must return written/flushed/step/record_sha256;
    journal.close() must return closed/rows_written/rows_flushed/closing_errors.
    Sink and callback claims remain externally authenticated responsibilities.
    """
    def __init__(self, rows, reader, crop, *, mode, declared_steps, consume, journal_factory,
                 checkpoint=lambda: None, adapter_factory=make_serial_adapter,
                 build=build_cpu_tensors, verify=verify_cpu_tensors):
        require(type(declared_steps) is int and 1 <= declared_steps <= 20000, "declared step count in1..20000 required")
        require(type(mode) is str and mode in MODES, "explicit crop arm required")
        require(callable(getattr(rows, "__next__", None)), "single-pass iterator required; no full-run list")
        require(all(callable(v) for v in (crop, consume, journal_factory, checkpoint, adapter_factory, build, verify))
            and callable(getattr(reader, "read", None)) and callable(getattr(reader, "close", None)),
            "explicit reader and lifecycle callbacks required")
        self._pid, self._thread = os.getpid(), threading.get_ident()
        self._rows, self._reader, self._crop, self._mode, self._total = rows, reader, crop, mode, declared_steps
        self._consume, self._journal_factory, self._checkpoint = consume, journal_factory, checkpoint
        self._adapter_factory, self._build, self._verify = adapter_factory, build, verify
        self._busy = self._started = self._closed = self._reader_closed = False
        self._active_callback, self._phase_index, self._active_step = False, -1, None
        self._adapter = self._journal = self._facade = None
        self._chunk_rows = []
        self._phase, self._failure, self._tail = "PREPARED", None, None
        self._cleanup_errors, self._last_chunk, self._last_journal = [], None, None
        self._counts = dict(rows_examined=0, chunks_opened=0, chunks_closed=0, produced=0,
            CPU_checked=0, optimizer_attempted=0, optimizer_returned=0, ACK_returned=0,
            fully_journaled_steps=0, journal_rows_written=0, maximum_buffered_rows=0, outer_reader_close_calls=0)
        self._phase_counts = {phase: 0 for phase in PHASES}
        self._journal_sha = hashlib.sha256()

    def _owner(self):
        require(os.getpid() == self._pid and threading.get_ident() == self._thread,
                "boundary belongs to one owner process/thread")

    def _guard(self):
        self._owner()
        self._checkpoint()
        self._owner()

    def _mark(self, phase):
        self._owner()
        require(self._active_callback and type(phase) is str and self._phase_index + 1 < len(PHASES)
            and phase == PHASES[self._phase_index + 1], "numeric callback phase skipped/repeated/outside callback")
        # HH_261012 - Record a returned optimizer before a later checkpoint can fail.
        self._phase_index += 1
        self._phase = phase
        self._phase_counts[phase] += 1
        if phase == "OPTIMIZER_ATTEMPTED":
            self._counts["optimizer_attempted"] += 1
        elif phase == "OPTIMIZER_RETURNED":
            self._counts["optimizer_returned"] += 1
        self._guard()

    def _append(self, record):
        self._guard()
        pin = metadata_sha(record)
        receipt = self._journal.append(record)
        require(type(receipt) is dict and set(receipt) == {"written", "flushed", "step", "record_sha256"}
            and receipt["written"] is True and receipt["flushed"] is True
            and type(receipt["step"]) is int and receipt["step"] == record["step"]
            and receipt["record_sha256"] == pin,
            "strict journal written/flushed receipt differs")
        require(metadata_sha(record) == pin, "journal callback mutated the record")
        self._counts["journal_rows_written"] += 1
        self._journal_sha.update(bytes.fromhex(pin))
        self._guard()

    def _step(self, baseline):
        step = baseline["step"]
        self._active_step, self._phase_index = step, -1
        self._phase, self._tail = "PRODUCE", dict(step=step, baseline=deepcopy(baseline))
        baseline_pin = metadata_sha(baseline)
        source = self._adapter.next_batch()
        require(type(source.step) is int and source.step == step, "produced global step differs")
        self._counts["produced"] += 1
        source_receipt = source.receipt
        require(len(encoded(source_receipt)) <= 16 * 1024 and type(source_receipt) is dict
            and type(source_receipt.get("step")) is int and source_receipt["step"] == step
            and source_receipt.get("mode") == self._mode, "bounded native receipt chronology differs")
        source_receipt = deepcopy(source_receipt)
        source_pin = metadata_sha(source_receipt)
        # HH_261012 - Preserve selected crop/native witnesses, not merely their non-reconstructable hash.
        self._tail["source_receipt"] = deepcopy(source_receipt)
        self._phase = "BUILD_CPU"
        bundle = self._build(source, baseline=baseline, mode=self._mode, checkpoint=self._guard)
        self._phase = "VERIFY_CPU"
        proof = self._verify(bundle, checkpoint=self._guard)
        require(len(encoded(proof)) <= 16 * 1024, "bounded CPU proof required")
        require(type(proof) is dict and type(proof.get("step")) is int and proof["step"] == step
                and proof.get("mode") == self._mode, "CPU proof chronology differs")
        proof_pin = metadata_sha(proof)
        consumer = proof.get("consumer_receipt")
        require(type(consumer) is dict and set(consumer) == {"consumer", "accepted", "host_RGB_sha256", "host_target_sha256"}
            and type(consumer["consumer"]) is str and consumer["accepted"] is True
            and sha(consumer["host_RGB_sha256"]) and sha(consumer["host_target_sha256"]), "positive CPU input receipt required")
        consumer = deepcopy(consumer)
        self._counts["CPU_checked"] += 1
        self._tail["CPU_proof_sha256"] = proof_pin
        self._active_callback = True
        self._phase = "NUMERIC_CALLBACK"
        try:
            update = self._consume(bundle, self._mark)
        finally:
            self._active_callback = False
        require(len(encoded(update)) <= 4096, "bounded update receipt required")
        require(self._phase_index == len(PHASES) - 1 and type(update) is dict
            and type(update.get("step")) is type(update.get("optimizer_updates")) is int
            and update["step"] == update["optimizer_updates"] == step, "returned update chronology/phases differ")
        self._tail["update_receipt"] = deepcopy(update)
        self._phase = "REVERIFY_BEFORE_JOURNAL"
        require(metadata_sha(self._verify(bundle, checkpoint=self._guard)) == proof_pin
            and metadata_sha(baseline) == baseline_pin and metadata_sha(source.receipt) == source_pin,
            "numeric callback changed input proof, native receipt or baseline")
        self._phase = "UPDATE_JOURNAL"
        self._append(dict(schema=SCHEMA + ".event", step=step, mode=self._mode,
            event="OPTIMIZER_RETURNED_NOT_ACKED", baseline=deepcopy(baseline),
            source_receipt=deepcopy(source_receipt), input_proof=deepcopy(proof),
            update=deepcopy(update), actual_serial_ACK_returned=False))
        self._phase = "REVERIFY_BEFORE_ACK"
        require(metadata_sha(self._verify(bundle, checkpoint=self._guard)) == proof_pin
            and metadata_sha(source.receipt) == source_pin,
                "journal callback changed input proof")
        self._phase = "SERIAL_ACK"
        ack = self._adapter.acknowledge(step, consumer_receipt=consumer)
        # HH_261012 - A returned ACK stays returned even if validating or journaling it later fails.
        self._counts["ACK_returned"] += 1
        expected = dict(step=step, consumer_receipt=consumer, native_and_host_buffers_rehashed=True,
                        consumer_execution_authenticated=False)
        encoded(ack)
        require(ack == expected and type(ack.get("step")) is int
            and ack.get("native_and_host_buffers_rehashed") is True
            and ack.get("consumer_execution_authenticated") is False, "exact serial ACK receipt differs")
        self._tail["serial_ACK"] = deepcopy(ack)
        self._phase = "ACK_JOURNAL"
        self._append(dict(schema=SCHEMA + ".event", step=step, mode=self._mode,
            event="SERIAL_ACK_RETURNED", actual_serial_ACK_returned=True, ACK=deepcopy(ack),
            preceding_update_receipt_sha256=metadata_sha(update)))
        self._counts["fully_journaled_steps"] += 1
        self._active_step, self._tail = None, None
        # Source/bundle/proof are local only; no history of numeric objects is retained.

    def _record_failure(self, error):
        if self._failure is None:
            self._failure = dict(phase=self._phase, step=self._active_step, error=safe_error(error))

    def _cleanup(self, name, operation):
        try:
            return operation()
        except BaseException as error:
            require(len(self._cleanup_errors) < 8, "bounded cleanup error ledger exceeded")
            self._cleanup_errors.append(dict(scope=name, error=safe_error(error)))
            return None

    def _close_chunk(self, *, successful):
        if self._adapter is not None:
            if not successful and self._failure is not None:
                self._cleanup("adapter_abort", lambda: self._adapter.abort("outer boundary failed", step=self._active_step))
            summary = self._cleanup("adapter_close", self._adapter.close)
            if summary is not None:
                encoded(summary)
                self._last_chunk = deepcopy(summary)
                if successful:
                    require(summary.get("closed") is True and summary.get("PASS_batches") == len(self._chunk_rows)
                        and summary.get("FAIL_batches") == summary.get("UNEXECUTED_batches") == 0
                        and summary.get("pending_step") is None and not summary.get("cleanup_errors"),
                        "closed chunk chronology/cleanup differs")
            if successful:
                require(summary is not None, "adapter close failed")
            self._adapter = None
        if self._facade is not None:
            self._cleanup("reader_facade_close", self._facade.close)
            self._facade = None
        if self._journal is not None:
            summary = self._cleanup("journal_close", self._journal.close)
            if summary is not None:
                encoded(summary)
                self._last_journal = deepcopy(summary)
                if successful:
                    require(summary.get("closed") is True and type(summary.get("rows_written")) is int
                        and type(summary.get("rows_flushed")) is int
                        and summary["rows_written"] == summary["rows_flushed"] == 2 * len(self._chunk_rows)
                        and not summary.get("closing_errors"), "journal closing rows/flush/errors differ")
            if successful:
                require(summary is not None, "journal close failed")
            self._journal = None
        if successful:
            self._counts["chunks_closed"] += 1
        self._chunk_rows.clear()

    def run(self):
        self._owner()
        require(not self._busy and not self._started and not self._closed, "boundary cannot reenter/retry/resume")
        self._busy = self._started = True
        try:
            next_step = 1
            while next_step <= self._total:
                self._phase = "READ_DECLARATION_CHUNK"
                self._chunk_rows = []
                for step in range(next_step, min(next_step + CHUNK_STEPS, self._total + 1)):
                    self._active_step = step
                    self._guard()
                    row = next(self._rows)
                    self._counts["rows_examined"] += 1
                    validate_row(row, step)
                    self._chunk_rows.append(deepcopy(row))
                    self._counts["maximum_buffered_rows"] = max(self._counts["maximum_buffered_rows"], len(self._chunk_rows))
                self._phase, self._active_step = "OPEN_CHUNK", next_step
                self._facade = NonOwningReader(self._reader, self._owner)
                self._adapter = self._adapter_factory(deepcopy(self._chunk_rows), self._facade, self._crop,
                    mode=self._mode, checkpoint=self._guard)
                self._journal = self._journal_factory(self._counts["chunks_opened"] + 1)
                self._counts["chunks_opened"] += 1
                for row in self._chunk_rows:
                    self._step(row)
                next_step += len(self._chunk_rows)
                self._phase = "CLOSE_CHUNK"
                self._close_chunk(successful=True)
                self._guard()
            self._phase, self._active_step = "VERIFY_STREAM_END", None
            try:
                next(self._rows)
            except StopIteration:
                pass
            else:
                raise ValueError("declaration iterator has extra rows outside fixed endpoint")
            self._phase = "CLOSE_OUTER_READER"
        except BaseException as error:
            self._record_failure(error)
        finally:
            self._active_callback = False
            if self._adapter is not None or self._journal is not None or self._facade is not None:
                self._cleanup("chunk_cleanup", lambda: self._close_chunk(successful=False))
            self._close_outer_reader()
            self._closed = True
            self._busy = False
        return self.snapshot()

    def _close_outer_reader(self):
        if not self._reader_closed:
            self._reader_closed = True
            self._counts["outer_reader_close_calls"] += 1
            self._cleanup("outer_reader_close", self._reader.close)
            try:
                errors = getattr(self._reader, "closing_errors", [])
                encoded(errors)
                require(type(errors) is list and len(errors) <= 6, "bounded reader closing observations required")
                if errors:
                    self._cleanup_errors.append(dict(scope="outer_reader_closing_observations", errors=deepcopy(errors)))
            except BaseException as error:
                self._cleanup_errors.append(dict(scope="outer_reader_closing_observations", error=safe_error(error)))

    def close(self):
        self._owner()
        require(not self._busy, "cannot close boundary from a running callback")
        if not self._closed:
            self._close_outer_reader()
            self._closed = True
        return self.snapshot()

    def snapshot(self):
        self._owner()
        completed = self._counts["fully_journaled_steps"]
        failed_step = self._failure["step"] if self._failure is not None else None
        failed = int(type(failed_step) is int and completed < failed_step <= self._total)
        status = ("FAILED_RETAINED_NO_RETRY" if self._failure or self._cleanup_errors else
                  "COMPLETED_BOUNDARY_CALLBACKS_NOT_MODEL_APPROVAL" if self._closed and self._started else
                  "CLOSED_NOT_EXECUTED" if self._closed else "RUNNING" if self._started else "PREPARED")
        value = dict(schema=SCHEMA + ".summary", status=status, mode=self._mode, declared_steps=self._total,
            PASS_batches=completed, FAIL_batches=failed, UNEXECUTED_batches=self._total - completed - failed,
            counts=deepcopy(self._counts), callback_phase_counts=deepcopy(self._phase_counts),
            optimizer_partial_state=("UNKNOWN_PARTIAL_NO_RETRY" if self._counts["optimizer_attempted"] > self._counts["optimizer_returned"]
                                     else "NO_UNRETURNED_OPTIMIZER_ATTEMPT"),
            failure=deepcopy(self._failure), failed_tail=deepcopy(self._tail), cleanup_errors=deepcopy(self._cleanup_errors),
            last_closed_chunk=deepcopy(self._last_chunk), last_closed_journal=deepcopy(self._last_journal),
            journal_record_metadata_sequence_sha256=self._journal_sha.hexdigest(),
            current_buffered_rows=len(self._chunk_rows), closed=self._closed, scope=dict(SCOPE))
        encoded(value)
        return value
