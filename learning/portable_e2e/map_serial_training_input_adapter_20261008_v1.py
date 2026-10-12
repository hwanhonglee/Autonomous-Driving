"""HH_261008 - Prepare one-pass, worker0 native-to-NumPy input production with explicit consumer ACK.

The caller must authenticate the fixed original exposure rows, package/source
and provider bytes, ownership, permissions and budgets. This library does not
authenticate ROOT, acquire a lease, open files, create a reader, train a model or
authorize TRAIN. An injected reader/crop is not proof of those external checks.
Only NumPy CPU host arrays are produced; no Torch/GPU numerical equivalence or
trainer/epoch integration is claimed. Returned arrays remain mutable. Their
original descriptors are privately snapshotted and rechecked at consumer ACK.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from functools import wraps
import hashlib
import json
import os
import threading

from . import map_crop_batch_worker_20261008_v3 as worker
from . import map_crop_preservation_journal_20261008_v1 as journal

SCHEMA = "portable_e2e.map_serial_training_input_adapter.20261008.v1"
ORIGINAL_CROP_CONTROL = "ORIGINAL_CROP_CONTROL"
SIGN_PRESERVING_CANDIDATE = "SIGN_PRESERVING_CANDIDATE"
MODES = (ORIGINAL_CROP_CONTROL, SIGN_PRESERVING_CANDIDATE)
RECEIPT_FIELDS = frozenset(("schema", "mode", "step", "provided_baseline", "provided_baseline_sha256",
    "control_exposure", "candidate_worker_receipt", "native_slots", "native_arrays", "host_arrays", "scope"))
CONSUMER_FIELDS = frozenset(("consumer", "accepted", "host_RGB_sha256", "host_target_sha256"))
MAX_RECEIPT_BYTES = 16 * 1024
MAX_ARRAY_BYTES = 8 * 3 * 360 * 640 * 4
NATIVE_SHAPES = {"RGB": (8, 360, 640, 3), "target": (8, 360, 640)}
HOST_SHAPES = {"RGB": (8, 3, 360, 640), "target": (8, 360, 640)}
SCOPE = dict(worker_count=0, epochs_supported=False, multiprocess_supported=False,
    ROOT_or_data_authenticated_by_adapter=False, injected_crop_source_authenticated_by_adapter=False,
    TRAIN_admitted_by_adapter=False, consumer_execution_authenticated_by_adapter=False,
    Torch_or_GPU_numerical_equivalence_claimed=False, filesystem_written_by_adapter=False,
    public_arrays_immutable=False, owner_thread_bound=True, reentrant_calls_supported=False,
    sampled_guards_are_OS_containment=False, optimizer_steps=0, NN=0)


def require(value, message):
    if not value:
        raise ValueError(message)


def _encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _metadata_sha(value):
    return hashlib.sha256(_encoded(value)).hexdigest()


def _typed_equal(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(_typed_equal(left[key], right[key]) for key in left)
    if type(left) in (list, tuple):
        return len(left) == len(right) and all(_typed_equal(a, b) for a, b in zip(left, right))
    return left == right


def _error(error):
    # HH_261008 - Preserve the first real error with bounded scalar evidence, without invoking foreign repr/str.
    if isinstance(error, BaseException):
        arguments = BaseException.args.__get__(error, type(error))
        message = arguments[0] if type(arguments) is tuple and arguments and type(arguments[0]) is str else "non-string exception message"
        return dict(type=type(error).__name__[:64], message=message[:2048], message_truncated=len(message) > 2048)
    require(type(error) is str and 0 < len(error) <= 2048, "explicit exception or bounded error text required")
    return dict(type="CallerAbort", message=error, message_truncated=False)


def _steps(values):
    require(type(values) is list and 1 <= len(values) <= 20000
        and all(type(step) is int and 1 <= step <= 20000 for step in values)
        and values == list(range(values[0], values[0]+len(values))), "fixed distinct contiguous original steps required")
    return list(values)


def plan_contiguous_shards(declared_steps, *, batches_per_shard=8):
    """HH_261008 - Plan scalar chronology only; no file, epoch or payload permission is created."""
    steps = _steps(declared_steps)
    require(type(batches_per_shard) is int and 1 <= batches_per_shard <= 64, "fixed shard size in1..64 required")
    return [dict(ordinal=ordinal, declared_steps=group, first_step=group[0], last_step=group[-1], expected_batches=len(group))
        for ordinal, group in enumerate(steps[start:start+batches_per_shard]
            for start in range(0, len(steps), batches_per_shard))]


def _array_sha(value):
    import numpy as np
    require(type(value) is np.ndarray and value.flags.c_contiguous and value.nbytes <= MAX_ARRAY_BYTES,
        "bounded exact contiguous NumPy array required")
    view = memoryview(value).cast("B")
    try:
        return hashlib.sha256(view).hexdigest()
    finally:
        view.release()


def _descriptor(value, *, shape, dtype, maximum):
    import numpy as np
    require(type(value) is np.ndarray and value.shape == shape and value.dtype == dtype
        and value.flags.owndata and value.flags.c_contiguous and not value.flags.f_contiguous,
        "owned C-contiguous exact native/host array ABI required")
    finite = bool(np.isfinite(value).all())
    lower, upper = value.min().item(), value.max().item()
    require(finite and 0 <= lower <= upper <= maximum, "finite declared native/host range required")
    return dict(shape=list(value.shape), dtype=str(value.dtype), nbytes=value.nbytes,
        owned=True, C_contiguous=True, finite=True, minimum=lower, maximum=upper, sha256=_array_sha(value))


def _descriptors(native_RGB, native_target, host_RGB, host_target):
    import numpy as np
    arrays = (native_RGB, native_target, host_RGB, host_target)
    require(not any(np.shares_memory(a, b) for i, a in enumerate(arrays) for b in arrays[i+1:]),
        "native and host arrays must own separate non-aliasing storage")
    native = dict(RGB=_descriptor(native_RGB, shape=NATIVE_SHAPES["RGB"], dtype=np.uint8, maximum=255),
        target=_descriptor(native_target, shape=NATIVE_SHAPES["target"], dtype=np.uint8, maximum=6))
    host = dict(RGB=_descriptor(host_RGB, shape=HOST_SHAPES["RGB"], dtype=np.float32, maximum=1),
        target=_descriptor(host_target, shape=HOST_SHAPES["target"], dtype=np.int64, maximum=6))
    return native, host


def _observed_arrays(arrays):
    """Retain current failed buffer descriptors without hashing an oversized consumer replacement."""
    import numpy as np
    result = {}
    for name, value in zip(("native_RGB", "native_target", "host_RGB", "host_target"), arrays):
        row = dict(raw_type=type(value).__name__, sha256=None)
        if type(value) is np.ndarray:
            row.update(shape=list(value.shape), dtype=str(value.dtype), nbytes=value.nbytes,
                owned=bool(value.flags.owndata), C_contiguous=bool(value.flags.c_contiguous))
            if value.flags.c_contiguous and value.nbytes <= MAX_ARRAY_BYTES:
                row["sha256"] = _array_sha(value)
        result[name] = row
    return result


def _observed_receipt(value):
    # HH_261008 - Retain bounded builtin metadata mutations without invoking caller copy/JSON hooks.
    count = 0
    def check(item, depth=0):
        nonlocal count
        count += 1
        require(count <= 4096 and depth <= 16, "exported receipt observation exceeds structural bounds")
        if type(item) is dict:
            require(len(item) <= 128 and all(type(key) is str and len(key) <= 128 for key in item), "receipt key bounds")
            for child in item.values():
                check(child, depth+1)
        elif type(item) in (list, tuple):
            require(len(item) <= 128, "receipt list bounds")
            for child in item:
                check(child, depth+1)
        else:
            require(type(item) in (str, int, float, bool, type(None)), "builtin receipt scalar required")
            require(type(item) is not str or len(item) <= MAX_RECEIPT_BYTES, "receipt text bounds")
            require(type(item) is not int or item.bit_length() <= 128, "receipt integer bounds")
    check(value)
    require(len(_encoded(value)) <= MAX_RECEIPT_BYTES, "exported receipt byte bounds")
    return deepcopy(value)


@dataclass(frozen=True)
class ProducedBatch:
    """HH_261008 - Frozen field bindings do not make these actual array buffers immutable."""
    step: int
    native_RGB: object
    native_target: object
    host_RGB: object
    host_target: object
    receipt: dict


class _TrackedReader:
    def __init__(self, adapter, row):
        self.adapter, self.row, self.native_slots = adapter, row, []

    def read(self, slot):
        self.adapter._guard()
        position = len(self.native_slots)
        require(position < 8 and type(slot) is int and slot == self.row["physical_slot_indices"][position],
            "reader query differs from the fixed original slot order")
        self.adapter.read_attempts += 1
        rgb, target = self.adapter.reader.read(slot)
        worker.native_pair(rgb, target)
        require(rgb.flags.owndata and target.flags.owndata, "reader must return owned native source slices")
        self.native_slots.append(dict(position=position, slot=slot, native_RGB_sha256=worker.array_hash(rgb),
            native_target_sha256=worker.array_hash(target), reader_provenance_authenticated_by_worker=False))
        self.adapter._partial_native_slots = deepcopy(self.native_slots)
        return rgb, target


def _serialized(operation):
    # HH_261008 - Bind mutation to the owner thread and deny reentry before any injected callback.
    @wraps(operation)
    def wrapped(self, *args, **kwargs):
        self._owner()
        require(not self._busy, "reentrant adapter operations are denied")
        self._busy = True
        try:
            return operation(self, *args, **kwargs)
        finally:
            self._busy = False
    return wrapped


class SerialTrainingInputAdapter:
    """HH_261008 - One owner process, one traversal, one pending batch and explicit consumption ACK."""
    def __init__(self, rows, reader, original_crop, *, mode, checkpoint=None):
        require(type(rows) is list and 1 <= len(rows) <= 20000, "fixed supplied exposure rows required")
        for row in rows:
            journal.validate_baseline(row)
        self.steps = _steps([row["step"] for row in rows])
        require(type(mode) is str and mode in MODES and callable(getattr(reader, "read", None))
            and callable(getattr(reader, "close", None)) and callable(original_crop)
            and (checkpoint is None or callable(checkpoint)), "explicit mode/reader/crop/checkpoint required")
        self._rows, self.reader, self.crop, self.mode = deepcopy(rows), reader, original_crop, mode
        self._checkpoint, self.owner_pid = checkpoint, os.getpid()
        self.owner_thread_ident, self._busy = threading.get_ident(), False
        self._cursor = self.produced = self.production_failed = self.production_attempts = self.read_attempts = self.acked = 0
        self.consumption_attempts = self.consumption_attempted_batches = 0
        self._consumption_active_step, self._pending_step = None, None
        self._closed, self._failed = False, False
        self._pending = self._pending_arrays = self._pending_expected = None
        self._partial_native_slots, self._active_step = [], None
        self.failure, self.failed_tail, self.cleanup_errors, self.reader_closing_observations = None, None, [], []
        self.acknowledgements = []

    def __getstate__(self):
        raise TypeError("single-process adapter cannot be pickled, fork-shared or used as a multiprocess dataset")

    def _owner(self):
        require(os.getpid() == self.owner_pid, "adapter owner PID differs; inherited input traversal denied")
        require(threading.get_ident() == self.owner_thread_ident, "adapter belongs to one owner thread")

    def _guard(self):
        self._owner()
        if self._checkpoint is not None:
            self._checkpoint()

    def _consumer_attempt(self):
        if self._pending is not None:
            self.consumption_attempts += 1
            if self._consumption_active_step is None:
                self.consumption_attempted_batches += 1
                self._consumption_active_step = self._active_step

    def _fail(self, error, phase):
        if self.failure is None:
            self.failure = dict(phase=phase, step=self._active_step, error=_error(error))
        if not self._failed:
            self._failed = True
            self.failed_tail = dict(step=self._active_step, phase=phase,
                original_provided_baseline=deepcopy(self._rows[self._cursor]) if self._cursor < len(self._rows) else None,
                actual_partial_native_slots=deepcopy(self._partial_native_slots),
                produced_receipt=deepcopy(self._pending_expected))
            if self._pending is not None:
                try:
                    self.failed_tail["exported_production_receipt"] = _observed_receipt(self._pending.receipt)
                    self.failed_tail["exported_buffer_bindings"] = _observed_arrays((self._pending.native_RGB,
                        self._pending.native_target, self._pending.host_RGB, self._pending.host_target))
                except BaseException as secondary:
                    self.failed_tail["exported_observation_error"] = _error(secondary)
            if self._pending_arrays is not None:
                try:
                    self.failed_tail["current_buffers"] = _observed_arrays(self._pending_arrays)
                except BaseException as secondary:
                    self.failed_tail["current_buffer_observation_error"] = _error(secondary)

    def _original(self, row, tracked):
        import numpy as np
        rgb, target = np.empty(NATIVE_SHAPES["RGB"], np.uint8), np.empty(NATIVE_SHAPES["target"], np.uint8)
        for position, (slot, proposal) in enumerate(zip(row["physical_slot_indices"], row["shared_crop_boxes"])):
            source_rgb, source_target = tracked.read(slot)
            source_rgb, source_target = source_rgb.copy(), source_target.copy()
            before = worker.array_hash(source_rgb), worker.array_hash(source_target)
            out_rgb, out_target = self.crop(source_rgb, source_target, proposal)
            worker.native_pair(out_rgb, out_target)
            require(before == (worker.array_hash(source_rgb), worker.array_hash(source_target)), "original crop changed its source arrays")
            rgb[position], target[position] = out_rgb, out_target
            self._guard()
        require(worker.array_hash(rgb) == row["RGB_sha256"] and worker.array_hash(target) == row["target_sha256"],
            "CONTROL must reproduce BOTH original RGB and GT batch SHA")
        return rgb, target

    @_serialized
    def next_batch(self):
        require(not self._closed and not self._failed, "closed or failed adapter cannot be rerun")
        if self._cursor == len(self._rows):
            self._guard()
            raise StopIteration("fixed traversal exhausted; no epoch or resume")
        try:
            self._guard()
            require(self._pending is None, "pending batch must be acknowledged before another production")
            self._active_step = self.steps[self._cursor]
            self._partial_native_slots = []
            self.production_attempts += 1
            row = deepcopy(self._rows[self._cursor])
            tracked = _TrackedReader(self, row)
            candidate = None
            if self.mode == ORIGINAL_CROP_CONTROL:
                rgb, target = self._original(row, tracked)
                control = deepcopy(row)
            else:
                rgb, target, candidate = worker.build_batch(row, tracked, self.crop, checkpoint=self._guard)
                control = None
            require(_typed_equal(row, self._rows[self._cursor]), "injected crop changed the fixed provided baseline")
            require(len(tracked.native_slots) == 8, "exact original eight native source witnesses required")
            if candidate is not None:
                require(_typed_equal(candidate["native_slots"], tracked.native_slots), "candidate native witnesses differ from actual reads")
            import numpy as np
            host_RGB = rgb.transpose(0, 3, 1, 2).astype(np.float32, order="C", copy=True)
            host_RGB /= np.float32(255)
            host_target = target.astype(np.int64, order="C", copy=True)
            native, host = _descriptors(rgb, target, host_RGB, host_target)
            receipt = dict(schema=SCHEMA+".batch", mode=self.mode, step=self._active_step,
                provided_baseline=deepcopy(row), provided_baseline_sha256=_metadata_sha(row),
                control_exposure=control, candidate_worker_receipt=candidate,
                native_slots=deepcopy(tracked.native_slots), native_arrays=native, host_arrays=host, scope=dict(SCOPE))
            require(set(receipt) == RECEIPT_FIELDS and len(_encoded(receipt)) <= MAX_RECEIPT_BYTES,
                "bounded exact adapter receipt required")
            self._pending_arrays = (rgb, target, host_RGB, host_target)
            self._pending_expected = deepcopy(receipt)
            self._pending = ProducedBatch(self._active_step, rgb, target, host_RGB, host_target, deepcopy(receipt))
            self._pending_step = self._active_step
            self.produced += 1
            return self._pending
        except BaseException as error:
            if self._pending is None and self._active_step is not None and self.production_attempts > self.produced+self.production_failed:
                self.production_failed += 1
            self._fail(error, "production" if self._pending is None else "unacknowledged_production_request")
            raise

    @_serialized
    def acknowledge(self, step, *, consumer_receipt):
        require(not self._closed and not self._failed, "closed or failed adapter cannot acknowledge again")
        self._consumer_attempt()
        try:
            self._guard()
            require(self._pending is not None and type(step) is int and step == self._active_step,
                "exact pending step required for ordered consumer ACK")
            require(type(consumer_receipt) is dict and set(consumer_receipt) == CONSUMER_FIELDS
                and type(consumer_receipt["consumer"]) is str and 0 < len(consumer_receipt["consumer"]) <= 128
                and "\0" not in consumer_receipt["consumer"] and consumer_receipt["accepted"] is True
                and journal.sha(consumer_receipt["host_RGB_sha256"]) and journal.sha(consumer_receipt["host_target_sha256"]),
                "exact strict positive consumer receipt required")
            expected = self._pending_expected
            require(_typed_equal(self._pending.receipt, expected), "consumer changed its supplied production receipt")
            require(type(self._pending.step) is int and self._pending.step == self._active_step and all(a is b for a, b in zip(
                (self._pending.native_RGB, self._pending.native_target, self._pending.host_RGB, self._pending.host_target), self._pending_arrays)),
                "consumer rebound a produced buffer or step")
            native, host = _descriptors(*self._pending_arrays)
            require(_typed_equal(native, expected["native_arrays"]) and _typed_equal(host, expected["host_arrays"]),
                "pending native or host bytes changed before consumer ACK")
            require(consumer_receipt["host_RGB_sha256"] == host["RGB"]["sha256"]
                and consumer_receipt["host_target_sha256"] == host["target"]["sha256"], "consumer host SHA differs from actual pending arrays")
            self.acknowledgements.append(dict(step=step, consumer_receipt=deepcopy(consumer_receipt),
                native_and_host_buffers_rehashed=True, consumer_execution_authenticated=False))
            self.acked += 1
            self._cursor += 1
            self._pending = self._pending_arrays = self._pending_expected = None
            self._pending_step = self._consumption_active_step = None
            self._active_step = None
            self._partial_native_slots = []
            return deepcopy(self.acknowledgements[-1])
        except BaseException as error:
            self._fail(error, "consumer_acknowledgement")
            raise

    @_serialized
    def abort(self, error, *, step=None):
        require(not self._closed, "closed adapter cannot receive a new abort")
        require(step is None or type(step) is int and step == self._active_step, "abort must identify the actual active step")
        if self._failed:
            return self.stats_snapshot()
        self._consumer_attempt()
        # HH_261008 - The actual consumer error precedes cleanup guards; a later guard cannot replace it.
        self._fail(error, "consumer_abort" if self._pending is not None else "owner_abort")
        try:
            self._guard()
        except BaseException as secondary:
            self.cleanup_errors.append(dict(scope="abort_checkpoint", error=_error(secondary)))
        return self.stats_snapshot()

    @_serialized
    def close(self):
        if self._closed:
            return self.stats_snapshot()
        if self._pending is not None and not self._failed:
            self._fail(ValueError("produced batch closed without positive consumer ACK"), "unacknowledged_close")
        self._closed = True
        try:
            self._guard()
        except BaseException as error:
            self.cleanup_errors.append(dict(scope="before_reader_close", error=_error(error)))
        try:
            self.reader.close()
        except BaseException as error:
            self.cleanup_errors.append(dict(scope="reader_close", error=_error(error)))
        finally:
            try:
                rows = getattr(self.reader, "closing_errors", [])
                require(type(rows) is list and len(rows) <= 6 and all(type(row) is dict and set(row) == {"scope", "type", "message"}
                    and type(row["scope"]) is str and len(row["scope"]) <= 64
                    and type(row["type"]) is str and len(row["type"]) <= 64
                    and type(row["message"]) is str and len(row["message"]) <= 512 for row in rows),
                    "bounded closing ledger required")
                self.reader_closing_observations = deepcopy(rows)
            except BaseException as error:
                self.cleanup_errors.append(dict(scope="reader_closing_observations", error=_error(error)))
        try:
            self._guard()
        except BaseException as error:
            self.cleanup_errors.append(dict(scope="after_reader_close", error=_error(error)))
        # HH_261008 - Preserve scalar failed-tail evidence, then release only the adapter's buffer references.
        # A caller can still hold its ProducedBatch; this is not a claim that all process memory is freed.
        self._pending = self._pending_arrays = self._pending_expected = None
        return self.stats_snapshot()

    def stats_snapshot(self):
        # HH_261008 - Produced pending data remains produced PASS even when its consumption fails.
        self._owner()
        failed = int(self._failed and self._active_step is not None)
        consumption_failed = int(self._failed and self._consumption_active_step is not None)
        return dict(schema=SCHEMA+".summary", mode=self.mode, declared_steps=list(self.steps),
            expected_batches=len(self.steps), PASS_batches=self.acked, FAIL_batches=failed,
            UNEXECUTED_batches=len(self.steps)-self.acked-failed,
            produced_batches=self.produced, production_attempts=self.production_attempts,
            production_FAIL_batches=self.production_failed,
            production_UNEXECUTED_batches=len(self.steps)-self.produced-self.production_failed,
            consumption_attempts=self.consumption_attempts,
            consumption_attempted_batches=self.consumption_attempted_batches,
            consumption_PASS_batches=self.acked, consumption_FAIL_batches=consumption_failed,
            consumption_UNEXECUTED_batches=len(self.steps)-self.acked-consumption_failed,
            actual_native_read_attempts=self.read_attempts, pending_step=self._pending_step,
            failure=deepcopy(self.failure), failed_tail=deepcopy(self.failed_tail),
            cleanup_errors=deepcopy(self.cleanup_errors), reader_closing_observations=deepcopy(self.reader_closing_observations),
            acknowledgements=deepcopy(self.acknowledgements), owner_PID=self.owner_pid,
            owner_thread_ident=self.owner_thread_ident, closed=self._closed,
            status="FAILED_RETAINED" if self.failure is not None or self.cleanup_errors else
                "COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL" if self._closed and self.acked == len(self.steps) else
                "CLOSED_UNEXECUTED_INPUTS" if self._closed else "PREPARED_OR_PARTIAL_INPUT_TRAVERSAL",
            provided_rows_sha256=_metadata_sha(self._rows), original20000_batches_outside_declaration=20000-len(self.steps),
            scope=dict(SCOPE))
