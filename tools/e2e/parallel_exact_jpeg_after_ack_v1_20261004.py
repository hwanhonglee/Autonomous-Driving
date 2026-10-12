"""HH_261004 - Encode one acknowledged six-camera bundle with three bounded workers.

All worker tasks join before any original-role-ordered, create-only write.
Nothing crosses the next native tick: this is recording research, not a new
policy, asynchronous capture, camera source, or learned driving capability.
"""

from __future__ import annotations

import ast
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import copy
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import threading
import time
import types

from . import defer_exact_jpeg_until_ACK_research_20261001 as original


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.parallel_exact_JPEG_after_ACK.20261004.v1"
WORKERS = 3
CAMERA_ORDER = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT",
                "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
SOURCE_SHA256 = {
    "tools/e2e/defer_exact_jpeg_until_ACK_research_20261001.py":
        "14efe6c55f3ef082b340d7ccaa5d3d4b5de0fb619aee09523ebc1245565c8a35",
    "tools/e2e/collect_carla_vad_expert.py":
        "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08",
    "tools/e2e/collect_temporal_route_recovery_fast45_20261001.py":
        "4b8e3d5ffbab373fa714c3e854858b9262343ad5a3d28c7eae7717eb5bc38447",
    "tools/e2e/collect_temporal_route_joint_town03_straight_v2_20261004.py":
        "0286de67c7bf16497f311a16bf0b9d9d5406fedda55108a085cfa604052bd574",
}
_active = False
_pure_encoder = None


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def verify_frozen():
    # HH_261004 - Hash immutable inputs once at setup, never on every frame.
    for name, expected in SOURCE_SHA256.items():
        path = ROOT / name
        require(path.is_file() and not path.is_symlink()
                and hashlib.sha256(path.read_bytes()).hexdigest() == expected,
                "frozen exact JPEG dependency changed: " + name)


def encoder_AST():
    """HH_261004 - Split only the final write from the exact original encoder AST."""
    source = ROOT / "tools/e2e/collect_carla_vad_expert.py"
    payload = source.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == SOURCE_SHA256[
        "tools/e2e/collect_carla_vad_expert.py"], "original JPEG source pin differs")
    definition = next(node for node in ast.parse(payload).body
                      if isinstance(node, ast.FunctionDef) and node.name == "_save_jpeg")
    changed = copy.deepcopy(definition)
    require(ast.unparse(changed.body[-1]) == "output.write_bytes(encoded.tobytes())",
            "one exact final original JPEG write required")
    write = copy.deepcopy(changed.body[-1])
    changed.body[-1] = ast.Return(value=copy.deepcopy(write.value.args[0]))
    restored = copy.deepcopy(changed)
    restored.body[-1] = write
    require(ast.dump(restored) == ast.dump(definition),
            "another original color, quality, encoder or exception statement changed")
    return definition, changed


def exact_encoder(save_jpeg):
    """HH_261004 - Use the actual original function globals, including its exception class."""
    definition, changed = encoder_AST()
    require(isinstance(save_jpeg, types.FunctionType)
            and save_jpeg.__name__ == "_save_jpeg"
            and Path(inspect.getsourcefile(save_jpeg)).resolve()
            == (ROOT / "tools/e2e/collect_carla_vad_expert.py").resolve(),
            "actual original collector JPEG provider required")
    actual = ast.parse(inspect.getsource(save_jpeg)).body[0]
    require(ast.dump(actual) == ast.dump(definition), "actual encoder source differs")
    namespace = dict(save_jpeg.__globals__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":original_encoder_without_write", "exec"), namespace)
    return namespace["_save_jpeg"]


def _encode_image(image, target, quality):
    """HH_261004 - Pure replay seam; compile the frozen numerical encoder once without collector/Torch/CARLA imports."""
    global _pure_encoder
    if _pure_encoder is None:
        _, changed = encoder_AST()
        source = ast.parse((ROOT / "tools/e2e/collect_carla_vad_expert.py").read_bytes())
        error = copy.deepcopy(next(node for node in source.body
                                   if isinstance(node, ast.ClassDef) and node.name == "CollectionError"))
        require(ast.unparse(error.bases[0]) == "RuntimeError" and len(error.body) == 1,
                "exact original replay exception definition required")
        namespace = dict(Any=object, Path=Path)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[error, changed], type_ignores=[])),
                     str(Path(__file__)) + ":pure_original_encoder_replay", "exec"), namespace)
        _pure_encoder = namespace["_save_jpeg"]
    return _pure_encoder(image, target, quality)


def _canonical_target(output, image, target, ordinal):
    target = Path(target)
    require(target.is_absolute() and target == target.resolve()
            and output.is_absolute() and output == output.resolve(),
            "canonical absolute owned JPEG output required")
    relative = target.relative_to(output)
    require(relative.parts == ("images", CAMERA_ORDER[ordinal], f"{image.frame:08d}.jpg"),
            "original six-camera role/order/frame JPEG target required")
    require(target.parent.is_dir() and not target.parent.is_symlink()
            and not target.exists() and not target.is_symlink(),
            "fresh regular owned JPEG target required")
    return target


def _write_create_only(target, payload):
    # HH_261004 - Recheck canonical parents, then never truncate an existing file or follow a final symlink.
    require(target == target.resolve() and target.parent.is_dir()
            and not target.parent.is_symlink(), "JPEG parent alias changed before write")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(target, flags, 0o666)
    with os.fdopen(descriptor, "wb") as stream:
        count = stream.write(payload)
        require(count == len(payload), "short exact JPEG write")


class ParallelExactJPEGAfterACK(original.ExactJPEGAfterACK):
    """HH_261004 - Inherit the original one-bundle queue; change only its synchronous encode scheduling."""

    def __init__(self, save_jpeg, output):
        super().__init__(save_jpeg, output)
        self.encoder = exact_encoder(save_jpeg)
        self.executor = ThreadPoolExecutor(max_workers=WORKERS, thread_name_prefix="hh_exactjpeg")
        self.closed = False
        self.flush_failed = False

    def _encode(self, image, target, quality, ordinal):
        row = dict(ordinal=ordinal, role=CAMERA_ORDER[ordinal], image_object_id=id(image),
                   quality=quality, worker_thread_id=threading.get_ident(),
                   encoding_started_monotonic_ns=time.perf_counter_ns(),
                   source_pixels_changed=False, source_timestamp_changed=False)
        payload = None
        error = None
        try:
            raw = image.raw_data
            require(type(image.frame) is int and image.frame >= 0
                    and type(image.timestamp) in (int, float) and math.isfinite(image.timestamp)
                    and image.timestamp >= 0 and image.width == 640 and image.height == 360
                    and isinstance(raw, bytes) and len(raw) == 640 * 360 * 4,
                    "immutable exact640x360 original BGRA source required")
            row.update(native_frame=image.frame,
                       source_image_timestamp_simulation_ns=int(round(image.timestamp * 1.e9)))
            source = hashlib.sha256(b"bgra8")
            source.update(raw)
            row.update(source_BGRA_encoding_prefixed_sha256=source.hexdigest(), source_byte_count=len(raw))
            payload = self.encoder(image, target, quality)
            require(isinstance(payload, bytes) and payload, "nonempty exact original encoded JPEG bytes required")
            row.update(encoded_JPEG_sha256=hashlib.sha256(payload).hexdigest(), encoded_JPEG_byte_count=len(payload))
        except BaseException as caught:
            # HH_261004 - Join every task; raise this exact exception later in original camera-role order.
            error = caught
        finally:
            row.update(encoding_finished_monotonic_ns=time.perf_counter_ns(),
                       encoding_failure=None if error is None else type(error).__name__ + ": " + str(error))
        return payload, error, row

    def close(self):
        if not self.closed:
            self.executor.shutdown(wait=True, cancel_futures=False)
            self.closed = True

    def _join_all(self, futures, items):
        # HH_261004 - Executor/cancellation failures also join every submitted future, without replacing an earlier validation error.
        results = []
        for ordinal, future in enumerate(futures):
            try:
                results.append(future.result())
            except BaseException as failed:
                image, target, quality = items[ordinal]
                row = dict(ordinal=ordinal, role=CAMERA_ORDER[ordinal], image_object_id=id(image),
                           native_frame=image.frame, quality=quality,
                           unexpected_future_failure=True, worker_execution_or_start_time_known=False,
                           future_result_observed_monotonic_ns=time.perf_counter_ns(),
                           encoding_failure=type(failed).__name__ + ": " + str(failed),
                           source_pixels_changed=False, source_timestamp_changed=False)
                results.append((None, failed, row))
        return results

    def flush(self, *, acknowledged, receipt=None):
        require(not self.closed and not self._flushing, "closed or reentrant parallel JPEG flush")
        if not self.pending:
            return
        require(not self.flush_failed, "failed original bundle cannot be retried or relabelled complete")
        self._flushing = True
        started = time.perf_counter_ns()
        items = tuple(self.pending)
        written, encodings, futures = [], [], []
        error = None
        joined = None
        try:
            require(0 < len(items) <= 6, "original one-six-camera-bundle bound required")
            for ordinal, (image, target, quality) in enumerate(items):
                _canonical_target(self.output, image, target, ordinal)
                futures.append(self.executor.submit(self._encode, image, target, quality, ordinal))
            # HH_261004 - No role write, subsequent frame, or tick begins before all submitted encodes join.
            encodings = self._join_all(futures, items)
            joined = time.perf_counter_ns()
            for (image, target, quality), (payload, failed, row) in zip(items, encodings):
                if failed is not None:
                    raise failed
                _canonical_target(self.output, image, target, row["ordinal"])
                row["write_started_monotonic_ns"] = time.perf_counter_ns()
                try:
                    _write_create_only(target, payload)
                finally:
                    row["write_finished_monotonic_ns"] = time.perf_counter_ns()
                written.append(str(target))
        except BaseException as caught:
            error = type(caught).__name__ + ": " + str(caught)
            self.flush_failed = True
            raise
        finally:
            # HH_261004 - Even submit/validation failure cannot leave workers using this source after flush.
            if len(encodings) != len(futures):
                encodings = self._join_all(futures, items)
            if joined is None:
                joined = time.perf_counter_ns()
            finished = time.perf_counter_ns()
            self.pending = list(items[len(written):])
            row = dict(schema=original.SCHEMA, frame=int(items[0][0].frame), enqueued_camera_count=len(items),
                       written_camera_count=len(written), written_files=written,
                       recording_started_monotonic_ns=started, recording_finished_monotonic_ns=finished,
                       recording_seconds=(finished-started)/1.e9, acknowledged_before_recording=bool(acknowledged),
                       ACK_receipt_sequence=receipt.get("sequence") if isinstance(receipt, dict) else None,
                       failure=error, all_queued_images_written=not self.pending,
                       no_source_image_dropped=True, source_timestamp_changed=False, source_pixels_changed=False,
                       synchronous_flush_before_next_tick=True,
                       legacy_jpeg_encode_write_timing_stage_measures_enqueue_only=True,
                       actual_encoding_timing_is_this_receipt=True, full_tick_10Hz_wall_rate_claimed=False,
                       parallel_encoding_research=dict(schema=SCHEMA, workers=WORKERS,
                           all_submitted_encodes_joined=True, submitted_encode_count=len(futures),
                           all_encodes_joined_monotonic_ns=joined, original_role_ordered_writes=True,
                           create_only_file_descriptors=True, encode_rows=[result[2] for result in encodings],
                           background_work_after_flush=False, sources_or_quality_changed=False,
                           actual_speed_gain_proven=False, controller_or_model_changed=False,
                           training_data_approved=False, vehicle_control_approved=False))
            self.events.append(row)
            self._flushing = False
            # HH_261004 - Preserve original create-only receipt creation and close-before-next-tick persistence.
            self.output.mkdir(parents=True, exist_ok=True)
            mode = "a" if self._receipt_created else "x"
            with (self.output / "recording_after_ack.jsonl").open(mode) as stream:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
            self._receipt_created = True


def _delegate_recording():
    """HH_261004 - Clone the byte-identical original generator with one private class alias, not a new ACK path."""
    template = inspect.unwrap(original.deferred_recording)
    namespace = dict(template.__globals__, ExactJPEGAfterACK=ParallelExactJPEGAfterACK)
    delegated = types.FunctionType(template.__code__, namespace, template.__name__,
                                   template.__defaults__, template.__closure__)
    require(delegated.__code__ is template.__code__
            and template.__globals__["ExactJPEGAfterACK"] is original.ExactJPEGAfterACK,
            "original ACK/exception/context-manager bytecode changed")
    return contextmanager(delegated)


@contextmanager
def parallel_deferred_recording(collector, output):
    recorder = None
    try:
        with _delegate_recording()(collector, output) as recorder:
            yield recorder
    finally:
        if recorder is not None:
            recorder.close()


def assert_runtime_recorder(collector):
    """HH_261004 - The actual copied save callback and inherited real ACK must own the same recorder."""
    save = collector._save_jpeg
    recorder = getattr(save, "__self__", None)
    require(type(recorder) is ParallelExactJPEGAfterACK
            and getattr(save, "__func__", None) is original.ExactJPEGAfterACK.enqueue
            and getattr(collector, "_exact_jpeg_after_ACK_active", False) is True,
            "actual three-worker enqueue provider was not installed")
    transports = [kind for kind in collector.AcknowledgedControlTransport.__mro__
                  if kind.__name__ == "RecordingAfterACKTransport"]
    require(len(transports) == 1
            and inspect.getclosurevars(transports[0].send).nonlocals.get("recorder") is recorder
            and not recorder.closed,
            "actual ACK closure and parallel recording provider differ")
    return recorder


@contextmanager
def installed_parallel_recording(recovery_module, straight_recording_module, *, collector):
    """HH_261004 - Enter BEFORE inherited capture activation; restore the two actually consumed aliases."""
    global _active
    verify_frozen()
    require(not _active and not getattr(collector, "_exact_jpeg_after_ACK_active", False)
            and recovery_module.deferred_recording is original.deferred_recording
            and straight_recording_module.ExactJPEGAfterACK is original.ExactJPEGAfterACK
            and getattr(recovery_module, "_active", False) is False
            and getattr(straight_recording_module, "_active", False) is False,
            "parallel recording cannot nest or replace unknown actual runtime aliases")
    require(Path(recovery_module.__file__).resolve()
            == ROOT / "tools/e2e/collect_temporal_route_recovery_fast45_20261001.py"
            and Path(straight_recording_module.__file__).resolve()
            == ROOT / "tools/e2e/collect_temporal_route_joint_town03_straight_v2_20261004.py",
            "exact actual recovery and copied-recorder provider modules required")
    saved = recovery_module.deferred_recording, straight_recording_module.ExactJPEGAfterACK
    _active = True
    try:
        recovery_module.deferred_recording = parallel_deferred_recording
        straight_recording_module.ExactJPEGAfterACK = ParallelExactJPEGAfterACK
        yield
    finally:
        recovery_module.deferred_recording, straight_recording_module.ExactJPEGAfterACK = saved
        _active = False


@contextmanager
def activated():
    """HH_261004 - Use only in a fresh owner's worker before the original capture activation."""
    from . import collect_temporal_route_recovery_fast45_20261001 as recovery
    from . import collect_temporal_route_joint_town03_straight_v2_20261004 as straight_recording
    with installed_parallel_recording(recovery, straight_recording, collector=straight_recording.collector):
        yield
