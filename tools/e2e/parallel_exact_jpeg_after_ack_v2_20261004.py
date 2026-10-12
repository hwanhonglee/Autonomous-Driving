"""HH_261004 - Snapshot actual original camera buffers before exact synchronous three-worker JPEG encoding.

This fresh revision preserves frozen V1 failures. Original image identity and
metadata stay recorded; only a private immutable encoding view is created.
Every encode joins before the original role-ordered create-only write/next tick.
"""

from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import inspect
import math
from pathlib import Path
import threading
import time
from types import FunctionType

from . import parallel_exact_jpeg_after_ack_v1_20261004 as previous


ROOT,original=previous.ROOT,previous.original
SCHEMA="portable_e2e.parallel_exact_JPEG_after_ACK.20261004.v2"
WORKERS,CAMERA_ORDER=previous.WORKERS,previous.CAMERA_ORDER
PARENT_SHA256="b3d1903541f9f599b5739655d9fb4bb3a6cdd60e4c0a278db0128a9423508e8b"
PARENT_TEST_SHA256="48c4df5152032b51c32b254525520636f6f8cce2e8d510475707742757b9c21b"
require=previous.require
_active=False


def verify_frozen():
    # HH_261004 - Keep the failed helper/test immutable and authenticate the original encoder/ACK dependencies.
    for path,sha in ((Path(previous.__file__),PARENT_SHA256),
            (ROOT/"learning/tests/test_parallel_exact_jpeg_after_ack_v1_20261004.py",PARENT_TEST_SHA256)):
        require(path.resolve()==path and path.is_file() and not path.is_symlink()
            and hashlib.sha256(path.read_bytes()).hexdigest()==sha,"frozen JPEG V1 source/test changed")
    previous.verify_frozen()


@dataclass(frozen=True)
class SnapshotImage:
    """HH_261004 - Only the original encoder consumes this immutable private view, never a camera queue or policy."""
    frame:int
    timestamp:float
    width:int
    height:int
    raw_data:bytes


def _prefixed_sha(buffer):
    value=hashlib.sha256(b"bgra8");value.update(buffer);return value.hexdigest()


def _safe_scalar(value):
    # HH_261004 - Failed ABI probes must remain JSON-safe, including invalid/nonfinite source metadata.
    return value if type(value) in (int,str,bool) or type(value) is float and math.isfinite(value) else repr(value)


def _probe(image,row):
    """HH_261004 - Record actual types/metadata/buffer ABI before rejecting any shape, type or timestamp predicate."""
    abi=row["source_ABI"]
    metadata=tuple(getattr(image,name) for name in ("frame","timestamp","width","height"))
    abi.update({name:_safe_scalar(value) for name,value in zip(("frame","timestamp","width","height"),metadata)})
    abi["metadata_types"]=[type(value).__module__+"."+type(value).__qualname__ for value in metadata]
    raw=image.raw_data
    abi["raw_data_type"]=type(raw).__module__+"."+type(raw).__qualname__
    try:
        view=memoryview(raw)
        abi.update(buffer_protocol_observed=True,readonly=view.readonly,contiguous=view.contiguous,
            c_contiguous=view.c_contiguous,format=view.format,itemsize=view.itemsize,ndim=view.ndim,
            nbytes=view.nbytes,shape=list(view.shape),strides=list(view.strides),source_len=len(raw))
    except BaseException as error:
        abi.update(buffer_protocol_observed=False,buffer_probe_error=type(error).__name__+": "+str(error))
        raise
    return metadata,raw,view


class ParallelExactJPEGAfterACK(previous.ParallelExactJPEGAfterACK):
    """HH_261004 - Preserve queue/ACK/join/write semantics; encode an equal immutable copy of the genuine source buffer."""

    def _encode(self,image,target,quality,ordinal):
        row=dict(ordinal=ordinal,role=CAMERA_ORDER[ordinal],image_object_id=id(image),quality=quality,
            worker_thread_id=threading.get_ident(),encoding_started_monotonic_ns=time.perf_counter_ns(),
            source_pixels_changed=False,source_timestamp_changed=False,
            source_ABI=dict(image_type=type(image).__module__+"."+type(image).__qualname__),
            private_snapshot_immutable=False,source_stable_through_encoding=False,
            original_image_identity_preserved=True,private_snapshot_used_only_for_encoder=True)
        payload=error=None
        try:
            metadata,raw,view=_probe(image,row)
            frame,stamp,width,height=metadata
            require(type(frame) is int and frame>=0 and type(stamp) in (int,float) and math.isfinite(stamp)
                and stamp>=0 and type(width) is int and type(height) is int and (width,height)==(640,360),
                "exact actual original640x360 frame/stamp/geometry required")
            require(isinstance(raw,(bytes,bytearray,memoryview)) and view.c_contiguous and view.itemsize==1
                and len(raw)==view.nbytes==640*360*4,"original readable contiguous byte-buffer ABI required")
            row.update(native_frame=frame,source_image_timestamp_simulation_ns=int(round(stamp*1.e9)),
                source_byte_count=view.nbytes,source_snapshot_started_monotonic_ns=time.perf_counter_ns())
            row["source_pre_snapshot_BGRA_encoding_prefixed_sha256"]=_prefixed_sha(view)
            snapshot=bytes(view)
            row.update(source_snapshot_finished_monotonic_ns=time.perf_counter_ns(),
                immutable_snapshot_BGRA_encoding_prefixed_sha256=_prefixed_sha(snapshot),
                immutable_snapshot_byte_count=len(snapshot),private_snapshot_immutable=type(snapshot) is bytes)
            require(row["private_snapshot_immutable"] and len(snapshot)==view.nbytes
                and row["source_pre_snapshot_BGRA_encoding_prefixed_sha256"]==row["immutable_snapshot_BGRA_encoding_prefixed_sha256"],
                "full original source changed while taking its private snapshot")
            private=SnapshotImage(frame,stamp,width,height,snapshot)
            row["private_encoding_image_object_id"]=id(private)
            payload=self.encoder(private,target,quality)
            require(type(payload) is bytes and payload,"nonempty exact original encoded JPEG bytes required")
            after,after_raw,after_view=_probe(image,dict(source_ABI={}))
            row["source_after_encoding_metadata"]={name:_safe_scalar(value) for name,value in zip(
                ("frame","timestamp","width","height"),after)}
            require(after==metadata and isinstance(after_raw,(bytes,bytearray,memoryview))
                and after_view.c_contiguous and after_view.itemsize==1
                and len(after_raw)==after_view.nbytes==len(snapshot),"original source metadata/buffer changed during encoding")
            row["source_post_encoding_BGRA_encoding_prefixed_sha256"]=_prefixed_sha(after_view)
            require(row["source_post_encoding_BGRA_encoding_prefixed_sha256"]
                ==row["immutable_snapshot_BGRA_encoding_prefixed_sha256"],"full original source pixels changed during encoding")
            row.update(source_BGRA_encoding_prefixed_sha256=row["immutable_snapshot_BGRA_encoding_prefixed_sha256"],
                source_stable_through_encoding=True,source_metadata_unchanged_after_encoding=True,
                encoded_JPEG_sha256=hashlib.sha256(payload).hexdigest(),encoded_JPEG_byte_count=len(payload))
        except BaseException as caught:
            # HH_261004 - Keep the exact first role-ordered error and all ABI/hash rows; inherited flush joins every task.
            error=caught
        finally:
            row.update(encoding_finished_monotonic_ns=time.perf_counter_ns(),
                encoding_failure=None if error is None else type(error).__name__+": "+str(error))
        return payload,error,row


# HH_261004 - The entire frozen join/order/create-only/failure-tail bytecode is reused, changing only its research schema.
_flush=previous.ParallelExactJPEGAfterACK.flush
ParallelExactJPEGAfterACK.flush=FunctionType(_flush.__code__,{**_flush.__globals__,"SCHEMA":SCHEMA},
    _flush.__name__,_flush.__defaults__,_flush.__closure__)
ParallelExactJPEGAfterACK.flush.__kwdefaults__=_flush.__kwdefaults__.copy()
ParallelExactJPEGAfterACK.flush.__annotations__=dict(_flush.__annotations__)
assert ParallelExactJPEGAfterACK.flush.__code__ is _flush.__code__


def _delegate_recording():
    # HH_261004 - Use the genuine original ACK/context-manager bytecode with only a new private recorder class alias.
    template=inspect.unwrap(original.deferred_recording)
    delegated=FunctionType(template.__code__,{**template.__globals__,"ExactJPEGAfterACK":ParallelExactJPEGAfterACK},
        template.__name__,template.__defaults__,template.__closure__)
    require(delegated.__code__ is template.__code__,"original recording/ACK generator bytecode changed")
    return contextmanager(delegated)


@contextmanager
def parallel_deferred_recording(collector,output):
    recorder=None
    try:
        with _delegate_recording()(collector,output) as recorder:yield recorder
    finally:
        if recorder is not None:recorder.close()


def assert_runtime_recorder(collector):
    # HH_261004 - Run the genuine frozen class/ACK-closure guard with the sole prospective new-class identity seam.
    function=previous.assert_runtime_recorder
    private=FunctionType(function.__code__,{**function.__globals__,"ParallelExactJPEGAfterACK":ParallelExactJPEGAfterACK},
        function.__name__,function.__defaults__,function.__closure__)
    return private(collector)


@contextmanager
def activated():
    """HH_261004 - Bind both actual copied-provider aliases before the inherited capture and restore them on every exit."""
    global _active
    from . import collect_temporal_route_recovery_fast45_20261001 as recovery
    from . import collect_temporal_route_joint_town03_straight_v2_20261004 as recording
    verify_frozen()
    require(not _active and not recovery._active and not recording._active
        and not getattr(recording.collector,"_exact_jpeg_after_ACK_active",False)
        and recovery.deferred_recording is original.deferred_recording
        and recording.ExactJPEGAfterACK is original.ExactJPEGAfterACK,"fresh JPEG V2 cannot nest or replace unknown providers")
    saved=recovery.deferred_recording,recording.ExactJPEGAfterACK;_active=True
    try:
        recovery.deferred_recording=parallel_deferred_recording;recording.ExactJPEGAfterACK=ParallelExactJPEGAfterACK
        yield
    finally:
        recovery.deferred_recording,recording.ExactJPEGAfterACK=saved;_active=False
