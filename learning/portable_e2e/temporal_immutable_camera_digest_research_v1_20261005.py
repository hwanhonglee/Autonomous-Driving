"""HH_261005 - Prepared-only immutable digest candidate; no live policy activation or speed claim."""
from __future__ import annotations

from collections import OrderedDict
from contextlib import contextmanager

SCHEMA = "portable_e2e.immutable_camera_digest_research.20261005.v1"
BGRA_SHAPE = (360, 640, 4)
BGRA_BYTES = 360 * 640 * 4
MAX_SOURCES = 60
MAX_OWNER_BYTES = MAX_SOURCES * BGRA_BYTES


def _numpy():
    import numpy as np
    return np


def _original_digest(camera):
    # HH_261005 - Misses/unknown sources use the genuine frozen encoding-prefixed SHA, not a substitute hash.
    from .temporal_simulation_research_v2_20261001 import TemporalSimulationResearchPolicyV2
    return TemporalSimulationResearchPolicyV2._camera_digest(camera)


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _array_metadata(value):
    return (value.dtype, value.shape, value.strides, value.nbytes,
            value.flags.c_contiguous, value.flags.writeable, value.flags.owndata,
            value.__array_interface__["data"], id(value.base))


def immutable_bgra_owner(payload):
    """HH_261005 - Readonly flags alone never prove immutable ownership; check every base on every call."""
    np = _numpy()
    if type(payload) is not np.ndarray or payload.dtype != np.uint8 or payload.shape != BGRA_SHAPE:
        return None
    if payload.nbytes != BGRA_BYTES or payload.flags.writeable or not payload.flags.c_contiguous:
        return None
    metadata, value = [], payload
    for _ in range(4):
        if type(value) is bytes:
            if len(value) != BGRA_BYTES:
                return None
            address = np.frombuffer(value, dtype=np.uint8).__array_interface__["data"][0]
            if address != payload.__array_interface__["data"][0]:
                return None
            return value, tuple(metadata)
        if type(value) is not np.ndarray or value.dtype != np.uint8 or value.nbytes != BGRA_BYTES:
            return None
        if value.flags.writeable or value.flags.owndata or not value.flags.c_contiguous:
            return None
        metadata.append((value, _array_metadata(value)))
        value = value.base
    return None


def own_observed_bgra(payload):
    """HH_261005 - New-source preparation only: copy actual pixels into bytes, then form a readonly NumPy view."""
    # HH_261005 - Future declared integration must time copy/proof/hash inside the unchanged 100 ms policy budget.
    np = _numpy()
    _require(type(payload) is np.ndarray and payload.dtype == np.uint8
             and payload.shape == BGRA_SHAPE and payload.flags.c_contiguous,
             "exact actual contiguous BGRA source required")
    before = _array_metadata(payload)
    owner = payload.tobytes(order="C")
    _require(type(owner) is bytes and len(owner) == BGRA_BYTES and _array_metadata(payload) == before,
             "source metadata changed during immutable copy")
    result = np.frombuffer(owner, dtype=np.uint8).reshape(BGRA_SHAPE)
    proof = immutable_bgra_owner(result)
    _require(proof is not None and proof[0] is owner, "exact immutable copy owner required")
    return result


def _valid_scope(scope):
    return (type(scope) is tuple and len(scope) == 4 and type(scope[0]) is str and bool(scope[0])
            and type(scope[1]) is str and scope[1] in ("train", "val", "test")
            and type(scope[2]) is str and bool(scope[2])
            and type(scope[3]) is str and len(scope[3]) == 64
            and all(c in "0123456789abcdef" for c in scope[3]))


class ImmutableCameraDigestCache:
    """HH_261005 - Explicit lifetime/context only; <=60 strong references and <=55,296,000 retained bytes."""

    def __init__(self):
        self._entries = OrderedDict()
        self._scope = None
        self.retained_owner_bytes = 0

    def reset(self):
        self._entries.clear()
        self.retained_owner_bytes = 0

    @contextmanager
    def context(self, scope):
        _require(_valid_scope(scope) and self._scope is None, "valid nonnested explicit context required")
        self.reset()
        self._scope = scope
        try:
            yield self
        finally:
            self.reset()
            self._scope = None

    def digest(self, camera, *, scope=None):
        # HH_261005 - Default denies reuse; this helper is not installed into any existing live collector/policy.
        if self._scope is None or scope is None:
            return _original_digest(camera)
        if scope != self._scope:
            self.reset()
            raise ValueError("camera digest context changed")
        if type(camera.timestamp_ns) is not int or camera.timestamp_ns < 0:
            return _original_digest(camera)
        payload, encoding = camera.payload, camera.encoding
        key = id(payload), camera.timestamp_ns, encoding
        entry = self._entries.get(key)
        if encoding == "bgra8":
            proof = immutable_bgra_owner(payload)
        elif encoding == "jpeg" and type(payload) is bytes and 0 < len(payload) <= 2 * 1024**2:
            proof = payload, ()
        else:
            proof = None
        if entry is not None:
            _require(proof is not None and payload is entry[0] and proof[0] is entry[1]
                     and len(proof[1]) == len(entry[2]) and all(
                         new[0] is old[0] and new[1] == old[1] for new, old in zip(proof[1], entry[2])),
                     "cached camera metadata or immutable owner changed")
            self._entries.move_to_end(key)
            return entry[3]
        sha = _original_digest(camera)
        if proof is None:
            return sha
        self._entries[key] = payload, proof[0], proof[1], sha
        self.retained_owner_bytes += len(proof[0])
        while len(self._entries) > MAX_SOURCES or self.retained_owner_bytes > MAX_OWNER_BYTES:
            _, removed = self._entries.popitem(last=False)
            self.retained_owner_bytes -= len(removed[1])
        return sha
