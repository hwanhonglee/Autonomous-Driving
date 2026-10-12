"""HH_261001 - Memoize source SHA only for explicitly owned immutable camera bytes; keep frozen V2 math."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, replace
import hashlib
import time

import numpy as np
import torch

from .contract import ContractError
from .runtime import RuntimeCameraFrame
from .temporal_simulation_research_v2_20261001 import TemporalSimulationResearchPolicyV2


SCHEMA = "portable_e2e.temporal_owned_immutable_camera_history.20261001.v1"
MAX_OWNED_CAMERAS = 60


def immutable_camera_backing(payload):
    """HH_261001 - A readonly flag is insufficient: require exact immutable bytes and unchanged byte-zero pointer."""
    if type(payload) is not np.ndarray or payload.dtype != np.uint8 or payload.shape != (360, 640, 4):
        return None
    if payload.flags.writeable or not payload.flags.c_contiguous or payload.flags.owndata:
        return None
    value = payload
    for _ in range(4):
        if type(value) is bytes:
            if len(value) != 360*640*4:
                return None
            address = np.frombuffer(value, dtype=np.uint8).__array_interface__["data"][0]
            return value if address == payload.__array_interface__["data"][0] else None
        if type(value) is not np.ndarray or value.dtype != np.uint8 or value.flags.writeable:
            return None
        if not value.flags.c_contiguous or value.flags.owndata:
            return None
        value = value.base
    return None


def own_observed_camera_bytes(frame):
    """HH_261001 - Copy exact observed source pixels into immutable owned bytes, never synthesize missing frames."""
    cameras = []
    for camera in frame.inputs.camera_frames:
        value = camera.payload
        if camera.encoding != "bgra8" or type(value) is not np.ndarray or value.dtype != np.uint8 \
                or value.shape != (360, 640, 4):
            raise ContractError("immutable owned source requires the actual six-camera BGRA ABI")
        backing = value.tobytes(order="C")
        payload = np.frombuffer(backing, dtype=np.uint8).reshape(360, 640, 4)
        cameras.append(RuntimeCameraFrame(camera.timestamp_ns, payload, "bgra8"))
    return replace(frame, inputs=replace(frame.inputs, camera_frames=tuple(cameras)))


def normalized_tensor_hashes(tensors):
    """HH_261001 - Record every exact normalized input tensor before NN execution, without another data copy."""
    result = {}
    for name, tensor in tensors.items():
        value = tensor.detach().cpu().contiguous().numpy()
        digest = hashlib.sha256()
        digest.update(str(value.dtype).encode("ascii"))
        digest.update(str(value.shape).encode("ascii"))
        digest.update(memoryview(value).cast("B"))
        result[name] = digest.hexdigest()
    return result


class OwnedCameraTemporalSimulationResearchPolicyV2(TemporalSimulationResearchPolicyV2):
    """HH_261001 - Cache proven immutable source bytes only; all math, gates, stop/PI and 100 ms remain inherited."""

    def __init__(self, *args, **kwargs):
        self._owned_camera_hashes = OrderedDict()
        self._last_owned_input_audit = None
        self._owned_hash_new = self._owned_hash_reuse = self._fallback_rehash = 0
        super().__init__(*args, **kwargs)

    def reset(self, reason="explicit_reset"):
        super().reset(reason)
        self._owned_camera_hashes.clear()

    def _admit_camera(self, camera):
        backing = immutable_camera_backing(camera.payload) if camera.encoding == "bgra8" else None
        if backing is None:
            return
        key = id(camera.payload), camera.timestamp_ns, camera.encoding
        cached = self._owned_camera_hashes.get(key)
        if cached is None:
            sha = super()._camera_digest(camera)
            self._owned_camera_hashes[key] = camera.payload, backing, sha
            self._owned_hash_new += 1
        elif cached[0] is not camera.payload or cached[1] is not backing:
            raise ContractError("owned source backing identity changed")
        self._owned_camera_hashes.move_to_end(key)
        while len(self._owned_camera_hashes) > MAX_OWNED_CAMERAS:
            self._owned_camera_hashes.popitem(last=False)

    def _camera_digest(self, camera):
        key = id(camera.payload), camera.timestamp_ns, camera.encoding
        entry = self._owned_camera_hashes.get(key)
        if entry is not None:
            backing = immutable_camera_backing(camera.payload) if camera.encoding == "bgra8" else None
            if backing is not entry[1] or camera.payload is not entry[0]:
                raise ContractError("owned camera source byte backing, shape, dtype or writeability changed")
            self._owned_hash_reuse += 1
            return entry[2]
        # HH_261001 - Mutable, unknown, readonly-view-over-mutable and JPEG inputs never receive memoized proof.
        self._fallback_rehash += 1
        return super()._camera_digest(camera)

    def build_inputs(self, frame, *, past_frames=(), **kwargs):
        started = time.perf_counter()
        self._last_owned_input_audit = None
        self._owned_hash_new = self._owned_hash_reuse = self._fallback_rehash = 0
        if not isinstance(past_frames, (list, tuple)) or len(past_frames) > 9:
            raise ContractError("bounded actual past frame list required")
        self._validate_frame(frame)
        for source in tuple(past_frames) + (frame,):
            self._validate_frame(source)
            for camera in source.inputs.camera_frames:
                self._admit_camera(camera)
        minimum = frame.timestamp_ns - 900_001_000
        for key in tuple(self._owned_camera_hashes):
            if not minimum <= key[1] <= frame.timestamp_ns:
                del self._owned_camera_hashes[key]
        tensors, window = super().build_inputs(frame, past_frames=past_frames, **kwargs)
        tensor_hashes = normalized_tensor_hashes(tensors)
        self._last_owned_input_audit = dict(schema=SCHEMA,
            frame_timestamp_ns=frame.timestamp_ns,
            every_source_camera_SHA_preserved=True, immutable_byte_backing_proof_required=True,
            readonly_flag_alone_sufficient=False, source_hashes_new=self._owned_hash_new,
            source_hashes_reused=self._owned_hash_reuse, mutable_or_unknown_source_rehashes=self._fallback_rehash,
            bounded_strong_reference_camera_count=len(self._owned_camera_hashes),
            strong_reference_camera_limit=MAX_OWNED_CAMERAS,
            source_camera_frames=[asdict(item) if item else None for item in window.frames],
            normalized_input_tensor_sha256=tensor_hashes,
            build_and_source_tensor_hash_seconds=time.perf_counter()-started,
            inside_unchanged_hard_policy_deadline=True, exact_normalized_tensor_archive_retained=False,
            raw_NN_decoder_gate_control_math_changed=False, source_pixels_changed=False)
        return tensors, window

    def prefill(self, *args, **kwargs):
        self._last_owned_input_audit = None
        result = super().prefill(*args, **kwargs)
        result["owned_camera_history"] = self._last_owned_input_audit
        return result

    def step(self, *args, **kwargs):
        self._last_owned_input_audit = None
        result = super().step(*args, **kwargs)
        result["owned_camera_history"] = self._last_owned_input_audit
        return result
