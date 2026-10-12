"""HH_261001 - Explicit research-only cached CNN API for the frozen temporal v2.

Only raw per-camera CNN tokens are reused. The inherited model still recomputes
all conditioning and fusion. This is not Autoware integration, runtime approval,
or evidence of a 10 Hz end-to-end pipeline. Ordinary ``forward`` is uncached.

Call ``forward_cached`` in eval mode inside ``torch.no_grad()`` with recorded
timestamps, six camera-file hashes per valid slot, and episode/split/rig/source
provenance. Tensor payload hashes additionally detect changed normalized pixels.
Nine frame groups retain both interleaved chains of a 10 Hz stream whose history
slots are 0.2 seconds apart. Missing slots never populate the cache. The frozen
model's 3 microsecond offset tolerance remains authoritative; this API does not
widen it to the dataset adapter's 1 millisecond matching tolerance.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Mapping

import torch
from torch import Tensor, nn

from .contract import ContractError
from .temporal_model_v2 import SpatialImageEncoderV2, TemporalModelConfigV2, TemporalModelV2


FROZEN_MODEL_SOURCE_SHA256 = "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435"
CACHED_OUTPUT_ATOL = 1.0e-6
CACHED_OUTPUT_RTOL = 1.0e-6
MAX_CACHED_FRAME_GROUPS = 9
MODEL_INPUT_NAMES = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy",
                     "route_mask", "time_valid", "relative_pose", "time_offsets_s")


@dataclass(frozen=True)
class CameraFrameIdentityV2:
    """HH_261001 - A recorded frame identity, never a dataset-global row index."""

    timestamp_ns: int
    camera_sha256: tuple[str, ...]


@dataclass(frozen=True)
class TemporalCacheWindowV2:
    """HH_261001 - Provenance for exactly the five supplied causal image slots."""

    episode_id: str
    split: str
    rig_id: str
    source_sha256: str
    anchor_timestamp_ns: int
    frames: tuple[CameraFrameIdentityV2 | None, ...]


@dataclass(frozen=True)
class TemporalCacheStatsV2:
    """HH_261001 - CNN work for the last successful explicit cached call."""

    cnn_calls: int
    cnn_images: int
    reused_frame_groups: int
    encoded_frame_groups: int
    retained_frame_groups: int
    cold_start: bool
    reset_reason: str | None


@dataclass(frozen=True)
class _Entry:
    pixel_sha256: str
    features: Tensor


def _tensor_sha256(value: Tensor) -> str:
    # HH_261001 - Hash actual bytes, including in-place .data edits that evade Tensor._version.
    tensor = value.detach().contiguous().cpu()
    if tensor.dtype == torch.bfloat16:
        tensor = tensor.float()
    digest = hashlib.sha256()
    digest.update(str((value.dtype, tuple(value.shape))).encode("ascii"))
    digest.update(memoryview(tensor.numpy()).cast("B"))
    return digest.hexdigest()


def _is_sha256(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


class _CachedSpatialImageEncoderV2(SpatialImageEncoderV2):
    """HH_261001 - Preserve all frozen encoder children and state_dict names."""

    def __init__(self, original: SpatialImageEncoderV2) -> None:
        nn.Module.__init__(self)
        self.features = original.features
        self.pool = original.pool
        self.projection = original.projection
        self.entries: dict[CameraFrameIdentityV2, _Entry] = {}
        self.request: tuple[CameraFrameIdentityV2 | None, ...] | None = None
        self.work = (0, 0, 0, False)

    def forward(self, images: Tensor) -> Tensor:
        if self.request is None:
            return super().forward(images)
        frames = self.request
        cold = not self.entries
        hits: dict[int, Tensor] = {}
        missing: list[int] = []
        pixels: dict[int, str] = {}
        for slot, frame in enumerate(frames):
            if frame is None:
                continue
            pixels[slot] = _tensor_sha256(images[slot * 6:(slot + 1) * 6])
            entry = self.entries.get(frame)
            if entry is not None and entry.pixel_sha256 == pixels[slot]:
                hits[slot] = entry.features
            else:
                missing.append(slot)
        if cold:
            # HH_261001 - Cold start performs the identical full masked 30-image CNN call.
            encoded = super().forward(images)
            for slot in missing:
                hits[slot] = encoded[slot * 6:(slot + 1) * 6].detach().clone()
            cnn_images = 30
        else:
            # HH_261001 - Invalid slot tokens are masked again by the frozen model after conditioning.
            encoded = images.new_zeros((30, 60, 128))
            cnn_images = len(missing) * 6
            if missing:
                new_images = torch.cat([images[slot * 6:(slot + 1) * 6] for slot in missing])
                new_features = super().forward(new_images)
                for index, slot in enumerate(missing):
                    hits[slot] = new_features[index * 6:(index + 1) * 6].detach().clone()
            for slot, features in hits.items():
                encoded[slot * 6:(slot + 1) * 6] = features
        for slot in missing:
            frame = frames[slot]
            # HH_261001 - A timestamp with changed file provenance replaces its old identity.
            for previous in tuple(self.entries):
                if previous.timestamp_ns == frame.timestamp_ns:
                    del self.entries[previous]
            self.entries[frame] = _Entry(pixels[slot], hits[slot])
        self.work = (cnn_images, len(hits) - len(missing), len(missing), cold)
        return encoded


class CachedTemporalModelV2(TemporalModelV2):
    """HH_261001 - Same checkpoint ABI; cached evaluation requires an explicit API.

    Construct, load the original checkpoint with ``strict=True``, call ``eval()``,
    then use ``forward_cached(inputs, window=...)`` under ``torch.no_grad()``.
    ``source_sha256`` identifies the validated dataset/source; camera hashes are
    ordered as the actual six model inputs. Unsupported training, batches other
    than one, autocast and incomplete provenance are rejected before CNN work.
    The CPU numerical equivalence contract is atol=rtol=1e-6, not bit equality.
    All calls on one wrapper, including ordinary forward, must be serial. Hooks
    may observe tensors; semantics-changing hooks or monkeypatched forwards are
    outside this cache contract. Use independent wrappers for independent streams.
    """

    def __init__(self, config: TemporalModelConfigV2 | None = None) -> None:
        source = Path(__file__).with_name("temporal_model_v2.py")
        if hashlib.sha256(source.read_bytes()).hexdigest() != FROZEN_MODEL_SOURCE_SHA256:
            raise ContractError("cached encoder requires the pinned frozen temporal v2 source")
        super().__init__(config)
        self.image_encoder = _CachedSpatialImageEncoderV2(self.image_encoder)
        self._cache_scope: tuple | None = None
        self._cache_state: tuple | None = None
        self._last_anchor_ns: int | None = None
        self._pending_reset_reason: str | None = "initial"
        self.cache_stats: TemporalCacheStatsV2 | None = None

    def reset_cache(self, reason: str = "explicit") -> None:
        """HH_261001 - Clear unregistered research state without changing learned tensors."""
        if isinstance(self.image_encoder, _CachedSpatialImageEncoderV2):
            self.image_encoder.entries.clear()
        self._cache_scope = None
        self._cache_state = None
        self._last_anchor_ns = None
        self._pending_reset_reason = reason
        self.cache_stats = None

    def train(self, mode: bool = True):
        if mode and hasattr(self, "_cache_scope"):
            self.reset_cache("training")
        return super().train(mode)

    def _apply(self, fn):
        if hasattr(self, "_cache_scope"):
            self.reset_cache("device_or_dtype_changed")
        return super()._apply(fn)

    def load_state_dict(self, state_dict, strict: bool = True):
        self.reset_cache("checkpoint_loaded")
        return super().load_state_dict(state_dict, strict=strict)

    def _state_identity(self) -> tuple:
        # HH_261001 - Whole-state content and module topology detect checkpoint and silent .data changes.
        digest = hashlib.sha256()
        for name, value in self.state_dict().items():
            digest.update(name.encode("utf-8"))
            digest.update(str(value.device).encode("ascii"))
            digest.update(_tensor_sha256(value).encode("ascii"))
        # HH_261001 - State tensors omit behavior settings such as GroupNorm.eps and pooling flags.
        topology = tuple((name, id(module), type(module), module.extra_repr(),
                          tuple((key, repr(value)) for key, value in vars(module).items()
                                if module is not self and module is not self.image_encoder
                                and not key.startswith("_")
                                and isinstance(value, (str, int, float, bool, tuple, type(None)))),
                          tuple((key, id(hook)) for key, hook in module._forward_hooks.items()),
                          tuple((key, id(hook)) for key, hook in module._forward_pre_hooks.items()))
                         for name, module in self.named_modules())
        return digest.hexdigest(), topology

    def _validate_window(self, window: TemporalCacheWindowV2, inputs: Mapping[str, Tensor]) -> None:
        if not isinstance(window, TemporalCacheWindowV2):
            raise ContractError("cached inference requires TemporalCacheWindowV2 provenance")
        for name in ("episode_id", "rig_id"):
            value = getattr(window, name)
            if not isinstance(value, str) or not value.strip():
                raise ContractError(f"cached {name} must be a nonempty recorded identity")
        if window.split not in ("train", "val", "test") or not _is_sha256(window.source_sha256):
            raise ContractError("cached split/source provenance is invalid")
        anchor = window.anchor_timestamp_ns
        if isinstance(anchor, bool) or not isinstance(anchor, int) or anchor < 0:
            raise ContractError("cached anchor timestamp must be nonnegative integer nanoseconds")
        if not isinstance(window.frames, tuple) or len(window.frames) != 5:
            raise ContractError("cached provenance must contain exactly five frame slots")
        previous_time = -1
        for slot, frame in enumerate(window.frames):
            valid = bool(inputs["time_valid"][0, slot].item())
            if not valid:
                if frame is not None:
                    raise ContractError("missing camera times must have no cached frame identity")
                continue
            if not isinstance(frame, CameraFrameIdentityV2):
                raise ContractError("valid camera times require recorded frame identities")
            timestamp = frame.timestamp_ns
            if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
                raise ContractError("cached frame timestamp must be nonnegative integer nanoseconds")
            if timestamp > anchor:
                raise ContractError("future cached camera frames are forbidden")
            if timestamp <= previous_time:
                raise ContractError("cached camera timestamps must increase strictly")
            previous_time = timestamp
            if abs((timestamp - anchor) / 1.0e9 - float(inputs["time_offsets_s"][0, slot])) > 3.0e-6:
                raise ContractError("cached frame timestamps differ from supplied camera time offsets")
            if slot == 4 and timestamp != anchor:
                raise ContractError("cached current frame must equal the anchor timestamp")
            if not isinstance(frame.camera_sha256, tuple) or len(frame.camera_sha256) != 6 or not all(_is_sha256(value) for value in frame.camera_sha256):
                raise ContractError("cached frame requires six recorded camera SHA256 hashes in model order")

    def forward_cached(
        self,
        inputs: Mapping[str, Tensor],
        *,
        window: TemporalCacheWindowV2,
        history_mode: str = "temporal",
    ) -> dict[str, Tensor]:
        if torch.is_grad_enabled() or any(module.training for module in self.modules()):
            raise ContractError("cached inference requires eval mode and torch.no_grad()")
        cpu_autocast = getattr(torch, "is_autocast_cpu_enabled", lambda: False)
        if torch.is_autocast_enabled() or cpu_autocast():
            raise ContractError("cached inference does not support autocast")
        if not isinstance(inputs, Mapping) or set(inputs) != set(MODEL_INPUT_NAMES):
            raise ContractError("cached inference requires exactly the frozen model input fields")
        if self._validate_inputs(inputs, history_mode) != 1:
            raise ContractError("cached inference supports one causal stream, batch size one")
        self._validate_window(window, inputs)
        if not isinstance(self.image_encoder, _CachedSpatialImageEncoderV2):
            raise ContractError("cached image encoder was replaced; construct a new research wrapper")
        encoder = self.image_encoder
        if encoder.request is not None:
            raise ContractError("cached inference is not reentrant; use one wrapper per stream")
        scope = (window.episode_id, window.split, window.rig_id, window.source_sha256,
                 _tensor_sha256(inputs["calibration"]))
        state = self._state_identity()
        if self._cache_scope is not None and scope != self._cache_scope:
            self.reset_cache("episode_split_rig_or_source_changed")
        elif self._cache_state is not None and state != self._cache_state:
            self.reset_cache("model_state_changed")
        elif self._last_anchor_ns is not None and window.anchor_timestamp_ns < self._last_anchor_ns:
            self.reset_cache("clock_reversal")
        reason = self._pending_reset_reason
        # HH_261001 - Never retain future frames or arbitrary stream history beyond the causal horizon.
        oldest = window.anchor_timestamp_ns - 800_003_000
        encoder.entries = {frame: entry for frame, entry in encoder.entries.items()
                           if oldest <= frame.timestamp_ns <= window.anchor_timestamp_ns}
        encoder.request = tuple(frame if history_mode == "temporal" or slot == 4 else None
                                for slot, frame in enumerate(window.frames))
        try:
            result = super().forward(**inputs, history_mode=history_mode)
        except Exception:
            self.reset_cache("failed_forward")
            raise
        finally:
            encoder.request = None
        newest = sorted(encoder.entries, key=lambda frame: frame.timestamp_ns, reverse=True)
        encoder.entries = {frame: encoder.entries[frame] for frame in newest[:MAX_CACHED_FRAME_GROUPS]}
        cnn_images, reused, encoded, cold = encoder.work
        self.cache_stats = TemporalCacheStatsV2(int(cnn_images > 0), cnn_images, reused, encoded,
                                               len(encoder.entries), cold, reason)
        self._cache_scope, self._cache_state = scope, state
        self._last_anchor_ns = window.anchor_timestamp_ns
        self._pending_reset_reason = None
        return result
