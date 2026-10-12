"""HH_261001 - Evaluation-only JPEG tensor reuse around the frozen temporal adapter.

Construct with explicit ``mode="evaluation"`` and ``allow_training=False``.
The dataset partition may be train, val or test: reading the train partition for
offline evaluation does not authorize optimization. Each image access still
reads the original bounded regular file and verifies its recorded SHA256. A
miss runs the unchanged parent decoder; a hit returns a private tensor copy.

Calls on one adapter must be serial. Its LRU retains at most 54 normalized
images, retiring frames older than the causal horizon and protecting the images
needed by the current window while evicting other entries. This retains both interleaved history chains of a
10 Hz stream without adding future images or changing labels or fingerprints.
This is research preprocessing, not sensor capture or runtime approval.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path
from typing import Mapping, Sequence

from torch import Tensor

from .contract import ContractError, _sha256
from .dataset import TrainingExample
from .model import ModelConfig
from .temporal_dataset_v2 import TIME_OFFSETS_NS, TemporalCommon10TorchDataset, TemporalSampleMetadata


MAX_CACHED_IMAGES = 54
FROZEN_TEMPORAL_DATASET_SOURCE_SHA256 = "9f27fd12f7302ed654df4c2c956697d794bb15243c9576681e0300a226cefafe"
FROZEN_TORCH_DATASET_SOURCE_SHA256 = "a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9"
FROZEN_CONTRACT_SOURCE_SHA256 = "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09"


@dataclass(frozen=True)
class DecodedCacheStatsV2:
    """HH_261001 - Successful image work and storage, plus failed/reset call counts."""

    hits: int
    misses: int
    hash_checks: int
    evictions: int
    retained_images: int
    retained_bytes: int
    peak_retained_images: int
    peak_retained_bytes: int
    reset_count: int
    successful_calls: int
    failed_calls: int
    last_call_hits: int
    last_call_misses: int
    last_call_hash_checks: int
    last_call_evictions: int
    reset_reason: str | None


class DecodedCacheTemporalDatasetV2(TemporalCommon10TorchDataset):
    """HH_261001 - Preserve the frozen adapter ABI and explicitly opt into eval reuse."""

    def __init__(
        self,
        examples: Sequence[TrainingExample],
        model_config: ModelConfig | None = None,
        *,
        raw_metadata: Mapping[str, TemporalSampleMetadata],
        mode: str,
        allow_training: bool = False,
        verify_image_sha256: bool = True,
        split: str = "train",
        timestamp_tolerance_ns: int = 1_000_000,
        max_cached_images: int = MAX_CACHED_IMAGES,
    ) -> None:
        if mode != "evaluation" or allow_training is not False:
            raise ContractError("decoded cache requires explicit evaluation mode and allow_training=False")
        if verify_image_sha256 is not True:
            raise ContractError("decoded cache requires verify_image_sha256=True on every access")
        if (isinstance(max_cached_images, bool) or not isinstance(max_cached_images, int)
                or not 1 <= max_cached_images <= MAX_CACHED_IMAGES):
            raise ContractError("decoded cache limit must be an integer between 1 and 54 images")
        source_pins = {
            "temporal_dataset_v2.py": FROZEN_TEMPORAL_DATASET_SOURCE_SHA256,
            "torch_dataset.py": FROZEN_TORCH_DATASET_SOURCE_SHA256,
            "contract.py": FROZEN_CONTRACT_SOURCE_SHA256,
        }
        for filename, expected in source_pins.items():
            if hashlib.sha256(Path(__file__).with_name(filename).read_bytes()).hexdigest() != expected:
                raise ContractError(f"decoded cache requires pinned frozen source: {filename}")
        super().__init__(examples, model_config, raw_metadata=raw_metadata,
                         verify_image_sha256=True, split=split,
                         timestamp_tolerance_ns=timestamp_tolerance_ns)
        self.mode = mode
        self.allow_training = allow_training
        self._max_cached_images = max_cached_images
        # HH_261001 - Cache provenance is separate from the unchanged parent dataset fingerprint.
        self.cache_source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.cache_source_pin = {
            "schema": "portable_e2e.temporal_decoded_cache.v2",
            "adapter_sha256": self.cache_source_sha256,
            "frozen_source_sha256": source_pins,
            "mode": mode,
            "allow_training": False,
            "max_cached_images": max_cached_images,
            "image_validation": "original_regular_file_bounded_read_and_sha256_every_access",
        }
        self._entries: OrderedDict[tuple, Tensor] = OrderedDict()
        self._protected_keys: set[tuple] = set()
        self._entry_timestamps: dict[tuple, int] = {}
        self._window_timestamps: dict[tuple, int] = {}
        self._entry_provenance: dict[tuple, dict[int, tuple]] = {}
        self._window_provenance: dict[tuple, dict[int, tuple]] = {}
        self._scope: tuple | None = None
        self._config_identity: tuple | None = None
        self._last_anchor_ns: int | None = None
        self._in_getitem = False
        self._counts = {name: 0 for name in (
            "hits", "misses", "hash_checks", "evictions", "retained_bytes",
            "peak_retained_images", "peak_retained_bytes", "reset_count",
            "successful_calls", "failed_calls",
        )}
        self._last_work = (0, 0, 0, 0)
        self._last_reset_reason: str | None = "initial"

    @property
    def max_cached_images(self) -> int:
        """HH_261001 - Keep the allocation bound fixed for one adapter."""
        return self._max_cached_images

    @property
    def cache_stats(self) -> DecodedCacheStatsV2:
        """HH_261001 - Return a detached statistics snapshot suitable for asdict()."""
        return DecodedCacheStatsV2(**self._counts, retained_images=len(self._entries),
                                  last_call_hits=self._last_work[0],
                                  last_call_misses=self._last_work[1],
                                  last_call_hash_checks=self._last_work[2],
                                  last_call_evictions=self._last_work[3],
                                  reset_reason=self._last_reset_reason)

    def reset_cache(self, reason: str = "explicit") -> None:
        """HH_261001 - Drop research tensors without changing examples or labels."""
        if not isinstance(reason, str) or not reason:
            raise ContractError("decoded cache reset reason must be a nonempty string")
        self._entries.clear()
        self._protected_keys.clear()
        self._entry_timestamps.clear()
        self._window_timestamps.clear()
        self._entry_provenance.clear()
        self._window_provenance.clear()
        self._scope = None
        self._config_identity = None
        self._last_anchor_ns = None
        self._counts["retained_bytes"] = 0
        self._counts["reset_count"] += 1
        self._last_reset_reason = reason

    def _validate_usage(self) -> tuple:
        # HH_261001 - Mutable public flags cannot silently weaken evaluation or byte verification.
        if self.mode != "evaluation" or self.allow_training is not False:
            raise ContractError("decoded cache requires evaluation mode and allow_training=False")
        if self.verify_image_sha256 is not True:
            raise ContractError("decoded cache requires verify_image_sha256=True on every access")
        self.config.validate()
        return tuple(asdict(self.config).items())

    def _scope_for(self, example: TrainingExample, metadata: TemporalSampleMetadata) -> tuple:
        self._validate_metadata(example, metadata)
        if example.source_manifest_sha256:
            _sha256(example.source_manifest_sha256, "decoded cache source manifest SHA256")
        return (example.episode_id, self.split, example.rig_id, metadata.sample_jsonl_sha256,
                example.source_manifest_sha256, example.source_dataset_id,
                example.source_dataset_version, example.domain, example.license_id,
                example.map_id, example.site_id, example.route_id)

    def _image_key(self, path: Path, expected_sha256: str, config_identity: tuple) -> tuple:
        # HH_261001 - Absolute lexical paths do not resolve or disguise a replaced symbolic link.
        return str(Path(path).absolute()), expected_sha256, config_identity

    def _evict_one(self) -> None:
        # HH_261001 - Parent decodes current first; protect all requested past frames before insertion.
        victim = next((key for key in self._entries if key not in self._protected_keys),
                      next(iter(self._entries)))
        self._remove_entry(victim)

    def _remove_entry(self, key: tuple) -> None:
        tensor = self._entries.pop(key)
        self._entry_timestamps.pop(key)
        self._entry_provenance.pop(key)
        self._counts["retained_bytes"] -= tensor.numel() * tensor.element_size()
        self._counts["evictions"] += 1

    def _decode_image(self, path: Path, expected_sha256: str) -> Tensor:
        config_identity = self._validate_usage()
        if not self._in_getitem:
            raise ContractError("decoded image reuse requires a complete __getitem__ evaluation window")
        key = self._image_key(path, expected_sha256, config_identity)
        entry = self._entries.get(key)
        if entry is not None:
            # HH_261001 - Reuse the exact parent regular-file, symlink, size and stability gates.
            payload = self._read_regular_file(path)
            if hashlib.sha256(payload).hexdigest() != expected_sha256:
                raise ContractError(f"training image changed after validation: {path}")
            self._counts["hash_checks"] += 1
            self._counts["hits"] += 1
            self._entries.move_to_end(key)
            self._entry_timestamps[key] = self._window_timestamps[key]
            self._entry_provenance[key] = dict(self._window_provenance[key])
            return entry.clone()
        # HH_261001 - Misses run the unchanged PIL resize and normalization on the actual original file.
        decoded = super()._decode_image(path, expected_sha256)
        self._counts["hash_checks"] += 1
        self._counts["misses"] += 1
        cached = decoded.detach().clone()
        if len(self._entries) >= self._max_cached_images:
            self._evict_one()
        self._entries[key] = cached
        self._entry_timestamps[key] = self._window_timestamps[key]
        self._entry_provenance[key] = dict(self._window_provenance[key])
        self._counts["retained_bytes"] += cached.numel() * cached.element_size()
        self._counts["peak_retained_images"] = max(self._counts["peak_retained_images"], len(self._entries))
        self._counts["peak_retained_bytes"] = max(self._counts["peak_retained_bytes"], self._counts["retained_bytes"])
        return decoded

    def __getitem__(self, index: int) -> dict[str, Tensor | str]:
        if self._in_getitem:
            raise ContractError("decoded cache requires serial non-reentrant evaluation access")
        before = tuple(self._counts[name] for name in ("hits", "misses", "hash_checks", "evictions"))
        try:
            config_identity = self._validate_usage()
            example = self.examples[index]
            current_scope = self._scope_for(example, self.raw_metadata[example.token])
            window_timestamps = {}
            window_provenance = {}
            source_scopes = set()
            for offset_ns in TIME_OFFSETS_NS:
                source_index = self._past_index(example, offset_ns)
                if source_index is None:
                    continue
                source = self.examples[source_index]
                source_scope = self._scope_for(source, self.raw_metadata[source.token])
                source_scopes.add(source_scope)
                for path, sha in zip(source.camera_paths, source.camera_sha256):
                    key = self._image_key(path, sha, config_identity)
                    window_timestamps[key] = max(window_timestamps.get(key, -1), source.anchor_timestamp_ns)
                    window_provenance.setdefault(key, {})[source.anchor_timestamp_ns] = source_scope
            # HH_261001 - Distinct source scopes catch historical-only drift without binding row identities.
            scope = current_scope, tuple(sorted(source_scopes))
            reason = None
            if self._scope is not None and scope != self._scope:
                reason = "provenance_changed"
            elif any(timestamp in self._entry_provenance.get(key, {})
                     and recorded_scope != self._entry_provenance[key][timestamp]
                     for key, identities in window_provenance.items()
                     for timestamp, recorded_scope in identities.items()):
                # HH_261001 - Per-frame identity detects swaps that preserve the window's scope set.
                reason = "provenance_changed"
            elif self._config_identity is not None and config_identity != self._config_identity:
                reason = "configuration_changed"
            elif self._last_anchor_ns is not None and example.anchor_timestamp_ns < self._last_anchor_ns:
                reason = "clock_reversed"
            if reason is not None:
                self.reset_cache(reason)
            elif self._scope is not None:
                self._last_reset_reason = None
            self._scope = scope
            self._config_identity = config_identity
            self._window_timestamps = window_timestamps
            self._window_provenance = window_provenance
            self._protected_keys = set(window_timestamps)
            # HH_261001 - Current-first decoding must not evict the opposite parity's newer frames.
            cutoff_ns = example.anchor_timestamp_ns + TIME_OFFSETS_NS[0] - self.timestamp_tolerance_ns
            for key, timestamp_ns in tuple(self._entry_timestamps.items()):
                if timestamp_ns < cutoff_ns and key not in self._protected_keys:
                    self._remove_entry(key)
            self._in_getitem = True
            output = super().__getitem__(index)
            self._last_anchor_ns = example.anchor_timestamp_ns
            self._counts["successful_calls"] += 1
            return output
        except BaseException:
            self._counts["failed_calls"] += 1
            self.reset_cache("failed_access")
            raise
        finally:
            self._in_getitem = False
            self._protected_keys.clear()
            self._window_timestamps.clear()
            self._window_provenance.clear()
            after = tuple(self._counts[name] for name in ("hits", "misses", "hash_checks", "evictions"))
            self._last_work = tuple(right - left for left, right in zip(before, after))
