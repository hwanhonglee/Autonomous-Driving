"""HH_261008 - Build actual same-box RGB/GT batches without training or live control.

The external owner must admit and seal original TRAIN packages, whole baseline
journals, providers and source before using this worker. Supplied readers and
baseline rows alone are not data permission or source authentication. Each batch
must reproduce BOTH original RGB and GT hashes before any candidate is returned.
No frozen trainer or global crop function is patched. No optimizer is imported.
"""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import stat

from . import map_crop_preservation_journal_20261008_v1 as journal

SCHEMA = "portable_e2e.map_crop_batch_worker.20261008.v1"
CROP_SOURCE_SHA = "9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96"
CROP_AST_SHA = "e2ae084b3783bb1f4d618c05c75b2afea00abe02545235c8b6fe449aa6e01809"
MAX_SOURCE_BYTES = 256 * 1024


def require(value, message):
    if not value:
        raise ValueError(message)


def compile_original_crop(path):
    """HH_261008 - Compile only the RAW-pinned original function, never its loader."""
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=True)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical crop source required")
    before = path.stat()
    require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1
        and before.st_size <= MAX_SOURCE_BYTES, "bounded single-link crop source required")
    with path.open("rb") as source:
        opened = os.fstat(source.fileno())
        raw = source.read(MAX_SOURCE_BYTES + 1)
        closed = os.fstat(source.fileno())
    after = path.stat()
    fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns", "st_mode", "st_nlink")
    require(all(getattr(before, key) == getattr(opened, key) == getattr(closed, key)
        == getattr(after, key) for key in fields), "crop source generation changed")
    require(hashlib.sha256(raw).hexdigest() == CROP_SOURCE_SHA, "original crop RAW source differs")
    tree = ast.parse(raw, filename=str(path))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "apply_crop"]
    require(len(nodes) == 1, "exact single original crop function required")
    dumped = ast.dump(nodes[0], include_attributes=False).encode()
    require(hashlib.sha256(dumped).hexdigest() == CROP_AST_SHA, "original crop AST differs")
    import numpy as np
    namespace = {"np": np, "require": require}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), namespace)
    return namespace["apply_crop"]


def array_hash(value):
    """HH_261008 - Hash actual contiguous bytes without a full tobytes copy."""
    import numpy as np
    require(type(value) is np.ndarray and value.dtype == np.uint8
        and value.flags.c_contiguous, "exact contiguous uint8 array required")
    return hashlib.sha256(memoryview(value).cast("B")).hexdigest()


def native_pair(rgb, target):
    import numpy as np
    require(type(rgb) is type(target) is np.ndarray and rgb.dtype == target.dtype == np.uint8
        and rgb.shape == (360, 640, 3) and target.shape == (360, 640)
        and rgb.flags.c_contiguous and target.flags.c_contiguous, "native RGB/GT ABI differs")
    require(int(target.max()) < 7, "actual GT class out of range")


def build_batch(baseline, reader, original_crop, *, checkpoint=lambda: None):
    """Return owned candidate arrays and a new byte-derived receipt, transactionally.

    Baseline file and reader provenance remain the external owner's obligation.
    The injected crop is authenticated only when compiled by compile_original_crop
    under the owner's sealed source/provider contract; fixtures may inject a fake.
    """
    journal.validate_baseline(baseline)
    require(callable(original_crop) and callable(getattr(reader, "read", None)), "explicit reader/crop required")
    import numpy as np
    owned_baseline = deepcopy(baseline)
    rgb = np.empty((8, 360, 640, 3), dtype=np.uint8)
    target = np.empty((8, 360, 640), dtype=np.uint8)
    original_rgb_hash, original_target_hash = hashlib.sha256(), hashlib.sha256()
    selected, full_counts, proposed_counts, native_receipts = [], [], [], []
    for position, (slot, proposal) in enumerate(zip(baseline["physical_slot_indices"], baseline["shared_crop_boxes"])):
        checkpoint()
        source_rgb, source_target = reader.read(slot)
        native_pair(source_rgb, source_target)
        # HH_261008 - Snapshot callback-owned inputs and deny silent crop mutation.
        source_rgb, source_target = source_rgb.copy(), source_target.copy()
        input_rgb_sha, input_gt_sha = array_hash(source_rgb), array_hash(source_target)
        original_rgb, original_target = original_crop(source_rgb, source_target, proposal)
        native_pair(original_rgb, original_target)
        original_rgb_hash.update(memoryview(original_rgb).cast("B"))
        original_target_hash.update(memoryview(original_target).cast("B"))
        full = int(np.count_nonzero(source_target == 5))
        proposed = int(np.count_nonzero(original_target == 5))
        ordinal = journal.policy.exposure_index(baseline["step"] - 1, position)
        choice = journal.policy.select_box(ordinal, proposal, full, proposed)
        chosen_box = list(choice["selected_box"])
        if choice["rescued"]:
            chosen_rgb, chosen_target = original_crop(source_rgb, source_target, chosen_box)
            native_pair(chosen_rgb, chosen_target)
        else:
            chosen_rgb, chosen_target = original_rgb, original_target
        require(array_hash(source_rgb) == input_rgb_sha and array_hash(source_target) == input_gt_sha,
            "crop changed its native source arrays")
        rgb[position], target[position] = chosen_rgb, chosen_target
        selected.append(chosen_box)
        full_counts.append(full)
        proposed_counts.append(proposed)
        native_receipts.append(dict(position=position, slot=slot, native_RGB_sha256=input_rgb_sha,
            native_target_sha256=input_gt_sha, reader_provenance_authenticated_by_worker=False))
        checkpoint()
    require(baseline == owned_baseline, "baseline changed during batch construction")
    require(original_rgb_hash.hexdigest() == baseline["RGB_sha256"], "historical original RGB B8 SHA mismatch")
    require(original_target_hash.hexdigest() == baseline["target_sha256"], "historical original GT B8 SHA mismatch")
    wrapped = journal.batch_receipt(owned_baseline, selected, full_counts, proposed_counts,
        array_hash(rgb), array_hash(target))
    receipt = dict(schema=SCHEMA + ".batch", step=baseline["step"], journal=wrapped,
        native_slots=native_receipts, actual_baseline_RGB_and_GT_hashes_reproduced=True,
        candidate_hashes_computed_from_actual_owned_arrays=True, same_box_for_actual_RGB_and_GT=True,
        source_and_baseline_RAW_authenticated_by_worker=False, training_steps=0, NN=0)
    return rgb, target, receipt


class BatchJournalWriter:
    """HH_261008 - Create-only bounded journal; do not reuse an old experiment path."""
    def __init__(self, path, declared_steps, *, maximum_bytes=256 * 1024):
        require(type(declared_steps) is list and 1 <= len(declared_steps) <= 20000
            and all(type(step) is int and 1 <= step <= 20000 for step in declared_steps)
            and declared_steps == sorted(set(declared_steps)), "fixed distinct increasing steps required")
        require(type(maximum_bytes) is int and 1 <= maximum_bytes <= 1024 * 1024, "bounded journal allowance required")
        self.path = Path(path)
        require(self.path.is_absolute() and not self.path.exists()
            and self.path == self.path.resolve(strict=False)
            and not any(p.is_symlink() for p in (self.path, *self.path.parents)), "fresh canonical journal path required")
        self.steps, self.maximum_bytes = list(declared_steps), maximum_bytes
        self.count = self.bytes_written = 0
        self.closed = False
        self.stream = self.path.open("xb")
        self.open_generation = os.fstat(self.stream.fileno())

    def append(self, rgb, target, receipt):
        require(not self.closed and self.count < len(self.steps), "closed/full declared journal")
        require(type(receipt) is dict and receipt.get("schema") == SCHEMA + ".batch"
            and receipt.get("step") == self.steps[self.count], "declared receipt order differs")
        row = receipt.get("journal")
        require(type(row) is dict, "actual byte journal wrapper required")
        journal.validate_batch(row, row["baseline_exposure"])
        require(array_hash(rgb) == row["candidate_RGB_sha256"]
            and array_hash(target) == row["candidate_target_sha256"], "candidate arrays changed before append")
        raw = (json.dumps(receipt, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
        require(self.bytes_written + len(raw) <= self.maximum_bytes, "declared journal byte allowance exceeded")
        require(self.stream.write(raw) == len(raw), "partial journal write")
        self.stream.flush()
        self.bytes_written += len(raw)
        self.count += 1

    def close(self):
        if not self.closed:
            self.closed = True
            self.stream.close()
            current = self.path.stat()
            require(current.st_dev == self.open_generation.st_dev and current.st_ino == self.open_generation.st_ino
                and current.st_nlink == 1 and not self.path.is_symlink(), "journal path replaced during write")
            require(current.st_size == self.bytes_written, "journal bytes changed outside writer")

    def summary(self, failed=False):
        require(type(failed) is bool, "actual failure flag required")
        fail = int(failed and self.count < len(self.steps))
        return dict(declared_batches=len(self.steps), PASS_batches=self.count, FAIL_batches=fail,
            UNEXECUTED_batches=len(self.steps) - self.count - fail,
            original20k_batches_outside_this_declaration=20000 - len(self.steps),
            written_bytes=self.bytes_written, candidate_journal_not_original_full20k=True)
