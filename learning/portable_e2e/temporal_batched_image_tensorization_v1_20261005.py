"""HH_261005 - Batch only six image normalization operations; retain every original runtime guard.

This research candidate is not active on import and does not prove a timing gain.
It privately replaces the frozen runtime's image-building block, proves its
inverse AST, and scopes the actual imported SingleCopy provider only when asked.
No source pixels, camera order, resize, small tensor, cache or finite guard is
removed. Timing receipts contain scalars only and never suppress an exception.
"""

from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
from pathlib import Path
import threading
import time
from types import FunctionType

import numpy as np
import torch
from PIL import Image

from .contract import ContractError
from . import runtime
from . import temporal_single_copy_history_research_20261001 as single_copy


SCHEMA = "portable_e2e.image_only_batched_tensorization.20261005.v1.research"
PARENT_SOURCE_PINS = {
    "runtime.py": "bd1f672efe5cf571744bd4c5ef7655435dc67dddadc41c23fddddb82d721a57b",
    "temporal_single_copy_history_research_20261001.py": "42d98271cddee6f19e01c35c29167cd9d1a8dfb1492f34f2334b74b17274d053",
}
_ORIGINAL = runtime.build_runtime_tensors
_SINGLE_COPY_METHOD = single_copy._SingleCopyBuildMixin.build_inputs
_ACTIVE = False
_TRUSTED_BINDINGS = {
    "rgb": runtime._rgb_array, "np": runtime.np, "torch": runtime.torch,
    "Image": runtime.Image, "mean": runtime.RGB_MEAN, "std": runtime.RGB_STD,
    "from_numpy": torch.from_numpy, "stack": torch.stack,
    "fromarray": Image.fromarray, "resize": Image.Image.resize,
    "new_tensor": torch.Tensor.new_tensor, "div_": torch.Tensor.div_,
}


def verify_frozen_sources():
    """HH_261005 - Bind the tiny immutable source closure at setup, never rehash it per image."""
    folder = Path(__file__).parent
    for name, expected in PARENT_SOURCE_PINS.items():
        path = folder / name
        if path.is_symlink() or path.resolve() != path or not path.is_file() \
                or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ContractError("batched image candidate requires frozen source: " + name)
    if runtime.build_runtime_tensors is not _ORIGINAL \
            or single_copy._SingleCopyBuildMixin.build_inputs is not _SINGLE_COPY_METHOD:
        raise ContractError("batched image candidate requires original runtime and SingleCopy methods")


def _trusted_image_bindings():
    current = {
        "rgb": runtime._rgb_array, "np": runtime.np, "torch": runtime.torch,
        "Image": runtime.Image, "mean": runtime.RGB_MEAN, "std": runtime.RGB_STD,
        "from_numpy": torch.from_numpy, "stack": torch.stack,
        "fromarray": Image.fromarray, "resize": Image.Image.resize,
        "new_tensor": torch.Tensor.new_tensor, "div_": torch.Tensor.div_,
    }
    return all(current[name] is expected for name, expected in _TRUSTED_BINDINGS.items())


def image_block_trees():
    """HH_261005 - Return an exact inverse proof for only the original three image-building statements."""
    verify_frozen_sources()
    module = ast.parse(Path(runtime.__file__).read_text(encoding="utf-8"))
    methods = [node for node in module.body
               if isinstance(node, ast.FunctionDef) and node.name == "build_runtime_tensors"]
    if len(methods) != 1:
        raise ContractError("one frozen runtime builder required")
    original = ast.Module(body=[copy.deepcopy(methods[0])], type_ignores=[])
    body = original.body[0].body
    starts = [index for index, node in enumerate(body)
              if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)
              and node.target.id == "image_tensors"]
    if len(starts) != 1:
        raise ContractError("exact original image_tensors annotation required")
    start = starts[0]
    block = body[start:start + 3]
    if len(block) != 3 or not isinstance(block[1], ast.Assign) \
            or not isinstance(block[1].targets[0], ast.Name) \
            or block[1].targets[0].id != "resampling" or not isinstance(block[2], ast.For) \
            or len(block[2].body) != 8:
        raise ContractError("frozen three-statement image block changed")
    changed = copy.deepcopy(original)
    changed.body[0].body[start:start + 3] = ast.parse(
        "image_tensors = _research_image_provider(inputs, config)"
    ).body
    restored = copy.deepcopy(changed)
    restored.body[0].body[start:start + 1] = copy.deepcopy(block)
    if ast.dump(restored, include_attributes=False) != ast.dump(original, include_attributes=False):
        raise ContractError("batched image inverse differs from the complete original runtime builder")
    return original, changed, restored


def _private_builder(provider):
    original, changed, restored = image_block_trees()
    # HH_261005 - Compile once with a private globals dictionary; all original nonimage globals retain identity.
    namespace = dict(_ORIGINAL.__globals__)
    namespace["_research_image_provider"] = provider
    exec(compile(ast.fix_missing_locations(changed), str(Path(__file__)), "exec"), namespace)
    compiled = namespace["build_runtime_tensors"]
    result = FunctionType(compiled.__code__, namespace, _ORIGINAL.__name__,
                          _ORIGINAL.__defaults__, _ORIGINAL.__closure__)
    result.__kwdefaults__ = copy.copy(_ORIGINAL.__kwdefaults__)
    result.__annotations__ = dict(_ORIGINAL.__annotations__)
    result._research_inverse_ast_equal = True
    result._research_original_ast_sha256 = hashlib.sha256(
        ast.dump(original, include_attributes=False).encode()).hexdigest()
    return result


class BatchedImageTensorizer:
    """HH_261005 - Keep original checks around one CPU float32 batch and last-call scalar timing."""

    def __init__(self):
        self.last_timing = None
        self._call_ordinal = 0
        self._current_timing = None
        self.runtime_builder = _private_builder(self._images)

    def _images(self, inputs, config):
        receipt = self._current_timing
        rows = receipt["camera_preprocessing"]
        resized_tensors = []
        resampling = getattr(Image, "Resampling", Image).BILINEAR
        for index, (camera_name, frame) in enumerate(zip(runtime.CAMERA_ORDER, inputs.camera_frames)):
            row = dict(role_index=index, role=camera_name, encoding=frame.encoding,
                       started_monotonic_ns=time.perf_counter_ns(), status="RUNNING")
            rows.append(row)
            try:
                array = runtime._rgb_array(frame, camera_name=camera_name)
                image = Image.fromarray(array, mode="RGB")
                resized = image.resize((config.image_width, config.image_height), resample=resampling)
                resized_array = np.asarray(resized, dtype=np.float32).copy()
                resized_tensors.append(torch.from_numpy(resized_array).permute(2, 0, 1).contiguous())
                row["status"] = "PASS"
            except BaseException as error:
                row.update(status="FAILED", error_type=type(error).__name__, error=str(error))
                raise
            finally:
                row["finished_monotonic_ns"] = time.perf_counter_ns()
        start = time.perf_counter_ns()
        receipt["batch_normalization"] = dict(started_monotonic_ns=start, status="RUNNING")
        try:
            # HH_261005 - Keep float32 div255, subtraction and division order; do not fuse, approximate or mutate sources.
            batch = torch.stack(resized_tensors).div_(255.0)
            mean = batch.new_tensor(runtime.RGB_MEAN).view(1, 3, 1, 1)
            std = batch.new_tensor(runtime.RGB_STD).view(1, 3, 1, 1)
            normalized = (batch - mean) / std
            receipt["batch_normalization"]["status"] = "PASS"
            # HH_261005 - Leave the original final torch.stack(...).unsqueeze(0) and finite tensor loop unchanged.
            return list(normalized.unbind(0))
        except BaseException as error:
            receipt["batch_normalization"].update(status="FAILED", error_type=type(error).__name__, error=str(error))
            raise
        finally:
            receipt["batch_normalization"]["finished_monotonic_ns"] = time.perf_counter_ns()

    def __call__(self, inputs, config):
        if self._current_timing is not None:
            raise ContractError("batched image tensorization cannot recurse or overlap")
        self._call_ordinal += 1
        receipt = dict(schema=SCHEMA + ".timing", research_only=True,
                       actual_speedup_proven=False, call_ordinal=self._call_ordinal,
                       thread_id=threading.get_ident(), started_monotonic_ns=time.perf_counter_ns(),
                       camera_preprocessing=[], status="RUNNING", fallback_to_original=False,
                       original_final_finite_validation_retained=True,
                       original_small_tensors_and_history_cache_unchanged=True)
        self._current_timing = receipt
        try:
            # HH_261005 - Unknown image implementations retain the exact ordinary path instead of gaining batch admission.
            if not _trusted_image_bindings():
                receipt["fallback_to_original"] = True
                result = _ORIGINAL(inputs, config)
            else:
                result = self.runtime_builder(inputs, config)
            receipt["status"] = "PASS"
            return result
        except BaseException as error:
            receipt.update(status="FAILED", error_type=type(error).__name__, error=str(error))
            raise
        finally:
            receipt["finished_monotonic_ns"] = time.perf_counter_ns()
            self.last_timing = receipt
            self._current_timing = None


@contextmanager
def activated(provider=None):
    """HH_261005 - Scope only the actual SingleCopy imported provider; restore it on every exit."""
    global _ACTIVE
    if _ACTIVE:
        raise ContractError("batched image provider activation cannot nest")
    verify_frozen_sources()
    actual_globals = _SINGLE_COPY_METHOD.__globals__
    if actual_globals is not vars(single_copy) or actual_globals.get("build_runtime_tensors") is not _ORIGINAL:
        raise ContractError("actual SingleCopy runtime provider is not the frozen original")
    value = BatchedImageTensorizer() if provider is None else provider
    if type(value) is not BatchedImageTensorizer:
        raise ContractError("exact batched image research provider required")
    saved = actual_globals["build_runtime_tensors"]
    _ACTIVE = True
    try:
        actual_globals["build_runtime_tensors"] = value
        yield value
    finally:
        actual_globals["build_runtime_tensors"] = saved
        _ACTIVE = False
