"""HH_261012 - Copy one frozen serial NumPy batch into checked CPU tensors, without a new traversal or ACK.

ROOT/source/data/provider admission remains the external owner's responsibility.
This helper neither normalizes again nor evaluates the frozen trainer recipe.
Its CPU clone-byte proof is not trainer/GPU numerical equivalence, an external
consumer authentication, a TRAIN grant or model approval. NumPy and Torch are
lazy imports. Public array/tensor buffers and exported metadata remain mutable.
Private evidence retains references and bounded metadata/hashes, never duplicate
source buffers. The caller keeps the original SerialAdapter lifecycle and ACK.
"""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, field
import os
import threading

from . import map_serial_training_input_adapter_20261008_v1 as serial

SCHEMA = "portable_e2e.map_torch_cpu_input_bridge.20261012.v1"
CONSUMER = "map_torch_cpu_value_consumer_v1"
MAX_RECEIPT_BYTES = 16 * 1024
RECEIPT_FIELDS = frozenset(("schema", "step", "mode", "source_receipt_sha256",
    "provided_baseline_sha256", "native_arrays", "host_arrays", "tensor_arrays",
    "copy_from_host_bytes_verified", "scope"))
TENSOR_FIELDS = frozenset(("shape", "dtype", "device", "layout", "C_contiguous", "strides",
    "storage_offset", "requires_grad", "grad_fn_is_None", "is_pinned", "has_no_base",
    "nbytes", "storage_nbytes", "minimum", "maximum", "finite", "sha256"))
PROOF_FIELDS = frozenset(("schema", "status", "step", "mode", "source_receipt_sha256",
    "provided_baseline_sha256", "native_and_host_buffers_rechecked", "CPU_tensor_buffers_rechecked",
    "copy_from_host_bytes_verified", "tensor_arrays", "consumer_receipt", "scope"))
SCOPE = dict(ROOT_authenticated_by_bridge=False, data_authenticated_by_bridge=False,
    provider_authenticated_by_bridge=False, TRAIN_admitted_by_bridge=False,
    consumer_execution_authenticated_by_bridge=False, frozen_trainer_reference_authenticated_by_bridge=False,
    frozen_trainer_CPU_recipe_parity_claimed=False, GPU_numeric_equivalence_claimed=False,
    public_buffers_immutable=False, unique_storage_against_arbitrary_external_aliases_claimed=False,
    sampled_guards_are_OS_containment=False, source_buffer_copies_retained=0,
    source_normalization_performed=False, lifecycle_or_ACK_implemented=False,
    CUDA_API_invoked_by_bridge=False, filesystem_written_by_bridge=False,
    worker_count=0, model_forwards=0, backward=0, optimizer_updates=0, epochs=0, NN=0, GPU=0)
_local = threading.local()
require = serial.require


def _owner(pid, thread):
    require(type(pid) is type(thread) is int and pid > 0 and thread > 0
        and os.getpid() == pid and threading.get_ident() == thread, "exact owner PID/thread required before bridge work")


@contextmanager
def _operation(pid, thread, checkpoint):
    # HH_261012 - A temporary operation guard is not an input traversal, counter or lifetime state machine.
    _owner(pid, thread)
    require(checkpoint is None or callable(checkpoint), "optional no-argument owner checkpoint required")
    require(not getattr(_local, "busy", False), "reentrant CPU bridge operation denied")
    _local.busy = True
    def poll():
        _owner(pid, thread)
        if checkpoint is not None:
            checkpoint()
        _owner(pid, thread)
    try:
        yield poll
    finally:
        _local.busy = False


def _bindings(source):
    import numpy as np
    values = (source.native_RGB, source.native_target, source.host_RGB, source.host_target)
    require(all(type(value) is np.ndarray for value in values), "exact NumPy source buffers required")
    return values, tuple(value.ctypes.data for value in values)


def _source_metadata(source, step, mode, baseline):
    require(type(source) is serial.ProducedBatch and type(step) is int and 1 <= step <= 20000
        and type(source.step) is int and source.step == step and type(mode) is str and mode in serial.MODES,
        "exact ProducedBatch and declared original step/mode required")
    serial.journal.validate_baseline(baseline, step)
    original = deepcopy(baseline)
    receipt = serial._observed_receipt(source.receipt)
    require(type(receipt) is dict and set(receipt) == serial.RECEIPT_FIELDS
        and receipt["schema"] == serial.SCHEMA+".batch" and type(receipt["step"]) is int and receipt["step"] == step
        and type(receipt["mode"]) is str and receipt["mode"] == mode
        and serial._typed_equal(receipt["provided_baseline"], original)
        and receipt["provided_baseline_sha256"] == serial._metadata_sha(original)
        and serial._typed_equal(receipt["scope"], serial.SCOPE), "frozen source receipt/baseline/mode/scope differs")
    slots = receipt["native_slots"]
    require(type(slots) is list and len(slots) == 8, "eight source native slot witnesses required")
    for position, row in enumerate(slots):
        require(type(row) is dict and set(row) == {"position", "slot", "native_RGB_sha256", "native_target_sha256",
            "reader_provenance_authenticated_by_worker"} and type(row["position"]) is int and row["position"] == position
            and type(row["slot"]) is int and row["slot"] == original["physical_slot_indices"][position]
            and serial.journal.sha(row["native_RGB_sha256"]) and serial.journal.sha(row["native_target_sha256"])
            and row["reader_provenance_authenticated_by_worker"] is False, "source native slot witness differs")
    if mode == serial.ORIGINAL_CROP_CONTROL:
        require(serial._typed_equal(receipt["control_exposure"], original) and receipt["candidate_worker_receipt"] is None,
            "CONTROL must preserve exact six-field exposure, without a candidate journal")
        rgb_pin, gt_pin = original["RGB_sha256"], original["target_sha256"]
    else:
        candidate = receipt["candidate_worker_receipt"]
        require(receipt["control_exposure"] is None and type(candidate) is dict and set(candidate) == {
            "schema", "step", "journal", "native_slots", "actual_baseline_RGB_and_GT_hashes_reproduced",
            "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT",
            "source_and_baseline_RAW_authenticated_by_worker", "training_steps", "NN"}
            and candidate["schema"] == serial.worker.SCHEMA+".batch" and type(candidate["step"]) is int and candidate["step"] == step
            and serial._typed_equal(candidate["native_slots"], slots), "exact candidate worker V3 source receipt required")
        serial.journal.validate_batch(candidate["journal"], original)
        require(all(candidate[key] is True for key in ("actual_baseline_RGB_and_GT_hashes_reproduced",
            "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT"))
            and candidate["source_and_baseline_RAW_authenticated_by_worker"] is False
            and type(candidate["training_steps"]) is type(candidate["NN"]) is int
            and candidate["training_steps"] == candidate["NN"] == 0, "candidate source scope differs")
        rgb_pin, gt_pin = candidate["journal"]["candidate_RGB_sha256"], candidate["journal"]["candidate_target_sha256"]
    return original, receipt, rgb_pin, gt_pin


def _source_check(source, step, expected, receipt_binding, values, pointers, rgb_pin, gt_pin):
    require(type(source) is serial.ProducedBatch and type(source.step) is int and source.step == step
        and source.receipt is receipt_binding and serial._typed_equal(source.receipt, expected),
        "source step or receipt was mutated/rebound")
    current, current_pointers = _bindings(source)
    require(all(a is b for a, b in zip(current, values)) and current_pointers == pointers,
        "source buffer or storage was rebound")
    native, host = serial._descriptors(*values)
    require(serial._typed_equal(native, expected["native_arrays"]) and serial._typed_equal(host, expected["host_arrays"])
        and native["RGB"]["sha256"] == rgb_pin and native["target"]["sha256"] == gt_pin,
        "actual source native/host descriptor or byte SHA differs")
    return native, host


def _tensor_descriptor(value, *, shape, dtype, maximum):
    import numpy as np
    import torch
    strides = (691200, 230400, 640, 1) if len(shape) == 4 else (230400, 640, 1)
    require(type(value) is torch.Tensor and value.device.type == "cpu" and value.device.index is None
        and value.dtype == dtype and tuple(value.shape) == shape and value.layout == torch.strided
        and value.is_contiguous() and tuple(value.stride()) == strides and value.storage_offset() == 0
        and value.requires_grad is False and value.grad_fn is None and value.grad is None
        and value._base is None and value.is_pinned() is False, "exact independent gradient-free unpinned CPU tensor ABI required")
    # HH_261012 - storage().size() is available in the existing Torch1.8 provider; no modern untyped_storage API.
    storage = value.storage()
    amount = value.numel()*value.element_size()
    require(storage.size()*value.element_size() == amount <= serial.MAX_ARRAY_BYTES,
        "bounded exact CPU clone storage required")
    view = value.numpy()
    require(type(view) is np.ndarray and view.flags.c_contiguous and view.shape == shape
        and view.dtype == (np.float32 if dtype == torch.float32 else np.int64), "exact CPU observation view required")
    low, high = view.min().item(), view.max().item()
    require(bool(np.isfinite(view).all()) and 0 <= low <= high <= maximum, "finite declared CPU tensor range required")
    descriptor = dict(shape=list(shape), dtype=str(view.dtype), device="cpu", layout="strided", C_contiguous=True,
        strides=list(strides), storage_offset=0, requires_grad=False, grad_fn_is_None=True, is_pinned=False,
        has_no_base=True, nbytes=amount, storage_nbytes=amount, minimum=low, maximum=high,
        finite=True, sha256=serial._array_sha(view))
    require(set(descriptor) == TENSOR_FIELDS, "exact bounded CPU descriptor required")
    return descriptor, view, (value.data_ptr(), storage.data_ptr())


def _tensor_pair(rgb, target, values, host):
    import numpy as np
    import torch
    RGB, rgb_view, rgb_pointer = _tensor_descriptor(rgb, shape=(8, 3, 360, 640), dtype=torch.float32, maximum=1)
    GT, gt_view, gt_pointer = _tensor_descriptor(target, shape=(8, 360, 640), dtype=torch.int64, maximum=6)
    require(not np.shares_memory(rgb_view, gt_view) and not any(np.shares_memory(view, source)
        for view in (rgb_view, gt_view) for source in values), "CPU tensors must not alias each other or any native/host source buffer")
    require(RGB["sha256"] == host["RGB"]["sha256"] and GT["sha256"] == host["target"]["sha256"],
        "CPU clone differs from actual supplied host bits")
    return dict(RGB=RGB, target=GT), (rgb_pointer, gt_pointer)


@dataclass(frozen=True)
class _Evidence:
    source: object
    step: int
    mode: str
    source_receipt: dict
    source_receipt_binding: object
    source_arrays: tuple
    source_pointers: tuple
    rgb_pin: str
    gt_pin: str
    tensors: tuple
    tensor_pointers: tuple
    receipt: dict
    receipt_binding: object
    owner_pid: int
    owner_thread_ident: int


@dataclass(frozen=True)
class TensorBatch:
    """HH_261012 - Frozen exported bindings do not freeze buffers or prove exclusive external ownership."""
    step: int
    mode: str
    RGB: object
    target: object
    receipt: dict
    _private: object = field(repr=False, compare=False)

    def __getstate__(self):
        raise TypeError("owner-bound CPU TensorBatch cannot be pickled or fork-shared")


def _verify(bundle, evidence):
    require(type(bundle.step) is int and bundle.step == evidence.step and type(bundle.mode) is str
        and bundle.mode == evidence.mode and bundle.receipt is evidence.receipt_binding
        and serial._typed_equal(bundle.receipt, evidence.receipt)
        and bundle.RGB is evidence.tensors[0] and bundle.target is evidence.tensors[1],
        "exported TensorBatch step/mode/receipt/tensor was mutated or rebound")
    native, host = _source_check(evidence.source, evidence.step, evidence.source_receipt,
        evidence.source_receipt_binding, evidence.source_arrays, evidence.source_pointers, evidence.rgb_pin, evidence.gt_pin)
    tensors, pointers = _tensor_pair(bundle.RGB, bundle.target, evidence.source_arrays, host)
    require(pointers == evidence.tensor_pointers and serial._typed_equal(tensors, evidence.receipt["tensor_arrays"]),
        "CPU tensor storage/descriptor/bits changed since construction")
    return native, host, tensors


def build_cpu_tensor_batch(source, *, expected_step, expected_mode, expected_baseline,
                           owner_pid, owner_thread_ident, checkpoint=None):
    """HH_261012 - Return CPU clones only; supplied rows/owner identity are checks, not admission authority."""
    with _operation(owner_pid, owner_thread_ident, checkpoint) as poll:
        baseline, source_expected, rgb_pin, gt_pin = _source_metadata(source, expected_step, expected_mode, expected_baseline)
        receipt_binding = source.receipt
        values, pointers = _bindings(source)
        poll()
        _source_check(source, expected_step, source_expected, receipt_binding, values, pointers, rgb_pin, gt_pin)
        import torch
        poll()
        _source_check(source, expected_step, source_expected, receipt_binding, values, pointers, rgb_pin, gt_pin)
        rgb = torch.from_numpy(values[2]).clone()
        poll()
        _source_check(source, expected_step, source_expected, receipt_binding, values, pointers, rgb_pin, gt_pin)
        target = torch.from_numpy(values[3]).clone()
        poll()
        native, host = _source_check(source, expected_step, source_expected, receipt_binding, values, pointers, rgb_pin, gt_pin)
        tensors, tensor_pointers = _tensor_pair(rgb, target, values, host)
        receipt = dict(schema=SCHEMA+".batch", step=expected_step, mode=expected_mode,
            source_receipt_sha256=serial._metadata_sha(source_expected), provided_baseline_sha256=serial._metadata_sha(baseline),
            native_arrays=native, host_arrays=host, tensor_arrays=tensors, copy_from_host_bytes_verified=True, scope=dict(SCOPE))
        require(set(receipt) == RECEIPT_FIELDS and len(serial._encoded(receipt)) <= MAX_RECEIPT_BYTES,
            "bounded exact CPU bridge receipt required")
        evidence = _Evidence(source, expected_step, expected_mode, source_expected, receipt_binding, values, pointers,
            rgb_pin, gt_pin, (rgb, target), tensor_pointers, deepcopy(receipt), receipt, owner_pid, owner_thread_ident)
        bundle = TensorBatch(expected_step, expected_mode, rgb, target, receipt, evidence)
        poll()
        _verify(bundle, evidence)
        return bundle


def verify_cpu_tensor_batch(bundle, *, checkpoint=None):
    """HH_261012 - Recheck current buffers against private snapshots; external reference/consumer claims remain false."""
    require(type(bundle) is TensorBatch and type(bundle._private) is _Evidence, "actual owner-bound TensorBatch required")
    evidence = bundle._private
    with _operation(evidence.owner_pid, evidence.owner_thread_ident, checkpoint) as poll:
        poll()
        _verify(bundle, evidence)
        poll()
        _, host, tensors = _verify(bundle, evidence)
        proof = dict(schema=SCHEMA+".proof", status="CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE",
            step=evidence.step, mode=evidence.mode, source_receipt_sha256=evidence.receipt["source_receipt_sha256"],
            provided_baseline_sha256=evidence.receipt["provided_baseline_sha256"],
            native_and_host_buffers_rechecked=True, CPU_tensor_buffers_rechecked=True,
            copy_from_host_bytes_verified=True, tensor_arrays=tensors,
            consumer_receipt=dict(consumer=CONSUMER, accepted=True, host_RGB_sha256=host["RGB"]["sha256"],
                host_target_sha256=host["target"]["sha256"]), scope=dict(SCOPE))
        require(set(proof) == PROOF_FIELDS and len(serial._encoded(proof)) <= MAX_RECEIPT_BYTES, "bounded exact CPU copy proof required")
        return proof
