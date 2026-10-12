"""HH_261012 - Author synthetic CPU B8 copy-boundary tests for ROOT; no archive, NN, GPU or trainer.

NumPy/Torch imports occur only in test helpers/cases when ROOT executes this
suite. The toy crop and supplied baseline are not actual-data authentication.
Torch clone-copy fidelity here is deliberately separate from the independently
sealed original trainer CPU reference exercised by the external runner.
"""
import ast
from copy import deepcopy
import os
from pathlib import Path
import threading

import pytest

from portable_e2e import map_torch_cpu_input_bridge_20261012_v1 as bridge
from portable_e2e import map_serial_training_input_adapter_20261008_v1 as serial


class Reader:
    def __init__(self, positive=True):
        import numpy as np
        self.rgb = np.empty((360, 640, 3), np.uint8)
        self.rgb[:] = [0, 127, 255]
        self.target = np.zeros((360, 640), np.uint8)
        if positive:
            self.target[300:320, 550:570] = 5

    def read(self, slot):
        return self.rgb.copy(), self.target.copy()

    def close(self):
        pass


def crop(rgb, target, box):
    # HH_261012 - A fictional deterministic crop fixture, not a frozen-original provider claim.
    if box == [0, 0, 360, 640]:
        return rgb.copy(), target.copy()
    out = target.copy()
    out.fill(0)
    return rgb.copy(), out


def source(mode=serial.ORIGINAL_CROP_CONTROL, positive=True):
    import numpy as np
    reader = Reader(positive)
    rgb, target = crop(reader.rgb, reader.target, [0, 0, 120, 300])
    row = dict(step=1, physical_slot_indices=list(range(8)), shared_crop_boxes=[[0, 0, 120, 300] for _ in range(8)],
        actual_paired_TRUE_bits=[0]*8, RGB_sha256=serial.worker.array_hash(np.stack([rgb]*8)),
        target_sha256=serial.worker.array_hash(np.stack([target]*8)))
    adapter = serial.SerialTrainingInputAdapter([row], reader, crop, mode=mode)
    return adapter, adapter.next_batch(), row


def build(batch, row, mode=serial.ORIGINAL_CROP_CONTROL, **kwargs):
    arguments = dict(expected_step=1, expected_mode=mode, expected_baseline=row,
        owner_pid=os.getpid(), owner_thread_ident=threading.get_ident())
    arguments.update(kwargs)
    return bridge.build_cpu_tensor_batch(batch, **arguments)


@pytest.mark.parametrize("mode,positive", [(serial.ORIGINAL_CROP_CONTROL, False),
    (serial.SIGN_PRESERVING_CANDIDATE, False), (serial.SIGN_PRESERVING_CANDIDATE, True)])
def test_real_cpu_clones_exact_shapes_bits_nonaliasing_and_unchanged_serial_ACK(mode, positive):
    import numpy as np
    adapter, batch, row = source(mode, positive)
    originals = tuple(serial._array_sha(value) for value in (batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target))
    bundle = build(batch, row, mode)
    assert bundle.step == 1 and bundle.mode == mode and set(bundle.receipt) == bridge.RECEIPT_FIELDS
    assert bundle.RGB.shape == (8, 3, 360, 640) and bundle.target.shape == (8, 360, 640)
    assert bundle.RGB.device.type == bundle.target.device.type == "cpu"
    assert bundle.RGB.requires_grad is bundle.target.requires_grad is False
    assert bundle.RGB.is_pinned() is bundle.target.is_pinned() is False
    assert np.array_equal(bundle.RGB.numpy().view(np.uint32), batch.host_RGB.view(np.uint32))
    assert np.array_equal(bundle.target.numpy(), batch.host_target)
    assert not any(np.shares_memory(view, value) for view in (bundle.RGB.numpy(), bundle.target.numpy())
        for value in (batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target))
    proof = bridge.verify_cpu_tensor_batch(bundle)
    assert set(proof) == bridge.PROOF_FIELDS and proof["status"] == "CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE"
    assert proof["scope"] == bridge.SCOPE and proof["scope"]["frozen_trainer_CPU_recipe_parity_claimed"] is False
    assert proof["scope"]["TRAIN_admitted_by_bridge"] is proof["scope"]["provider_authenticated_by_bridge"] is False
    assert proof["scope"]["consumer_execution_authenticated_by_bridge"] is False
    assert proof["scope"]["unique_storage_against_arbitrary_external_aliases_claimed"] is False
    assert tuple(serial._array_sha(value) for value in (batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target)) == originals
    assert set(proof["consumer_receipt"]) == serial.CONSUMER_FIELDS
    assert proof["consumer_receipt"]["consumer"] == "map_torch_cpu_value_consumer_v1"
    # HH_261012 - Only the original adapter ACK advances consumption; the bridge has no cursor or ACK.
    assert adapter.stats_snapshot()["PASS_batches"] == 0
    adapter.acknowledge(1, consumer_receipt=proof["consumer_receipt"])
    assert adapter.close()["PASS_batches"] == 1
    assert bridge.verify_cpu_tensor_batch(bundle)["consumer_receipt"] == proof["consumer_receipt"]


@pytest.mark.parametrize("field", ["native_RGB", "native_target", "host_RGB", "host_target"])
def test_source_mutation_after_clone_is_not_hidden_by_tensor_copy(field):
    _, batch, row = source()
    bundle = build(batch, row)
    getattr(batch, field).flat[0] += 1
    with pytest.raises(ValueError, match="source"):
        bridge.verify_cpu_tensor_batch(bundle)


@pytest.mark.parametrize("fault", ["RGB_bits", "target_bits", "RGB_nan", "RGB_range", "target_range",
    "RGB_dtype", "target_dtype", "RGB_shape", "requires_grad"])
def test_current_tensor_bits_dtype_range_shape_and_gradient_denials(fault):
    import torch
    _, batch, row = source()
    bundle = build(batch, row)
    if fault == "RGB_bits": bundle.RGB[0, 0, 0, 0] = 0.25
    elif fault == "target_bits": bundle.target[0, 0, 0] = 1
    elif fault == "RGB_nan": bundle.RGB[0, 0, 0, 0] = float("nan")
    elif fault == "RGB_range": bundle.RGB[0, 0, 0, 0] = 2
    elif fault == "target_range": bundle.target[0, 0, 0] = 7
    elif fault == "RGB_dtype": bundle.RGB.data = bundle.RGB.to(dtype=torch.float64)
    elif fault == "target_dtype": bundle.target.data = bundle.target.to(dtype=torch.int32)
    elif fault == "RGB_shape": bundle.RGB.resize_(8, 3, 360, 639)
    else: bundle.RGB.requires_grad_(True)
    with pytest.raises(ValueError):
        bridge.verify_cpu_tensor_batch(bundle)


@pytest.mark.parametrize("fault", ["step_bool", "step_future", "mode", "RGB_ref", "target_ref", "receipt_ref", "receipt_bits"])
def test_exported_tensor_batch_step_mode_receipt_and_tensor_rebinding_denied(fault):
    _, batch, row = source()
    bundle = build(batch, row)
    if fault == "step_bool": object.__setattr__(bundle, "step", True)
    elif fault == "step_future": object.__setattr__(bundle, "step", 2)
    elif fault == "mode": object.__setattr__(bundle, "mode", serial.SIGN_PRESERVING_CANDIDATE)
    elif fault == "RGB_ref": object.__setattr__(bundle, "RGB", bundle.RGB.clone())
    elif fault == "target_ref": object.__setattr__(bundle, "target", bundle.target.clone())
    elif fault == "receipt_ref": object.__setattr__(bundle, "receipt", deepcopy(bundle.receipt))
    else: bundle.receipt["scope"]["TRAIN_admitted_by_bridge"] = True
    with pytest.raises(ValueError, match="mutated|rebound"):
        bridge.verify_cpu_tensor_batch(bundle)


def test_simultaneous_public_receipt_tensor_and_source_forgery_denied():
    _, batch, row = source()
    bundle = build(batch, row)
    bundle.RGB[0, 0, 0, 0] = 0.25
    bundle.receipt["tensor_arrays"]["RGB"]["sha256"] = serial._array_sha(bundle.RGB.numpy())
    batch.host_RGB[0, 0, 0, 0] = 0.25
    batch.receipt["host_arrays"]["RGB"]["sha256"] = serial._array_sha(batch.host_RGB)
    with pytest.raises(ValueError, match="mutated|rebound"):
        bridge.verify_cpu_tensor_batch(bundle)


@pytest.mark.parametrize("field", ["native_RGB", "native_target", "host_RGB", "host_target", "receipt", "step"])
def test_source_binding_rebound_even_with_identical_bytes_is_denied(field):
    _, batch, row = source()
    bundle = build(batch, row)
    if field == "receipt": value = deepcopy(batch.receipt)
    elif field == "step": value = True
    else: value = getattr(batch, field).copy()
    object.__setattr__(batch, field, value)
    with pytest.raises(ValueError, match="source"):
        bridge.verify_cpu_tensor_batch(bundle)


def test_same_tensor_object_new_storage_is_denied_even_with_identical_bits():
    _, batch, row = source()
    bundle = build(batch, row)
    replacement = bundle.RGB.clone()
    bundle.RGB.set_(replacement)
    with pytest.raises(ValueError, match="storage"):
        bridge.verify_cpu_tensor_batch(bundle)


@pytest.mark.parametrize("fault", ["step_bool", "step_future", "mode_bool", "mode_wrong", "baseline_step", "baseline_SHA", "source_OWNDATA"])
def test_supplied_source_contract_and_baseline_denied_before_tensor_allocation(monkeypatch, fault):
    import torch
    _, batch, row = source()
    kwargs = {}
    if fault == "step_bool": kwargs["expected_step"] = True
    elif fault == "step_future": kwargs["expected_step"] = 2
    elif fault == "mode_bool": kwargs["expected_mode"] = True
    elif fault == "mode_wrong": kwargs["expected_mode"] = serial.SIGN_PRESERVING_CANDIDATE
    elif fault == "baseline_step": row["step"] = 2
    elif fault == "baseline_SHA": row["RGB_sha256"] = "f"*64
    else: object.__setattr__(batch, "host_RGB", batch.host_RGB.view())
    calls = []
    def forbidden(value):
        calls.append(value)
        raise AssertionError("invalid source reached tensor allocation")
    monkeypatch.setattr(torch, "from_numpy", forbidden)
    with pytest.raises(ValueError):
        build(batch, row, **kwargs)
    assert calls == []


def test_expected_baseline_argument_is_privately_isolated_without_array_copies():
    _, batch, row = source()
    expected = deepcopy(row)
    bundle = build(batch, row)
    row["step"] = 2
    row["shared_crop_boxes"][0][0] = 1
    proof = bridge.verify_cpu_tensor_batch(bundle)
    assert proof["provided_baseline_sha256"] == serial._metadata_sha(expected)
    assert all(a is b for a, b in zip(bundle._private.source_arrays,
        (batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target)))
    assert proof["scope"]["source_buffer_copies_retained"] == 0


@pytest.mark.parametrize("owner", ["pid", "thread", "bool_pid", "bool_thread"])
def test_owner_denied_before_checkpoint_or_tensor_allocation(owner):
    _, batch, row = source()
    kwargs = {}
    if owner == "pid": kwargs["owner_pid"] = os.getpid()+1
    elif owner == "thread": kwargs["owner_thread_ident"] = threading.get_ident()+1
    elif owner == "bool_pid": kwargs["owner_pid"] = True
    else: kwargs["owner_thread_ident"] = True
    calls = []
    with pytest.raises(ValueError, match="owner"):
        build(batch, row, checkpoint=lambda: calls.append(1), **kwargs)
    assert calls == []


@pytest.mark.parametrize("owner", ["pid", "thread"])
def test_verify_foreign_owner_denied_before_callback(monkeypatch, owner):
    _, batch, row = source()
    bundle = build(batch, row)
    calls = []
    with monkeypatch.context() as patch:
        if owner == "pid": patch.setattr(bridge.os, "getpid", lambda: bundle._private.owner_pid+1)
        else: patch.setattr(bridge.threading, "get_ident", lambda: bundle._private.owner_thread_ident+1)
        with pytest.raises(ValueError, match="owner"):
            bridge.verify_cpu_tensor_batch(bundle, checkpoint=lambda: calls.append(1))
    assert calls == []
    assert bridge.verify_cpu_tensor_batch(bundle)["copy_from_host_bytes_verified"] is True


def test_reentrant_build_verify_and_callback_source_mutation_rejected_and_guard_released():
    _, batch, row = source()
    with pytest.raises(ValueError, match="reentrant"):
        build(batch, row, checkpoint=lambda: build(batch, row))
    bundle = build(batch, row)
    with pytest.raises(ValueError, match="reentrant"):
        bridge.verify_cpu_tensor_batch(bundle, checkpoint=lambda: bridge.verify_cpu_tensor_batch(bundle))
    assert bridge.verify_cpu_tensor_batch(bundle)["copy_from_host_bytes_verified"] is True
    def mutate(): batch.host_RGB.flat[0] = 0.25
    with pytest.raises(ValueError, match="source"):
        bridge.verify_cpu_tensor_batch(bundle, checkpoint=mutate)


def test_copy_from_numpy_view_instead_of_clone_denied(monkeypatch):
    import torch
    _, batch, row = source()
    monkeypatch.setattr(torch.Tensor, "clone", lambda self: self)
    with pytest.raises(ValueError, match="alias"):
        build(batch, row)


def test_source_lazy_cpu_only_no_new_normalization_traversal_reference_or_ACK():
    tree = ast.parse(Path(bridge.__file__).read_bytes())
    top_imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any(alias.name in {"numpy", "torch"} for node in top_imports for alias in node.names)
    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert not functions & {"next_batch", "acknowledge", "abort", "close", "train", "run"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not calls & {"torch.tensor", "torch.cuda.is_initialized", "torch.cuda.is_available", "np.load", "open"}
    assert not any(name.endswith((".cuda", ".backward", ".acknowledge", ".next_batch", ".read", ".write", ".set_num_threads")) for name in calls)
    assert not any(isinstance(node, (ast.Div, ast.FloorDiv)) for node in ast.walk(tree))
