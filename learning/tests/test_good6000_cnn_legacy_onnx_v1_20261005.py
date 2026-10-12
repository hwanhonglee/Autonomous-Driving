"""HH_261005 - Test source/scalar/synthetic-array admission only; no Torch import, real weights, tracing, ONNX export or GPU job."""

from array import array
import ast
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_good6000_cnn_legacy_onnx_v1_20261005 as helper


def legacy_sources():
    return {"__init__.py": "def export(): pass\n", "utils.py": "from torch._C import _check_onnx_proto\n"
        "def export(graph, proto):\n graph._export_onnx()\n _check_onnx_proto(proto)\n", "symbolic_opset9.py": ""}


def test_legacy_source_dependency_proof_uses_builtin_not_missing_Python_onnx():
    assert helper.legacy_dependency_report(legacy_sources()) == dict(Python_onnx_import_required_in_reviewed_sources=False,
        builtin_checker="torch._C._check_onnx_proto", actual_export_feasibility_verified=False)


@pytest.mark.parametrize("fault", ("import", "from", "missing_source", "missing_checker", "missing_export"))
def test_missing_or_changed_legacy_dependency_proof_denies(fault):
    sources = legacy_sources()
    if fault == "import": sources["symbolic_opset9.py"] = "import onnx\n"
    elif fault == "from": sources["symbolic_opset9.py"] = "from onnx.helper import make_tensor\n"
    elif fault == "missing_source": sources.pop("__init__.py")
    elif fault == "missing_checker": sources["utils.py"] = "def export(graph):\n graph._export_onnx()\n"
    elif fault == "missing_export": sources["utils.py"] = "from torch._C import _check_onnx_proto\n_check_onnx_proto(proto)\n"
    with pytest.raises(RuntimeError): helper.legacy_dependency_report(sources)


@pytest.mark.parametrize("batch", (6, 30))
def test_fixed_batch_tensor_names_dtypes_shapes_and_full_cold_bytes(batch):
    contract = helper.fixed_batch_contract(batch)
    assert contract["input_shape"] == [batch, 3, 180, 320] and contract["output_shape"] == [batch, 60, 128]
    assert contract["input_dtype"] == contract["output_dtype"] == "float32"
    assert contract["input_name"] == "normalized_images" and contract["output_name"] == "cnn_features"
    assert contract["input_bytes"] == (4147200 if batch == 6 else 20736000)
    assert contract["dynamic_axes"] is None and not contract["do_constant_folding"] and contract["enable_onnx_checker"]
    assert not contract["runtime_activation"]


@pytest.mark.parametrize("batch", (True, 1, 5, 7, 29, 31, 6.0, "30"))
def test_no_unplanned_batch_or_boolean_numeric_alias(batch):
    with pytest.raises(RuntimeError): helper.fixed_batch_contract(batch)


class SyntheticTensor:
    def __init__(self, value=1.25):
        self.values = array("f", [value])
        self.dtype, self.device, self.requires_grad = "float32", SimpleNamespace(type="cpu"), False
        self.shape, self._version = (1,), 0
    def detach(self): return self
    def contiguous(self): return self
    def numpy(self): return self.values
    def data_ptr(self): return id(self.values)
    def stride(self): return (1,)


def synthetic_state():
    names = ["image_encoder." + name for name in helper.CNN_NAMES] + ["synthetic_%03d" % index for index in range(87)]
    return {name: SyntheticTensor() for name in names}


def finite(value): return all(math.isfinite(number) for number in value.values)


def test_all101_synthetic_values_hashed_and_exact14_CNN_mapping():
    state = synthetic_state()
    rows = helper.tensor_inventory(state, "float32", finite)
    assert len(rows) == 101 and len(helper.cnn_inventory(rows)) == 14
    assert all(row["raw_bytes"] == 4 and row["raw_sha256"] == hashlib.sha256(array("f", [1.25]).tobytes()).hexdigest() for row in rows.values())
    state["synthetic_086"].values[0] = 2.0
    changed = helper.tensor_inventory(state, "float32", finite)
    assert changed["synthetic_086"]["raw_sha256"] != rows["synthetic_086"]["raw_sha256"]


@pytest.mark.parametrize("fault", ("count", "dtype", "device", "grad", "finite", "CNN_name"))
def test_every_bad_synthetic_weight_or_CNN_shape_namespace_denies(fault):
    state = synthetic_state()
    value = next(iter(state.values()))
    if fault == "count": state.pop("synthetic_086")
    elif fault == "dtype": value.dtype = "float16"
    elif fault == "device": value.device.type = "cuda"
    elif fault == "grad": value.requires_grad = True
    elif fault == "finite": value.values[0] = float("nan")
    elif fault == "CNN_name": state["image_encoder.extra.weight"] = state.pop("image_encoder.projection.weight")
    with pytest.raises(RuntimeError): helper.cnn_inventory(helper.tensor_inventory(state, "float32", finite))


class SyntheticSequence(list): pass
class SyntheticConv:
    def __init__(self, input_channels, output_channels, *, kernel=3, stride=2, padding=1, bias=False):
        self.in_channels, self.out_channels = input_channels, output_channels
        self.kernel_size, self.stride, self.padding, self.dilation = kernel, stride, padding, 1
        self.groups, self.bias, self.padding_mode = 1, object() if bias else None, "zeros"
class SyntheticNorm:
    def __init__(self, width): self.num_groups, self.num_channels, self.eps, self.affine = 8, width, 1e-5, True
class SyntheticActivation:
    def __init__(self): self.inplace = False
class SyntheticPool:
    def __init__(self):
        self.kernel_size, self.stride, self.padding = 2, 2, 0
        self.ceil_mode, self.count_include_pad, self.divisor_override = False, True, None


def synthetic_CNN_structure():
    classes = SimpleNamespace(Sequential=SyntheticSequence, Conv2d=SyntheticConv, GroupNorm=SyntheticNorm,
        SiLU=SyntheticActivation, AvgPool2d=SyntheticPool)
    features, previous = SyntheticSequence(), 3
    for width in (32, 64, 128, 192):
        features.extend((SyntheticConv(previous, width), SyntheticNorm(width), SyntheticActivation()))
        previous = width
    return SimpleNamespace(features=features, pool=SyntheticPool(), projection=SyntheticConv(192, 128, kernel=1, stride=1, padding=0, bias=True)), classes


def test_synthetic_exact_original_CNN_structure_seals_nonweight_attributes():
    encoder, classes = synthetic_CNN_structure()
    layout = helper.cnn_structure(encoder, classes)
    assert len(layout["features"]) == 4 and layout["output_grid"] == [6, 10] and layout["output_tokens"] == [60, 128]


@pytest.mark.parametrize("fault", ("stride", "padding", "norm_eps", "norm_groups", "inplace", "pool", "pool_divisor", "projection_stride", "projection_bias", "order"))
def test_synthetic_same_weight_names_cannot_hide_changed_CNN_structure(fault):
    encoder, classes = synthetic_CNN_structure()
    if fault == "stride": encoder.features[0].stride = 1
    elif fault == "padding": encoder.features[0].padding_mode = "reflect"
    elif fault == "norm_eps": encoder.features[1].eps = .001
    elif fault == "norm_groups": encoder.features[1].num_groups = 4
    elif fault == "inplace": encoder.features[2].inplace = True
    elif fault == "pool": encoder.pool.ceil_mode = True
    elif fault == "pool_divisor": encoder.pool.divisor_override = 2
    elif fault == "projection_stride": encoder.projection.stride = 2
    elif fault == "projection_bias": encoder.projection.bias = None
    elif fault == "order": encoder.features[0], encoder.features[1] = encoder.features[1], encoder.features[0]
    with pytest.raises(RuntimeError): helper.cnn_structure(encoder, classes)


class SyntheticModule:
    def __init__(self):
        self._forward_hooks, self._forward_pre_hooks, self._backward_hooks = {}, {}, {}
        self._state_dict_hooks, self._load_state_dict_pre_hooks = {}, {}
    def forward(self): raise AssertionError("no synthetic neural forward is permitted")


class SyntheticModel(SyntheticModule):
    def __init__(self):
        super().__init__()
        self.image_encoder, self.curvature_head = SyntheticModule(), SyntheticModule()
        self.image_encoder.entries, self.image_encoder.request, self.image_encoder.work = {}, None, (0, 0, 0, False)
        self._recovery_active, self._recovery_features, self.cache_stats = False, None, None
        self._cache_scope, self._cache_state, self._last_anchor_ns, self._pending_reset_reason = None, None, None, "checkpoint_loaded"
        self._recovery_hook_handle, self._recovery_hook_function = SimpleNamespace(id=9), self.recovery_hook
        self.curvature_head._forward_hooks[9] = self._recovery_hook_function
    def recovery_hook(self): raise AssertionError("no recovery forward is permitted")
    def named_modules(self): return (("", self), ("image_encoder", self.image_encoder), ("curvature_head", self.curvature_head))


def test_idle_synthetic_callback_cache_inventory_does_not_call_math():
    model = SyntheticModel()
    assert helper.callbacks_and_cache(model) == helper.callbacks_and_cache(model)


def test_callback_identity_changed_after_admission_cannot_match_prior_snapshot():
    model = SyntheticModel()
    before = helper.callbacks_and_cache(model)
    replacement = model.recovery_hook
    model._recovery_hook_function = replacement
    model.curvature_head._forward_hooks[9] = replacement
    assert helper.callbacks_and_cache(model) != before


@pytest.mark.parametrize("fault", ("CNN_hook", "missing_head", "replaced_head", "pre_hook", "state_hook", "instance_forward", "cache", "request", "active", "stats"))
def test_synthetic_hook_callback_or_cache_changes_are_denied(fault):
    model = SyntheticModel()
    if fault == "CNN_hook": model.image_encoder._forward_hooks[2] = lambda: None
    elif fault == "missing_head": model.curvature_head._forward_hooks.clear()
    elif fault == "replaced_head": model.curvature_head._forward_hooks[9] = lambda: None
    elif fault == "pre_hook": model.image_encoder._forward_pre_hooks[2] = lambda: None
    elif fault == "state_hook": model._state_dict_hooks[2] = lambda: None
    elif fault == "instance_forward": model.image_encoder.forward = lambda: None
    elif fault == "cache": model.image_encoder.entries["synthetic"] = 1
    elif fault == "request": model.image_encoder.request = ("synthetic",)
    elif fault == "active": model._recovery_active = True
    elif fault == "stats": model.cache_stats = SimpleNamespace(calls=1)
    with pytest.raises(RuntimeError): helper.callbacks_and_cache(model)


def test_actual_entry_denies_before_Torch_weights_or_export_without_reviewed_declaration():
    with pytest.raises(RuntimeError): helper.strict_load_original({})
    assert "torch" not in sys.modules and "onnx" not in sys.modules


@pytest.mark.parametrize("entry", ("strict_load_original", "export_two_fixed_batches"))
def test_prepared_only_revision_denies_even_valid_looking_caller_authority(entry, monkeypatch):
    # HH_261005 - Scalar caller claims must not bypass the absent sealed ROOT authority, lease and budget before any asset/source read.
    draft = dict(schema=helper.SCHEMA + ".prospective_cpu_export", ROOT_full_source_review_completed=True,
        explicit_cpu_export_authorized=True, asset_pins=dict(helper.PINS), fixed_batches=[6, 30],
        GPU_or_engine_build_authorized=False)
    monkeypatch.setattr(helper, "admit_declaration", lambda value: pytest.fail("draft admission cannot run in prepared-only V1"))
    monkeypatch.setattr(helper, "file_sha", lambda path: pytest.fail("no asset/source read before unconditional denial"))
    with pytest.raises(RuntimeError, match="PREPARED_ONLY"):
        if entry == "strict_load_original": helper.strict_load_original(draft)
        else: helper.export_two_fixed_batches(object(), draft)
    assert "torch" not in sys.modules and "onnx" not in sys.modules


def test_source_uses_lazy_Torch_original_CNN_forward_and_no_runtime_GPU_control():
    tree = ast.parse(helper.HERE.read_text())
    assert not any(isinstance(node, ast.Import) and any(alias.name == "torch" for alias in node.names) for node in tree.body)
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "SpatialImageEncoderV2" and node.func.attr == "forward" for node in ast.walk(tree))
    forbidden = {"cuda", "half", "bfloat16", "tick", "apply_control", "load_world", "spawn_actor", "buildSerializedNetwork", "enqueueV3"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in forbidden for node in ast.walk(tree))
