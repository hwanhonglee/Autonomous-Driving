"""HH_261005 - Test source/scalar/synthetic-array admission only; no Torch import, real weights, tracing, ONNX export or GPU job."""

from array import array
import ast
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_good6000_cnn_legacy_onnx_v3_20261005 as helper


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
    monkeypatch.setattr(helper, "admit_declaration", lambda value: pytest.fail("draft admission cannot run in unadmitted V3"))
    monkeypatch.setattr(helper, "file_sha", lambda path: pytest.fail("no asset/source read before unconditional denial"))
    with pytest.raises(RuntimeError, match="fixed ROOT-authorized"):
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


# HH_261005 - Additional V3 fixture-only tests prove fixed authority/budget/lifecycle admission and a physical, unchanged original CNN/state guard seam.
def function_nodes(path):
    return {node.name: node for node in ast.parse(path.read_text()).body if isinstance(node, ast.FunctionDef)}


def test_physical_copy_original_CNN_state_and_structure_guards_are_AST_equal():
    original, candidate = function_nodes(helper.V1), function_nodes(helper.HERE)
    for name in ("require", "file_sha", "legacy_dependency_report", "fixed_batch_contract", "tensor_inventory",
            "cnn_inventory", "cnn_structure", "callbacks_and_cache"):
        assert ast.dump(original[name], include_attributes=False) == ast.dump(candidate[name], include_attributes=False)
    left, right = original["strict_load_original"], candidate["strict_load_original"]
    left.body[0].value.func.id = right.body[0].value.func.id
    assert ast.dump(left, include_attributes=False) == ast.dump(right, include_attributes=False)


def test_export_math_AST_equals_frozen_V1_except_exact_input_seals_and_admission():
    original, candidate = function_nodes(helper.V1)["export_two_fixed_batches"], function_nodes(helper.HERE)["export_two_fixed_batches"]
    original.body[0].value.func.id = candidate.body[0].value.func.id
    loop = next(node for node in candidate.body if isinstance(node, ast.For) and isinstance(node.target, ast.Name) and node.target.id == "batch")
    trial = next(node for node in loop.body if isinstance(node, ast.Try))
    keep, removed = [], []
    for node in trial.body:
        drop = isinstance(node, ast.Assign) and ((isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in {"normalized", "input_before"}) or isinstance(node.targets[0], ast.Subscript)
            and isinstance(node.targets[0].slice, ast.Constant) and node.targets[0].slice.value == "normalized_input_unchanged_manifest")
        drop = drop or isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) \
            and node.value.func.id == "require" and any(isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                and child.func.id == "input_manifest" for child in ast.walk(node))
        (removed if drop else keep).append(node)
    assert len(removed) == 4
    zero = next(node.value for node in removed if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "normalized")
    trial.body = keep
    for node in ast.walk(candidate):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "export":
            assert isinstance(node.args[1], ast.Name) and node.args[1].id == "normalized"
            node.args[1] = zero
    assert ast.dump(original, include_attributes=False) == ast.dump(candidate, include_attributes=False)


def fake_plan():
    return dict(schema=helper.SCHEMA + ".ROOT_plan", source_sha256={"fake_source": "a" * 64},
        model_asset_sha256=dict(helper.MODEL_ASSETS), runtime_source_sha256={"fake_runtime": "b" * 64})


def fake_authority(plan):
    return dict(schema=helper.SCHEMA + ".ROOT_execution_authority", explicit_cpu_only_export_approved=True,
        plan_path=str(helper.PLAN), plan_sha256="c" * 64, source_sha256=plan["source_sha256"],
        model_asset_sha256=plan["model_asset_sha256"], runtime_source_sha256=plan["runtime_source_sha256"])


def test_fixed_ROOT_authority_admission_seals_all7_original_assets(monkeypatch):
    plan, observed = fake_plan(), []
    authority = fake_authority(plan)
    monkeypatch.setattr(helper, "contract", lambda: plan)
    monkeypatch.setattr(helper, "read_pinned", lambda path, sha: plan if path == helper.PLAN else authority)
    def seal(path):
        observed.append(str(path))
        return helper.MODEL_ASSETS[str(path)]
    monkeypatch.setattr(helper, "file_sha", seal)
    assert helper.verify("c" * 64, "d" * 64) == plan
    assert set(observed) == set(helper.MODEL_ASSETS) and len(observed) == 7


@pytest.mark.parametrize("fault", ("schema", "boolean", "plan_path", "plan_sha", "source", "asset", "runtime", "plan_contract", "asset_bytes"))
def test_forged_or_changed_fixed_authority_or_source_assets_deny(fault, monkeypatch):
    plan, authority = fake_plan(), fake_authority(fake_plan())
    if fault == "schema": authority["schema"] = "old"
    elif fault == "boolean": authority["explicit_cpu_only_export_approved"] = 1
    elif fault == "plan_path": authority["plan_path"] = "/tmp/fake_plan.json"
    elif fault == "plan_sha": authority["plan_sha256"] = "e" * 64
    elif fault == "source": authority["source_sha256"] = {}
    elif fault == "asset": authority["model_asset_sha256"] = {}
    elif fault == "runtime": authority["runtime_source_sha256"] = {}
    elif fault == "plan_contract": plan = dict(plan, runtime_activation=True)
    monkeypatch.setattr(helper, "contract", fake_plan)
    monkeypatch.setattr(helper, "read_pinned", lambda path, sha: plan if path == helper.PLAN else authority)
    monkeypatch.setattr(helper, "file_sha", lambda path: "0" * 64 if fault == "asset_bytes" else helper.MODEL_ASSETS[str(path)])
    with pytest.raises(RuntimeError): helper.verify("c" * 64, "d" * 64)


@pytest.mark.parametrize("sha", ("z" * 64, "a" * 63, "A" * 64, True))
def test_nonhex_root_authority_sha_denied_without_file_reads(sha, monkeypatch):
    monkeypatch.setattr(helper, "file_sha", lambda path: pytest.fail("invalid SHA must deny before file read"))
    with pytest.raises(RuntimeError): helper.read_pinned(helper.PLAN, sha)


@pytest.mark.parametrize("fault", ("low_startup", "low_closing", "quota", "symlink"))
def test_original_floor_shared_attempt_closing_and_allfile_census_guard(fault, monkeypatch, tmp_path):
    monkeypatch.setattr(helper, "OUTPUT", tmp_path)
    free = helper.FLOOR + helper.SHARED + (helper.ATTEMPT if fault == "low_startup" else helper.CLOSING) - 1
    if fault in {"quota", "symlink"}: free = helper.FLOOR + helper.SHARED + helper.ATTEMPT
    monkeypatch.setattr(helper.shutil, "disk_usage", lambda path: SimpleNamespace(free=free))
    if fault == "quota":
        monkeypatch.setattr(helper, "ATTEMPT", helper.CLOSING + 1)
        (tmp_path / "owned").write_bytes(b"xx")
    elif fault == "symlink": (tmp_path / "bad").symlink_to("/proc")
    with pytest.raises(RuntimeError): helper.budget(startup=fault == "low_startup")


def test_complete_small_output_census_and_unchanged_fixed_budget(monkeypatch, tmp_path):
    monkeypatch.setattr(helper, "OUTPUT", tmp_path)
    monkeypatch.setattr(helper.shutil, "disk_usage", lambda path: SimpleNamespace(free=helper.FLOOR + helper.SHARED + helper.ATTEMPT))
    (tmp_path / "one").write_bytes(b"abc")
    (tmp_path / "nested").mkdir()
    (tmp_path / "nested/two").write_bytes(b"de")
    assert helper.budget(startup=True)["used_bytes"] == 5


@pytest.mark.parametrize("event,args", (("socket.connect", ()), ("subprocess.Popen", ()), ("os.fork", ()),
    ("os.remove", ()), ("os.rename", ()), ("os.symlink", ()), ("open", ("/tmp/foreign_output", "w", 1)),
    ("os.mkdir", ("/tmp/foreign_directory", 0o700))))
def test_selective_Python_network_process_foreign_write_mutation_denied(event, args):
    with pytest.raises(RuntimeError): helper.audit_policy(event, args)


def test_readonly_source_reads_BytesIO_and_explicit_owned_output_are_permitted(monkeypatch, tmp_path):
    monkeypatch.setattr(helper, "OUTPUT", tmp_path)
    helper.audit_policy("open", ("/usr/lib/fake_source.py", "r", 0))
    helper.audit_policy("open", (str(tmp_path / "cnn_fixed6.onnx"), "wb", 577))
    helper.audit_policy("os.mkdir", (str(tmp_path / "export"), 0o700))
    helper.audit_policy("compile", (b"fixture", "fixture.py"))


@pytest.mark.parametrize("fault", ("absent_admission", "deadline", "parent_generation"))
def test_actual_admission_cannot_reuse_missing_expired_or_wrong_owned_parent(fault, monkeypatch):
    active = None if fault == "absent_admission" else dict(deadline=0 if fault == "deadline" else 99999999999, parent={"pid": 10}, lease_fd=4)
    monkeypatch.setattr(helper, "_ACTIVE", active)
    monkeypatch.setattr(helper, "verify_parent_lease", lambda *args: (_ for _ in ()).throw(RuntimeError("owned parent changed")))
    monkeypatch.setattr(helper, "budget", lambda: pytest.fail("invalid lifecycle must deny first"))
    with pytest.raises(RuntimeError): helper._require_actual_admission()


def test_foreign_or_reused_child_generation_is_never_signalled(monkeypatch):
    process = SimpleNamespace(pid=123, poll=lambda: None)
    monkeypatch.setattr(helper, "process_core", lambda pid: dict(pid=pid, start_ticks=999))
    monkeypatch.setattr(helper.os, "killpg", lambda *args: pytest.fail("foreign/reused generation cannot be signalled"))
    with pytest.raises(RuntimeError): helper.stop_owned(process, dict(pid=123, start_ticks=1))


def test_source_actual_entry_has_no_FunctionType_cloning_old_gate_bypass_or_gpu_engine():
    source = helper.HERE.read_text()
    assert "FunctionType" not in source and "_deny_actual_execution" not in source
    assert 'process.wait(timeout=60)' in source and "pass_fds=(lease_fd,)" in source
    assert "torch.set_num_threads(6); torch.set_num_interop_threads(1)" in source
    assert "sys.addaudithook(audit_policy)" in source and "Python_audit_is_OS_sandbox=False" in source
    assert 'all_planned_batches=[dict(batch=batch, status="FAIL_NOT_VERIFIED") for batch in BATCHES]' in source


def test_synthetic_full_input_manifest_detects_mutation_without_Torch(monkeypatch):
    value = SyntheticTensor()
    value.is_contiguous = lambda: True
    fake_torch = SimpleNamespace(float32="float32", isfinite=lambda tensor: SimpleNamespace(all=lambda: finite(tensor)))
    monkeypatch.setattr(helper, "fixed_batch_contract", lambda batch: dict(input_shape=[1], input_bytes=4))
    before = helper.input_manifest(value, fake_torch, 6)
    assert before["raw_bytes"] == 4 and before["shape"] == [1]
    value.values[0] = 2.
    assert helper.input_manifest(value, fake_torch, 6)["raw_sha256"] != before["raw_sha256"]
    value.values[0] = float("nan")
    with pytest.raises(RuntimeError): helper.input_manifest(value, fake_torch, 6)


# HH_261005 - V3-only regression: real inherited stdlib FD/flock under the installed Python audit hook, no ROOT authority, Torch or export.
def test_real_readonly_parent_lock_FD_verification_after_audit_and_append_denial(tmp_path):
    lock, owned = tmp_path / "outside_output.lock", tmp_path / "owned_output"
    lock.touch()
    owned.mkdir()
    parent = helper.process_core(helper.os.getpid())
    code = """
import importlib.util, json, sys
from pathlib import Path
# HH_261005 - Canonical tools is not a top-level package; load the exact explicit bootstrap file as the real workspace entry does.
spec = importlib.util.spec_from_file_location('fixture_workspace_python', Path(sys.argv[5]))
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)
workspace.configure()
from scripts.e2e import prepare_good6000_cnn_legacy_onnx_v3_20261005 as h
h.LOCK, h.OUTPUT = Path(sys.argv[1]), Path(sys.argv[2])
parent, fd = json.loads(sys.argv[3]), int(sys.argv[4])
sys.addaudithook(h.audit_policy)
h.verify_parent_lease(parent, fd)
try:
    h.LOCK.open('a+')
except RuntimeError:
    append_denied = True
else:
    raise AssertionError('foreign append-open unexpectedly permitted')
assert 'torch' not in sys.modules and 'onnx' not in sys.modules
print(json.dumps(dict(readonly_FD_verified=True, append_open_denied=append_denied)))
"""
    with lock.open("a+") as stream:
        helper.fcntl.flock(stream, helper.fcntl.LOCK_EX | helper.fcntl.LOCK_NB)
        result = helper.subprocess.run([sys.executable, "-B", "-c", code, str(lock), str(owned),
            helper.json.dumps(parent), str(stream.fileno()), str(helper.ROOT / "tools/workspace_python.py")], cwd=helper.ROOT, pass_fds=(stream.fileno(),),
            stdout=helper.subprocess.PIPE, stderr=helper.subprocess.PIPE, timeout=3, check=False)
        assert result.returncode == 0, result.stderr.decode()
        assert helper.json.loads(result.stdout) == dict(readonly_FD_verified=True, append_open_denied=True)
        # The child's separate readonly verification must not unlock the parent's original description.
        with lock.open("r") as check:
            with pytest.raises(BlockingIOError):
                helper.fcntl.flock(check, helper.fcntl.LOCK_EX | helper.fcntl.LOCK_NB)


def test_ROOT_plan_authority_exact_bytes_archived_and_closing_seals_retained(monkeypatch, tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    plan_path, authority_path = tmp_path / "ROOT_plan.json", tmp_path / "ROOT_execution_authority.json"
    plan_path.write_bytes(b'{"fixture_plan":true}\n')
    authority_path.write_bytes(b'{"fixture_authority":true}\n')
    monkeypatch.setattr(helper, "OUTPUT", output)
    monkeypatch.setattr(helper, "PLAN", plan_path)
    monkeypatch.setattr(helper, "AUTHORITY", authority_path)
    monkeypatch.setattr(helper, "MODEL_ASSETS", {})
    plan = dict(source_sha256={}, runtime_source_sha256={})
    plan_sha, authority_sha = helper.file_sha(plan_path), helper.file_sha(authority_path)
    helper.archive(plan, plan_sha, authority_sha)
    assert (output / "ROOT_plan.json").read_bytes() == plan_path.read_bytes()
    assert (output / "ROOT_execution_authority.json").read_bytes() == authority_path.read_bytes()
    helper.verify_archive(plan, plan_sha, authority_sha)
    authority_path.write_bytes(b'{"fixture_authority":false}\n')
    with pytest.raises(RuntimeError): helper.verify_archive(plan, plan_sha, authority_sha)


def test_minimal_V3_only_changes_are_readonly_child_lock_bootstrap_closure_and_ROOT_provenance():
    tree = function_nodes(helper.HERE)
    lock_modes = lambda node: [call.args[0].value for call in ast.walk(node) if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute) and call.func.attr == "open" and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "LOCK"]
    assert lock_modes(tree["verify_parent_lease"]) == ["r"]
    assert lock_modes(tree["runtime_lease"]) == ["a+"]
    closure = ast.unparse(tree["closure"])
    assert "tools/workspace_python.py" in closure and "tools/canonical_workspace.py" in closure and "V2_PINS" in closure
    trial = next(node for node in tree["execute"].body[0].body if isinstance(node, ast.Try))
    archive_index = next(index for index, node in enumerate(trial.body) if isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id == "archive")
    popen_index = next(index for index, node in enumerate(trial.body) if isinstance(node, ast.With))
    assert archive_index < popen_index
    archived = helper.archived(dict(source_sha256={}, runtime_source_sha256={}), "a" * 64, "b" * 64)
    assert archived[-2:] == [(helper.PLAN, helper.OUTPUT / "ROOT_plan.json", "a" * 64),
        (helper.AUTHORITY, helper.OUTPUT / "ROOT_execution_authority.json", "b" * 64)]
