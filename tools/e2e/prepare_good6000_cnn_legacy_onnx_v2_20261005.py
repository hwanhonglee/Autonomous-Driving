"""HH_261005 - Prepare a lazy, CPU-only legacy ONNX feasibility export of the exact GOOD6000 CNN; never activate a runtime or build a GPU engine.

No Torch import, model loading, tracing or file output occurs on module import.
Actual CPU-only execution requires fixed sealed ROOT plan/authority, an inherited
exclusive runtime lease, bounded child lifetime and resource admission.
This proves neither TensorRT support nor cached/full-output numerical parity.
"""

import argparse
import ast
from contextlib import contextmanager
import fcntl
import os
import re
import resource
import shutil
import signal
import stat
import subprocess
import sys
import time
from io import BytesIO
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
SCHEMA = "portable_e2e.good6000_cnn_legacy_onnx_feasibility.20261005.v2"
RUN = ROOT / "storage/experiments/training/2026-10-02/temporal_route_joint_v1"
WEIGHTS, METADATA = RUN / "exports/step_006000/model_state.npz", RUN / "exports/step_006000/metadata.json"
PARENT = ROOT / "storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000"
BACKBONE = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000"
PINS = dict(weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")
BATCHES = (6, 30)
LEGACY_ONNX = Path("/usr/lib/python3/dist-packages/torch/onnx")
CNN_NAMES = tuple(sorted(["features.%d.weight" % index for index in (0, 3, 6, 9)]
    + ["features.%d.%s" % (index, suffix) for index in (1, 4, 7, 10) for suffix in ("weight", "bias")]
    + ["projection.weight", "projection.bias"]))


def require(value, message):
    if not value:
        raise RuntimeError(message)


def file_sha(path):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=True)
        and all(not item.is_symlink() for item in (path, *path.parents)) and path.is_file(), "canonical regular sealed file required")
    before = path.stat()
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    after = path.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns), "sealed file generation changed")
    return value


def legacy_dependency_report(source_texts):
    # HH_261005 - Inspect Python source, not imports: this installed legacy path checks protobuf through torch._C, not Python onnx.
    require({"__init__.py", "utils.py", "symbolic_opset9.py"} <= set(source_texts), "complete key legacy exporter source texts required")
    direct = []
    for name, text in source_texts.items():
        for node in ast.walk(ast.parse(text)):
            names = [alias.name for alias in node.names] if isinstance(node, ast.Import) else (
                [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
            direct.extend(name for imported in names if imported == "onnx" or imported.startswith("onnx."))
    require(not direct, "Python onnx dependency found; stop and seek authority instead of installing")
    tree = ast.parse(source_texts["utils.py"])
    require(any(isinstance(node, ast.ImportFrom) and node.module == "torch._C"
        and any(alias.name == "_check_onnx_proto" for alias in node.names) for node in ast.walk(tree)),
        "legacy built-in C++ protobuf checker source not established")
    require("graph._export_onnx(" in source_texts["utils.py"] and "_check_onnx_proto(proto)" in source_texts["utils.py"],
        "reviewed legacy graph export/check path differs")
    return dict(Python_onnx_import_required_in_reviewed_sources=False, builtin_checker="torch._C._check_onnx_proto",
        actual_export_feasibility_verified=False)


def fixed_batch_contract(batch):
    require(type(batch) is int and batch in BATCHES, "only exact warm6/cold30 CNN batches are admitted")
    return dict(batch=batch, input_name="normalized_images", input_dtype="float32", input_shape=[batch, 3, 180, 320],
        input_bytes=batch * 3 * 180 * 320 * 4, output_name="cnn_features", output_dtype="float32",
        output_shape=[batch, 60, 128], output_bytes=batch * 60 * 128 * 4, opset=12, dynamic_axes=None,
        export_params=True, do_constant_folding=False, keep_initializers_as_inputs=False,
        enable_onnx_checker=True, use_external_data_format=False, runtime_activation=False)


def admit_declaration(declaration):
    # HH_261005 - The fixed child admission must own this exact execution declaration, not merely matching caller claims.
    require(_ACTIVE is not None and declaration is _ACTIVE["declaration"], "exact admitted execution declaration required")
    require(declaration.get("schema") == SCHEMA + ".prospective_cpu_export"
        and declaration.get("ROOT_full_source_review_completed") is True
        and declaration.get("explicit_cpu_export_authorized") is True and declaration.get("asset_pins") == PINS
        and declaration.get("fixed_batches") == [6, 30] and declaration.get("GPU_or_engine_build_authorized") is False,
        "a separate exact ROOT-reviewed CPU export declaration is required before Torch or weights")
    output = Path(declaration["output"])
    require(output.is_absolute() and ".." not in output.parts and output == output.resolve()
        and output.is_relative_to(ROOT / "storage/experiments/diagnostics/2026-10-05") and not output.exists(),
        "fresh canonical export output required; never retry or overwrite")
    require(declaration["source_sha256"].get(str(HERE.relative_to(ROOT))) == file_sha(HERE), "export helper source seal differs")
    return output


def _require_actual_admission():
    # HH_261005 - Only the sealed, owned child path can establish actual admission; draft caller booleans are insufficient.
    require(_ACTIVE is not None, "actual export requires the fixed ROOT-authorized owned child path")
    require(time.monotonic() <= _ACTIVE["deadline"], "fixed60s CPU export deadline exceeded")
    verify_parent_lease(_ACTIVE["parent"], _ACTIVE["lease_fd"])
    verify_parent_command(_ACTIVE["parent"], _ACTIVE["plan_sha"], _ACTIVE["authority_sha"])
    budget()


def tensor_inventory(state, float32, finite):
    # HH_261005 - Record all101 raw FP32 values, shapes and storage/version identities; do not fingerprint only the exported CNN subset.
    rows = {}
    for name, value in sorted(state.items()):
        require(value.dtype == float32 and value.device.type == "cpu" and not value.requires_grad
            and finite(value), "all original state must be finite readonly CPU float32")
        raw = memoryview(value.detach().contiguous().numpy()).cast("B")
        rows[name] = dict(shape=list(value.shape), dtype="float32", raw_sha256=hashlib.sha256(raw).hexdigest(),
            raw_bytes=len(raw), storage_ptr=value.data_ptr(), version=value._version, stride=list(value.stride()))
    require(len(rows) == 101, "complete original101 state tensors required")
    return rows


def cnn_inventory(full):
    values = {name.removeprefix("image_encoder."): value for name, value in full.items() if name.startswith("image_encoder.")}
    require(tuple(sorted(values)) == CNN_NAMES, "exact original14 CNN tensor names required")
    return values


def cnn_structure(encoder, nn):
    # HH_261005 - State names alone do not seal stride, normalization epsilon or pooling; reject any after-loader structural mutation.
    pair = lambda value: (value, value) if type(value) is int else tuple(value)
    require(type(encoder.features) is nn.Sequential and len(encoder.features) == 12, "exact original12 CNN feature modules required")
    rows, previous = [], 3
    for block, width in enumerate((32, 64, 128, 192)):
        conv, norm, activation = [encoder.features[block * 3 + index] for index in range(3)]
        require(type(conv) is nn.Conv2d and (conv.in_channels, conv.out_channels) == (previous, width)
            and pair(conv.kernel_size) == (3, 3) and pair(conv.stride) == (2, 2) and pair(conv.padding) == (1, 1)
            and pair(conv.dilation) == (1, 1) and conv.groups == 1 and conv.bias is None and conv.padding_mode == "zeros",
            "original CNN convolution attributes differ")
        require(type(norm) is nn.GroupNorm and norm.num_groups == 8 and norm.num_channels == width
            and norm.eps == 1e-5 and norm.affine is True, "original CNN GroupNorm attributes differ")
        require(type(activation) is nn.SiLU and activation.inplace is False, "original CNN SiLU attributes differ")
        rows.append(dict(input_channels=previous, output_channels=width, kernel=[3, 3], stride=[2, 2], padding=[1, 1],
            groups=1, norm_groups=8, norm_eps=1e-5, activation="SiLU", inplace=False))
        previous = width
    pool, projection = encoder.pool, encoder.projection
    require(type(pool) is nn.AvgPool2d and pair(pool.kernel_size) == (2, 2) and pair(pool.stride) == (2, 2)
        and pair(pool.padding) == (0, 0) and pool.ceil_mode is False and pool.count_include_pad is True
        and pool.divisor_override is None, "original CNN fixed pooling attributes differ")
    require(type(projection) is nn.Conv2d and (projection.in_channels, projection.out_channels) == (192, 128)
        and pair(projection.kernel_size) == (1, 1) and pair(projection.stride) == (1, 1) and pair(projection.padding) == (0, 0)
        and pair(projection.dilation) == (1, 1) and projection.groups == 1 and projection.bias is not None
        and projection.padding_mode == "zeros", "original CNN projection attributes differ")
    return dict(features=rows, pool="original AvgPool2d2/stride2", projection=[192, 128, 1, 1], output_grid=[6, 10], output_tokens=[60, 128])


def callbacks_and_cache(model):
    require(model._recovery_active is False and model._recovery_features is None
        and not model.image_encoder.entries and model.image_encoder.request is None
        and model._cache_scope is None and model._cache_state is None and model._last_anchor_ns is None and model.cache_stats is None,
        "export only an idle fresh original model; never disturb a causal cache")
    callbacks = []
    for name, module in model.named_modules():
        require(not any(method in module.__dict__ for method in ("forward", "state_dict", "load_state_dict", "named_modules", "modules",
            "named_buffers", "named_parameters")), "instance model method replacement forbidden")
        for field in ("_forward_pre_hooks", "_backward_hooks", "_state_dict_hooks", "_load_state_dict_pre_hooks"):
            require(not getattr(module, field, {}), "extra state/forward/backward callbacks forbidden")
        hooks = module._forward_hooks
        if module is model.curvature_head:
            require(set(hooks) == {model._recovery_hook_handle.id}
                and hooks[model._recovery_hook_handle.id] is model._recovery_hook_function,
                "original permanent route-recovery head hook must remain genuine")
        else:
            require(not hooks, "CNN or unrelated forward callback forbidden")
        callbacks.append((name, id(module), id(type(module)), id(type(module).forward), tuple((key, id(value)) for key, value in hooks.items())))
    return dict(callbacks=callbacks, entries=dict(model.image_encoder.entries), request=model.image_encoder.request,
        encoder_work=model.image_encoder.work, cache_stats=None,
        cache_scope=model._cache_scope, cache_state=model._cache_state, anchor=model._last_anchor_ns,
        pending_reset=model._pending_reset_reason)


def strict_load_original(declaration):
    # HH_261005 - This function is intentionally lazy and must not be called during source preparation or synthetic scalar tests.
    _require_actual_admission()
    admit_declaration(declaration)
    import torch
    from portable_e2e import temporal_route_joint_export_research_20261002 as loader
    from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
    require(loader.load_route_joint_research_npz.__globals__["RouteRecoveryTemporalModelV3"] is RouteRecoveryTemporalModelV3,
        "genuine original strict101 constructor global was replaced")
    binding = dict(schema=loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=SCHEMA,
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, fixed_endpoint_updates=6000, **PINS)
    model = loader.load_route_joint_research_npz(WEIGHTS, METADATA, weights_sha256=PINS["weights_sha256"],
        metadata_sha256=PINS["metadata_sha256"], parent_weights=PARENT / "model_state.npz", parent_metadata=PARENT / "metadata.json",
        backbone_parent_weights=BACKBONE / "model_state.npz", backbone_parent_metadata=BACKBONE / "metadata.json",
        research_owner_binding=binding, source_root=ROOT)
    require(type(model) is RouteRecoveryTemporalModelV3 and model._research_recovery_simulator_control_authorized is False,
        "only the exact original numeric readonly101 model is admitted")
    require(loader.load_route_joint_research_npz.__globals__["RouteRecoveryTemporalModelV3"] is RouteRecoveryTemporalModelV3,
        "strict loading changed the original constructor global")
    return model


def export_two_fixed_batches(model, declaration):
    _require_actual_admission()
    output = admit_declaration(declaration)
    source_pins = declaration["source_sha256"]
    require(all(not Path(name).is_absolute() and ".." not in Path(name).parts
        and file_sha(ROOT / name) == sha for name, sha in source_pins.items()), "prospective source closure differs")
    legacy_pins = declaration["legacy_exporter_source_sha256"]
    require(all(Path(name).parent == LEGACY_ONNX and Path(name).suffix == ".py"
        and file_sha(name) == sha for name, sha in legacy_pins.items()), "exact installed legacy exporter source bytes required")
    legacy = {Path(name).name: Path(name).read_text() for name in legacy_pins}
    dependency = legacy_dependency_report(legacy)
    import torch
    import torch.onnx
    from portable_e2e.temporal_model_v2 import SpatialImageEncoderV2
    from portable_e2e.temporal_cached_encoder_v2 import _CachedSpatialImageEncoderV2
    from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
    require(Path(torch.__file__) == LEGACY_ONNX.parent / "__init__.py" and Path(torch.onnx.__file__) == LEGACY_ONNX / "__init__.py"
        and torch.__version__ == "1.8.0a0" and torch.version.cuda is None
        and torch.get_num_threads() == 6 and torch.get_num_interop_threads() == 1
        and not torch.is_autocast_enabled() and not getattr(torch, "is_autocast_cpu_enabled", lambda: False)(),
        "reviewed local CPU Torch1.8/FP32/CPU6 interop1 only; no implicit backend or precision intervention")
    require(type(model) is RouteRecoveryTemporalModelV3 and type(model.image_encoder) is _CachedSpatialImageEncoderV2
        and all(not module.training for module in model.modules()) and all(not value.requires_grad for value in model.parameters())
        and {name for name, value in model.named_buffers()} == {"image_grid_xy"}
        and model._research_temporal_export_npz_sha256 == PINS["weights_sha256"]
        and model._research_temporal_metadata_sha256 == PINS["metadata_sha256"]
        and model._research_recovery_simulator_control_authorized is False, "exact unchanged original GOOD6000 CPU model required")
    finite = lambda value: bool(torch.isfinite(value).all().item())
    before_callbacks = callbacks_and_cache(model)
    require(not any(isinstance(value, torch.Tensor) for module in model.modules() for value in module.__dict__.values()),
        "unregistered hidden tensor state forbidden")
    structure = cnn_structure(model.image_encoder, torch.nn)
    before = tensor_inventory(model.state_dict(), torch.float32, finite)
    cnn = cnn_inventory(before)
    metadata = model._research_temporal_export_metadata
    require(set(before) == set(metadata["state_tensor_shapes"]) == set(metadata["state_tensor_dtypes"])
        and all(value["shape"] == metadata["state_tensor_shapes"][name] and metadata["state_tensor_dtypes"][name] == "float32"
            for name, value in before.items()), "loaded101 shape/dtype declaration differs")
    require(not torch.nn.modules.module._global_forward_hooks and not torch.nn.modules.module._global_forward_pre_hooks
        and not torch.nn.modules.module._global_backward_hooks, "global module callbacks forbidden")
    class ExactCNN(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.features, self.pool, self.projection = model.image_encoder.features, model.image_encoder.pool, model.image_encoder.projection
        def forward(self, normalized_images):
            # HH_261005 - Call the exact frozen base CNN math, not cached forward or a rewritten normalization/convolution implementation.
            return SpatialImageEncoderV2.forward(self, normalized_images)
    wrapper = ExactCNN().eval()
    require(tuple(sorted(wrapper.state_dict())) == CNN_NAMES
        and all(wrapper.state_dict()[name].data_ptr() == model.state_dict()["image_encoder." + name].data_ptr() for name in CNN_NAMES),
        "wrapper must share the exact original CNN state, not copied or substituted weights")
    output.mkdir(parents=True, exist_ok=False)
    rows, failures, guard_failed = [], [], False
    for batch in BATCHES:
        row = dict(contract=fixed_batch_contract(batch), status="FAIL", error=None)
        rows.append(row)
        if guard_failed:
            row["error"] = "Not executed after original state/callback/cache guard failure; planned denominator retained"
            failures.append(row["error"])
            continue
        try:
            require(callbacks_and_cache(model) == before_callbacks and cnn_structure(model.image_encoder, torch.nn) == structure,
                "callbacks/cache/CNN structure changed before trace")
            buffer = BytesIO()
            normalized = torch.zeros((batch, 3, 180, 320), dtype=torch.float32, device="cpu")
            input_before = input_manifest(normalized, torch, batch)
            with torch.no_grad():
                torch.onnx.export(wrapper, normalized, buffer,
                    input_names=["normalized_images"], output_names=["cnn_features"], opset_version=12,
                    export_params=True, do_constant_folding=False, dynamic_axes=None, keep_initializers_as_inputs=False,
                    training=torch.onnx.TrainingMode.EVAL, operator_export_type=torch.onnx.OperatorExportTypes.ONNX,
                    enable_onnx_checker=True, use_external_data_format=False)
            require(input_manifest(normalized, torch, batch) == input_before, "export changed the exact bounded normalized FP32 input")
            row["normalized_input_unchanged_manifest"] = input_before
            payload = buffer.getvalue()
            require(payload and len(payload) <= 32 * 1024**2, "bounded standalone checked CNN protobuf required")
            with (output / ("cnn_fixed%d.onnx" % batch)).open("xb") as stream:
                stream.write(payload)
            row.update(status="EXPORTED_BUILTIN_CHECKED_ONLY", bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())
        except Exception as error:
            row["error"] = type(error).__name__ + ": " + str(error)
            failures.append(row["error"])
        finally:
            try:
                require(callbacks_and_cache(model) == before_callbacks and cnn_structure(model.image_encoder, torch.nn) == structure
                    and not any(isinstance(value, torch.Tensor) for module in model.modules() for value in module.__dict__.values())
                    and tensor_inventory(model.state_dict(), torch.float32, finite) == before,
                    "export changed original101 state, storage, callbacks or cache; do not continue")
                row["original101_state_callbacks_cache_unchanged"] = True
            except Exception as error:
                guard_failed = True
                row.update(status="FAIL", original101_state_callbacks_cache_unchanged=False,
                    post_guard_error=type(error).__name__ + ": " + str(error))
                failures.append(row["post_guard_error"])
    try:
        sources_unchanged = all(file_sha(ROOT / name) == sha for name, sha in source_pins.items()) and all(
            file_sha(name) == sha for name, sha in legacy_pins.items())
    except Exception as error:
        sources_unchanged = False
        failures.append("final source verification: " + type(error).__name__ + ": " + str(error))
    if not sources_unchanged:
        failures.append("source closure changed during export")
    result = dict(schema=SCHEMA, status="FAIL" if failures else "EXPORTED_FEASIBILITY_ONLY", dependency=dependency,
        fixed_asset_pins=PINS, full101_state=before, exact14_CNN_state=cnn, CNN_structure=structure, all_fixed_batches_including_failures=rows,
        actual_original_camera_or_full_cached_NN_replay=False, all_original101_state_callbacks_cache_unchanged=not guard_failed,
        prospective_sources_unchanged=sources_unchanged, model_or_controller_intervention_activated=False,
        original_cold_history_input_bytes=20736000, engine_or_GPU_tested=False, numerical_cached_parity_verified=False,
        ONNX_initializer_and_runtime_ABI_independent_parser_verified=False, runtime_activation=False)
    with (output / "feasibility.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    return result


# HH_261005 - Fresh physical V2 admission/owner code; all CNN math and original101/14 structure/state/callback/cache guards above remain V1-equivalent.
TEST = ROOT / "learning/tests/test_good6000_cnn_legacy_onnx_v2_20261005.py"
V1 = ROOT / "tools/e2e/prepare_good6000_cnn_legacy_onnx_v1_20261005.py"
V1_TEST = ROOT / "learning/tests/test_good6000_cnn_legacy_onnx_v1_20261005.py"
V1_PINS = {str(V1.relative_to(ROOT)): "4dd34b6c9498e86831fc6412db4fd840ec5fa818ae4c44725ed8b994c29a588d",
    str(V1_TEST.relative_to(ROOT)): "300697bb4d0a34c7c22bbd57773566146a3d48130b7e4809c4de3096cc357790"}
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/good6000_cnn_legacy_onnx_v2"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_legacy_onnx_v2/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
FLOOR, SHARED, ATTEMPT, CLOSING = 5 * 1024**3, 64 * 1024**2, 768 * 1024**2, 16 * 1024**2
MODULE = "scripts.e2e.prepare_good6000_cnn_legacy_onnx_v2_20261005"
PREFIX = ["/usr/bin/python3.10", "-B", str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--"]
MODEL_ASSETS = {str(WEIGHTS): PINS["weights_sha256"], str(METADATA): PINS["metadata_sha256"],
    str(PARENT / "model_state.npz"): "0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9",
    str(PARENT / "metadata.json"): "4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d",
    str(BACKBONE / "model_state.npz"): "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735",
    str(BACKBONE / "metadata.json"): "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec",
    str(RUN / "declaration.json"): PINS["declaration_sha256"]}
_ACTIVE = None


def input_manifest(value, torch, batch):
    # HH_261005 - Seal the actual fixed input bytes before/after tracing without substituting a composed or reduced image digest.
    expected = fixed_batch_contract(batch)
    require(value.dtype == torch.float32 and value.device.type == "cpu" and not value.requires_grad
        and list(value.shape) == expected["input_shape"] and value.is_contiguous() and bool(torch.isfinite(value).all()),
        "exact finite readonly contiguous FP32 normalized input required")
    raw = memoryview(value.detach().numpy()).cast("B")
    require(len(raw) == expected["input_bytes"], "complete fixed normalized input bytes required")
    return dict(shape=list(value.shape), dtype="float32", raw_sha256=hashlib.sha256(raw).hexdigest(),
        raw_bytes=len(raw), storage_ptr=value.data_ptr(), version=value._version, stride=list(value.stride()))


def read_pinned(path, sha):
    require(type(sha) is str and re.fullmatch(r"[0-9a-f]{64}", sha), "explicit hexadecimal authority SHA required")
    require(Path(path).stat().st_size <= 4 * 1024**2 and file_sha(path) == sha, "bounded exact sealed JSON required")
    raw = Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == sha, "JSON changed during read")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key denied")
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=unique, parse_constant=lambda value: require(False, "nonfinite JSON denied"))


def closure():
    # HH_261005 - Merge the three original sealed metadata closures before importing any model; preserve all training-source pins.
    sources = dict(V1_PINS)
    for path in (METADATA, PARENT / "metadata.json", BACKBONE / "metadata.json"):
        for name, sha in read_pinned(path, MODEL_ASSETS[str(path)])["source_sha256"].items():
            require(not Path(name).is_absolute() and ".." not in Path(name).parts and (name not in sources or sources[name] == sha),
                "original metadata source closure conflict/escape")
            sources[name] = sha
    for path in (HERE, TEST, ROOT / "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py",
            ROOT / "tools/e2e/prepare_ctrack_isolated_probe_20261001.py"):
        sources[str(path.relative_to(ROOT))] = file_sha(path)
    require(all(file_sha(ROOT / name) == sha for name, sha in sources.items()), "complete original source closure differs")
    torch_root = LEGACY_ONNX.parent
    # Installed framework Python families used by import/tracing/CNN are sealed separately from the original model metadata closure.
    runtime_sources = {path for folder in (torch_root, LEGACY_ONNX, torch_root / "jit", torch_root / "nn",
        torch_root / "nn/modules", torch_root / "autograd") for path in folder.glob("*.py")}
    return sources, {str(path): file_sha(path) for path in sorted(runtime_sources)}


def contract():
    sources, runtime = closure()
    return dict(schema=SCHEMA + ".ROOT_plan", output=str(OUTPUT), source_sha256=sources,
        model_asset_sha256=dict(MODEL_ASSETS), runtime_source_sha256=runtime,
        interpreter_path=PREFIX[0], interpreter_sha256=file_sha(PREFIX[0]), runtime_lock=str(LOCK),
        fixed_batches=[fixed_batch_contract(batch) for batch in BATCHES], CPU_threads=6, interop_threads=1,
        free_floor_bytes=FLOOR, shared_reserve_bytes=SHARED, attempt_quota_bytes=ATTEMPT, closing_reserve_bytes=CLOSING,
        child_wall_timeout_seconds=60, ONNX_file_limit_bytes=32 * 1024**2, log_limit_bytes=CLOSING,
        GPU_or_engine_or_CARLA_or_training_or_install_authorized=False, runtime_activation=False,
        actual_original_BGRA_or_full_cached_NN_replay=False, owner_prefix=PREFIX,
        Python_audit_is_OS_sandbox=False, native_syscalls_traced=False)


def verify(plan_sha, authority_sha):
    plan, authority = read_pinned(PLAN, plan_sha), read_pinned(AUTHORITY, authority_sha)
    equal = lambda left, right: json.dumps(left, sort_keys=True, allow_nan=False) == json.dumps(right, sort_keys=True, allow_nan=False)
    require(equal(plan, contract()), "fixed prospective plan/source/runtime contract differs")
    require(equal(authority, dict(schema=SCHEMA + ".ROOT_execution_authority", explicit_cpu_only_export_approved=True,
        plan_path=str(PLAN), plan_sha256=plan_sha, source_sha256=plan["source_sha256"],
        model_asset_sha256=plan["model_asset_sha256"], runtime_source_sha256=plan["runtime_source_sha256"])),
        "exact fresh ROOT authority required; caller booleans cannot authorize execution")
    require(all(file_sha(path) == sha for path, sha in MODEL_ASSETS.items()), "all original7 model assets must match")
    return plan


def archived(plan):
    rows = [(ROOT / name, OUTPUT / "archive" / name, sha) for name, sha in plan["source_sha256"].items()]
    rows += [(Path(name), OUTPUT / "archive" / Path(name).relative_to(ROOT), sha) for name, sha in MODEL_ASSETS.items()]
    rows += [(Path(name), OUTPUT / "archive_external" / Path(name).relative_to("/"), sha)
        for name, sha in plan["runtime_source_sha256"].items()]
    return rows


def archive(plan):
    for original, target, sha in archived(plan):
        require(not target.exists(), "fresh archived source/asset required")
        target.parent.mkdir(parents=True, exist_ok=True)
        with original.open("rb") as src, target.open("xb") as dest: shutil.copyfileobj(src, dest, 1024**2)
        require(file_sha(original) == sha == file_sha(target), "actual/archived source or model asset differs")


def verify_archive(plan):
    require(all(file_sha(original) == sha == file_sha(target) for original, target, sha in archived(plan)),
        "all actual/archived original sources/runtime/model assets must remain unchanged")


def budget(startup=False):
    require(shutil.disk_usage(ROOT).free >= FLOOR + SHARED + (ATTEMPT if startup else CLOSING), "original free floor/shared/reserve required")
    total = 0
    if OUTPUT.exists():
        require(OUTPUT == OUTPUT.resolve(strict=True) and not OUTPUT.is_symlink(), "canonical owned output required")
        for parent, directories, files in os.walk(OUTPUT, followlinks=False):
            for name in directories + files:
                path = Path(parent) / name
                value = path.lstat()
                require(not stat.S_ISLNK(value.st_mode) and (stat.S_ISREG(value.st_mode) or stat.S_ISDIR(value.st_mode)), "special output object denied")
                if stat.S_ISREG(value.st_mode): total += value.st_size
    require(total <= ATTEMPT - CLOSING, "complete every-file attempt census must retain closing reserve")
    return dict(used_bytes=total, free_floor_bytes=FLOOR, shared_reserve_bytes=SHARED, closing_reserve_bytes=CLOSING)


def process_core(pid):
    try: fields = Path("/proc/" + str(pid) + "/stat").read_text().rsplit(")", 1)[1].split()
    except FileNotFoundError: return None
    return dict(pid=pid, start_ticks=int(fields[19]), ppid=int(fields[1]), pgid=int(fields[2]), sid=int(fields[3]))


@contextmanager
def runtime_lease():
    # HH_261005 - Use the existing exact shared runtime lock, not a new independent lock or advisory caller label.
    require(LOCK == LOCK.resolve(strict=True) and LOCK.is_file() and not LOCK.is_symlink(), "existing canonical runtime lock required")
    with LOCK.open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try: yield stream.fileno()
        finally: fcntl.flock(stream, fcntl.LOCK_UN)


def verify_parent_lease(parent, fd):
    require(process_core(parent["pid"]) == parent and parent["pid"] == os.getppid(), "exact live owned parent generation required")
    value, target = os.fstat(fd), LOCK.stat()
    require((value.st_dev, value.st_ino) == (target.st_dev, target.st_ino) and stat.S_ISREG(value.st_mode)
        and os.path.samefile("/proc/" + str(parent["pid"]) + "/fd/" + str(fd), LOCK), "inherited exact parent runtime lease FD required")
    # A separately opened description must conflict, proving the shared exclusive lease remains held.
    with LOCK.open("a+") as check:
        try: fcntl.flock(check, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: pass
        else:
            fcntl.flock(check, fcntl.LOCK_UN)
            raise RuntimeError("exclusive runtime lease not held")


def verify_parent_command(parent, plan_sha, authority_sha):
    argv = [part.decode() for part in Path("/proc/" + str(parent["pid"]) + "/cmdline").read_bytes().split(b"\0") if part]
    require(argv == PREFIX + ["--mode", "execute", "--plan-sha256", plan_sha, "--authority-sha256", authority_sha],
        "exact admitted parent execution CLI required")


def environment():
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="6",
        MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS="1", XDG_CACHE_HOME=str(OUTPUT / "UserCache"),
        CUDA_CACHE_PATH=str(OUTPUT / "UserCache/CUDA"))
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"): env.pop(key, None)
    return env


def audit_policy(event, args):
    # HH_261005 - This is a selective Python hook, not an OS sandbox or proof about unobserved native syscalls.
    if event == "open" and (args[2] or 0) & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND):
        require(isinstance(args[0], (str, bytes, os.PathLike)) and Path(os.fsdecode(args[0])).is_absolute(),
            "only explicit absolute owned output writes permitted")
        path = Path(os.fsdecode(args[0]))
        require(path.is_relative_to(OUTPUT) and path == path.resolve()
            and all(not item.is_symlink() for item in (path, *path.parents)), "only canonical fresh owned output writes permitted")
    elif event.startswith("socket.") or event in {"subprocess.Popen", "os.system", "os.exec", "os.fork", "os.posix_spawn",
            "os.remove", "os.rename", "os.rmdir", "os.chmod", "os.chown", "os.utime", "os.link", "os.symlink", "os.truncate"}:
        raise RuntimeError("Python network/process/destructive mutation denied: " + event)
    elif event == "os.mkdir":
        path = Path(os.fsdecode(args[0]))
        require(path.is_absolute() and path.is_relative_to(OUTPUT) and path == path.resolve()
            and all(not item.is_symlink() for item in (path, *path.parents)), "only owned output directories permitted")


def limits():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    resource.setrlimit(resource.RLIMIT_FSIZE, (32 * 1024**2, 32 * 1024**2))


def child(plan_sha, authority_sha, parent, lease_fd):
    global _ACTIVE
    require(_ACTIVE is None and sys.version_info[:2] == (3, 10) and sys.executable == PREFIX[0]
        and os.environ.get("CUDA_VISIBLE_DEVICES") == "" and "torch" not in sys.modules, "fresh exact CPU-only child required")
    deadline = time.monotonic() + 60
    verify_parent_command(parent, plan_sha, authority_sha)
    verify_parent_lease(parent, lease_fd)
    plan = verify(plan_sha, authority_sha); verify_archive(plan); budget()
    require(resource.getrlimit(resource.RLIMIT_FSIZE) == (32 * 1024**2, 32 * 1024**2), "exact bounded ONNX file limit required")
    declaration = dict(schema=SCHEMA + ".prospective_cpu_export", ROOT_full_source_review_completed=True,
        explicit_cpu_export_authorized=True, asset_pins=PINS, fixed_batches=[6, 30], GPU_or_engine_build_authorized=False,
        output=str(OUTPUT / "export"), source_sha256=plan["source_sha256"],
        legacy_exporter_source_sha256={name: sha for name, sha in plan["runtime_source_sha256"].items() if Path(name).parent == LEGACY_ONNX})
    _ACTIVE = dict(parent=parent, lease_fd=lease_fd, deadline=deadline, declaration=declaration,
        plan_sha=plan_sha, authority_sha=authority_sha)
    try:
        sys.addaudithook(audit_policy)
        import torch
        require(Path(torch.__file__) == LEGACY_ONNX.parent / "__init__.py" and torch.__version__ == "1.8.0a0"
            and torch.version.cuda is None, "exact existing CPU-only Torch required before any weights")
        torch.set_num_threads(6); torch.set_num_interop_threads(1); torch.use_deterministic_algorithms(True)
        model = strict_load_original(declaration)
        result = export_two_fixed_batches(model, declaration)
        _require_actual_admission(); verify(plan_sha, authority_sha); verify_archive(plan)
        return 0 if result["status"] == "EXPORTED_FEASIBILITY_ONLY" else 2
    finally: _ACTIVE = None


def stop_owned(process, core):
    if process is not None and process.poll() is None:
        require(core and process_core(process.pid) == core and core["ppid"] == os.getpid()
            and core["pgid"] == core["sid"] == process.pid, "exact owned export child generation required before signal")
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2)


def execute(plan_sha, authority_sha):
    with runtime_lease() as lease_fd:
        plan = verify(plan_sha, authority_sha); budget(startup=True)
        require(not OUTPUT.exists() and OUTPUT.parent.is_dir(), "fresh output and existing canonical parent required")
        OUTPUT.mkdir(exist_ok=False)
        result = dict(schema=SCHEMA + ".result", status="FAIL_RETAINED", fixed_batches=[6, 30],
            all_planned_batches=[dict(batch=batch, status="FAIL_NOT_VERIFIED") for batch in BATCHES],
            GPU_or_engine_or_controller_activated=False, numeric_full_cached_parity_verified=False)
        process, core, argv = None, None, None
        started = time.monotonic()
        try:
            archive(plan); budget()
            parent = process_core(os.getpid())
            require(parent is not None, "owned parent generation required")
            argv = PREFIX + ["--mode", "child", "--plan-sha256", plan_sha, "--authority-sha256", authority_sha,
                "--parent-record", json.dumps(parent, separators=(",", ":")), "--lease-fd", str(lease_fd)]
            with (OUTPUT / "stdout.log").open("xb") as out, (OUTPUT / "stderr.log").open("xb") as err:
                process = subprocess.Popen(argv, cwd=ROOT, env=environment(), stdout=out, stderr=err,
                    start_new_session=True, pass_fds=(lease_fd,), preexec_fn=limits)
                core = process_core(process.pid)
                require(core and core["ppid"] == os.getpid() and core["pgid"] == core["sid"] == process.pid, "owned export child core required")
                result["exit_code"] = process.wait(timeout=60)
            report_path = OUTPUT / "export/feasibility.json"
            if report_path.exists():
                require(report_path.stat().st_size <= 4 * 1024**2, "bounded complete export receipt required")
                with report_path.open() as stream: report = json.load(stream)
                require(report["schema"] == SCHEMA and [row["contract"]["batch"]
                    for row in report["all_fixed_batches_including_failures"]] == [6, 30], "complete two fixed export rows required")
                result["all_planned_batches"] = report["all_fixed_batches_including_failures"]
            require(result["exit_code"] == 0 and report_path.exists() and report["status"] == "EXPORTED_FEASIBILITY_ONLY",
                "CPU export child failed; all logs/rows retained")
            result["status"] = "EXPORTED_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
        except BaseException as error: result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            try:
                stop_owned(process, core)
                from scripts.e2e import run_owned_ctrack_isolated_probe_20261001 as guards
                require(core is None or not any(row["pgid"] == core["pgid"] for row in guards.all_processes()),
                    "owned export process-group survivor retained")
            except BaseException as error: result.update(status="FAIL_RETAINED", cleanup_error=str(error))
            result.update(actual_child_argv=argv, child_creation_core=core, elapsed_seconds=time.monotonic() - started)
            try:
                verify(plan_sha, authority_sha); verify_archive(plan)
                result["closing_budget"] = budget()
                require(all((OUTPUT / name).stat().st_size <= CLOSING for name in ("stdout.log", "stderr.log")), "bounded logs required")
            except BaseException as error: result.update(status="FAIL_RETAINED", final_guard_error=str(error))
            with (OUTPUT / "result.json").open("x") as stream: json.dump(result, stream, indent=2, allow_nan=False)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("execute", "child"), required=True)
    parser.add_argument("--plan-sha256", required=True); parser.add_argument("--authority-sha256", required=True)
    parser.add_argument("--parent-record"); parser.add_argument("--lease-fd", type=int)
    args = parser.parse_args(argv)
    if args.mode == "child": return child(args.plan_sha256, args.authority_sha256, json.loads(args.parent_record), args.lease_fd)
    result = execute(args.plan_sha256, args.authority_sha256)
    print(json.dumps({name: result.get(name) for name in ("schema", "status", "exit_code", "elapsed_seconds")}))
    return 0 if result["status"] == "EXPORTED_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY" else 2


if __name__ == "__main__": raise SystemExit(main())
