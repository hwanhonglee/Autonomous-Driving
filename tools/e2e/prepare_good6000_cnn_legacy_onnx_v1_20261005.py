"""HH_261005 - Prepare a lazy, CPU-only legacy ONNX feasibility export of the exact GOOD6000 CNN; never activate a runtime or build a GPU engine.

No Torch import, model loading, tracing or file output occurs on module import.
Actual entry is unconditionally denied in this prepared-only revision: fixed
ROOT authority/plan, runtime lease and resource admission are not implemented.
This proves neither TensorRT support nor cached/full-output numerical parity.
"""

import ast
from io import BytesIO
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
SCHEMA = "portable_e2e.good6000_cnn_legacy_onnx_feasibility.20261005.v1"
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
    # HH_261005 - Validate a draft scalar contract only; caller booleans are not actual ROOT authority or runtime admission.
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


def _deny_actual_execution():
    # HH_261005 - No caller declaration can authorize this source-only V1; a fresh reviewed revision must implement real admission.
    raise RuntimeError("PREPARED_ONLY: fixed ROOT authority/plan, exclusive runtime lease and output/resource budget admission are not implemented; a fresh reviewed revision is required")


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
    _deny_actual_execution()
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
    _deny_actual_execution()
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
            with torch.no_grad():
                torch.onnx.export(wrapper, torch.zeros((batch, 3, 180, 320), dtype=torch.float32, device="cpu"), buffer,
                    input_names=["normalized_images"], output_names=["cnn_features"], opset_version=12,
                    export_params=True, do_constant_folding=False, dynamic_axes=None, keep_initializers_as_inputs=False,
                    training=torch.onnx.TrainingMode.EVAL, operator_export_type=torch.onnx.OperatorExportTypes.ONNX,
                    enable_onnx_checker=True, use_external_data_format=False)
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
