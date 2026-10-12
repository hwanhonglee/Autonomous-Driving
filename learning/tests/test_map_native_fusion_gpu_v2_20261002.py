"""HH_261002 - Synthetic-only residual/source/worker guards; no real GPU, model, TRAIN or SSH calls."""
import ast
import copy
import hashlib
import inspect
import json
import multiprocessing
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning")); sys.path.insert(0, str(ROOT / "tools/e2e"))
import train_map_native_fusion_pair_v2_20261002 as revision


def runtime():
    names = ("python_executable", "numpy", "numpy_native", "PIL.Image", "PIL._imaging", "torch", "torch._C", "torch.lib/libtorch.so")
    return dict(python_version="synthetic", python_implementation="CPython", python_prefix="/synthetic/private/venv",
        numpy="synthetic", pillow="synthetic", torch="synthetic", torch_compiled_CUDA="synthetic",
        provider_files={k: dict(path="/synthetic/provider/" + str(i), bytes=1, sha256="b" * 64) for i, k in enumerate(names)})


def authority(workers=0):
    p = revision.protocol; source = dict(revision.FROZEN_SOURCE_SHA, **{Path(revision.__file__).name: "a" * 64})
    proof = {"synthetic_not_actual_schedule": True}
    value = dict(schema=p.AUTHORIZATION_SCHEMA, contract=p.prepared_contract(workers), source_sha256=source,
        protocol_sha256=p.document_sha(p.prepared_contract(workers)), equal_global_schedule_sha256=p.document_sha(proof),
        runtime_identity=runtime(), reviewer="synthetic-not-real-authority", research_fit_authorized=True,
        native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False, automatic_model_promotion=False, vehicle_control_approved=False)
    return value, source, proof


@pytest.mark.parametrize("workers", [0, 2])
def test_fresh_protocol_truth_and_exact_original_math_contract(workers):
    old = revision.previous.prepared_contract(workers); new = revision.protocol.prepared_contract(workers)
    changed = {k for k in old if old[k] != new[k]}
    assert changed == {"schema", "study_id", "source", "output", "source_count", "numeric_export_schema", "GPU_idle_precondition"}
    assert new["schema"].endswith(".v2") and new["source_count"] == 14
    assert new["source"] == str(revision.SOURCE) and new["output"] == str(revision.OUTPUT)
    assert revision.SOURCE != revision.previous.SOURCE and revision.OUTPUT != revision.previous.OUTPUT
    assert new["frozen_original_schema"] == old["schema"] and new["fresh_revision_is_not_an_original_V1_execution"] is True
    assert new["gpu_preflight_revision"] == revision.GPU_REVISION
    assert new["examples_per_arm"] == 160000 and new["parameter_count_by_arm"] == old["parameter_count_by_arm"]
    assert new["semantic_loss"] == old["semantic_loss"] and new["all_existing_27_screens_unchanged"] is True
    for name in ("schedule", "schedule_proof", "train_package", "postcheck", "environment_contract", "original_27_screen"):
        assert getattr(revision.protocol, name).__code__ is getattr(revision.previous, name).__code__
    value, source, proof = authority(workers)
    revision.protocol.validate_authorization(value, source, proof, workers)
    assert revision.previous.SOURCE.name.endswith("v1") and revision.previous.SOURCE_NAMES != revision.SOURCE_NAMES


def test_authority_inverse_AST_only_source_count():
    node = revision.count_revision_ast()
    hits = [n for n in ast.walk(node) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 14]
    assert len(hits) == 1; hits[0].value = 13
    tree = ast.parse(Path(revision.previous.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "validate_authorization")
    assert ast.dump(node) == ast.dump(old)


@pytest.mark.parametrize("fault", ["oldschema", "count", "source", "wrapper", "frozen", "GPU", "float", "fit", "scope"])
def test_new_authority_rejects_original_or_changed_contract(fault):
    value, source, proof = authority()
    if fault == "oldschema": value["schema"] = revision.previous.AUTHORIZATION_SCHEMA
    elif fault == "count": value["contract"]["source_count"] = 13
    elif fault == "source": source["unknown.py"] = "c" * 64
    elif fault == "wrapper": source.pop(Path(revision.__file__).name)
    elif fault == "frozen": source[next(iter(revision.FROZEN_SOURCE_SHA))] = "c" * 64
    elif fault == "GPU": value["contract"]["gpu_preflight_revision"]["maximum_driver_residual_MiB"] = 17
    elif fault == "float": value["contract"]["gradient_norm_cap"] = 5
    elif fault == "fit": value["research_fit_authorized"] = False
    else: value["DEV_or_HOLDOUT_used_for_fit"] = True
    with pytest.raises(ValueError): revision.protocol.validate_authorization(value, source, proof, 0)


def test_frozen_thirteen_and_fresh_fourteen_snapshot(tmp_path):
    revision.verify_frozen()
    for name in revision.SOURCE_NAMES:
        source = Path(revision.__file__).parent / name
        if not source.is_file(): source = ROOT / "learning/portable_e2e" / name
        shutil.copy2(source, tmp_path / name)
    actual = revision.source_snapshot(tmp_path.resolve())
    assert len(actual) == 14 and all(actual[k] == v for k, v in revision.FROZEN_SOURCE_SHA.items())
    assert actual[Path(revision.__file__).name] == hashlib.sha256(Path(revision.__file__).read_bytes()).hexdigest()
    name = next(iter(revision.FROZEN_SOURCE_SHA)); (tmp_path / name).write_bytes(b"changed")
    with pytest.raises(ValueError): revision.source_snapshot(tmp_path.resolve())


def fake_gpu(monkeypatch, *, memory="1", utilization="0", apps="", index="0", uuid=None,
             available=True, count=1, properties=None, initialized=False, rows=None):
    seen = []; uuid = revision.original.mining.GPU_UUID if uuid is None else uuid
    row = f"{index}, {uuid}, {memory}, {utilization}\n" if rows is None else rows
    def query(command, **kwargs):
        seen.append(("query", tuple(command)))
        assert command[:3] == ["nvidia-smi", "-i", "0"] and kwargs["timeout"] == 20
        return apps if "compute-apps" in command[3] else row
    def cuda_available(): seen.append(("CUDA_available",)); return available
    def cuda_count(): seen.append(("CUDA_count",)); return count
    def props(device):
        seen.append(("CUDA_properties", device))
        return SimpleNamespace(uuid=uuid if properties is None else properties, name="synthetic-not-GPU")
    torch = SimpleNamespace(cuda=SimpleNamespace(is_initialized=lambda: initialized, is_available=cuda_available,
        device_count=cuda_count, get_device_properties=props), device=lambda value: value)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", revision.original.mining.GPU_UUID)
    monkeypatch.setattr(revision.original.subprocess, "check_output", query)
    monkeypatch.setattr(revision, "_providers", lambda: (torch, None, None, None, None))
    return torch, seen


@pytest.mark.parametrize("memory", ["0", "1", "16"])
def test_both_physical_queries_before_CUDA_and_honest_residual(monkeypatch, memory):
    _, seen = fake_gpu(monkeypatch, memory=memory)
    revision.gpu_idle_before_cuda()
    assert len(seen) == 2 and all(v[0] == "query" for v in seen)
    value = revision.preflight_gpu()
    assert [v[0] for v in seen] == ["query"] * 4 + ["CUDA_available", "CUDA_count", "CUDA_properties"]
    assert value["memory_used_mib_before_torch"] == int(memory)
    assert value["GPU0_compute_process_count"] == 0 and value["gpu_preflight_revision"] == revision.GPU_REVISION


@pytest.mark.parametrize("settings", [dict(memory="17"), dict(memory="-1"), dict(memory="1.0"), dict(utilization="1"),
    dict(index="1"), dict(uuid="GPU-foreign"), dict(apps="123, GPU-active, 1\n"), dict(initialized=True),
    dict(rows=""), dict(rows="0,a,1,0\n0,a,1,0\n"), dict(rows="malformed\n")])
def test_busy_wrong_or_malformed_GPU_rejects_before_CUDA(monkeypatch, settings):
    _, seen = fake_gpu(monkeypatch, **settings)
    with pytest.raises(ValueError): revision.preflight_gpu()
    assert all(v[0] == "query" for v in seen)


@pytest.mark.parametrize("settings", [dict(available=False), dict(count=0), dict(count=2), dict(properties="GPU-foreign")])
def test_Torch_single_physical_GPU_properties_required(monkeypatch, settings):
    fake_gpu(monkeypatch, **settings)
    with pytest.raises(ValueError): revision.preflight_gpu()


def test_environment_UUID_rejected_before_physical_query(monkeypatch):
    _, seen = fake_gpu(monkeypatch); monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0,1")
    with pytest.raises(ValueError): revision.preflight_gpu()
    assert seen == []


def test_private_guard_preserves_exact_original_lease_without_module_mutation(monkeypatch):
    lease = object(); preflight = object()
    guard = SimpleNamespace(GPU0_UUID=revision.original.mining.GPU_UUID, __file__="/synthetic/train_map_photometric_pilot.py",
        gpu0_lease=lease, preflight_gpu=preflight, seed_everything=object())
    monkeypatch.setattr(revision, "_providers", lambda: (None, None, None, None, guard))
    proxy = revision.providers()[-1]
    assert proxy is not guard and proxy.gpu0_lease is lease and proxy.preflight_gpu is revision.preflight_gpu
    assert guard.preflight_gpu is preflight and proxy.seed_everything is guard.seed_everything


def test_scope_restores_all_four_bindings_after_exception_and_reentry_rejected():
    old = revision.original.protocol, revision.original.providers, revision.original.gpu_idle_before_cuda, revision.original.CropBatches
    with pytest.raises(RuntimeError, match="synthetic"):
        with revision.activated():
            assert revision.original.protocol is revision.protocol and revision.original.providers is revision.providers
            assert revision.original.CropBatches is revision.CropBatches
            with pytest.raises(ValueError):
                with revision.activated(): pass
            raise RuntimeError("synthetic")
    assert (revision.original.protocol, revision.original.providers, revision.original.gpu_idle_before_cuda, revision.original.CropBatches) == old
    assert revision._active is False


@pytest.mark.parametrize("name", ["protocol", "providers", "gpu_idle_before_cuda", "CropBatches"])
def test_unknown_binding_not_overwritten(monkeypatch, name):
    bad = object(); monkeypatch.setattr(revision.original, name, bad)
    with pytest.raises(ValueError):
        with revision.activated(): pass
    assert getattr(revision.original, name) is bad and not revision._active


def test_spawn_reader_exact_original_bytecode_and_new_protocol_no_CUDA(monkeypatch):
    old = revision._batches.__getitem__
    assert revision.CropBatches.__getitem__.__code__ is old.__code__
    assert revision.CropBatches.__getitem__.__globals__["protocol"] is revision.protocol
    assert revision.CropBatches.__getstate__ is revision._batches.__getstate__
    data = np.zeros((1, 1), np.uint8); schedule = {k: np.zeros((1, 1), np.uint8) for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits")}
    descriptor = revision.CropBatches(data, schedule, {"wrapper.py": "a" * 64})
    assert descriptor.__getstate__()["arrays"] is None
    monkeypatch.setattr(revision.protocol.previous.previous, "batch_arrays", lambda arrays, ids, boxes, bits, arm: (arrays, arrays, bits.copy()))
    step, rgb, target, bits = descriptor[0]
    assert step == 0 and rgb is data and target is data and not bits.any()


def test_public_contract_mode_no_payload_model_GPU_or_output(monkeypatch, capsys):
    monkeypatch.setattr(revision.original, "run", lambda args: pytest.fail("query is not a fit"))
    assert revision.main(["--mode", "contract"]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value["schema"] == revision.SCHEMA and value["source_count"] == 14
    assert value["training_authorized"] is False and value["model_promoted"] is False


def spawned_reader(connection, descriptor):
    # HH_261002 - A genuine spawn imports the new wrapper independently; all payload/providers below are synthetic.
    try:
        assert descriptor.arrays is None
        assert type(descriptor) is revision.CropBatches
        assert descriptor.__getitem__.__globals__["protocol"] is revision.protocol
        assert revision.protocol.SOURCE.name.endswith("v2") and len(revision.protocol.SOURCE_NAMES) == 14
        sys.modules["torch"] = SimpleNamespace(cuda=SimpleNamespace(is_initialized=lambda: False))
        revision.protocol.source_snapshot = lambda root: descriptor.source
        revision.protocol.train_package = lambda: (None, np.zeros((1, 1), np.uint8), None, None)
        revision.protocol.previous.previous.batch_arrays = lambda arrays, ids, boxes, bits, arm: (arrays, arrays, bits.copy())
        step, rgb, target, bits = descriptor[0]
        connection.send(dict(schema=revision.protocol.SCHEMA, source_count=len(revision.protocol.SOURCE_NAMES),
            step=step, bytes_equal=np.array_equal(rgb, target), FALSE_only=not bits.any(), Torch_imported=False))
    except Exception as error: connection.send(dict(error=type(error).__name__ + ": " + str(error)))
    finally: connection.close()


def test_genuine_spawn_restores_new_worker_protocol_without_V1_path_or_CUDA():
    schedule = {k: np.zeros((1, 1), np.uint8) for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits")}
    descriptor = revision.CropBatches(np.zeros((1, 1), np.uint8), schedule, {"synthetic.py": "a" * 64})
    context = multiprocessing.get_context("spawn"); receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=spawned_reader, args=(sender, descriptor)); process.start(); sender.close()
    try:
        assert receiver.poll(20), "synthetic spawn did not return"
        value = receiver.recv(); process.join(20)
        assert process.exitcode == 0 and "error" not in value
        assert value == dict(schema=revision.SCHEMA, source_count=14, step=0, bytes_equal=True, FALSE_only=True, Torch_imported=False)
    finally:
        receiver.close()
        if process.is_alive(): process.terminate(); process.join(5)


def test_frozen_run_reaches_both_new_guards_under_original_lease_only():
    body = ast.parse(inspect.getsource(revision.original.run))
    lease = next(n for n in ast.walk(body) if isinstance(n, ast.With)
        and any(isinstance(i.context_expr, ast.Call) and isinstance(i.context_expr.func, ast.Attribute)
            and i.context_expr.func.attr == "gpu0_lease" for i in n.items))
    assert ast.unparse(lease.body[0]) == "gpu_idle_before_cuda()"
    assert ast.unparse(lease.body[1]) == "gpu = guard.preflight_gpu()"
    assert revision.original.strict_preflights.__globals__["protocol"] is revision.previous
    assert revision.protocol.original_27_screen.__code__ is revision.previous.original_27_screen.__code__
