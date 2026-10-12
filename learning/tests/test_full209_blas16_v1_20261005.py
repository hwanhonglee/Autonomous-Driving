"""HH_261005 - Prepare scalar-only tests of exact original worker and pre-import BLAS environment seams; never run weights, images or NN."""

import ast
import copy
import os
from pathlib import Path
import sys

import pytest

from scripts.e2e import benchmark_full209_blas16_v1_20261005 as probe


def test_controlled_two_arms_keep_every_original_context_tolerance_and_refusal():
    value = probe.contract()
    assert value["thread_arms"] == [6, 6] and value["interop_threads"] == 1
    assert value["OPENBLAS_environment_values"] == ["1", "6"]
    assert value["fixed_OMP_NUM_THREADS"] == value["fixed_MKL_NUM_THREADS"] == "6"
    assert value["total_contexts"] == 1776 and value["total_timed_calls"] == 60
    assert value["first_selected_context_cold_cache_per_fresh_arm"] is True
    assert value["serial_AB_not_thirty_interleaved_pairs"] is True
    assert value["raw_absolute_tolerance"] == value["raw_relative_tolerance"] == 1e-6
    assert value["ordinary_decoder_XY_pointwise_Euclidean_limit_m"] == .05
    assert value["ordinary_decoder_XY_chord_speed_absolute_limit_mps"] == .1
    assert value["original_prefills"] == 23 and value["original_fresh_NN"] == 865
    assert value["first_five_prefill_decoder_refusals_per_arm_retained"] is True
    assert value["active_BLAS_pool_identification_proven"] is False
    assert value["CPU_affinity_changed"] is False
    assert value["main_functions_approved"] == value["subfunctions_approved"] == 0
    value["fixed_serial_environment_arms"][0]["name"] = "not_the_constant"
    assert probe.ARMS[0]["name"] == "OPENBLAS1"


def test_exact_two_worker_seams_inverse_original_AST_with_setup_before_Torch():
    tree = ast.parse(Path(probe.original.__file__).read_text())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "worker")
    fresh = probe.private_worker_AST()
    output = next(n for n in fresh.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "output" for t in n.targets))
    assert ast.unparse(output.value) == "OUTPUT / ARMS[CURRENT_ARM]['name']"
    output.value = copy.deepcopy(next(n for n in old.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "output" for t in n.targets)).value)
    attempt = next(n for n in fresh.body if isinstance(n, ast.Try))
    setup = [i for i, n in enumerate(attempt.body) if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name) and n.value.func.id == "apply_environment"]
    torch_import = next(i for i, n in enumerate(attempt.body) if isinstance(n, ast.Import) and n.names[0].name == "torch")
    assert setup == [torch_import - 1]
    attempt.body.pop(setup[0])
    assert ast.dump(fresh, include_attributes=False) == ast.dump(old, include_attributes=False)


def test_private_scope_retains_exact_code_and_actual_new_source_contract_binding(monkeypatch):
    monkeypatch.setattr(probe, "_SCOPE", None)
    before = dict(probe.original.__dict__)
    scope = probe._private_scope()
    for name in ("raw_values", "raw_comparison", "ordinary_decoder_screen", "decoder_comparison", "compact_cache",
                 "initialize_worker_runtime", "set_threads_after_load", "timed_forward", "compare_arms", "bounded_append"):
        assert scope[name].__code__ is getattr(probe.original, name).__code__
        assert scope[name].__globals__ is scope
    assert scope["_snapshot_original"].__code__ is probe.original.snapshot.__code__
    assert scope["_snapshot_original"].__globals__ is scope
    assert scope["source_snapshot"] is probe.source_snapshot and scope["contract"] is probe.contract
    assert scope["THREAD_ARMS"] == (6, 6)
    assert probe.original.__dict__ == before
    assert not any(name in sys.modules for name in ("torch", "numpy", "carla"))


@pytest.fixture
def fake_setup(monkeypatch):
    # HH_261005 - Mutate only pytest-restored env entries; stub admission files/syscall metadata and never import native libraries.
    for name in probe.ENV_NAMES: monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    monkeypatch.setattr(probe, "ENVIRONMENT_STATE", None)
    monkeypatch.setattr(probe, "CURRENT_ARM", 0)
    monkeypatch.setattr(probe.os, "sched_getaffinity", lambda pid: set(range(24)))
    actual_iterdir = Path.iterdir
    monkeypatch.setattr(Path, "iterdir", lambda path: iter([Path("/proc/self/task/99")]) if path == Path("/proc/self/task") else actual_iterdir(path))
    saved = []
    monkeypatch.setattr(probe.original, "write_new_json", lambda path, value: saved.append((path, copy.deepcopy(value))))
    return dict(contract=probe.contract(), environment={"environment": probe.selected_environment()}), saved


@pytest.mark.parametrize("arm,value", [(0, "1"), (1, "6")])
def test_environment_set_once_before_import_and_receipt_binds_original_values(fake_setup, monkeypatch, arm, value):
    plan, saved = fake_setup
    monkeypatch.setattr(probe, "CURRENT_ARM", arm)
    probe.apply_environment(plan)
    assert probe.selected_environment() == dict(plan["environment"]["environment"], OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS=value)
    assert len(saved) == 1 and saved[0][0].parent.name == probe.ARMS[arm]["name"]
    assert saved[0][1]["original_environment"] == plan["environment"]["environment"]
    assert saved[0][1]["before_any_Torch_NumPy_or_model_import"] is True
    with pytest.raises(RuntimeError, match="one fresh"): probe.apply_environment(plan)
    assert len(saved) == 1


@pytest.mark.parametrize("fault", ["torch", "numpy", "carla", "contract", "original_env", "affinity", "other_thread", "bool_arm"])
def test_bad_admission_never_assigns_thread_environment_or_writes_success(fake_setup, monkeypatch, fault):
    plan, saved = fake_setup
    if fault in ("torch", "numpy", "carla"): monkeypatch.setitem(sys.modules, fault, object())
    elif fault == "contract": plan["contract"]["fixed_intraop_threads"] = 8
    elif fault == "original_env": plan["environment"]["environment"]["OMP_NUM_THREADS"] = "unexpected"
    elif fault == "affinity": monkeypatch.setattr(probe.os, "sched_getaffinity", lambda pid: set(range(16)))
    elif fault == "other_thread": monkeypatch.setattr(Path, "iterdir", lambda path: iter([Path("1"), Path("2")]))
    else: monkeypatch.setattr(probe, "CURRENT_ARM", True)
    before = probe.selected_environment()
    with pytest.raises(RuntimeError): probe.apply_environment(plan)
    assert probe.selected_environment() == before and probe.ENVIRONMENT_STATE is None and not saved


def test_post_snapshot_inverts_only_three_declared_values_not_any_other_guard(fake_setup, monkeypatch):
    plan, saved = fake_setup
    probe.apply_environment(plan)
    document = dict(sources={"immutable": "a"*64}, inputs={"complete": 7366}, assets={"weights": "b"*64},
        environment={"environment": probe.selected_environment(), "inherited_CPU_affinity": list(range(24)),
                     "interpreter": "/usr/bin/python3", "GPU_used": False}, contract={"exact": True})
    monkeypatch.setattr(probe, "_private_scope", lambda: {"_snapshot_original": lambda: (copy.deepcopy(document), ["camera_metadata"], ["event_metadata"])})
    actual, cameras, events = probe.snapshot()
    expected = copy.deepcopy(document)
    expected["environment"]["environment"] = plan["environment"]["environment"]
    assert actual == expected and cameras == ["camera_metadata"] and events == ["event_metadata"]
    monkeypatch.setenv("OPENBLAS_NUM_THREADS", "8")
    with pytest.raises(RuntimeError, match="changed"): probe.snapshot()


def test_original_frozen_source_change_denies_compilation_before_any_model_import(monkeypatch):
    actual = probe.digest
    monkeypatch.setattr(probe, "digest", lambda path: "changed" if Path(path) == Path(probe.frozen.__file__) else actual(path))
    with pytest.raises(RuntimeError, match="frozen thread468 V2"): probe.private_worker_AST()


def test_no_parent_environment_mutation_global_activation_or_suppressed_worker_error_in_prepared_source():
    tree = ast.parse(probe.HERE.read_text())
    worker = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "worker")
    assert not any(isinstance(n, ast.Try) for n in ast.walk(worker))
    parent = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_parent")
    assert not any(isinstance(n, ast.Assign) and any(isinstance(t, ast.Subscript) and isinstance(t.value, ast.Attribute) and t.value.attr == "environ" for t in n.targets) for n in ast.walk(parent))
    assert "failed_or_unexecuted_tails_retained" in ast.unparse(parent)
    assert "no_retry" in ast.unparse(parent)
    assert "torch" not in sys.modules and "carla" not in sys.modules
