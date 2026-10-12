"""HH_261005 - Prove fresh affinity block admission and exact original worker seams with scalar fakes only, never weights or replay."""

import ast
import copy
from pathlib import Path
import sys

import pytest

from scripts.e2e import benchmark_full209_pcore_affinity_v1_20261005 as probe


def test_four_fresh_ABBA_blocks_keep_whole_denominator_and_disclose_not_interleaved_pairs():
    value = probe.contract()
    assert [row["variant"] for row in value["serial_ABBA_blocks"]] == list("ABBA")
    assert len({row["name"] for row in value["serial_ABBA_blocks"]}) == 4
    assert value["context_denominator_per_block"] == 888 and value["total_contexts"] == 3552
    assert value["timed_calls_per_block"] == 30 and value["total_timed_calls"] == 120
    assert value["thirty_interleaved_AB_pairs_claimed"] is False
    assert value["serial_input_and_OS_cache_warming_disclosed"] is True
    assert value["first_selected_context_cold_cache_per_fresh_block"] is True
    assert value["raw_absolute_tolerance"] == value["raw_relative_tolerance"] == 1e-6
    assert value["ordinary_decoder_XY_pointwise_Euclidean_limit_m"] == .05
    assert value["ordinary_decoder_XY_chord_speed_absolute_limit_mps"] == .1
    assert value["all_camera_bundles"] == 901 and value["original_prefills"] == 23 and value["original_fresh_NN"] == 865
    assert value["original_V2_fresh_ordinary"] == 802 and value["original_V2_fresh_terminal"] == 63
    assert value["selected_context_indices"][0] == 0 and value["selected_context_indices"][-1] == 887
    assert value["main_functions_approved"] == value["subfunctions_approved"] == 0
    assert value["complete_safety_engine_and_control_parity_verified"] is False and value["actual_runtime_activated"] is False


def test_contract_is_detached_from_constant_block_selection():
    value = probe.contract()
    value["serial_ABBA_blocks"][0]["name"] = "changed"
    assert probe.BLOCKS[0]["name"] == "A1_ALL24"
    assert value == __import__("json").loads(__import__("json").dumps(value, allow_nan=False))


def test_private_worker_three_seams_inverse_is_exact_original_AST():
    source = ast.parse(Path(probe.original.__file__).read_text())
    old = next(row for row in source.body if isinstance(row, ast.FunctionDef) and row.name == "worker")
    revised = probe.private_worker_AST()
    old_output = next(row for row in old.body if isinstance(row, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "output" for target in row.targets))
    output = next(row for row in revised.body if isinstance(row, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "output" for target in row.targets))
    assert ast.unparse(output.value) == "OUTPUT / BLOCKS[CURRENT_BLOCK]['name']"
    attempt = next(row for row in revised.body if isinstance(row, ast.Try))
    names = [row.value.func.id for row in attempt.body if isinstance(row, ast.Expr) and isinstance(row.value, ast.Call) and isinstance(row.value.func, ast.Name)]
    assert names.count("apply_affinity") == names.count("verify_thread_affinity") == 1
    output.value = copy.deepcopy(old_output.value)
    attempt.body = [row for row in attempt.body if not (isinstance(row, ast.Expr) and isinstance(row.value, ast.Call)
        and isinstance(row.value.func, ast.Name) and row.value.func.id in ("apply_affinity", "verify_thread_affinity"))]
    assert ast.dump(revised, include_attributes=False) == ast.dump(old, include_attributes=False)


def test_private_scope_preserves_neural_numeric_decoder_and_loader_code_objects(monkeypatch):
    monkeypatch.setattr(probe, "_SCOPE", None)
    before = dict(probe.original.__dict__)
    scope = probe._private_scope()
    for name in ("raw_values", "raw_comparison", "ordinary_decoder_screen", "decoder_comparison", "compact_cache",
                 "initialize_worker_runtime", "set_threads_after_load", "compare_arms", "bounded_append"):
        assert scope[name].__code__ is getattr(probe.original, name).__code__
        assert scope[name].__globals__ is scope
    assert probe.original.__dict__ == before
    assert scope["THREAD_ARMS"] == (6, 6, 6, 6) and scope["OUTPUT"] == probe.OUTPUT
    assert "torch" not in sys.modules and "carla" not in sys.modules


@pytest.fixture
def fake_affinity(monkeypatch):
    # HH_261005 - Stub syscall providers so the test itself never changes its real scheduler affinity or starts a model thread.
    current, calls = [list(probe.ALL_CPUS)], []
    monkeypatch.setattr(probe, "AFFINITY_STATE", None)
    monkeypatch.setattr(probe, "CURRENT_BLOCK", 1)
    monkeypatch.setattr(probe.os, "getpid", lambda: 99)
    monkeypatch.setattr(probe.os, "sched_getaffinity", lambda pid: set(current[0]))
    def setter(pid, mask):
        calls.append((pid, list(mask)))
        current[0] = list(mask)
    monkeypatch.setattr(probe.os, "sched_setaffinity", setter)
    monkeypatch.setattr(probe.original, "write_new_json", lambda path, value: None)
    monkeypatch.setattr(probe, "process_thread_masks", lambda: [dict(tid=99, start_ticks=123, CPU_affinity=list(current[0]))])
    return current, calls


def test_mask_is_set_once_before_Torch_on_genuine_single_thread_admission(fake_affinity):
    current, calls = fake_affinity
    probe.apply_affinity(dict(contract=probe.contract()))
    assert calls == [(0, list(probe.PCORE_CPUS))] and current[0] == list(probe.PCORE_CPUS)
    assert probe.AFFINITY_STATE["before_any_Torch_or_model_import"] is True
    assert probe.AFFINITY_STATE["before_thread_identities"] == [dict(tid=99, start_ticks=123, CPU_affinity=list(probe.ALL_CPUS))]
    with pytest.raises(RuntimeError, match="single fresh"): probe.apply_affinity(dict(contract=probe.contract()))
    assert len(calls) == 1


@pytest.mark.parametrize("fault", ["preimported_Torch", "changed_contract", "original_mask", "other_thread", "wrong_thread_generation"])
def test_failed_admission_never_changes_scheduler(fake_affinity, monkeypatch, fault):
    current, calls = fake_affinity
    plan = dict(contract=probe.contract())
    if fault == "preimported_Torch": monkeypatch.setitem(sys.modules, "torch", object())
    elif fault == "changed_contract": plan["contract"]["fixed_intraop_threads"] = 8
    elif fault == "original_mask": current[0] = list(probe.PCORE_CPUS)
    elif fault == "other_thread": monkeypatch.setattr(probe, "process_thread_masks", lambda: [dict(tid=99), dict(tid=100)])
    else: monkeypatch.setattr(probe, "process_thread_masks", lambda: [dict(tid=100, start_ticks=123, CPU_affinity=list(probe.ALL_CPUS))])
    with pytest.raises(RuntimeError): probe.apply_affinity(plan)
    assert not calls and probe.AFFINITY_STATE is None


@pytest.mark.parametrize("text, expected", [("0-15", list(range(16))), ("0-3,8,10-11", [0,1,2,3,8,10,11]), ("0-23\n", list(range(24)))])
def test_actual_proc_mask_parser(text, expected):
    assert probe.parse_cpu_list(text) == expected


@pytest.mark.parametrize("text", ["", "1-0", "0,0", "0-2,2-4", "bad", "0-999999", "0-1-2"])
def test_malformed_or_duplicated_mask_is_denied(text):
    with pytest.raises(RuntimeError): probe.parse_cpu_list(text)


def test_all_observed_thread_mask_escape_is_denied_before_receipt_success(fake_affinity, monkeypatch, tmp_path):
    probe.apply_affinity(dict(contract=probe.contract()))
    monkeypatch.setattr(probe, "process_thread_masks", lambda: [dict(tid=99, start_ticks=123, CPU_affinity=list(probe.PCORE_CPUS)),
        dict(tid=100, start_ticks=124, CPU_affinity=list(probe.ALL_CPUS))])
    with pytest.raises(RuntimeError, match="escaped"): probe.verify_thread_affinity(tmp_path, "synthetic")
    assert not list(tmp_path.iterdir())


def test_only_declared_affinity_field_is_normalized_after_worker_change(monkeypatch):
    document = dict(sources={"frozen": "a"*64}, inputs={"all_original_file_count": 7366}, assets={"GOOD": "b"*64},
                    environment=dict(inherited_CPU_affinity=list(probe.PCORE_CPUS), interpreter="unchanged"), contract={"original": True})
    monkeypatch.setattr(probe, "AFFINITY_STATE", dict(declared_mask=list(probe.PCORE_CPUS)))
    monkeypatch.setattr(probe, "_private_scope", lambda: {"_snapshot_original": lambda: (copy.deepcopy(document), ["camera_metadata_only"], ["event_metadata_only"])})
    monkeypatch.setattr(probe, "cpu_topology", lambda: {"synthetic_named_metadata": True})
    actual, cameras, events = probe.snapshot()
    expected = copy.deepcopy(document)
    expected["environment"]["inherited_CPU_affinity"] = list(probe.ALL_CPUS)
    expected["CPU_topology"] = {"synthetic_named_metadata": True}
    assert actual == expected and cameras == ["camera_metadata_only"] and events == ["event_metadata_only"]
    document["environment"]["inherited_CPU_affinity"] = [0]
    with pytest.raises(RuntimeError, match="worker affinity changed"): probe.snapshot()


def test_affinity_proofs_are_outside_original_timer_and_original_forward_exception_is_not_masked(monkeypatch, tmp_path):
    calls, exception = [], RuntimeError("original retained forward failure")
    monkeypatch.setattr(probe, "verify_thread_affinity", lambda output, stage: calls.append(stage))
    def forward(*args):
        calls.append("original_timer_and_journal")
        return "unchanged_raw_heads"
    monkeypatch.setattr(probe, "_private_scope", lambda: {"_timed_original": forward})
    assert probe.timed_forward(None, None, None, 6, 0, [], tmp_path) == "unchanged_raw_heads"
    assert calls == ["before_selected_forward_0", "original_timer_and_journal", "after_selected_forward_0"]
    calls.clear()
    def fail(*args):
        calls.append("original_failed_timer_and_journal")
        raise exception
    monkeypatch.setattr(probe, "_private_scope", lambda: {"_timed_original": fail})
    with pytest.raises(RuntimeError) as caught: probe.timed_forward(None, None, None, 6, 0, [], tmp_path)
    assert caught.value is exception and calls == ["before_selected_forward_0", "original_failed_timer_and_journal"]


def test_frozen_source_mutation_fails_before_compilation_or_model_import(monkeypatch):
    real = probe.digest
    monkeypatch.setattr(probe, "digest", lambda path: "changed" if Path(path) == Path(probe.frozen.__file__) else real(path))
    with pytest.raises(RuntimeError, match="frozen thread468 V2"): probe.private_worker_AST()
