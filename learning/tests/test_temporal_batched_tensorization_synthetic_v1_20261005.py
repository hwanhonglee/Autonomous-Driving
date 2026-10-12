"""HH_261005 - Check prospective synthetic CPU benchmark contracts without any actual timed experiment."""

import ast
from dataclasses import replace
import inspect
import itertools
import json
from pathlib import Path

import pytest

from scripts.e2e import benchmark_temporal_batched_tensorization_synthetic_v1_20261005 as bench


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    return bench.modules()


def test_contract_fixes_all_180_calls_and_disclaims_every_actual_gain():
    contract = bench.contract()
    assert contract["patterns"] == list(bench.PATTERNS)
    assert contract["timed_calls"] == 180 and contract["pairs_per_pattern"] == 30
    assert contract["warmup_builder_calls"] == 0
    assert contract["first_call_cold_and_all_slow_or_rejected_calls_retained"] is True
    assert contract["source_image_shape"] == [6, 360, 640, 4]
    assert contract["output_image_shape"] == [1, 6, 3, 180, 320]
    assert contract["intraop"] == 6 and contract["interop"] == 1
    assert contract["interop_mutated_by_runner"] is False
    assert contract["intraop_restored_after_context"] is True
    assert contract["formal_main_function_approval"] == contract["formal_subfunction_approval"] == 0
    for name in ("actual_recorded_BGRA_parity_proven", "actual_sensor_latency_gain_proven",
                 "actual_closed_loop_gain_proven", "NN_or_weights_read", "model_promoted", "training", "installs"):
        assert contract[name] is False


def test_fixed_parent_candidate_and_legacy_fixture_pins_and_complete_source_closure():
    sources = bench.source_snapshot()
    assert all(sources[name] == sha for name, sha in bench.FIXED_PINS.items())
    assert sources[str(bench.HERE.relative_to(bench.ROOT))] == bench.digest(bench.HERE)
    assert sources[str(bench.TEST.relative_to(bench.ROOT))] == bench.digest(bench.TEST)
    assert "learning/portable_e2e/runtime.py" in sources
    assert "learning/portable_e2e/torch_dataset.py" in sources
    assert "learning/portable_e2e/runtime_contract.py" in sources
    assert not any(name.startswith(("storage/experiments/validation/", "datasets/")) for name in sources)


def test_synthetic_fixtures_are_reproducible_immutable_six_distinct_BGRA_roles_and_exact_legacy_nonimages(modules):
    _, _, np, runtime = modules
    config, rows = bench.fixtures()
    config2, rows2 = bench.fixtures()
    one, two = bench.fixture_snapshot(config, rows), bench.fixture_snapshot(config2, rows2)
    assert one == two and bench.metadata_sha(one) == bench.metadata_sha(two)
    assert (config.ego_history_frames, config.route_points) == (10, 128)
    original = bench.exact_function(bench.FIXTURE_SOURCE, "_runtime_inputs", dict(np=np,
        RuntimeInputs=runtime.RuntimeInputs, RuntimeCameraFrame=runtime.RuntimeCameraFrame,
        FAKE_TRAJECTORY_SPEED_MPS=__import__("math").hypot(.05, .001) / .1))(config)
    assert original.ego_history_mask == (False, True, True)
    for pattern, inputs in rows:
        assert inputs.calibration == original.calibration
        assert inputs.ego_history == original.ego_history
        assert inputs.route_xy_base_m == original.route_xy_base_m
        assert inputs.ego_history_mask == (True,) * 10
        for frame in inputs.camera_frames:
            assert frame.payload.shape == (360, 640, 4)
            assert frame.payload.flags.writeable is False
            with pytest.raises(ValueError): frame.payload.flags.writeable = True
    black, white = rows[0][1].camera_frames[:2]
    assert np.all(black.payload[:, :, :3] == 0) and np.all(white.payload[:, :, :3] == 255)


@pytest.mark.parametrize("pattern_index", range(3))
def test_real_original_and_frozen_candidate_whole_default_tensors_and_hashes_equal_without_timers(modules, pattern_index):
    candidate, torch, _, runtime = modules
    # HH_261005 - Untimed correctness is not the prospective cold timing run; restore this test's intra-op setting.
    old = torch.get_num_threads()
    try:
        torch.set_num_threads(6)
        config, rows = bench.fixtures()
        inputs = rows[pattern_index][1]
        before = bench.fixture_snapshot(config, rows)
        original = runtime.build_runtime_tensors
        provider = candidate.BatchedImageTensorizer()
        a, b = original(inputs, config), provider(inputs, config)
        assert a["images"].shape == (1, 6, 3, 180, 320)
        assert a["route_xy"].shape == (1, 128, 2)
        assert bench.exact_tensor_equal(a, b)
        assert bench.tensor_snapshot(a) == bench.tensor_snapshot(b)
        assert bench.fixture_snapshot(config, rows) == before
        assert runtime.build_runtime_tensors is original
        assert provider.last_timing["fallback_to_original"] is False
        assert len(provider.last_timing["camera_preprocessing"]) == 6
    finally:
        torch.set_num_threads(old)


def fake_calls(*, failing=False, bad_snapshot=False, fail_persist=False):
    # HH_261005 - A deterministic fake clock validates ledger structure; these are not measured CPU timing rows.
    ticks = itertools.count(100, 11)
    rows = [(name, name) for name in bench.PATTERNS]
    functions = {"A": lambda inputs, config: (inputs, config), "B": lambda inputs, config: (inputs, config)}
    if failing:
        def deny(inputs, config): raise ValueError("retained original failure")
        functions["B"] = deny
    snapshot = lambda value: value
    if bad_snapshot:
        def reject(value): raise ValueError("outside-timer verification failure")
        snapshot = reject
    saved, calls, pairs = [], [], []
    def persist(row):
        saved.append(dict(row))
        if fail_persist and len(saved) == 7: raise OSError("simulated journal failure")
    result = bench.paired_calls(functions, "config", rows, clock=lambda: next(ticks), snapshot=snapshot,
        compare=lambda a, b: a == b, persist=persist, receipt=lambda: {"synthetic": True}, results=calls, pairs=pairs)
    return result, saved


def test_fixed_ABBA_180_row_denominator_cold_first_and_all_patterns_are_retained():
    (calls, pairs), saved = fake_calls()
    assert len(calls) == len(saved) == 180 and len(pairs) == 90
    assert [row["call_ordinal"] for row in calls] == list(range(180))
    assert all(row["elapsed_ns"] == 11 for row in calls)
    assert all(row["bitwise_all_tensor_parity"] for row in pairs)
    for pattern in bench.PATTERNS:
        rows = [row for row in calls if row["pattern"] == pattern]
        assert len(rows) == 60 and sum(row["variant"] == "A" for row in rows) == 30
        for pair in range(30):
            assert "".join(row["variant"] for row in rows if row["pair_ordinal"] == pair) == ("AB" if pair % 2 == 0 else "BA")
    assert [row["variant"] for row in calls[:2]] == ["A", "B"]


@pytest.mark.parametrize("fault", ["builder", "outside_snapshot"])
def test_every_error_call_remains_in_fixed_180_denominator_with_original_reason(fault):
    (calls, pairs), saved = fake_calls(failing=fault == "builder", bad_snapshot=fault == "outside_snapshot")
    assert len(calls) == len(saved) == 180 and len(pairs) == 90
    assert not any(row["bitwise_all_tensor_parity"] for row in pairs)
    assert sum(row["error"] is not None for row in calls) == (90 if fault == "builder" else 180)
    assert all(row["error"]["type"] == "ValueError" for row in calls if row["error"])


def test_journal_error_preserves_attempted_in_memory_prefix_instead_of_discarding_partial_calls():
    calls, pairs, saved = [], [], []
    def persist(row):
        saved.append(row)
        if len(saved) == 7: raise OSError("durability failure")
    ticks = itertools.count()
    with pytest.raises(OSError, match="durability"):
        bench.paired_calls(dict(A=lambda i, c: i, B=lambda i, c: i), None,
            [(name, name) for name in bench.PATTERNS], clock=lambda: next(ticks), snapshot=lambda v: v,
            compare=lambda a, b: a == b, persist=persist, results=calls, pairs=pairs)
    assert len(calls) == len(saved) == 7 and len(pairs) == 3


class FakeTorch:
    def __init__(self, intra=4, interop=1): self.intra, self.interop, self.mutations = intra, interop, []
    def get_num_threads(self): return self.intra
    def get_num_interop_threads(self): return self.interop
    def set_num_threads(self, value): self.mutations.append(value); self.intra = value


@pytest.mark.parametrize("fail", [False, True])
def test_CPU6_context_restores_original_intraop_on_every_exit_without_interop_mutation(fail):
    fake = FakeTorch()
    if fail:
        with pytest.raises(ValueError):
            with bench.cpu_six(fake):
                assert fake.intra == 6
                raise ValueError("declared failure")
    else:
        with bench.cpu_six(fake): assert fake.intra == 6
    assert fake.intra == 4 and fake.interop == 1 and fake.mutations == [6, 4]


def test_unknown_interop_rejected_before_any_thread_setting():
    fake = FakeTorch(interop=96)
    with pytest.raises(RuntimeError, match="interop1"):
        with bench.cpu_six(fake): pytest.fail("must not enter")
    assert fake.mutations == []


def test_declaration_has_no_builder_provider_or_timing_calls_and_execute_does_not_rewarm():
    tree = ast.parse(bench.HERE.read_text())
    declaration = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "declare")
    calls = [ast.unparse(node.func) for node in ast.walk(declaration) if isinstance(node, ast.Call)]
    assert not any("build_runtime_tensors" in name or "BatchedImageTensorizer" in name or "paired_calls" in name for name in calls)
    assert "set_num_interop_threads" not in bench.HERE.read_text()
    all_calls = [ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)]
    for forbidden in ("torch.load", "np.load", "Image.open", "torch.cuda", "load_runtime_weight_bundle", "subprocess.run"):
        assert not any(name == forbidden or name.startswith(forbidden + ".") for name in all_calls)


def test_no_undeclared_wrong_mode_or_missing_execution_SHA_can_dispatch(monkeypatch):
    monkeypatch.setattr(bench, "declare", lambda: pytest.fail("unexpected declaration"))
    monkeypatch.setattr(bench, "execute", lambda value: pytest.fail("unexpected execution"))
    with pytest.raises(RuntimeError, match="execution requires"):
        bench.main(["--mode", "execute"])
    with pytest.raises(RuntimeError, match="declaration cannot"):
        bench.main(["--mode", "declare", "--expected-declaration-sha256", "a" * 64])


def test_synthetic_input_snapshot_denies_timestamp_or_mutable_payload_retagging(modules):
    _, _, _, runtime = modules
    config, rows = bench.fixtures()
    pattern, inputs = rows[0]
    changed = replace(inputs, camera_frames=(replace(inputs.camera_frames[0], timestamp_ns=2),) + inputs.camera_frames[1:])
    with pytest.raises(RuntimeError, match="immutable synthetic"):
        bench.fixture_snapshot(config, [(pattern, changed)])


@pytest.mark.parametrize("fault", ["nonfinite_calibration", "overflow_history", "non_suffix_history_mask", "nonfinite_route"])
def test_derived_fixture_nonimage_rejections_are_exact_original_runtime_guards(modules, fault):
    candidate, torch, _, runtime = modules
    old = torch.get_num_threads()
    try:
        torch.set_num_threads(6)
        config, rows = bench.fixtures()
        inputs = rows[1][1]
        if fault == "nonfinite_calibration":
            values = [list(row) for row in inputs.calibration]; values[0][0] = float("nan")
            inputs = replace(inputs, calibration=tuple(tuple(row) for row in values))
        elif fault == "overflow_history":
            values = [list(row) for row in inputs.ego_history]; values[-1][2] = 1e308
            inputs = replace(inputs, ego_history=tuple(tuple(row) for row in values))
        elif fault == "non_suffix_history_mask":
            inputs = replace(inputs, ego_history_mask=(True, False) + (True,) * 8)
        else:
            inputs = replace(inputs, route_xy_base_m=((0., 0.), (float("nan"), 0.)))
        reasons = []
        for function in (runtime.build_runtime_tensors, candidate.BatchedImageTensorizer()):
            with pytest.raises(runtime.ContractError) as caught:
                function(inputs, config)
            reasons.append((type(caught.value), str(caught.value)))
        assert reasons[0] == reasons[1]
    finally:
        torch.set_num_threads(old)
