"""HH_260906 - Exercise decoder-only oracle plumbing with tiny synthetic tensors and no real-data optimization."""

import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import probe_portable_decoder_representability as probe


def sample(index=0, trial="run_001"):
    return {"anchor_index": index, "sample_id": f"{trial}:{index}", "trial_id": trial,
        "frame": index, "anchor_timestamp_ns": index * 100_000_000, "capture_phase": "driving",
        "current_vx_mps": 1.0, "route_xy": [[float(i), 0.] for i in range(101)],
        "target_xy": [[(i + 1) * .1, 0.] for i in range(64)], "target_speed": [1.] * 64,
        "target_yaw": [0.] * 64, "valid_mask": [True] * 64, "original_raw_violating_step_counts": {"xy_curvature": 1}}


def test_frozen_model_and_gate_sources_and_full_configuration():
    identity = probe.source_identity()
    assert identity["files"]["portable_e2e/model.py"] == probe.MODEL_SHA
    assert identity["files"]["portable_e2e/runtime_contract.py"] == probe.RUNTIME_SHA
    assert identity["model_config"] == probe.CONFIG.to_dict()
    assert probe.PLAN["iterations"] == 512 and probe.PLAN["batch_size"] == 256
    assert probe.PLAN["candidate_count"] == 6 and probe.PLAN["learning_rate"] == .05


def test_decoder_calls_existing_unbound_implementation_without_model_construction(monkeypatch):
    monkeypatch.setattr(probe.model.PerspectiveTrajectoryModel, "__init__", lambda *args: pytest.fail("learned model instantiated"))
    history, route, mask, _, _ = probe.tensor_batch([sample()], "cpu")
    latent = probe.initial_latents([0])
    expected = probe.model.PerspectiveTrajectoryModel._decode_physical_v1(SimpleNamespace(config=probe.CONFIG), latent, history, route, mask)
    actual = probe.decode(latent, history, route, mask)
    assert all(torch.equal(a, b) for a, b in zip(actual, expected))


def test_decoder_history_stub_uses_only_actual_latest_vx_and_route_prefix():
    history, route, mask, _, _ = probe.tensor_batch([sample()], "cpu")
    assert tuple(history.shape) == (1, 10, 13)
    assert history.count_nonzero() == 1 and history[0, -1, 1] == 1.
    assert mask[0].tolist() == [True] * 101 + [False] * 27
    assert not any(t.requires_grad for t in (history, route, mask))


def test_initializations_are_batch_independent_and_do_not_pollute_global_rng():
    before = torch.random.get_rng_state().clone()
    together = probe.initial_latents([3, 4])
    separate = torch.cat([probe.initial_latents([i]) for i in (3, 4)])
    assert torch.equal(together, separate) and torch.equal(before, torch.random.get_rng_state())
    assert together.dtype == torch.float32 and together[:, 0].count_nonzero() == 0
    assert not torch.equal(together[0, 1], together[0, 2])


def test_objective_has_declared_units_and_independent_candidate_reduction():
    xy, speed = torch.ones(2, 6, 64, 2), torch.ones(2, 6, 64)
    result = probe.objective(xy, speed, torch.zeros(2, 64, 2), torch.zeros(2, 64))
    assert tuple(result.shape) == (2, 6) and torch.equal(result, torch.full((2, 6), 3.))


def test_tiny_optimization_is_reproducible_and_preserves_inputs_and_raw_failure():
    rows = [sample()]
    before = copy.deepcopy(rows)
    history = []
    first = probe.optimize_batch(rows, "cpu", lambda: False, history.append, _test_iterations=3)
    second = probe.optimize_batch(rows, "cpu", lambda: False, lambda x: None, _test_iterations=3)
    assert first["status"] == "COMPLETE" and first["iterations_completed"] == 3
    assert len(history) == 4 and [r["iteration"] for r in history] == list(range(4))
    assert all(torch.equal(first[key], second[key]) for key in ("latent", "xy", "speed", "final_objective"))
    assert rows == before
    report = probe.candidate_reports(rows, first)[0]
    assert len(report["candidates"]) == 6 and report["original_raw_violating_step_counts"] == {"xy_curvature": 1}
    assert all(len(c["xy_error_by_step_m"]) == len(c["optimized_raw_latents"]) == 64 for c in report["candidates"])


def test_only_latent_tensor_is_registered_with_optimizer(monkeypatch):
    original, parameters = torch.optim.Adam, []
    def checked(values, **kwargs):
        values = list(values); parameters.extend(values)
        return original(values, **kwargs)
    monkeypatch.setattr(torch.optim, "Adam", checked)
    probe.optimize_batch([sample()], "cpu", lambda: False, lambda _: None, _test_iterations=1)
    assert len(parameters) == 1 and tuple(parameters[0].shape) == (1, 6, 64, 2)
    assert parameters[0].is_leaf and parameters[0].grad is not None


def test_deadline_before_first_iteration_retains_unoptimized_candidates_and_marks_partial():
    result = probe.optimize_batch([sample()], "cpu", lambda: True, lambda _: pytest.fail("late progress"), _test_iterations=3)
    assert result["status"] == "PARTIAL_OR_FAILED" and result["iterations_completed"] == 0
    reports = probe.candidate_reports([sample()], result)
    summary = probe.summarize_reports(reports)
    assert len(reports) == 1 and len(reports[0]["candidates"]) == 6
    assert summary["initialization_0"]["metrics"]["initial_objective_m2"]["missing_count"] == 1
    assert summary["initialization_0"]["metrics"]["ade_m"]["count"] == 1


def test_nonfinite_objective_is_failed_not_complete(monkeypatch):
    monkeypatch.setattr(probe, "objective", lambda *args: torch.full((1, 6), math.nan))
    result = probe.optimize_batch([sample()], "cpu", lambda: False, lambda _: None, _test_iterations=1)
    assert result["status"] == "PARTIAL_OR_FAILED" and result["failure"] == "nonfinite_objective"
    reports = probe.candidate_reports([sample()], result)
    assert len(reports) == 1 and len(reports[0]["candidates"]) == 6
    assert all(c["numeric_status"] == "NONFINITE_UNAVAILABLE" for c in reports[0]["candidates"])
    assert reports[0]["oracle_objective_minimum_initialization_index"] is None
    assert probe.summarize_reports(reports)["future_aware_best_final_objective_of_six"]["unavailable_all_initializations_anchor_count"] == 1


def test_one_nonfinite_candidate_does_not_erase_other_five_initializations():
    result = probe.optimize_batch([sample()], "cpu", lambda: False, lambda _: None, _test_iterations=1)
    result["xy"][0, 0, 0, 0] = math.nan
    result["terminal_values_finite"] = False
    reports = probe.candidate_reports([sample()], result)
    assert len(reports) == 1 and len(reports[0]["candidates"]) == 6
    assert reports[0]["candidates"][0]["numeric_status"] == "NONFINITE_UNAVAILABLE"
    assert all(c["numeric_status"] == "FINITE" for c in reports[0]["candidates"][1:])
    assert probe.summarize_reports(reports)["initialization_0"]["metrics"]["ade_m"]["missing_count"] == 1
    assert probe.summarize_reports(reports)["initialization_1"]["metrics"]["ade_m"]["count"] == 1
    probe.encoded(reports)


@pytest.mark.parametrize("change", [lambda r: r["valid_mask"].__setitem__(1, False),
    lambda r: r.update(current_vx_mps=math.nan), lambda r: r.update(route_xy=[[0., 0.]]),
    lambda r: r["target_xy"][0].__setitem__(0, math.inf), lambda r: r["target_speed"].pop()])
def test_bad_input_is_not_filtered_or_repaired(change):
    row = sample(); change(row)
    with pytest.raises((ValueError, RuntimeError)):
        probe.tensor_batch([row], "cpu")


def test_every_candidate_gate_failure_is_preserved(monkeypatch):
    result = probe.optimize_batch([sample()], "cpu", lambda: False, lambda _: None, _test_iterations=1)
    calls = []
    def reject(xy, speed, logits, *, current_speed_mps):
        calls.append((logits, current_speed_mps))
        raise probe.raw.contract.ContractError("deliberate unchanged gate failure")
    monkeypatch.setattr(probe.runtime, "validate_and_select_trajectory", reject)
    reports = probe.candidate_reports([sample()], result)
    assert len(calls) == 6 and all(c["output_runtime_gate"]["status"] == "FAIL" for c in reports[0]["candidates"])
    assert [values[0].index(1.) for values in calls] == list(range(6))
    assert all(values[1] == 1. for values in calls)


def test_distribution_uses_all_values_and_descriptive_thresholds_are_not_admission():
    assert probe.distribution([4., 1., 2., 3.]) == {"count": 4, "mean": 2.5, "min": 1., "p50": 2.5, "p90": 3.7, "p99": 3.9699999999999998, "max": 4.}
    assert probe.SCOPE["training_data_approved"] is False and probe.SCOPE["new_admission_threshold"] is False


def synthetic_original():
    states = [dict(frame=i, timestamp=i * .05, x=i * .05, y=0., z=0., yaw=0., vx=1., vy=0., ax=0., ay=0., yaw_rate=0., capture_phase="driving") for i in range(131)]
    timeline = probe.raw.measured_timeline(states)
    camera = {"timestamp": 0., "frame": 0, "capture_phase": "driving"}
    route = {"route_length_m": 100., "route": [{"x": 0., "y": 0.}, {"x": 100., "y": 0.}]}
    _, records = probe.raw.measured_future_audit(timeline, [camera], route, [{"timestamp_ns": 0, "same_recorded_frame_and_timestamp": True}])
    return timeline, camera, route, json.loads(json.dumps(records[0]))


def test_pure_extraction_reuses_measured_future_and_retains_original_failed_metric():
    timeline, camera, route, original = synthetic_original()
    original["diagnostic"]["steps"][3]["metrics"]["xy_curvature"][1] = True
    result = probe.extract_anchor(timeline, camera, route, original)
    assert result["target_xy"][-1] == pytest.approx([6.4, 0.]) and result["target_speed"] == [1.] * 64
    assert result["valid_mask"] == [True] * 64 and result["original_raw_violating_step_counts"]["xy_curvature"] == 1


def test_future_gap_and_old_ledger_disagreement_are_not_repaired():
    timeline, camera, route, original = synthetic_original()
    with pytest.raises(ValueError, match="missing"):
        probe.extract_anchor(timeline[:40], camera, route, original)
    original["diagnostic"]["steps"][1]["metrics"]["speed_limit"][0] += 1.
    with pytest.raises(ValueError, match="differs"):
        probe.extract_anchor(timeline, camera, route, original)


@pytest.mark.parametrize("budget", [0., -1., math.inf, math.nan, True])
def test_bad_wall_budget_is_rejected_before_real_input_access(tmp_path, budget):
    with pytest.raises(ValueError, match="wall budget"):
        probe.run_probe(tmp_path, tmp_path / "new", "0" * 64, probe.raw.scalar.sha(Path(probe.__file__)), "cpu", budget)


def test_cpu_device_requires_hidden_gpu_and_never_initializes_cuda(tmp_path, monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    monkeypatch.setattr(torch.cuda, "is_available", lambda: pytest.fail("CPU preflight queried GPU"))
    with pytest.raises(ValueError, match="hide CUDA"):
        probe.run_probe(tmp_path, tmp_path / "new", "0" * 64, probe.raw.scalar.sha(Path(probe.__file__)), "cpu", 1.)


@pytest.mark.parametrize("visible", ["0", "1", "0,1", "", "GPU-unknown"])
def test_gpu0_guard_rejects_other_or_multiple_devices_before_allocation(tmp_path, monkeypatch, visible):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", visible)
    monkeypatch.setattr(torch.cuda, "is_available", lambda: pytest.fail("wrong-device preflight queried GPU"))
    with pytest.raises(ValueError, match="physical GPU0"):
        probe.run_probe(tmp_path, tmp_path / "new", "0" * 64, probe.raw.scalar.sha(Path(probe.__file__)), "cuda:0", 1.)


@pytest.mark.parametrize("wrong_uuid", [None, "GPU-00000000-0000-0000-0000-000000000000"])
def test_cuda_property_uuid_must_confirm_exact_assigned_device(tmp_path, monkeypatch, wrong_uuid):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", probe.ASSIGNED_GPU_UUID)
    monkeypatch.setattr(torch.cuda, "is_available", lambda: True)
    monkeypatch.setattr(torch.cuda, "device_count", lambda: 1)
    monkeypatch.setattr(torch.cuda, "get_device_properties", lambda index: SimpleNamespace(uuid=wrong_uuid))
    with pytest.raises(ValueError, match="UUID"):
        probe.run_probe(tmp_path, tmp_path / "new", "0" * 64, probe.raw.scalar.sha(Path(probe.__file__)), "cuda:0", 1.)


def test_uuid_normalization_accepts_only_equivalent_identifier_formats():
    expected = "59f374a453f5c05034b256aab0e3c7e5"
    assert probe.normalized_gpu_uuid(probe.ASSIGNED_GPU_UUID) == expected
    assert probe.normalized_gpu_uuid(expected) == expected
    assert probe.normalized_gpu_uuid(bytes.fromhex(expected)) == expected


def test_extractor_rejects_visible_or_already_initialized_cuda_before_inputs(tmp_path, monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", probe.ASSIGNED_GPU_UUID)
    with pytest.raises(ValueError, match="CPU extraction"):
        probe.extract(tmp_path, tmp_path, tmp_path / "new", "0" * 64, probe.raw.scalar.sha(Path(probe.__file__)))


def test_extraction_and_probe_outputs_are_create_only_and_outside_inputs(tmp_path):
    with pytest.raises(ValueError): probe.fresh(tmp_path, [])
    with pytest.raises(ValueError): probe.fresh(tmp_path / "new", [tmp_path])
    (tmp_path / "link").symlink_to(tmp_path / "missing")
    with pytest.raises(ValueError): probe.fresh(tmp_path / "link", [])


def test_cli_has_no_optimizer_iteration_override_or_abbreviations():
    for args in (["probe", "--iterations", "1"], ["probe", "--max-wall", "1"]):
        with pytest.raises(SystemExit): probe.main(args)


@pytest.fixture
def mocked_full_job(tmp_path, monkeypatch):
    # HH_260906 - Only orchestration is exercised at the full count; every optimizer call is replaced with synthetic records.
    inputs, output = tmp_path / "inputs", tmp_path / "output"
    inputs.mkdir()
    rows = [sample(i, "run_001" if i < 671 else "run_002") for i in range(1337)]
    (inputs / "inputs.jsonl").write_bytes(b"\n".join(probe.encoded(row) for row in rows) + b"\n")
    identity = {"fixture_only": "no model or source execution"}
    monkeypatch.setattr(probe, "source_identity", lambda: identity)
    manifest = {"schema": "portable_e2e.decoder_oracle_inputs.v1", "input_count": 1337,
        "counts_by_trial": probe.EXPECTED_COUNTS, "inputs_sha256": probe.raw.scalar.sha(inputs / "inputs.jsonl"),
        "source_identity": identity, "plan": probe.PLAN, "scope": probe.SCOPE}
    probe.write_json(inputs / "manifest.json", manifest)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    seen = []
    def fake_optimize(rows, device, stop, progress):
        seen.append((len(rows), rows[0]["anchor_index"]))
        progress({"iteration": 512, "optimizer_steps_completed": 512})
        return {"status": "COMPLETE", "failure": None, "iterations_completed": 512,
            "initial_latent_sha256": "a" * 64, "terminal_values_finite": True}
    monkeypatch.setattr(probe, "optimize_batch", fake_optimize)
    def fake_reports(rows, result):
        return [{"sample_id": row["sample_id"], "optimization_status": result["status"], "anchor_index": row["anchor_index"]} for row in rows]
    monkeypatch.setattr(probe, "candidate_reports", fake_reports)
    monkeypatch.setattr(probe, "summarize_reports", lambda reports: {"mocked_count": len(reports)})
    args = (inputs, output, probe.raw.scalar.sha(inputs / "manifest.json"), probe.raw.scalar.sha(Path(probe.__file__)), "cpu", 10.)
    return args, seen


def test_full_count_orchestration_includes_final_partial_size_batch_without_real_optimization(mocked_full_job):
    args, seen = mocked_full_job
    result = probe.run_probe(*args)
    assert result["status"] == "COMPLETE_NOT_ADMITTED" and result["reported_anchor_count"] == 1337
    assert result["completed_batches"] == 6 and seen == [(256, 0), (256, 256), (256, 512), (256, 768), (256, 1024), (57, 1280)]
    for line in (args[1] / "SHA256SUMS").read_text().splitlines():
        value, name = line.split("  ")
        assert probe.raw.scalar.sha(args[1] / name) == value


def test_exception_retains_failed_last_batch_and_never_calls_it_complete(mocked_full_job, monkeypatch):
    args, _ = mocked_full_job
    def broken(rows, device, stop, progress):
        progress({"iteration": 7, "optimizer_steps_completed": 7})
        raise RuntimeError("synthetic failure")
    monkeypatch.setattr(probe, "optimize_batch", broken)
    result = probe.run_probe(*args)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED" and result["reported_anchor_count"] == 0
    assert result["batches"][0]["iterations_completed"] == 7
    assert result["batches"][0]["unreported_anchor_indices"] == list(range(256))
    assert result["unreported_anchor_count"] == 1337


def test_input_postcheck_failure_overrides_otherwise_complete_job(mocked_full_job, monkeypatch):
    args, _ = mocked_full_job
    original = probe.optimize_batch
    def mutate(rows, device, stop, progress):
        result = original(rows, device, stop, progress)
        if rows[0]["anchor_index"] == 1280:
            (args[0] / "inputs.jsonl").write_text("changed")
        return result
    monkeypatch.setattr(probe, "optimize_batch", mutate)
    result = probe.run_probe(*args)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED" and not result["source_and_input_postcheck_pass"]


def test_signal_requests_partial_result_and_restores_original_handlers(mocked_full_job, monkeypatch):
    args, _ = mocked_full_job
    old = probe.signal.getsignal(probe.signal.SIGTERM)
    def signaled(rows, device, stop, progress):
        probe.signal.getsignal(probe.signal.SIGTERM)(probe.signal.SIGTERM, None)
        assert stop()
        return {"status": "PARTIAL_OR_FAILED", "failure": "deadline_or_signal", "iterations_completed": 2,
            "initial_latent_sha256": "a" * 64, "terminal_values_finite": True}
    monkeypatch.setattr(probe, "optimize_batch", signaled)
    result = probe.run_probe(*args)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED" and result["received_signal"] == "SIGTERM"
    assert result["reported_anchor_count"] == 256 and probe.signal.getsignal(probe.signal.SIGTERM) == old


def test_startup_write_failure_always_restores_signal_handlers(mocked_full_job, monkeypatch):
    args, _ = mocked_full_job
    old = {s: probe.signal.getsignal(s) for s in (probe.signal.SIGINT, probe.signal.SIGTERM)}
    original = probe.write_json
    def fail_started(path, value):
        if path.name == "started.json":
            raise OSError("synthetic startup write failure")
        return original(path, value)
    monkeypatch.setattr(probe, "write_json", fail_started)
    result = probe.run_probe(*args)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED" and result["reported_anchor_count"] == 0
    assert all(probe.signal.getsignal(s) == handler for s, handler in old.items())


def test_mid_batch_serialization_failure_reconciles_persisted_anchor_count(mocked_full_job, monkeypatch):
    args, _ = mocked_full_job
    original = probe.encoded
    def fail_third(value):
        if isinstance(value, dict) and value.get("anchor_index") == 2 and "sample_id" in value:
            raise ValueError("synthetic third-row serialization failure")
        return original(value)
    monkeypatch.setattr(probe, "encoded", fail_third)
    result = probe.run_probe(*args)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED" and result["reported_anchor_count"] == 2
    assert result["batches"][0]["reported_anchor_count"] == 2
    assert result["batches"][0]["unreported_anchor_indices"] == list(range(2, 256))
    assert len((args[1] / "per_anchor.jsonl").read_text().splitlines()) == 2


def test_truncated_detail_tail_is_disclosed_and_never_deleted(tmp_path):
    content = b'{"anchor_index":0}\n{"anchor_index":1}\n{"anchor_index":2'
    path = tmp_path / "per_anchor.jsonl"
    path.write_bytes(content)
    rows, details = probe.persisted_reports(tmp_path)
    assert len(rows) == 2 and details["incomplete_tail_bytes"] == len(b'{"anchor_index":2')
    assert path.read_bytes() == content
