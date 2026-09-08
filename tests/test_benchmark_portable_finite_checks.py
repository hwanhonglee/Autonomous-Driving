"""HH_260906 - Prove finite-check boolean parity and owner boundaries without CUDA or actual learning."""

from collections import UserDict
import json
import math
import signal
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import benchmark_portable_finite_checks as module


@pytest.mark.parametrize("dtype", [torch.float16, torch.float32, torch.float64, torch.complex64, torch.complex128])
@pytest.mark.parametrize("special", [0., float("nan"), float("inf"), -float("inf")])
def test_all_floating_complex_finite_semantics(dtype, special):
    tensor = torch.tensor([special, 1.], dtype=dtype)
    value = UserDict({"nested": [(), {"x": tensor}, torch.empty(0, dtype=dtype)]})
    before = module.fingerprint(value)
    assert module.grouped_tensors_are_finite(value) == module._nested_tensors_are_finite(value) == math.isfinite(special)
    assert module.fingerprint(value) == before


@pytest.mark.parametrize("value", [None, float("nan"), float("inf"), "value", [], (), {},
    UserDict({"x": [float("nan"), ()]}), {"set_ignored": {float("nan")}},
    torch.tensor([True]), torch.tensor([1, -3], dtype=torch.int64), torch.empty(0)])
def test_ignored_non_tensor_integer_empty_semantics(value):
    assert module.grouped_tensors_are_finite(value) == module._nested_tensors_are_finite(value) is True


def test_complex_imaginary_nonfinite_is_checked():
    for value in (complex(1, float("nan")), complex(1, float("inf"))):
        tensor = torch.tensor([value], dtype=torch.complex64)
        assert module.grouped_tensors_are_finite(tensor) is module._nested_tensors_are_finite(tensor) is False


def test_exact_frozen47shape_parameter_count_and_cpu_scalar_boundaries():
    assert len(module.PARAMETER_SHAPES) == 47 and sum(math.prod(s) for s in module.PARAMETER_SHAPES) == 954590
    value = module.synthetic_state(torch.device("cpu"))
    leaves = [t for t in module.tensor_leaves(value) if torch.is_floating_point(t) or torch.is_complex(t)]
    assert len(leaves) == 188
    assert len(value["model"]) == len(value["optimizer_like"]["state"]) == 47
    assert all(item["step"].device.type == "cpu" and item["step"].ndim == 0 for item in value["optimizer_like"]["state"])
    assert module._nested_tensors_are_finite(value) and module.grouped_tensors_are_finite(value)


def arguments(tmp_path, **changes):
    values = dict(device="cpu", output_dir=tmp_path / "benchmark", max_wall_seconds=30., expected_source_commit=None,
                  finish_before_utc=None, repo=None, expected_worker_sha256=None)
    return SimpleNamespace(**dict(values, **changes))


@pytest.fixture
def tiny(tmp_path, monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    monkeypatch.setattr(module, "PARAMETER_SHAPES", ((2, 3), (4,)))
    monkeypatch.setattr(module, "WARMUP", 1); monkeypatch.setattr(module, "REPEATS", 2)
    def forbidden(*_a, **_k): raise AssertionError("CPU benchmark must not initialize CUDA")
    for name in ("device_count", "get_device_properties", "synchronize", "Event"):
        monkeypatch.setattr(module.torch.cuda, name, forbidden)
    return arguments(tmp_path)


def test_cpu_complete_abba_timing_integrity_and_restoration(tiny):
    before_threads = torch.get_num_threads()
    before_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    result = module.run(tiny)
    assert result["status"] == "COMPLETE" and result["completed_measurement_count"] == result["expected_measurement_count"] == 8
    assert [r["method"] for r in result["measurements"]] == list(module.ORDER) * 2
    assert result["input_sha256_before"] == result["input_sha256_after"]
    assert result["source_sha256"] == result["source_sha256_after"] and len(result["parity"]) == 12
    counts = result["all_finite_workload_scalar_reductions"]
    assert counts["recursive_item_calls"] == 8 and counts["grouped_item_calls"] == 2
    assert counts["recursive_item_calls_by_device"] == {"cpu": 8}
    assert all(r["cuda_event_elapsed_ms"] is None for r in result["measurements"])
    assert torch.get_num_threads() == before_threads
    assert all(signal.getsignal(s) == h for s, h in before_handlers.items())
    assert result["model_training"] is result["dataset_read"] is result["production_guard_changed"] is False
    for line in (tiny.output_dir / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ")
        assert name != "SHA256SUMS" and module.ownership.digest(tiny.output_dir / name) == digest


@pytest.mark.parametrize("fault", ["timeout", "signal", "input_changed", "postcheck_error", "source_changed"])
def test_partial_failure_retained_no_complete(tiny, monkeypatch, fault):
    original = module.measure
    observed = []
    def measure(name, value, device):
        observed.append(name)
        if fault == "timeout": raise TimeoutError("test budget")
        if fault == "signal": signal.raise_signal(signal.SIGTERM)
        if fault == "input_changed": value["model"]["0"].add_(1)
        return original(name, value, device)
    monkeypatch.setattr(module, "measure", measure)
    if fault == "postcheck_error":
        actual = module.fingerprint; count = []
        def fingerprint(value):
            count.append(1)
            if len(count) > 1: raise RuntimeError("test postcheck error")
            return actual(value)
        monkeypatch.setattr(module, "fingerprint", fingerprint)
    if fault == "source_changed":
        actual = module.source_identity; count = []
        def sources():
            value = actual(); count.append(1)
            if len(count) > 1: value["benchmark_worker.py"] = "f" * 64
            return value
        monkeypatch.setattr(module, "source_identity", sources)
    result = module.run(tiny)
    assert result["status"] == "PARTIAL" and (tiny.output_dir / "summary.json").is_file()
    assert not result["overall_training_speedup_established"]


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True, 301])
def test_invalid_budget_no_output(tmp_path, monkeypatch, value):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    args = arguments(tmp_path, max_wall_seconds=value)
    with pytest.raises(ValueError, match="wall budget"):
        module.run(args)
    assert not args.output_dir.exists()


def test_cpu_visibility_required(tmp_path, monkeypatch):
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    with pytest.raises(ValueError, match="CPU benchmark requires"):
        module.validate_options(arguments(tmp_path))


def test_preserved_model_then_optimizer_guard_call_order(monkeypatch):
    observed = []
    first, second = object(), object()
    def check(value): observed.append(value); return value is first
    monkeypatch.setattr(module, "_nested_tensors_are_finite", check)
    with pytest.raises(ValueError, match="unexpectedly failed"):
        module.measure("recursive", {"model": first, "optimizer_like": second}, torch.device("cpu"))
    assert observed == [first, second]


@pytest.fixture
def gpu_fixture(tmp_path, monkeypatch):
    # HH_260906 - Mock GPU identity queries only; the test creates one private advisory lock and no CUDA context.
    workspace = tmp_path / "personal/hwanhong/portable_e2e"
    repo = workspace / "autoware_e2e"; repo.mkdir(parents=True)
    lease = workspace / "runs/campaigns/.gpu0_training.lock"; lease.parent.mkdir(parents=True); lease.touch()
    monkeypatch.setattr(module, "ROOT", repo); monkeypatch.setattr(module.ownership, "WORKSPACE", workspace)
    monkeypatch.setattr(module.sys, "prefix", str(workspace / "venvs/py312"))
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", module.GPU_UUID)
    monkeypatch.setenv("PYTHONPATH", str(repo)); monkeypatch.setenv("PYTHONNOUSERSITE", "1")
    for key in ("PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP"): monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(module.inspect, "getsourcefile", lambda _: str(repo / "portable_e2e/train.py"))
    args = arguments(tmp_path, device="cuda:0", expected_source_commit="a" * 40, finish_before_utc=module.DEADLINE,
        repo=repo, expected_worker_sha256=module.ownership.digest(module.Path(module.__file__)))
    calls = []
    monkeypatch.setattr(module.ownership, "run_inventory", lambda command, _: "a" * 40 if command[1] == "rev-parse" else "")
    monkeypatch.setattr(module.ownership, "assert_gpu_idle", lambda _: calls.append("idle"))
    monkeypatch.setattr(torch.cuda, "device_count", lambda: calls.append("cuda_count") or 1)
    monkeypatch.setattr(torch.cuda, "get_device_properties", lambda _: SimpleNamespace(uuid=module.GPU_UUID, name="synthetic fixture GPU"))
    return args, calls


def test_gpu_idle_and_lease_before_cuda_initialization(gpu_fixture):
    args, calls = gpu_fixture
    with module.device_lease(args) as record:
        assert calls == ["idle", "cuda_count"]
        assert record["physical_gpu_index"] == 0 and record["physical_gpu_uuid"] == module.GPU_UUID


@pytest.mark.parametrize("fault", ["visibility", "venv", "worker_sha", "repo", "source", "uuid", "occupied_lease", "pythonpath", "loaded_guard"])
def test_gpu_scope_and_ownership_fail_closed(gpu_fixture, monkeypatch, fault):
    args, calls = gpu_fixture
    if fault == "visibility": monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    elif fault == "venv": monkeypatch.setattr(module.sys, "prefix", "/not-the-personal-venv")
    elif fault == "worker_sha": args.expected_worker_sha256 = "f" * 64
    elif fault == "repo": args.repo = None
    elif fault == "source": monkeypatch.setattr(module.ownership, "run_inventory", lambda *_: "bad")
    elif fault == "uuid": monkeypatch.setattr(torch.cuda, "get_device_properties", lambda _: SimpleNamespace(uuid="other", name="fixture"))
    elif fault == "pythonpath": monkeypatch.setenv("PYTHONPATH", "/unreviewed/module/path")
    elif fault == "loaded_guard": monkeypatch.setattr(module.inspect, "getsourcefile", lambda _: "/unreviewed/train.py")
    else:
        def locked(*_): raise BlockingIOError("fixture lease occupied")
        monkeypatch.setattr(module.fcntl, "flock", locked)
    with pytest.raises((ValueError, BlockingIOError)):
        with module.device_lease(args): pytest.fail("invalid GPU scope entered")
    if fault not in ("uuid",): assert not calls


def test_output_reuse_or_dataset_path_forbidden(tiny, monkeypatch, tmp_path):
    tiny.output_dir.mkdir()
    with pytest.raises(ValueError, match="fresh"): module.run(tiny)
    repo = tmp_path / "repo"; repo.mkdir(); physical = tmp_path / "physical"; physical.mkdir()
    (repo / "datasets").symlink_to(physical, target_is_directory=True)
    monkeypatch.setattr(module, "ROOT", repo); tiny.output_dir = physical / "benchmark"
    with pytest.raises(ValueError, match="outside datasets"): module.run(tiny)


@pytest.mark.parametrize("fault", ["short", "empty", "missing", "duplicate", "nan", "negative", "result", "event", "unterminated"])
def test_final_persisted_journal_corruption_never_claims_complete(tiny, monkeypatch, fault):
    # HH_260906 - Corrupt only a synthetic test journal after the final timed call, exactly where the former completion gap occurred.
    original, calls = module.source_identity, []
    def sources():
        result = original(); calls.append(1)
        if len(calls) == 2:
            path = tiny.output_dir / "measurements.jsonl"
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            if fault == "missing": path.unlink()
            elif fault == "unterminated": path.write_text(path.read_text().rstrip("\n"))
            else:
                if fault == "short": rows = rows[:-1]
                elif fault == "empty": rows = []
                elif fault == "duplicate": rows[-1] = rows[-2]
                elif fault == "nan": rows[-1]["function_host_ms"] = float("nan")
                elif fault == "negative": rows[-1]["drained_wall_ms"] = -1
                elif fault == "result": rows[-1]["result"] = False
                elif fault == "event": rows[-1]["cuda_event_elapsed_ms"] = .1
                path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        return result
    monkeypatch.setattr(module, "source_identity", sources)
    result = module.run(tiny)
    assert result["status"] == "PARTIAL" and result["journal_errors"]
    assert result["completed_measurement_count"] < result["expected_measurement_count"] == 8
    assert (tiny.output_dir / "summary.json").is_file()


def test_gpu_persisted_timing_validation_keeps_all80_and_rejects_nonfinite(monkeypatch):
    monkeypatch.setattr(module, "REPEATS", 20)
    rows = [{"block": i // 4, "position": i % 4, "method": module.ORDER[i % 4], "result": True,
             "function_host_ms": 1., "drained_wall_ms": 2., "cuda_event_elapsed_ms": 1.5} for i in range(80)]
    assert module.validate_measurements(rows, "cuda:0") == (rows, [])
    for invalid in (None, float("nan"), float("inf"), -1., True):
        rows[-1]["cuda_event_elapsed_ms"] = invalid
        valid, errors = module.validate_measurements(rows, "cuda:0")
        assert len(valid) == 79 and errors
