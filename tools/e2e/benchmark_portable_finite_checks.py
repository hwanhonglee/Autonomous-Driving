#!/usr/bin/env python3
"""HH_260906 - Measure synthetic finite-check synchronization only; never train, load weights or change production guards."""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Mapping
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import time

import torch

from portable_e2e.train import _nested_tensors_are_finite
from scripts.e2e import run_portable_training_campaign as ownership

# HH_260906 - The worker may be staged outside Git; the explicitly verified imported ownership module locates the repository.
ROOT = Path(ownership.__file__).resolve().parents[2]
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
DEADLINE = "2026-09-09T01:00:00Z"
WARMUP, REPEATS = 5, 20
ORDER = ("recursive", "grouped", "grouped", "recursive")
# HH_260906 - Freeze CPU-inspected physical-v1 parameter shapes only; these are not learned tensor values.
SHAPE_SOURCE_COMMIT = "05c5847b5ad6c31ac46a713e35ba7fdc2edd10a0"
MODEL_SHA256 = "b72c0fcbaf558254a3e7b02aa90406ed157724a07f63d0ef9f46d0d444f92fc4"
CONFIG_PATH = "portable_e2e/config/perspective_trajectory_physical_v1.model.json"
CONFIG_SHA256 = "e96e31c96cafa41b57b67b9531ae9fff6bf21fd7e418ea78f9062fcb1dcfe74d"
PARAMETER_SHAPES = (
    (6,96), (24,3,3,3), (24,), (24,), (48,24,3,3), (48,), (48,), (96,48,3,3), (96,), (96,),
    (144,96,3,3), (144,), (144,), (96,2160), (96,), (96,16), (96,), (96,96), (96,),
    (192,576), (192,), (192,), (192,), (64,13), (64,), (192,64), (192,64), (192,), (192,),
    (96,2), (96,), (96,96), (96,), (288,96), (288,96), (288,), (288,), (256,352), (256,),
    (256,), (256,), (256,256), (256,), (768,256), (768,), (6,256), (6,),
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def tensor_leaves(value):
    # HH_260906 - Traverse exactly Mapping values and list/tuple elements; ignore all other non-tensor leaves like the baseline.
    if isinstance(value, torch.Tensor):
        yield value
    elif isinstance(value, Mapping):
        for child in value.values(): yield from tensor_leaves(child)
    elif isinstance(value, (list, tuple)):
        for child in value: yield from tensor_leaves(child)


def grouped_tensors_are_finite(value):
    # HH_260906 - Every floating/complex leaf receives the same isfinite/all check; only the host scalar reductions are grouped.
    groups = defaultdict(list)
    for tensor in tensor_leaves(value):
        if torch.is_floating_point(tensor) or torch.is_complex(tensor):
            require(tensor.layout == torch.strided and tensor.device.type in ("cpu", "cuda"),
                    "prototype supports dense CPU/CUDA tensors only")
            groups[tensor.device].append(torch.isfinite(tensor).all())
    return all(bool(torch.stack(values).all().item()) for values in groups.values())


def synthetic_state(device):
    # HH_260906 - Match reviewed shapes with fixed synthetic values, never a live model or real Adam state snapshot.
    parameters = [torch.linspace(-1, 1, math.prod(shape), dtype=torch.float32).reshape(shape).to(device)
                  for shape in PARAMETER_SHAPES]
    return {"model": {str(i): p for i, p in enumerate(parameters)},
        "optimizer_like": {"state": [{"step": torch.tensor(1540., dtype=torch.float32),
            "exp_avg": torch.full_like(p, .125), "exp_avg_sq": torch.full_like(p, .25)} for p in parameters],
            "ignored": {"integer": torch.tensor([1, 2], dtype=torch.int64), "bool": torch.tensor([True]),
                        "python_nan": float("nan"), "text": "unchanged baseline semantics", "empty": []}}}


def fingerprint(value):
    # HH_260906 - Hash all retained tensor bytes outside timed regions, including CPU step scalars and ignored integer tensors.
    digest = hashlib.sha256()
    for tensor in tensor_leaves(value):
        header = {"shape": list(tensor.shape), "dtype": str(tensor.dtype), "device": str(tensor.device)}
        digest.update(json.dumps(header, sort_keys=True).encode())
        cpu = tensor.detach().cpu().contiguous()
        # HH_260906 - NumPy preserves these bytes on old local Torch; bfloat16 uses an equal-width view without conversion.
        if cpu.dtype == torch.bfloat16: cpu = cpu.view(torch.int16)
        digest.update(cpu.numpy().tobytes())
    return digest.hexdigest()


def parity_cases(device):
    # HH_260906 - Nonfinite Python scalars are intentionally ignored exactly as the production tensor-only helper does.
    finite = torch.tensor([1., -2., 0.], device=device)
    cases = {"nested_finite": {"a": finite, "b": [(), {}, [], torch.tensor([], device=device)]},
        "nan_tensor": [finite, torch.tensor([float("nan")], device=device)],
        "positive_inf": {"x": torch.tensor([float("inf")], device=device)},
        "negative_inf": (torch.tensor([-float("inf")], device=device), finite),
        "complex_finite": torch.tensor([1 + 2j], dtype=torch.complex64, device=device),
        "complex_nan": torch.tensor([complex(1, float("nan"))], dtype=torch.complex64, device=device),
        "complex_inf": torch.tensor([complex(float("inf"), 1)], dtype=torch.complex128, device=device),
        "integer_boolean": [torch.tensor([1], dtype=torch.int64, device=device), torch.tensor([True], device=device)],
        "empty_nested": {"x": [], "y": ()}, "python_nonfinite_ignored": [float("nan"), float("inf"), None],
        "mixed_device_finite": [torch.ones(2), finite],
        "mixed_device_nonfinite": [finite, torch.tensor([float("nan")])]}
    results = []
    for name, value in cases.items():
        baseline, grouped = _nested_tensors_are_finite(value), grouped_tensors_are_finite(value)
        require(baseline == grouped, "finite-check semantic parity failure")
        results.append({"case": name, "recursive": baseline, "grouped": grouped, "equal": True})
    return results


def source_identity():
    names = ("portable_e2e/train.py", "portable_e2e/torch_dataset.py", "portable_e2e/model.py", CONFIG_PATH,
             "scripts/e2e/run_portable_training_campaign.py")
    return {**{name: ownership.digest(ROOT / name) for name in names},
            "benchmark_worker.py": ownership.digest(Path(__file__).resolve())}


def shape_reference_sources():
    # HH_260906 - Later model IDs cannot rewrite the historical CPU-inspected shape reference; never fetch Git objects.
    expected = {"portable_e2e/model.py": MODEL_SHA256, CONFIG_PATH: CONFIG_SHA256}
    for name, digest in expected.items():
        result = subprocess.run(["git", "-c", "protocol.allow=never", "show", f"{SHAPE_SOURCE_COMMIT}:{name}"],
            cwd=ROOT, env=dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0"),
            check=True, capture_output=True, timeout=15)
        require(hashlib.sha256(result.stdout).hexdigest() == digest, "historical shape-reference source changed")
    return expected


def utc():
    return datetime.now(timezone.utc).isoformat()


def validate_options(args):
    require(args.device in ("cpu", "cuda:0"), "only CPU or the one visible assigned GPU0 is permitted")
    require(type(args.max_wall_seconds) in (int, float) and math.isfinite(args.max_wall_seconds)
            and 0 < args.max_wall_seconds <= 300, "wall budget must be finite and in (0,300]")
    if args.device == "cpu":
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "CPU benchmark requires CUDA_VISIBLE_DEVICES='' explicitly")
        return
    require(getattr(args, "repo", None) is not None and Path(args.repo).absolute() == ROOT,
            "GPU worker requires the explicitly approved imported repository path")
    require(Path(inspect.getsourcefile(_nested_tensors_are_finite)).resolve() == ROOT / "portable_e2e/train.py",
            "loaded recursive guard must come from the pinned personal repository")
    require(os.environ.get("PYTHONPATH") == str(ROOT) and os.environ.get("PYTHONNOUSERSITE") == "1"
            and not any(os.environ.get(k) for k in ("PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")),
            "GPU worker requires exact repository PYTHONPATH and no inherited user Python environment")
    expected_worker = getattr(args, "expected_worker_sha256", None)
    require(isinstance(expected_worker, str) and re.fullmatch(r"[0-9a-f]{64}", expected_worker)
        and ownership.digest(Path(__file__).resolve()) == expected_worker, "GPU worker SHA differs from explicit pin")
    require(args.finish_before_utc == DEADLINE, "GPU benchmark requires the exact user deadline")
    require(isinstance(args.expected_source_commit, str) and re.fullmatch(r"[0-9a-f]{40}", args.expected_source_commit),
            "GPU benchmark requires a pinned source commit")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID, "GPU visibility must be the exact assigned physical GPU0 UUID")
    require(Path(sys.prefix) == ownership.WORKSPACE / "venvs/py312", "GPU benchmark requires the existing personal py312 venv")
    require(ROOT == ownership.WORKSPACE / "autoware_e2e", "GPU benchmark must run from the personal repository")
    remaining = (datetime.fromisoformat(DEADLINE.replace("Z", "+00:00")) - datetime.now(timezone.utc)).total_seconds()
    require(remaining >= args.max_wall_seconds + 60, "not enough time for benchmark plus finalization reserve")


@contextmanager
def device_lease(args):
    # HH_260906 - A cooperative lease and physical-ID idle check precede every CUDA initialization; never signal another process.
    validate_options(args)
    if args.device == "cpu":
        yield {"device": "cpu", "cuda_initialized_by_guard": False}
        return
    require(ownership.run_inventory(["git", "rev-parse", "HEAD"], ROOT) == args.expected_source_commit
        and not ownership.run_inventory(["git", "status", "--porcelain", "--untracked-files=all"], ROOT), "GPU source must be pinned and clean")
    path = ownership.WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), "existing training lease must be regular without symlinks")
    with path.open("a") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ownership.assert_gpu_idle(ROOT)
        require(torch.cuda.device_count() == 1, "exactly one CUDA device must be visible")
        props = torch.cuda.get_device_properties(0)
        observed = str(getattr(props, "uuid", "")).removeprefix("GPU-").lower()
        require(observed == GPU_UUID.removeprefix("GPU-").lower(), "Torch visible device UUID differs from assigned GPU0")
        try:
            yield {"device": "cuda:0", "physical_gpu_uuid": GPU_UUID, "physical_gpu_index": 0,
                   "device_name": props.name, "cooperative_lease": "runs/campaigns/.gpu0_training.lock"}
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


def measure(name, value, device):
    function = _nested_tensors_are_finite if name == "recursive" else grouped_tensors_are_finite
    gpu = device.type == "cuda"
    if gpu:
        torch.cuda.synchronize(device)
        begin, end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
        begin.record()
    started = time.perf_counter_ns()
    # HH_260906 - Preserve the two production call boundaries instead of exaggerating savings by merging both states.
    result = function(value["model"]) and function(value["optimizer_like"])
    returned = time.perf_counter_ns()
    if gpu:
        end.record(); end.synchronize()
    finished = time.perf_counter_ns()
    require(result is True, "all-finite timed workload unexpectedly failed")
    return {"method": name, "result": result, "function_host_ms": (returned - started) / 1e6,
            "drained_wall_ms": (finished - started) / 1e6,
            "cuda_event_elapsed_ms": begin.elapsed_time(end) if gpu else None}


def write_new(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write("\n")


def validate_measurements(rows, device):
    # HH_260906 - Completion requires the entire persisted ABBA schedule; retain invalid raw bytes without summarizing them as timings.
    expected, valid, errors = REPEATS * len(ORDER), [], []
    for index, row in enumerate(rows):
        try:
            require(isinstance(row, dict) and set(row) == {"block", "position", "method", "result",
                "function_host_ms", "drained_wall_ms", "cuda_event_elapsed_ms"}, "unexpected measurement fields")
            block, position = divmod(index, len(ORDER))
            require(index < expected and type(row["block"]) is int and type(row["position"]) is int
                and row["block"] == block and row["position"] == position and row["method"] == ORDER[position],
                "persisted measurement differs from fixed ABBA order")
            require(row["result"] is True, "timed finite check did not pass")
            for key in ("function_host_ms", "drained_wall_ms"):
                require(type(row[key]) in (int, float) and math.isfinite(row[key]) and row[key] >= 0,
                        f"invalid {key}")
            require(row["drained_wall_ms"] >= row["function_host_ms"], "drained wall time precedes function return")
            event = row["cuda_event_elapsed_ms"]
            require(event is None if device == "cpu" else
                type(event) in (int, float) and math.isfinite(event) and event >= 0, "invalid device event timing")
            valid.append(row)
        except (ValueError, TypeError, KeyError) as error:
            errors.append(f"row {index}: {error}")
            break
    if len(rows) != expected or len(valid) != expected:
        errors.append(f"expected {expected} ordered valid measurements; parsed {len(rows)}, valid prefix {len(valid)}")
    return valid, errors


def run(args):
    validate_options(args)
    output = Path(args.output_dir).absolute()
    require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents))
        and not output.resolve().is_relative_to((ROOT / "datasets").resolve()), "output must be fresh outside datasets")
    if args.device == "cuda:0":
        require(output.resolve().is_relative_to(ownership.WORKSPACE / "runs/diagnostics"), "GPU output must be inside personal runs/diagnostics")
    sources = source_identity()
    shape_sources = shape_reference_sources()
    record = {"schema": "portable_e2e.finite_check_benchmark.v1", "status": "PARTIAL", "started_at_utc": utc(),
        "source_sha256": sources, "expected_source_commit": args.expected_source_commit, "torch_version": str(torch.__version__),
        "synthetic_only": True, "model_training": False, "model_or_checkpoint_loaded": False, "dataset_read": False,
        "production_guard_changed": False, "overall_training_speedup_established": False,
        "plan": {"tensor_count": len(PARAMETER_SHAPES), "parameter_shapes": [list(shape) for shape in PARAMETER_SHAPES],
            "synthetic_parameter_elements": sum(math.prod(shape) for shape in PARAMETER_SHAPES), "moment_tensors_per_parameter": 2,
            "cpu_step_scalars_per_parameter": 1, "warmup_per_method": WARMUP, "repeated_abba_blocks": REPEATS,
            "timing_order": list(ORDER), "max_wall_seconds": args.max_wall_seconds, "threads": 4,
            "finish_before_utc": args.finish_before_utc}, "measurements": [], "parity": [],
        "shape_reference": {"source_commit": SHAPE_SOURCE_COMMIT, "model_source_sha256": MODEL_SHA256,
            "model_config_sha256": CONFIG_SHA256, "learned_values_copied": False,
            "offline_git_source_sha256": shape_sources,
            "optimizer_layout": "Synthetic noncapturable/nonfused Adam-style two device moments and one CPU float step per parameter."},
        "limitations": ["Synthetic values matching reviewed parameter shapes; not actual model/optimizer state, data loading, forward/backward or end-to-end training timing.",
            "Both methods return the same boolean on supported noncyclic pure containers; grouped checks do not reproduce early-exit traversal timing.",
            "Python scalar NaN/Inf and non-Mapping/list/tuple containers remain ignored, exactly as production does.",
            "CUDA event elapsed time includes device-timeline host-dispatch gaps; it is not summed kernel execution time.",
            "Wall budget is cooperative between bounded checks; the owner must also use an external 360-second process timeout for GPU execution."]}
    with device_lease(args) as device_info:
        output.mkdir(parents=True, exist_ok=False)
        write_new(output / "plan.json", {k: record[k] for k in ("schema", "started_at_utc", "source_sha256", "plan")})
        record["hardware"] = device_info
        start, stop = time.monotonic(), []
        handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        threads = torch.get_num_threads()
        def requested(signum, _frame): stop.append(signum)
        def check():
            if stop: raise InterruptedError(f"stop signal {stop[0]} received")
            if time.monotonic() - start >= args.max_wall_seconds: raise TimeoutError("cooperative benchmark wall budget exhausted")
        value = None
        try:
            for sig in handlers: signal.signal(sig, requested)
            torch.set_num_threads(4)
            device = torch.device(args.device)
            with torch.no_grad():
                check(); record["parity"] = parity_cases(device); check()
                value = synthetic_state(device); record["input_sha256_before"] = fingerprint(value); check()
                leaves = [t for t in tensor_leaves(value) if torch.is_floating_point(t) or torch.is_complex(t)]
                by_device = {str(d): sum(t.device == d for t in leaves) for d in {t.device for t in leaves}}
                grouped = sum(len({t.device for t in tensor_leaves(value[key])
                    if torch.is_floating_point(t) or torch.is_complex(t)}) for key in ("model", "optimizer_like"))
                record["all_finite_workload_scalar_reductions"] = {"recursive_item_calls": len(leaves),
                    "recursive_item_calls_by_device": by_device, "grouped_item_calls": grouped,
                    "preserved_call_boundaries": ["model", "optimizer_like"],
                    "every_floating_or_complex_tensor_checked": True,
                    "notice": "Code-path scalar-read counts, not independently measured CUDA fence or kernel counts; CPU reads are not GPU synchronizations."}
                for _ in range(WARMUP):
                    for method in ("recursive", "grouped"): check(); measure(method, value, device)
                with (output / "measurements.jsonl").open("x") as journal:
                    for block in range(REPEATS):
                        for position, method in enumerate(ORDER):
                            check(); row = {"block": block, "position": position, **measure(method, value, device)}
                            journal.write(json.dumps(row, allow_nan=False) + "\n"); journal.flush()
                            record["measurements"].append(row)
                check()
            record["status"] = "COMPLETE"
        except BaseException as error:
            record["error"] = f"{type(error).__name__}: {error}"
            record["status"] = "PARTIAL"
        finally:
            # HH_260906 - Repeated stop requests cannot bypass CPU evidence persistence or restoration of caller state.
            for sig in handlers: signal.signal(sig, signal.SIG_IGN)
            try:
                try:
                    if value is not None:
                        record["input_sha256_after"] = fingerprint(value)
                        if record["input_sha256_after"] != record.get("input_sha256_before"):
                            record.update(status="PARTIAL", integrity_error="synthetic input bytes changed")
                    record["source_sha256_after"] = source_identity()
                    if record["source_sha256_after"] != sources: record.update(status="PARTIAL", integrity_error="source bytes changed")
                except Exception as error:
                    record.update(status="PARTIAL", integrity_error=f"postcheck failed: {type(error).__name__}: {error}")
                if time.monotonic() - start >= args.max_wall_seconds:
                    record.update(status="PARTIAL", budget_error="wall budget exhausted before integrity finalization")
                journal_path = output / "measurements.jsonl"
                saved, tail_error = [], None
                if journal_path.exists():
                    # HH_260906 - Reconcile persisted complete records after interruption, never claim a partial row as a measurement.
                    for line in journal_path.read_bytes().splitlines(keepends=True):
                        try:
                            if not line.endswith(b"\n"): raise ValueError("unterminated journal row")
                            saved.append(json.loads(line))
                        except (ValueError, UnicodeError) as error:
                            tail_error = f"{type(error).__name__}: {error}"
                            break
                else:
                    tail_error = "measurement journal is missing"
                record["persisted_complete_json_record_count"] = len(saved)
                record["measurements"], journal_errors = validate_measurements(saved, args.device)
                if tail_error: journal_errors.insert(0, tail_error)
                if journal_errors: record.update(status="PARTIAL", journal_errors=journal_errors)
                record["method_statistics"] = {}
                for method in ("recursive", "grouped"):
                    rows = [r for r in record["measurements"] if r["method"] == method]
                    if rows:
                        values = sorted(r["function_host_ms"] for r in rows)
                        record["method_statistics"][method] = {"count": len(values), "mean_function_host_ms": statistics.mean(values),
                            "median_function_host_ms": statistics.median(values), "p95_nearest_rank_function_host_ms": values[math.ceil(.95 * len(values)) - 1]}
                record["finished_at_utc"], record["wall_seconds"] = utc(), time.monotonic() - start
                record["completed_measurement_count"] = len(record["measurements"])
                record["expected_measurement_count"] = REPEATS * len(ORDER)
                write_new(output / "summary.json", record)
                paths = sorted(output.iterdir())
                with (output / "SHA256SUMS").open("x") as stream:
                    for path in paths: stream.write(f"{ownership.digest(path)}  {path.name}\n")
            finally:
                torch.set_num_threads(threads)
                for sig, handler in handlers.items(): signal.signal(sig, handler)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-wall-seconds", type=float, required=True)
    parser.add_argument("--expected-source-commit"); parser.add_argument("--finish-before-utc")
    parser.add_argument("--repo", type=Path); parser.add_argument("--expected-worker-sha256")
    args = parser.parse_args(argv)
    result = run(args)
    print(json.dumps({"status": result["status"], "measurements": result["completed_measurement_count"]}))
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
