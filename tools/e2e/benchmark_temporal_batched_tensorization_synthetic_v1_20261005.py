"""HH_261005 - Prospectively compare original and batched tensorization on synthetic CPU inputs only.

No images, weights, reserved evaluation payloads or simulator are read. The
three deterministic fixtures are not recorded CARLA BGRA. Every cold/slow/error
call remains in the fixed 180-row denominator; no warmup builder is called.
The fresh-process caller must configure interop1 before invoking this module.
Only the declared intra-op6 context is changed here, and it is always restored.
"""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
from dataclasses import asdict, replace
import fcntl
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
import time

from .benchmark_scandir_episode_quota_v1_20261004 import digest, read_json, require, safe_path, write_new_json


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_temporal_batched_tensorization_synthetic_v1_20261005.py"
HELPER = ROOT / "learning/portable_e2e/temporal_batched_image_tensorization_v1_20261005.py"
HELPER_TEST = ROOT / "learning/tests/test_temporal_batched_image_tensorization_v1_20261005.py"
FIXTURE_SOURCE = ROOT / "learning/tests/test_portable_e2e_runtime.py"
CLOSURE_SOURCE = ROOT / "tools/e2e/train_temporal_v2_balanced_replay_20261001.py"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/temporal_batched_tensorization_synthetic_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/temporal_batched_tensorization_synthetic_v1"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
SCHEMA = "portable_e2e.synthetic_batched_tensorization_CPU6.20261005.v1"
PATTERNS = ("constant_extremes", "checker_gradient", "seeded_random")
PAIRS, THREADS, INTEROP, RANDOM_SEED = 30, 6, 1, 261005
KEYS = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask")
FIXED_PINS = {
    "learning/portable_e2e/temporal_batched_image_tensorization_v1_20261005.py": "45297afb99eb8a8115ba1f2aecead69db7b60bdd1a5f820642d7c146cee7da38",
    "learning/tests/test_temporal_batched_image_tensorization_v1_20261005.py": "4387020e95ba37030d4f4c42eef638efdb3f4f7b680a2b5b955cfc2c24fbc209",
    "learning/tests/test_portable_e2e_runtime.py": "81ee9e8a173906899cc6550bcfeb4b9e6027b7f2dfe93bd35ab008b97d421cfa",
    "tools/e2e/train_temporal_v2_balanced_replay_20261001.py": "a1e3ad4ed9302b02701c7e3aa56de88afacf6681b275c1bc6986f79ebfd79103",
    "tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py": "3ed5b103fe30db4fac9ac13a0d4c6c5f7990ed17fc3b747ab5a06f883b81c73c",
    "tools/workspace_python.py": "2221cdf71e4ec840566fb19c180329887e53035ef1f319b02221c547141a5b21",
    "tools/canonical_workspace.py": "5d5ee618dba8dfdd84cc6da2877fb2dce8b5ba0017e4a503eb3c55d28401d0ed",
}
_modules = None


def metadata_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def exact_function(path, name, namespace):
    """HH_261005 - Execute only one pinned pure function, never its training/test module or CLI."""
    require(digest(path) == FIXED_PINS[str(path.relative_to(ROOT))], "frozen fixture/closure source changed")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    nodes = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == name]
    require(len(nodes) == 1, "one authentic named fixture/closure function required")
    scope = dict(namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(path), "exec"), scope)
    return scope[name]


def source_snapshot():
    require(all(digest(ROOT / name) == sha for name, sha in FIXED_PINS.items()), "sealed candidate/fixture/setup source differs")
    closure = exact_function(CLOSURE_SOURCE, "imported_source_closure", dict(Path=Path, ast=ast, require=require))
    seeds = (HERE, TEST, HELPER, HELPER_TEST, FIXTURE_SOURCE, CLOSURE_SOURCE,
             ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")
    result = {str(path.relative_to(ROOT)): digest(path) for path in closure(ROOT, seeds)}
    require(all(result.get(name) == sha for name, sha in FIXED_PINS.items()), "complete explicit source closure differs")
    return result


def modules():
    global _modules
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit GPU-disabled environment required before imports")
    if _modules is None:
        candidate = importlib.import_module("portable_e2e.temporal_batched_image_tensorization_v1_20261005")
        require(Path(candidate.__file__).resolve() == HELPER and digest(HELPER) == FIXED_PINS[str(HELPER.relative_to(ROOT))],
                "exact reviewed candidate import required")
        candidate.verify_frozen_sources()
        _modules = candidate, candidate.torch, candidate.np, candidate.runtime
    return _modules


@contextmanager
def cpu_six(torch):
    """HH_261005 - Never mutate interop or runtime bindings; restore only the declared intra-op change."""
    require(torch.get_num_interop_threads() == INTEROP, "fresh caller must already configure interop1")
    old = torch.get_num_threads()
    try:
        torch.set_num_threads(THREADS)
        require(torch.get_num_threads() == THREADS and torch.get_num_interop_threads() == INTEROP, "actual CPU6/interop1 readback required")
        yield
    finally:
        torch.set_num_threads(old)
        require(torch.get_num_threads() == old and torch.get_num_interop_threads() == INTEROP, "CPU threading restoration failed")


def environment(torch):
    expected = dict(CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS="6")
    require(all(os.environ.get(name) == value for name, value in expected.items()), "exact prospective CPU-only environment required")
    require(torch.get_num_threads() == THREADS and torch.get_num_interop_threads() == INTEROP, "actual CPU6/interop1 required")
    require(torch.version.cuda is None and "carla" not in sys.modules, "CPU-only Torch and no simulator import required")
    numpy = modules()[2]
    return dict(interpreter=sys.executable, python=platform.python_version(), torch_version=torch.__version__,
                torch_source=str(Path(torch.__file__).resolve()), torch_source_sha256=digest(Path(torch.__file__).resolve()),
                numpy_version=numpy.__version__, environment=expected, torch_intraop_threads=THREADS,
                torch_interop_threads=INTEROP, cpu_affinity=sorted(os.sched_getaffinity(0)), GPU_used=False)


def fixtures():
    """HH_261005 - Derive only images and a disclosed ten-valid-history mask from the genuine legacy fixture."""
    _, _, np, runtime = modules()
    config = runtime.ModelConfig()
    require(config.camera_count == 6 and (config.image_height, config.image_width, config.ego_history_frames, config.route_points)
            == (180, 320, 10, 128), "original actual SingleCopy ModelConfig ABI required")
    # HH_261005 - Preserve the source expression for the legacy finite speed constant, not a substitute guessed vehicle state.
    tree = ast.parse(FIXTURE_SOURCE.read_text(encoding="utf-8"))
    assignments = [node for node in tree.body if isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "FAKE_TRAJECTORY_SPEED_MPS" for target in node.targets)]
    require(len(assignments) == 1, "one original legacy speed expression required")
    namespace = dict(math=math, np=np, RuntimeInputs=runtime.RuntimeInputs, RuntimeCameraFrame=runtime.RuntimeCameraFrame)
    exec(compile(ast.fix_missing_locations(ast.Module(body=assignments, type_ignores=[])), str(FIXTURE_SOURCE), "exec"), namespace)
    legacy = exact_function(FIXTURE_SOURCE, "_runtime_inputs", namespace)(config)
    require(legacy.ego_history_mask == (False, True, True), "original three-slot fixture mask changed")
    rows = []
    yy, xx = np.indices((360, 640), dtype=np.uint16)
    for pattern in PATTERNS:
        cameras = []
        for role in range(6):
            array = np.empty((360, 640, 4), dtype=np.uint8)
            if pattern == "constant_extremes":
                code = (0, 7, 1, 2, 4, 5)[role]
                for channel in range(3): array[:, :, channel] = 255 * ((code >> channel) & 1)
            elif pattern == "checker_gradient":
                array[:, :, 0] = ((xx + 17 * role) % 256).astype(np.uint8)
                array[:, :, 1] = ((yy * 3 + 29 * role) % 256).astype(np.uint8)
                array[:, :, 2] = ((((xx // 7 + yy // 5 + role) % 2) * 255 + 13 * role) % 256).astype(np.uint8)
            else:
                array[:, :, :3] = np.random.RandomState(RANDOM_SEED + role).randint(0, 256, (360, 640, 3)).astype(np.uint8)
            array[:, :, 3] = 255
            immutable = np.frombuffer(array.tobytes(order="C"), dtype=np.uint8).reshape(360, 640, 4)
            cameras.append(runtime.RuntimeCameraFrame(timestamp_ns=1_000_000_000, payload=immutable, encoding="bgra8"))
        inputs = replace(legacy, camera_frames=tuple(cameras), ego_history_mask=(True,) * config.ego_history_frames)
        require(inputs.calibration == legacy.calibration and inputs.ego_history == legacy.ego_history
                and inputs.route_xy_base_m == legacy.route_xy_base_m, "legacy nonimage values were changed")
        rows.append((pattern, inputs))
    return config, rows


def fixture_snapshot(config, rows):
    values = []
    for pattern, inputs in rows:
        cameras = []
        for role, frame in zip(modules()[3].CAMERA_ORDER, inputs.camera_frames):
            array = frame.payload
            require(array.shape == (360, 640, 4) and str(array.dtype) == "uint8" and array.flags.c_contiguous
                    and not array.flags.writeable and frame.encoding == "bgra8" and frame.timestamp_ns == 1_000_000_000,
                    "exact immutable synthetic BGRA fixture required")
            cameras.append(dict(role=role, timestamp_ns=frame.timestamp_ns, encoding=frame.encoding,
                                shape=list(array.shape), strides=list(array.strides), dtype=str(array.dtype),
                                payload_sha256=hashlib.sha256(array.tobytes(order="C")).hexdigest()))
        require(len(cameras) == 6 and len({row["payload_sha256"] for row in cameras}) == 6, "six distinct fixed synthetic roles required")
        values.append(dict(pattern=pattern, camera_frames=cameras, calibration=inputs.calibration,
                           ego_history=inputs.ego_history, ego_history_mask=inputs.ego_history_mask,
                           route_xy_base_m=inputs.route_xy_base_m))
    # HH_261005 - JSON-roundtrip fixes tuple/list representation before prospective comparison; no numeric conversion is hidden.
    return json.loads(json.dumps(dict(config=asdict(config), fixtures=values), sort_keys=True, allow_nan=False))


def contract():
    return dict(patterns=list(PATTERNS), pairs_per_pattern=PAIRS, timed_calls=2 * PAIRS * len(PATTERNS),
                order="pair0..29, pattern in fixed declared order; AB for even pair, BA for odd; both calls share identical immutable input",
                first_call_cold_and_all_slow_or_rejected_calls_retained=True, warmup_builder_calls=0,
                fixture_source="pinned _runtime_inputs + original FAKE_TRAJECTORY_SPEED_MPS; only RGB->synthetic BGRA and mask3->full-valid10 disclosed",
                synthetic_patterns_seed=RANDOM_SEED, source_image_shape=[6, 360, 640, 4], output_image_shape=[1, 6, 3, 180, 320],
                exact_all_six_tensor_bytes_and_dtype_shape_SHA_required=True, exact_legacy_nonimage_values_required=True,
                timer_scope="one complete original build_runtime_tensors or exact BatchedImageTensorizer.__call__, including every original guard and candidate scalar stage receipts",
                timer_excludes="imports/AST/provider setup, fixture generation/hash, source verification, result parity/hash, receipt persistence",
                source_and_fixture_hashing_may_warm_host_cache=True, intraop=THREADS, interop=INTEROP,
                interop_mutated_by_runner=False, intraop_restored_after_context=True, no_runtime_alias_or_cache_activation=True,
                actual_recorded_BGRA_parity_proven=False, NN_or_weights_read=False, actual_sensor_latency_gain_proven=False,
                actual_closed_loop_gain_proven=False, training=False, installs=False, model_promoted=False,
                formal_main_function_approval=0, formal_subfunction_approval=0)


def tensor_snapshot(tensors):
    _, torch, _, _ = modules()
    require(type(tensors) is dict and set(tensors) == set(KEYS), "complete original six-tensor output required")
    result = {}
    for name in KEYS:
        tensor = tensors[name]
        require(isinstance(tensor, torch.Tensor) and tensor.device.type == "cpu", "original CPU tensor output required")
        result[name] = dict(shape=list(tensor.shape), dtype=str(tensor.dtype),
                            sha256=hashlib.sha256(tensor.detach().contiguous().numpy().tobytes(order="C")).hexdigest())
    return result


def exact_tensor_equal(left, right):
    """HH_261005 - Compare the actual contiguous CPU bytes as well as the separately recorded SHA values."""
    require(set(left) == set(right) == set(KEYS), "complete same six-tensor comparison required")
    return all(left[name].shape == right[name].shape and left[name].dtype == right[name].dtype
               and left[name].detach().contiguous().numpy().tobytes(order="C")
               == right[name].detach().contiguous().numpy().tobytes(order="C") for name in KEYS)


def paired_calls(functions, config, rows, *, clock=time.perf_counter_ns, snapshot=tensor_snapshot,
                 compare=exact_tensor_equal, persist=lambda row: None, receipt=lambda: None,
                 results=None, pairs=None):
    """HH_261005 - Keep every attempted call and pair failure in the fixed denominator; never retry or filter."""
    require(set(functions) == {"A", "B"} and [name for name, _ in rows] == list(PATTERNS), "exact prospective variants and pattern order required")
    results = [] if results is None else results
    pairs = [] if pairs is None else pairs
    require(not results and not pairs, "fresh empty call and pair ledgers required")
    for pair in range(PAIRS):
        for pattern, inputs in rows:
            observations, raw_outputs = {}, {}
            for position, variant in enumerate("AB" if pair % 2 == 0 else "BA"):
                row = dict(call_ordinal=len(results), pair_ordinal=pair, pattern=pattern, position=position, variant=variant, error=None)
                started = clock()
                try: value = functions[variant](inputs, config)
                except Exception as error:
                    value = None
                    row["error"] = dict(type=type(error).__name__, message=str(error))
                finished = clock()
                row.update(started_monotonic_ns=started, finished_monotonic_ns=finished, elapsed_ns=finished - started)
                require(type(started) is int and type(finished) is int and finished >= started, "monotonic integer timing required")
                try:
                    row["output"] = snapshot(value) if row["error"] is None else None
                except Exception as error:
                    row["error"] = dict(type=type(error).__name__, message=str(error), phase="outside-timer output verification")
                    row["output"] = None
                if variant == "B":
                    try: row["candidate_scalar_receipt"] = receipt()
                    except Exception as error:
                        row["candidate_scalar_receipt"] = None
                        row["error"] = dict(type=type(error).__name__, message=str(error), phase="outside-timer receipt verification")
                observations[variant] = row
                raw_outputs[variant] = value
                results.append(row)
                persist(row)
            pairs.append(dict(pair_ordinal=pair, pattern=pattern, bitwise_all_tensor_parity=(
                observations["A"]["error"] is None and observations["B"]["error"] is None
                and observations["A"]["output"] == observations["B"]["output"]
                and compare(raw_outputs["A"], raw_outputs["B"]))))
    require(len(results) == 180 and len(pairs) == 90, "complete fixed180/90 denominator required")
    return results, pairs


def append_row(row):
    path = safe_path(OUTPUT / "timed_calls.jsonl", missing_tail=True)
    with path.open("a" if path.exists() else "x", encoding="utf-8") as stream:
        stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def declare():
    require(not PLAN.exists() and not OUTPUT.exists(), "fresh prospective plan/output required")
    sources = source_snapshot()
    _, torch, _, _ = modules()
    with cpu_six(torch):
        env = environment(torch)
        config, rows = fixtures()
        inputs = fixture_snapshot(config, rows)
    require(source_snapshot() == sources, "source changed during declaration")
    document = dict(schema=SCHEMA + ".prospective", provenance="HH_261005 - Seal synthetic inputs/source/order before any timed call; no actual driving claim.",
                    output=str(OUTPUT), source_sha256=sources, contract=contract(), environment=env,
                    synthetic_inputs=inputs, synthetic_inputs_sha256=metadata_sha(inputs), measured=False)
    write_new_json(PLAN, document)
    return dict(status="DECLARED_NOT_MEASURED", declaration=str(PLAN), declaration_sha256=digest(PLAN))


def execute(expected_sha):
    require(type(expected_sha) is str and len(expected_sha) == 64 and digest(PLAN) == expected_sha, "exact external prospective SHA required")
    plan = read_json(PLAN)
    require(plan.get("schema") == SCHEMA + ".prospective" and plan.get("contract") == contract()
            and plan.get("output") == str(OUTPUT), "unchanged prospective experiment required")
    require(source_snapshot() == plan["source_sha256"], "source changed before execution")
    require(not OUTPUT.exists() and safe_path(OUTPUT, missing_tail=True) == OUTPUT and OUTPUT.resolve() == OUTPUT, "fresh canonical no-alias output required")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    write_new_json(OUTPUT / "started.json", dict(schema=SCHEMA + ".started", declaration_sha256=expected_sha,
                   started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), planned_timed_calls=180))
    results, pairs = [], []
    try:
        # HH_261005 - Take the existing shared ownership lease; never overlap an actual runtime or another CPU timing experiment.
        with safe_path(LOCK).open("r+") as lease:
            fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            candidate, torch, _, runtime = modules()
            with cpu_six(torch):
                require(environment(torch) == plan["environment"], "CPU environment/readback changed")
                config, inputs = fixtures()
                require(fixture_snapshot(config, inputs) == plan["synthetic_inputs"], "synthetic fixture bytes changed")
                provider = candidate.BatchedImageTensorizer()
                original = runtime.build_runtime_tensors
                results, pairs = paired_calls(dict(A=original, B=provider), config, inputs, persist=append_row,
                    receipt=lambda: json.loads(json.dumps(provider.last_timing, allow_nan=False)), results=results, pairs=pairs)
                require(runtime.build_runtime_tensors is original, "ordinary runtime alias changed")
                require(fixture_snapshot(config, inputs) == plan["synthetic_inputs"], "source fixtures mutated by tensorization")
            require(source_snapshot() == plan["source_sha256"] and digest(PLAN) == expected_sha, "post-source/plan integrity changed")
        complete = all(row["bitwise_all_tensor_parity"] for row in pairs)
        candidate_rows = [row for row in results if row["variant"] == "B"]
        complete = complete and all(type(row["candidate_scalar_receipt"]) is dict
            and row["candidate_scalar_receipt"].get("status") == "PASS"
            and row["candidate_scalar_receipt"].get("fallback_to_original") is False
            and len(row["candidate_scalar_receipt"].get("camera_preprocessing", [])) == 6 for row in candidate_rows)
        stats = {}
        for pattern in PATTERNS:
            stats[pattern] = {}
            for variant in ("A", "B"):
                values = [row["elapsed_ns"] / 1e6 for row in results if row["pattern"] == pattern and row["variant"] == variant]
                stats[pattern][variant] = dict(calls=len(values), mean_ms=statistics.mean(values), median_ms=statistics.median(values),
                    p95_ms=sorted(values)[math.ceil(.95 * len(values)) - 1], maximum_ms=max(values))
        summary = dict(schema=SCHEMA + ".summary", status="PASS_SYNTHETIC_PARITY_ONLY" if complete else "FAIL_SYNTHETIC_RETAINED",
            declaration_sha256=expected_sha, timed_call_count=len(results), pair_count=len(pairs),
            all_pair_parity_passed=all(row["bitwise_all_tensor_parity"] for row in pairs), pairs=pairs, timing_statistics=stats,
            all_source_and_fixture_sha_pre_post_unchanged=True, all_180_including_cold_and_failures_retained=True,
            actual_BGRA_or_sensor_latency_or_NN_or_closed_loop_gain_proven=False, actual_runtime_activated=False,
            model_promoted=False, formal_function_approved=False)
        write_new_json(OUTPUT / "summary.json", summary)
        return dict(status=summary["status"], timed_calls=len(results), pairs=len(pairs), output=str(OUTPUT), summary_sha256=digest(OUTPUT / "summary.json"))
    except BaseException as error:
        write_new_json(OUTPUT / "FAILED_attempt.json", dict(schema=SCHEMA + ".failed", declaration_sha256=expected_sha,
            status="FAILED_RETAINED", error_type=type(error).__name__, error=str(error),
            completed_in_memory_calls=len(results), completed_pairs=len(pairs), durable_call_journal_retained=True,
            all_attempted_call_rows=results, all_completed_pair_rows=pairs,
            expected_call_count=180, retry_forbidden=True, formal_function_approved=False))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("declare", "execute"))
    parser.add_argument("--expected-declaration-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None, "declaration cannot consume an execution SHA")
        result = declare()
    else:
        require(args.expected_declaration_sha256 is not None, "execution requires external declaration SHA")
        result = execute(args.expected_declaration_sha256)
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0 if not result["status"].startswith("FAIL") else 2


if __name__ == "__main__":
    raise SystemExit(main())
