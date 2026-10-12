"""HH_261005 - Prepare two fresh whole888 CPU6 arms differing only in pre-import OpenBLAS1 versus6.

Source preparation is not authority to execute. ROOT must review and seal a
fresh declaration before any lossy-JPEG reconstruction, weights or NN calls.
The original full forward, raw tolerance, decoder refusals, lease and disk caps
remain unchanged. This does not establish active BLAS pools or live gain.
"""

import argparse
import ast
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from types import FunctionType

from . import benchmark_full209_thread468_v2_20261004 as frozen


original, previous = frozen.original, frozen.original.previous
ROOT, require, digest, read_json = original.ROOT, original.require, original.digest, original.read_json
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_blas16_v1_20261005.py"
MODULE = "scripts.e2e.benchmark_full209_blas16_v1_20261005"
SCHEMA = "portable_e2e.full209_masked_copy_controlled_BLAS16.20261005.v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/full209_blas16_controlled_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/full209_blas16_controlled_counterfactual_v1"
V2_SOURCE_SHA = "1660292f4feb0feccd0815fa6ae8de5cb34ad15e798adfbbabe52478cf4cc7b9"
V2_TEST_SHA = "7b6b218fb45be7735730068d611d0c5e9adece7e01dd179ede3d7b584c663e1f"
ARMS = (dict(index=0, name="OPENBLAS1", OPENBLAS_NUM_THREADS="1"),
        dict(index=1, name="OPENBLAS6", OPENBLAS_NUM_THREADS="6"))
ENV_NAMES = ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")
CURRENT_ARM, ENVIRONMENT_STATE, _SCOPE = None, None, None


def contract():
    value = frozen.contract()
    value.update(thread_arms=[6, 6], fixed_intraop_threads=6, interop_threads=1,
        fixed_serial_environment_arms=copy.deepcopy(list(ARMS)),
        fixed_OMP_NUM_THREADS="6", fixed_MKL_NUM_THREADS="6",
        OPENBLAS_environment_values=["1", "6"], inherited_CPU_affinity=list(range(24)),
        environment_set_once_before_any_Torch_NumPy_or_model_import=True,
        original_snapshot_admitted_before_environment_change=True,
        post_snapshot_only_declared_three_environment_values_normalized=True,
        CPU_affinity_changed=False, active_BLAS_pool_identification_proven=False,
        original_neural_math_state_decoder_tolerance_and_worker_guards_unchanged=True,
        total_contexts=1776, contexts_per_arm=888, timed_calls_per_arm=30,
        total_timed_calls=60, first_selected_context_cold_cache_per_fresh_arm=True,
        serial_AB_not_thirty_interleaved_pairs=True,
        serial_input_and_OS_cache_warming_disclosed=True, no_hidden_warmup=True,
        first_five_prefill_decoder_refusals_per_arm_retained=True,
        lossy_JPEG_counterfactual_only=True, actual_sensor_latency_gain_proven=False)
    return value


def verify_frozen():
    require(digest(Path(frozen.__file__)) == V2_SOURCE_SHA and digest(frozen.TEST) == V2_TEST_SHA,
        "frozen thread468 V2 source/test changed")
    require(digest(frozen.ORIGINAL_SOURCE) == frozen.ORIGINAL_SOURCE_SHA
        and digest(frozen.ORIGINAL_TEST) == frozen.ORIGINAL_TEST_SHA,
        "frozen thread468 V1 source/test changed")


def source_snapshot():
    verify_frozen()
    pins = frozen.source_snapshot()
    pins.update({str(path.relative_to(ROOT)): digest(path)
        for path in (Path(frozen.__file__), frozen.TEST, HERE, TEST)})
    return pins


def selected_environment():
    return {name: os.environ.get(name) for name in ("CUDA_VISIBLE_DEVICES", *ENV_NAMES)}


def snapshot():
    value, cameras, events = _private_scope()["_snapshot_original"]()
    require(value["environment"]["inherited_CPU_affinity"] == list(range(24)),
        "unchanged admitted all24 CPU affinity required")
    if ENVIRONMENT_STATE is not None:
        require(selected_environment() == ENVIRONMENT_STATE["actual_environment"],
            "prospectively fixed pre-import thread environment changed")
        # HH_261005 - Invert only the three disclosed env assignments, never interpreter, GPU, affinity, source, input or numerical guards.
        actual = value["environment"]["environment"]
        for name in ENV_NAMES:
            require(actual[name] == ENVIRONMENT_STATE["actual_environment"][name], "environment readback differs")
            actual[name] = ENVIRONMENT_STATE["original_environment"][name]
    return value, cameras, events


def apply_environment(plan):
    global ENVIRONMENT_STATE
    require(ENVIRONMENT_STATE is None and type(CURRENT_ARM) is int and CURRENT_ARM in range(2)
        and not any(name in sys.modules for name in ("torch", "numpy", "carla")),
        "one fresh pre-Torch/NumPy environment setup required")
    require(plan["contract"] == contract()
        and selected_environment() == plan["environment"]["environment"]
        and sorted(os.sched_getaffinity(0)) == list(range(24)), "original admitted environment required")
    require(len(list(Path("/proc/self/task").iterdir())) == 1,
        "single fresh process thread required before configuring native pools")
    before = selected_environment()
    for name, value in (("OMP_NUM_THREADS", "6"), ("MKL_NUM_THREADS", "6"),
                        ("OPENBLAS_NUM_THREADS", ARMS[CURRENT_ARM]["OPENBLAS_NUM_THREADS"])):
        os.environ[name] = value
    after = selected_environment()
    require(after == dict(before, OMP_NUM_THREADS="6", MKL_NUM_THREADS="6",
        OPENBLAS_NUM_THREADS=ARMS[CURRENT_ARM]["OPENBLAS_NUM_THREADS"]), "exact fresh environment readback required")
    ENVIRONMENT_STATE = dict(arm=copy.deepcopy(ARMS[CURRENT_ARM]),
        original_environment=before, actual_environment=after,
        before_any_Torch_NumPy_or_model_import=True, mutation_rounds=1,
        CPU_affinity_changed=False, active_BLAS_pool_identification_proven=False)
    original.write_new_json(OUTPUT / ARMS[CURRENT_ARM]["name"] / "pre_import_environment.json", ENVIRONMENT_STATE)


def private_worker_AST():
    verify_frozen()
    tree = ast.parse(Path(original.__file__).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "worker")
    pristine = copy.deepcopy(node)
    output = next(n for n in node.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "output" for t in n.targets))
    old_output = copy.deepcopy(output.value)
    output.value = ast.parse('OUTPUT / ARMS[CURRENT_ARM]["name"]', mode="eval").body
    attempt = next(n for n in node.body if isinstance(n, ast.Try))
    index = next(i for i, n in enumerate(attempt.body) if isinstance(n, ast.Import) and n.names[0].name == "torch")
    attempt.body.insert(index, ast.parse("apply_environment(plan)").body[0])
    inverse = copy.deepcopy(node)
    next(n for n in inverse.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "output" for t in n.targets)).value = old_output
    inverse_try = next(n for n in inverse.body if isinstance(n, ast.Try))
    inverse_try.body = [n for n in inverse_try.body if not (isinstance(n, ast.Expr)
        and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
        and n.value.func.id == "apply_environment")]
    require(ast.dump(inverse, include_attributes=False) == ast.dump(pristine, include_attributes=False),
        "original full worker inverse AST differs")
    return ast.fix_missing_locations(node)


def _private_scope():
    global _SCOPE
    if _SCOPE is None:
        scope = dict(original.__dict__)
        scope.update(MODULE=MODULE, SCHEMA=SCHEMA, PLAN=PLAN, OUTPUT=OUTPUT, HERE=HERE, TEST=TEST,
            THREAD_ARMS=(6, 6), ARMS=ARMS, CURRENT_ARM=CURRENT_ARM,
            contract=contract, source_snapshot=source_snapshot, snapshot=snapshot,
            apply_environment=apply_environment, _snapshot_original=original.snapshot)
        for name, value in original.__dict__.items():
            if isinstance(value, FunctionType) and value.__globals__ is original.__dict__:
                scope[name] = FunctionType(value.__code__, scope, name, value.__defaults__, value.__closure__)
                scope[name].__kwdefaults__ = value.__kwdefaults__
        scope.update(contract=contract, source_snapshot=source_snapshot, snapshot=snapshot,
            apply_environment=apply_environment)
        scope["_snapshot_original"] = FunctionType(original.snapshot.__code__, scope)
        exec(compile(ast.Module(body=[private_worker_AST()], type_ignores=[]), str(HERE), "exec"), scope)
        _SCOPE = scope
    _SCOPE["CURRENT_ARM"] = CURRENT_ARM
    return _SCOPE


def worker(arm, expected, lease_fd):
    global CURRENT_ARM
    require(type(arm) is int and arm in range(2), "fixed fresh BLAS arm required")
    CURRENT_ARM = arm
    return _private_scope()["worker"](6, expected, lease_fd)


def run_parent(args, lease):
    previous.recorded.fresh_output(OUTPUT)
    before, _, _ = snapshot()
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None
            and not any(name in sys.modules for name in ("torch", "numpy", "carla")), "source-only declaration required")
        previous.recorded.fresh_output(PLAN)
        original.write_new_json(PLAN, dict(schema=SCHEMA + ".prospective",
            provenance="HH_261005 - Fix both whole888 native-thread environments, cold first and all refusals before any reconstruction or NN.", **before))
        print(json.dumps(dict(status="DECLARED_NOT_MEASURED", declaration_sha256=digest(PLAN))))
        return 0
    require(args.expected_declaration_sha256 == digest(PLAN), "exact fresh prospective declaration SHA required")
    plan = read_json(PLAN)
    require(plan["schema"] == SCHEMA + ".prospective"
        and all(plan[name] == value for name, value in before.items()), "source/input/asset/environment changed")
    previous.recorded.disk_guard()
    OUTPUT.mkdir(parents=True, exist_ok=False)
    attempts = [dict(**arm, status="UNEXECUTED", planned_contexts=888, planned_timed_calls=30) for arm in ARMS]
    arms = []
    try:
        for arm in ARMS:
            item = attempts[arm["index"]]
            item["status"] = "STARTED"
            command = [sys.executable, str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--",
                "--mode", "worker", "--arm", str(arm["index"]), "--expected-declaration-sha256",
                args.expected_declaration_sha256, "--runtime-lease-fd", str(lease.fileno())]
            with (OUTPUT / (arm["name"] + ".stdout.log")).open("xb") as stdout, (OUTPUT / (arm["name"] + ".stderr.log")).open("xb") as stderr:
                result = subprocess.run(command, cwd=ROOT, env=dict(os.environ), pass_fds=(lease.fileno(),), stdout=stdout, stderr=stderr, check=False)
            item.update(status="COMPLETED" if result.returncode == 0 else "REJECTED", exit_code=result.returncode, command=command)
            original.write_new_json(OUTPUT / (arm["name"] + ".process_receipt.json"), item)
            require(result.returncode == 0, "fresh BLAS arm rejected; prefixes retained without retry")
            root = OUTPUT / arm["name"]
            summary = read_json(root / "summary.json")
            require(summary["threads"] == 6 and summary["interop_threads"] == 1
                and summary["completed_contexts"] == 888 and summary["timed_calls"] == 30
                and digest(root / "all888_context_rows.jsonl") == summary["all_context_rows_sha256"]
                and digest(root / "all30_timing_rows.jsonl") == summary["all_timing_rows_sha256"], "complete original worker archive required")
            setup = read_json(root / "pre_import_environment.json")
            require(setup["arm"] == arm and setup["original_environment"] == plan["environment"]["environment"]
                and setup["actual_environment"] == dict(setup["original_environment"], OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS=arm["OPENBLAS_NUM_THREADS"])
                and setup["before_any_Torch_NumPy_or_model_import"] is True and setup["mutation_rounds"] == 1
                and setup["CPU_affinity_changed"] is False, "exact pre-import environment proof required")
            arms.append(dict(threads=6, summary=summary, rows=previous.jsonl(root / "all888_context_rows.jsonl"),
                arm=arm, pre_import_environment=setup, pre_import_environment_sha256=digest(root / "pre_import_environment.json")))
            previous.recorded.disk_guard()
        comparisons = _private_scope()["compare_arms"](arms)
        for i, row in enumerate(comparisons):
            _private_scope()["bounded_append"](OUTPUT, "all1776_comparison_rows.jsonl", dict(row, environment_arm=ARMS[i // 888]))
        after, _, _ = snapshot()
        require(before == after and sum(previous.recorded.tree_members(OUTPUT)["files"].values()) <= original.MAX_TOTAL_BYTES,
            "unchanged full original closure and original48MiB output cap required")
        ordinary = [r for r in comparisons if r["ordinary_decoder"]["status"] == "ORDINARY_GO_COUNTERFACTUAL_ONLY"]
        passed = bool(ordinary) and all(r["all_five_raw_heads_strict_allclose"] for r in comparisons)
        passed = passed and all(r["ordinary_decoder"]["ordinary_numeric_screen_passed"] for r in ordinary)
        passed = passed and not any(r["ordinary_decoder_rejected"] for r in comparisons)
        original.write_new_json(OUTPUT / "summary.json", dict(schema=SCHEMA,
            status="OFFLINE_NUMERIC_PASS_NOT_LIVE_GAIN" if passed else "FAIL_RETAINED",
            arm_summaries=[{k: v for k, v in arm.items() if k != "rows"} for arm in arms],
            completed_comparisons=len(comparisons), contexts_per_arm=888, timed_calls_per_arm=30, total_timed_calls=60,
            comparison_rows_sha256=digest(OUTPUT / "all1776_comparison_rows.jsonl"),
            ordinary_decoded_comparison_count=len(ordinary), ordinary_decoder_rejection_count=sum(r["ordinary_decoder_rejected"] for r in comparisons),
            terminal_unverified_count=sum(r["original_decoder_role"] == "ACTUAL_TERMINAL" for r in comparisons),
            all_original_source_input_assets_unchanged=True, raw_ATOL=1e-6, raw_RTOL=1e-6,
            serial_AB_not_interleaved_pairs=True, OS_warming_and_order_confounding_disclosed=True,
            active_BLAS_pool_identification_proven=False, full_safety_engine_parity_verified=False,
            terminal_proof_verified=False, actual_sensor_latency_gain_proven=False,
            original_actual_BGRA_or_raw_NN_equality_claimed=False, model_promoted=False,
            main_functions_approved=0, subfunctions_approved=0))
        return 0 if passed else 2
    except BaseException as error:
        original.write_new_json(OUTPUT / "FAILED_attempt.json", dict(schema=SCHEMA + ".failed",
            all_declared_arm_attempts=attempts, complete_prefix_arm_count=len(arms), planned_contexts=1776,
            planned_timed_calls=60, exception_type=type(error).__name__, exception=str(error),
            failed_or_unexecuted_tails_retained=True, no_retry=True, model_promoted=False))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "execute", "worker"), required=True)
    parser.add_argument("--expected-declaration-sha256")
    parser.add_argument("--arm", type=int, choices=range(2))
    parser.add_argument("--runtime-lease-fd", type=int)
    args = parser.parse_args(argv)
    if args.mode == "worker":
        require(args.arm is not None and args.runtime_lease_fd is not None, "internal declared worker only")
        return worker(args.arm, args.expected_declaration_sha256, args.runtime_lease_fd)
    require(args.arm is None and args.runtime_lease_fd is None, "worker arguments cannot change parent declaration")
    with original.safe_path(previous.LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            return run_parent(args, lease)
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
