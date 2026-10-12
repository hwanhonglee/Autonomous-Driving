"""HH_261005 - Prepare four fresh serial ABBA affinity blocks on the frozen full888 CPU6 cached-forward probe.

This is not thirty interleaved A/B pairs. Each fresh block reconstructs all
901 lossy-JPEG bundles and processes all888 original contexts, retaining the
same thirty fixed forwards including its cold first cache call. OS warming
and serial ordering are disclosed. No original BGRA, terminal/full-engine
parity, live gain, driving capability or model promotion is claimed.
Actual declaration and execution require ROOT review; source preparation does
not read model weights or recorded pixels, run inference, or connect to CARLA.
"""

from __future__ import annotations

import argparse
import ast
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import FunctionType

from . import benchmark_full209_thread468_v2_20261004 as frozen


original = frozen.original
previous = original.previous
ROOT, require, digest, read_json = original.ROOT, original.require, original.digest, original.read_json
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_pcore_affinity_v1_20261005.py"
MODULE = "scripts.e2e.benchmark_full209_pcore_affinity_v1_20261005"
SCHEMA = "portable_e2e.full209_masked_copy_pcore_affinity_serial_ABBA.20261005.v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/full209_pcore_affinity_serial_ABBA_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/full209_pcore_affinity_serial_ABBA_v1"
V2_SOURCE_SHA = "1660292f4feb0feccd0815fa6ae8de5cb34ad15e798adfbbabe52478cf4cc7b9"
V2_TEST_SHA = "7b6b218fb45be7735730068d611d0c5e9adece7e01dd179ede3d7b584c663e1f"
ALL_CPUS, PCORE_CPUS = tuple(range(24)), tuple(range(16))
BLOCKS = tuple(dict(index=index, name=name, variant=variant)
               for index, (name, variant) in enumerate((("A1_ALL24", "A"), ("B1_PCORE", "B"), ("B2_PCORE", "B"), ("A2_ALL24", "A"))))
CURRENT_BLOCK, AFFINITY_STATE = None, None
_SCOPE = None


def contract():
    value = frozen.contract()
    value.update(thread_arms=[6]*4, fixed_intraop_threads=6, interop_threads=1,
        serial_ABBA_blocks=copy.deepcopy(list(BLOCKS)), affinity_masks={"A": list(ALL_CPUS), "B": list(PCORE_CPUS)},
        affinity_set_before_any_Torch_or_model_import=True, original_snapshot_admitted_before_affinity_change=True,
        post_snapshot_only_inherited_CPU_affinity_normalized_to_original_declared_mask=True,
        all_observed_process_thread_masks_verified=True, continuous_thread_monitoring_claimed=False,
        observed_threads_not_falsely_identified_as_Torch_workers=True,
        thirty_interleaved_AB_pairs_claimed=False, timed_calls_per_block=30, total_timed_calls=120,
        context_denominator_per_block=888, total_contexts=3552,
        first_selected_context_cold_cache_per_fresh_block=True, no_hidden_warmup=True,
        serial_input_and_OS_cache_warming_disclosed=True,
        original_neural_math_state_decoder_and_tolerances_unchanged=True,
        affinity_comparison_is_offline_lossy_JPEG_counterfactual_only=True)
    return value


def verify_frozen():
    require(digest(Path(frozen.__file__)) == V2_SOURCE_SHA and digest(frozen.TEST) == V2_TEST_SHA,
            "frozen thread468 V2 source/test changed")
    require(digest(frozen.ORIGINAL_SOURCE) == frozen.ORIGINAL_SOURCE_SHA
            and digest(frozen.ORIGINAL_TEST) == frozen.ORIGINAL_TEST_SHA, "frozen thread468 V1 source/test changed")


def source_snapshot():
    verify_frozen()
    pins = frozen.source_snapshot()
    pins.update({str(path.relative_to(ROOT)): digest(path) for path in (Path(frozen.__file__), frozen.TEST, HERE, TEST)})
    return pins


def cpu_topology():
    # HH_261005 - Seal named local CPU metadata; frequency grouping is a scheduling hypothesis, not an observed inference improvement.
    base = Path("/sys/devices/system/cpu")
    rows = []
    for cpu in ALL_CPUS:
        path = base / f"cpu{cpu}"
        rows.append(dict(cpu=cpu, core_id=int((path/"topology/core_id").read_text()),
            package_id=int((path/"topology/physical_package_id").read_text()),
            thread_siblings_list=(path/"topology/thread_siblings_list").read_text().strip(),
            maximum_frequency_kHz=int((path/"cpufreq/cpuinfo_max_freq").read_text())))
    require(all(5300000 <= row["maximum_frequency_kHz"] <= 5400000 for row in rows[:16])
            and all(row["maximum_frequency_kHz"] == 4200000 for row in rows[16:]), "declared local P/E maximum-frequency grouping changed")
    return dict(online=(base/"online").read_text().strip(), named_cpu_rows=rows,
                grouping_from_static_named_metadata=True, runtime_gain_proven=False)


def snapshot():
    value, cameras, events = _private_scope()["_snapshot_original"]()
    actual = value["environment"]["inherited_CPU_affinity"]
    if AFFINITY_STATE is None:
        require(actual == list(ALL_CPUS), "original all24 inherited affinity required before fresh admission")
    else:
        require(actual == AFFINITY_STATE["declared_mask"], "declared worker affinity changed after application")
        # HH_261005 - Normalize only the single prospectively declared affinity field; every original source/input/asset/environment field remains exact.
        value["environment"]["inherited_CPU_affinity"] = list(ALL_CPUS)
    value["CPU_topology"] = cpu_topology()
    return value, cameras, events


def parse_cpu_list(text):
    result = []
    for item in text.strip().split(","):
        endpoints = item.split("-")
        require(len(endpoints) in (1, 2) and all(value.isdecimal() for value in endpoints), "exact CPU mask syntax required")
        lo, hi = int(endpoints[0]), int(endpoints[-1])
        require(lo <= hi <= 4095, "bounded CPU mask range required")
        result.extend(range(lo, hi+1))
    require(result and len(result) == len(set(result)), "nonempty nonduplicate CPU mask required")
    return sorted(result)


def process_thread_masks():
    rows = []
    for path in sorted(Path("/proc/self/task").iterdir(), key=lambda item: int(item.name)):
        status = path.joinpath("status").read_text()
        mask = next(line.split(":", 1)[1] for line in status.splitlines() if line.startswith("Cpus_allowed_list:"))
        fields = path.joinpath("stat").read_text().rsplit(")", 1)[1].split()
        rows.append(dict(tid=int(path.name), start_ticks=int(fields[19]), CPU_affinity=parse_cpu_list(mask)))
    require(rows, "actual process thread identities required")
    return rows


def apply_affinity(plan):
    global AFFINITY_STATE
    require(AFFINITY_STATE is None and CURRENT_BLOCK in range(4) and "torch" not in sys.modules and "carla" not in sys.modules,
            "single fresh pre-Torch affinity application required")
    require(plan["contract"] == contract() and sorted(os.sched_getaffinity(0)) == list(ALL_CPUS), "admitted original environment required")
    before = process_thread_masks()
    require(len(before) == 1 and before[0]["tid"] == os.getpid() and before[0]["CPU_affinity"] == list(ALL_CPUS),
            "fresh single-thread worker required before changing inherited affinity")
    mask = ALL_CPUS if BLOCKS[CURRENT_BLOCK]["variant"] == "A" else PCORE_CPUS
    os.sched_setaffinity(0, mask)
    require(sorted(os.sched_getaffinity(0)) == list(mask), "fresh process affinity application failed")
    AFFINITY_STATE = dict(block=copy.deepcopy(BLOCKS[CURRENT_BLOCK]), original_mask=list(ALL_CPUS),
        declared_mask=list(mask), before_thread_identities=before, applied_monotonic_ns=time.monotonic_ns(),
        before_any_Torch_or_model_import=True, mutation_count=1)
    original.write_new_json(OUTPUT/BLOCKS[CURRENT_BLOCK]["name"]/"affinity_applied.json", AFFINITY_STATE)


def verify_thread_affinity(output, stage):
    require(AFFINITY_STATE is not None and sorted(os.sched_getaffinity(0)) == AFFINITY_STATE["declared_mask"], "actual main thread affinity changed")
    rows = process_thread_masks()
    require(all(row["CPU_affinity"] == AFFINITY_STATE["declared_mask"] for row in rows), "an observed worker thread escaped the declared inherited mask")
    receipt = dict(stage=stage, observed_monotonic_ns=time.monotonic_ns(), block=BLOCKS[CURRENT_BLOCK],
        declared_mask=AFFINITY_STATE["declared_mask"], all_observed_process_thread_identities=rows,
        all_observed_thread_masks_equal=True, continuous_monitoring_claimed=False)
    _private_scope()["bounded_append"](output, "affinity_thread_receipts.jsonl", receipt)
    return receipt


def timed_forward(model, tensors, window, threads, index, timings, output):
    selected = index in previous.selected_indices()
    if selected: verify_thread_affinity(output, "before_selected_forward_" + str(index))
    # HH_261005 - An original forward exception propagates unchanged with its original timing journal; a later affinity check must not mask it.
    raw = _private_scope()["_timed_original"](model, tensors, window, threads, index, timings, output)
    if selected: verify_thread_affinity(output, "after_selected_forward_" + str(index))
    return raw


def private_worker_AST():
    # HH_261005 - Preserve the entire original worker body/inference/error/order; only output identity and pre-import affinity proof are new seams.
    verify_frozen()
    tree = ast.parse(Path(original.__file__).read_text())
    node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == "worker")
    pristine = copy.deepcopy(node)
    output = next(item for item in node.body if isinstance(item, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "output" for target in item.targets))
    old_output = copy.deepcopy(output.value)
    output.value = ast.parse('OUTPUT / BLOCKS[CURRENT_BLOCK]["name"]', mode="eval").body
    attempt = next(item for item in node.body if isinstance(item, ast.Try))
    first_import = next(index for index, item in enumerate(attempt.body) if isinstance(item, ast.Import) and item.names[0].name == "torch")
    attempt.body.insert(first_import, ast.parse("apply_affinity(plan)").body[0])
    loaded = next(index for index, item in enumerate(attempt.body) if isinstance(item, ast.Expr) and isinstance(item.value, ast.Call)
                  and isinstance(item.value.func, ast.Name) and item.value.func.id == "set_threads_after_load")
    attempt.body.insert(loaded+1, ast.parse('verify_thread_affinity(output, "after_strict_loader_and_CPU6_setup")').body[0])
    # HH_261005 - Exact inverse equality prevents a hidden neural/controller/decoder/guard rewrite.
    inverse = copy.deepcopy(node)
    next(item for item in inverse.body if isinstance(item, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "output" for target in item.targets)).value = old_output
    original_try = next(item for item in inverse.body if isinstance(item, ast.Try))
    original_try.body = [item for item in original_try.body if not (isinstance(item, ast.Expr) and isinstance(item.value, ast.Call)
        and isinstance(item.value.func, ast.Name) and item.value.func.id in ("apply_affinity", "verify_thread_affinity"))]
    require(ast.dump(inverse, include_attributes=False) == ast.dump(pristine, include_attributes=False), "private original worker inverse AST differs")
    return ast.fix_missing_locations(node)


def _private_scope():
    global _SCOPE
    if _SCOPE is None:
        scope = dict(original.__dict__)
        for name, value in original.__dict__.items():
            if isinstance(value, FunctionType) and value.__globals__ is original.__dict__:
                scope[name] = FunctionType(value.__code__, scope, name, value.__defaults__, value.__closure__)
        scope.update(MODULE=MODULE, SCHEMA=SCHEMA, PLAN=PLAN, OUTPUT=OUTPUT, HERE=HERE, TEST=TEST,
            THREAD_ARMS=(6,)*4, BLOCKS=BLOCKS, CURRENT_BLOCK=CURRENT_BLOCK,
            contract=contract, source_snapshot=source_snapshot, snapshot=snapshot,
            apply_affinity=apply_affinity, verify_thread_affinity=verify_thread_affinity, timed_forward=timed_forward)
        scope["_snapshot_original"] = FunctionType(original.snapshot.__code__, scope)
        scope["_timed_original"] = FunctionType(original.timed_forward.__code__, scope)
        exec(compile(ast.Module(body=[private_worker_AST()], type_ignores=[]), str(HERE), "exec"), scope)
        _SCOPE = scope
    _SCOPE["CURRENT_BLOCK"] = CURRENT_BLOCK
    return _SCOPE


def worker(block, expected, lease_fd):
    global CURRENT_BLOCK
    require(type(block) is int and block in range(4), "fixed serial ABBA block required")
    CURRENT_BLOCK = block
    return _private_scope()["worker"](6, expected, lease_fd)


def run_parent(args, lease):
    previous.recorded.fresh_output(OUTPUT)
    before, _, _ = snapshot()
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None and "torch" not in sys.modules and "carla" not in sys.modules, "source/metadata declaration only")
        previous.recorded.fresh_output(PLAN)
        original.write_new_json(PLAN, dict(schema=SCHEMA+".prospective", provenance="HH_261005 - Fix whole888 serial ABBA masks, cold first calls and unchanged numerical guards before any model work.", **before))
        print(json.dumps(dict(status="DECLARED_NOT_MEASURED", declaration_sha256=digest(PLAN))))
        return 0
    require(args.expected_declaration_sha256 == digest(PLAN), "exact prospective affinity declaration SHA required")
    plan = read_json(PLAN)
    require(plan["schema"] == SCHEMA+".prospective" and all(plan[name] == value for name, value in before.items()), "prospective source/input/asset/environment changed")
    previous.recorded.disk_guard()
    OUTPUT.mkdir(parents=True, exist_ok=False)
    attempts = [dict(**block, status="UNEXECUTED", planned_contexts=888, planned_timed_calls=30) for block in BLOCKS]
    arms = []
    try:
        for block in BLOCKS:
            item = attempts[block["index"]]
            item["status"] = "STARTED"
            command = [sys.executable, str(ROOT/"tools/workspace_python.py"), "-m", MODULE, "--", "--mode", "worker", "--block", str(block["index"]),
                "--expected-declaration-sha256", args.expected_declaration_sha256, "--runtime-lease-fd", str(lease.fileno())]
            with (OUTPUT/(block["name"]+".stdout.log")).open("xb") as stdout, (OUTPUT/(block["name"]+".stderr.log")).open("xb") as stderr:
                result = subprocess.run(command, cwd=ROOT, env=dict(os.environ), pass_fds=(lease.fileno(),), stdout=stdout, stderr=stderr, check=False)
            item.update(status="COMPLETED" if result.returncode == 0 else "REJECTED", exit_code=result.returncode, command=command)
            original.write_new_json(OUTPUT/(block["name"]+".process_receipt.json"), item)
            require(result.returncode == 0, "fresh affinity block rejected; all prefixes retained, no retry")
            root = OUTPUT/block["name"]
            summary = read_json(root/"summary.json")
            require(summary["threads"] == 6 and summary["completed_contexts"] == 888 and summary["timed_calls"] == 30
                and digest(root/"all888_context_rows.jsonl") == summary["all_context_rows_sha256"], "complete original worker archive required")
            rows = previous.jsonl(root/"all888_context_rows.jsonl")
            receipts = previous.jsonl(root/"affinity_thread_receipts.jsonl")
            mask = list(ALL_CPUS if block["variant"] == "A" else PCORE_CPUS)
            applied = read_json(root/"affinity_applied.json")
            require(applied["block"] == block and applied["declared_mask"] == mask and applied["original_mask"] == list(ALL_CPUS)
                and applied["before_any_Torch_or_model_import"] is True and applied["mutation_count"] == 1,
                "original pre-Torch mask application receipt required")
            require(len(receipts) == 61 and all(row["declared_mask"] == mask and row["all_observed_thread_masks_equal"] is True
                and row["all_observed_process_thread_identities"]
                and all(thread["CPU_affinity"] == mask for thread in row["all_observed_process_thread_identities"]) for row in receipts), "complete observed affinity proof required")
            arms.append(dict(threads=6, summary=summary, rows=rows, affinity_block=block,
                affinity_applied_sha256=digest(root/"affinity_applied.json"), affinity_thread_receipts_sha256=digest(root/"affinity_thread_receipts.jsonl")))
            previous.recorded.disk_guard()
        comparison = _private_scope()["compare_arms"](arms)
        for index, row in enumerate(comparison):
            _private_scope()["bounded_append"](OUTPUT, "all3552_comparison_rows.jsonl", dict(row, affinity_block=BLOCKS[index//888]))
        after, _, _ = snapshot()
        require(before == after and sum(previous.recorded.tree_members(OUTPUT)["files"].values()) <= original.MAX_TOTAL_BYTES, "unchanged original closure and evidence cap required")
        ordinary_count = sum(row["ordinary_decoder"]["status"] == "ORDINARY_GO_COUNTERFACTUAL_ONLY" for row in comparison)
        passed = ordinary_count > 0 and all(row["all_five_raw_heads_strict_allclose"] for row in comparison) and all(row["ordinary_decoder"]["ordinary_numeric_screen_passed"] for row in comparison
            if row["ordinary_decoder"]["status"] == "ORDINARY_GO_COUNTERFACTUAL_ONLY") and not any(row["ordinary_decoder_rejected"] for row in comparison)
        summary = dict(schema=SCHEMA, status="OFFLINE_NUMERIC_PASS_NOT_LIVE_GAIN" if passed else "FAIL_RETAINED", block_summaries=[dict(block=block, original_summary=arm["summary"],
                affinity_applied_sha256=arm["affinity_applied_sha256"], affinity_thread_receipts_sha256=arm["affinity_thread_receipts_sha256"]) for block, arm in zip(BLOCKS, arms)],
            complete_contexts_per_block=888, completed_comparisons=len(comparison), timed_calls_per_block=30, total_timed_calls=120,
            ordinary_decoded_comparison_count=ordinary_count, comparison_rows_sha256=digest(OUTPUT/"all3552_comparison_rows.jsonl"),
            serial_ABBA_not_interleaved_pairs=True, OS_warming_and_order_confounding_disclosed=True,
            raw_ATOL=1e-6, raw_RTOL=1e-6, full_safety_engine_parity_verified=False, terminal_proof_verified=False,
            source_input_assets_unchanged=True, original_actual_BGRA_or_raw_NN_equality_claimed=False,
            actual_sensor_latency_gain_proven=False, model_promoted=False, main_functions_approved=0, subfunctions_approved=0)
        original.write_new_json(OUTPUT/"summary.json", summary)
        return 0 if passed else 2
    except BaseException as error:
        original.write_new_json(OUTPUT/"FAILED_attempt.json", dict(schema=SCHEMA+".failed", all_four_declared_block_attempts=attempts,
            complete_prefix_block_count=len(arms), planned_contexts=3552, planned_timed_calls=120,
            exception_type=type(error).__name__, exception=str(error), failed_or_unexecuted_tails_retained=True, no_retry=True, model_promoted=False))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "execute", "worker"), required=True)
    parser.add_argument("--expected-declaration-sha256")
    parser.add_argument("--block", type=int, choices=range(4))
    parser.add_argument("--runtime-lease-fd", type=int)
    args = parser.parse_args(argv)
    if args.mode == "worker":
        require(args.block is not None and args.runtime_lease_fd is not None, "internal declared affinity worker only")
        return worker(args.block, args.expected_declaration_sha256, args.runtime_lease_fd)
    require(args.block is None and args.runtime_lease_fd is None, "worker arguments cannot change the parent declaration")
    with original.safe_path(previous.LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX|fcntl.LOCK_NB)
        try: return run_parent(args, lease)
        finally: fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
