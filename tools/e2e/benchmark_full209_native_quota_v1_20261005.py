"""HH_261005 - Compare complete old/native quota walks on all frozen V4 final-tree entries, not live sensor latency."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import statistics
import time

from . import benchmark_scandir_episode_quota_v1_20261004 as io
from . import audit_scandir_episode_quota_parity_v1_20261004 as original
from . import full_tree_quota_native_v1_20261005 as native


ROOT = io.ROOT
SCHEMA = "portable_e2e.V4_all_entries_native_quota_ABBA.20261005.v1"
CALLER = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v4/ROOT_actual_publication_caller.json"
CALLER_SHA = "39d0d2d6c03a39229f396e985ab1211064dd40561aaad1a8726ef470b569c0e0"
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v4"
EPISODE = RUN / "episode"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-05/full209_native_quota_v1"
PLAN = PLAN_DIR / "declaration.json"
LIBRARY = PLAN_DIR / "lib_full_tree_quota_v1.so"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v1"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
TEST = ROOT / "learning/tests/test_benchmark_full209_native_quota_v1_20261005.py"
HERE = Path(__file__).resolve()
PAIRS = 30
FIELDS = ("entries", "directories_including_root", "regular_files", "regular_file_bytes", "special_nodes_ignored_like_original")


def sources():
    # HH_261005 - Include the actual compiler binary and all helper/tests; no implicit changed native code is admitted.
    paths = (HERE, TEST, Path(io.__file__), Path(original.__file__), Path(native.__file__), native.SOURCE,
             ROOT / "learning/tests/test_full_tree_quota_native_v1_20261005.py",
             ROOT / "learning/tests/test_scandir_episode_quota_parity_v1_20261004.py",
             ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")
    require = io.require
    require(io.digest(Path(original.__file__)) == "92126ab2ffeaac19a546d3e87aef415d086f078af6b68a8fbdb29c370b65ec47",
            "frozen original complete quota helper changed")
    return {str(path): io.digest(path) for path in (*paths, Path("/usr/bin/gcc").resolve())}


def inputs():
    """HH_261005 - Verify all7486 original files and all episode membership; full SHA reads occur only outside timed calls."""
    io.require(io.digest(CALLER) == CALLER_SHA, "frozen complete V4 caller changed")
    value = io.read_json(CALLER)
    io.require(value["run_root"] == str(RUN) and len(value["run_files_sha256"]) == 7486, "wrong original run")
    inventory = io.tree_members(RUN)
    io.require(set(inventory["files"]) == set(value["run_files_sha256"]), "original whole run membership changed")
    for name, sha in value["run_files_sha256"].items():
        io.require(io.digest(RUN / name) == sha, "original capture changed: " + name)
    io.require(io.tree_members(RUN) == inventory and io.digest(CALLER) == CALLER_SHA, "original changed during seal")
    episode = io.tree_members(EPISODE)
    return dict(caller_sha256=CALLER_SHA, whole_file_count=7486,
                whole_membership_sha256=native.hashlib.sha256(json.dumps(inventory, sort_keys=True).encode()).hexdigest(),
                episode_membership=episode, episode_logical_bytes=sum(episode["files"].values()),
                all_originals_preserved=True)


def environment():
    return dict(python=os.path.realpath(os.sys.executable), uname=list(os.uname()),
                CPU_affinity=sorted(os.sched_getaffinity(0)), CUDA_VISIBLE_DEVICES=os.environ.get("CUDA_VISIBLE_DEVICES"))


def stats(values):
    ordered = sorted(values)
    # HH_261005 - Linear p95 is reported with every unfiltered call, not only a fast subset.
    position = .95 * (len(ordered) - 1)
    lower = int(position)
    p95 = ordered[lower] + (ordered[min(lower + 1, len(ordered) - 1)] - ordered[lower]) * (position - lower)
    return dict(count=len(values), mean_ms=statistics.mean(values), median_ms=statistics.median(values), p95_ms=p95, maximum_ms=max(values))


def paired_calls(census, episode, expected, sink):
    """HH_261005 - Preserve complete AB/BA order and counters; call scope includes traversal and exact metadata receipts."""
    rows = []
    for index in range(PAIRS):
        for arm in (("old", "native") if index % 2 == 0 else ("native", "old")):
            record = dict(pair=index, arm=arm, status="RUNNING", counters={})
            start = time.perf_counter_ns()
            try:
                call = original.scandir_directory_bytes if arm == "old" else census.directory_bytes
                returned = call(episode, receipt=record["counters"])
                finish = time.perf_counter_ns()
                record.update(start_monotonic_ns=start, finish_monotonic_ns=finish, elapsed_ms=(finish-start)/1e6,
                              returned_bytes=returned)
                io.require(returned == expected["regular_file_bytes"]
                           and all(record["counters"].get(key) == expected[key] for key in FIELDS)
                           and record["counters"].get("complete_walk") is True
                           and record["counters"].get("cached_or_incremental_byte_count_used") is False,
                           "complete original/native counters differ")
                record["status"] = "PASS"
            except BaseException as error:
                record.update(status="FAIL", error=type(error).__name__ + ": " + str(error),
                              failure_observed_monotonic_ns=time.perf_counter_ns())
                sink(record)
                raise
            sink(record)
            rows.append(record)
    return rows


def execute(plan):
    expected_sources = plan["sources"]
    io.require(sources() == expected_sources and environment() == plan["environment"] and inputs() == plan["inputs"],
               "prospective source/environment/full-input seal differs")
    census = native.NativeFullCensus(LIBRARY, expected_source_sha256=plan["build"]["source_sha256"],
                                    expected_library_sha256=plan["build"]["library_sha256"])
    io.safe_path(OUTPUT, missing_tail=True)
    OUTPUT.mkdir(exist_ok=False)
    expected = {}
    original.scandir_directory_bytes(EPISODE, receipt=expected)
    io.require(expected["regular_file_bytes"] == plan["inputs"]["episode_logical_bytes"], "whole logical bytes differ")
    journal = OUTPUT / "all60_walks.jsonl"
    with journal.open("x", encoding="utf-8") as stream:
        def sink(row):
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
        rows = paired_calls(census, EPISODE, expected, sink)
    io.require(inputs() == plan["inputs"] and sources() == expected_sources
               and io.digest(LIBRARY) == plan["build"]["library_sha256"], "post-run seals changed")
    summary = dict(schema=SCHEMA, status="COMPLETED_ALL60_EQUAL", original_counts={key:expected[key] for key in FIELDS},
                   timings={arm:stats([row["elapsed_ms"] for row in rows if row["arm"] == arm]) for arm in ("old", "native")},
                   actual_sensor_latency_proven=False, actual_capture_activated=False, model_executed=False,
                   no_membership_cache_or_guard_omission=True, failed_tail_cropped=False,
                   warm_OS_cache_after_full_integrity_scan_disclosed=True,
                   main_functions_approved=0, subfunctions_approved=0)
    io.write_new_json(OUTPUT / "summary.json", summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "execute"), required=True)
    parser.add_argument("--expected-declaration-sha256")
    args = parser.parse_args(argv)
    with io.safe_path(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.mode == "declare":
            io.require(not PLAN.exists() and not LIBRARY.exists(), "fresh declaration/artifact only")
            source = sources()
            build = native.compile_new(LIBRARY, expected_source_sha256=io.digest(native.SOURCE))
            plan = dict(schema=SCHEMA + ".declaration", sources=source, build=build, inputs=inputs(), environment=environment(),
                        PAIRS=PAIRS, fixed_order="AB on even pairs, BA on odd pairs", cold_calls_removed=0,
                        no_actual_model_simulator_or_optimization=True, every_entry_every_call=True,
                        original_quota_free_floor_and_reserves_unchanged=True,
                        formal_function_approval=False)
            io.write_new_json(PLAN, plan)
            print(json.dumps(dict(declaration=str(PLAN), sha256=io.digest(PLAN), source_count=len(source))))
        else:
            io.require(args.expected_declaration_sha256 and io.digest(PLAN) == args.expected_declaration_sha256,
                       "external prospective declaration SHA required")
            # HH_261005 - Preserve pre/post-seal exceptions as create-only attempt receipts, not just successful timed rows.
            started = PLAN_DIR / "execution_started.json"
            finished = PLAN_DIR / "execution_result.json"
            io.require(not started.exists() and not finished.exists(), "one fresh declared execution only")
            io.write_new_json(started, dict(schema=SCHEMA + ".attempt", status="RUNNING",
                                           declaration_sha256=args.expected_declaration_sha256,
                                           started_monotonic_ns=time.perf_counter_ns()))
            result = dict(schema=SCHEMA + ".attempt_result", status="RUNNING", output=str(OUTPUT))
            try:
                summary = execute(io.read_json(PLAN))
                result.update(status="COMPLETED", exit_code=0, summary_sha256=io.digest(OUTPUT / "summary.json"))
                print(json.dumps(summary, sort_keys=True))
            except BaseException as error:
                result.update(status="FAILED_PREFIX_RETAINED", exit_code=1,
                              error_type=type(error).__name__, error=str(error))
                raise
            finally:
                result["finished_monotonic_ns"] = time.perf_counter_ns()
                io.write_new_json(finished, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
