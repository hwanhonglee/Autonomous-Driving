"""HH_261005 - Independently recompute all60 timing/count rows and complete original/source/library seals, retaining V1 failure."""

import json
from pathlib import Path
import statistics

from . import benchmark_full209_native_quota_v2_20261005 as fresh


def main():
    io = fresh.previous.io
    with fresh.activated():
        benchmark = fresh.previous
        plan = io.read_json(benchmark.PLAN)
        summary = io.read_json(fresh.OUTPUT / "summary.json")
        rows = [json.loads(line) for line in (fresh.OUTPUT / "all60_walks.jsonl").read_text().splitlines()]
        io.require(len(rows) == 60 and summary["status"] == "COMPLETED_ALL60_EQUAL", "complete result required")
        counts = summary["original_counts"]
        for index, row in enumerate(rows):
            pair, position = divmod(index, 2)
            arm = (("old", "native") if pair % 2 == 0 else ("native", "old"))[position]
            io.require(row["pair"] == pair and row["arm"] == arm and row["status"] == "PASS", "fixed unfiltered order differs")
            start, finish = row["start_monotonic_ns"], row["finish_monotonic_ns"]
            io.require(type(start) is int and type(finish) is int and finish >= start
                       and row["elapsed_ms"] == (finish-start)/1e6, "actual clock/timer differs")
            io.require(row["returned_bytes"] == counts["regular_file_bytes"]
                       and all(row["counters"].get(key) == value for key, value in counts.items()), "every full counter differs")
            if arm == "native":
                io.require(row["counters"]["native_census_library_sha256"] == plan["build"]["library_sha256"]
                           and row["counters"]["native_census_source_sha256"] == plan["build"]["source_sha256"],
                           "actual native engine/source identity differs")
        recomputed = {}
        for arm in ("old", "native"):
            values = [row["elapsed_ms"] for row in rows if row["arm"] == arm]
            ordered = sorted(values)
            point = .95 * 29
            recomputed[arm] = dict(count=30, mean_ms=statistics.mean(values), median_ms=statistics.median(values),
                                   p95_ms=ordered[27] + (ordered[28]-ordered[27])*(point-27), maximum_ms=max(values))
        io.require(recomputed == summary["timings"], "independent statistics differ")
        io.require(benchmark.inputs() == plan["inputs"] and benchmark.sources() == plan["sources"]
                   and io.digest(benchmark.LIBRARY) == plan["build"]["library_sha256"], "whole original/source/engine seal differs")
        attempt = io.read_json(benchmark.PLAN_DIR / "execution_result.json")
        failure = io.read_json(benchmark.ROOT / "storage/experiments/plans/2026-10-05/full209_native_quota_v1/execution_result.json")
        io.require(attempt["exit_code"] == 0 and failure["exit_code"] == 1
                   and failure["error_type"] == "FileNotFoundError", "original failure/completion changed")
        result = dict(provenance="HH_261005 - Independently verify complete counterfactual without promoting a runtime/function.",
                      status="PASS_COMPLETE_INDEPENDENT_COUNTERFACTUAL_POSTCHECK", all60_rows_checked=True,
                      original7486_files_unchanged=True, source_count=len(plan["sources"]), original_V1_failure_retained=True,
                      declaration_sha256=io.digest(benchmark.PLAN), library_sha256=plan["build"]["library_sha256"],
                      C_source_sha256=plan["build"]["source_sha256"], original_counts=counts, timings=recomputed,
                      checker_sha256=io.digest(Path(__file__).resolve()), actual_live_gain_proven=False,
                      formal_function_approval=False)
        io.write_new_json(fresh.OUTPUT / "ROOT_complete_independent_postcheck.json", result)
        print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
