"""HH_261004 - Compare two complete quota walks on one sealed final tree; never activate a runtime or claim sensor-latency improvement."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path, PurePosixPath
import stat
import statistics
import time


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
HELPER = ROOT / "tools/e2e/audit_scandir_episode_quota_parity_v1_20261004.py"
TEST = ROOT / "learning/tests/test_benchmark_scandir_episode_quota_v1_20261004.py"
FIXED_SOURCE_SHA = {
    "tools/e2e/audit_scandir_episode_quota_parity_v1_20261004.py": "92126ab2ffeaac19a546d3e87aef415d086f078af6b68a8fbdb29c370b65ec47",
    "learning/tests/test_scandir_episode_quota_parity_v1_20261004.py": "8f1c527b7345d2e64dabfd83b1758bc99af60bfa6bf266f5ae7a30fdac5aba98",
    "tools/e2e/research_temporal_v2_owned_camera_town07_20261001.py": "fbeb39844f07a4e1928665253ded08038fa0cc27f705d2c6fad2728d934d800b",
    "tools/canonical_workspace.py": "5d5ee618dba8dfdd84cc6da2877fb2dce8b5ba0017e4a503eb3c55d28401d0ed",
    "tools/workspace_python.py": "2221cdf71e4ec840566fb19c180329887e53035ef1f319b02221c547141a5b21",
}
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v3"
CALLER = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_callback45_v3/ROOT_actual_publication_caller.json"
CALLER_SHA = "a7c9ca4f2d5d3698a65cf4a46c0aee924306ba3225630576e6c32310c72de4f0"
CALLER_SCHEMA = "portable_e2e.actual_town03_known_dev_straight_fixed45_publication.20261004.callback45_v3.caller_pins"
EPISODE = RUN / "episode.partial"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/callback45_v3_complete_quota_scandir_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/callback45_v3_complete_quota_scandir_counterfactual_v1"
SCHEMA = "portable_e2e.sealed_final_tree_complete_quota_ABBA_counterfactual.20261004.v1"
PAIRS = 30
MAX_JSON = 2 * 1024**2


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def safe_path(path, *, directory=False, missing_tail=False):
    """HH_261004 - Reject aliases in every existing ancestor before reading inputs or creating only the declared new output."""
    path = Path(os.path.abspath(path))
    require(path == Path(path.anchor).joinpath(*path.parts[1:]), "canonical absolute path required")
    for item in reversed((path, *path.parents)):
        try:
            metadata = item.lstat()
        except FileNotFoundError:
            require(missing_tail, "missing sealed input: " + str(item))
            continue
        require(not stat.S_ISLNK(metadata.st_mode), "symlink forbidden: " + str(item))
        if item != path:
            require(stat.S_ISDIR(metadata.st_mode), "real ancestor directory required")
    if not missing_tail:
        metadata = path.lstat()
        require(stat.S_ISDIR(metadata.st_mode) if directory else stat.S_ISREG(metadata.st_mode), "wrong input type")
    return path


def digest(path):
    """HH_261004 - Hash whole sealed files only outside timed scopes; integrity reads are not model inputs."""
    path = safe_path(path)
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            checksum.update(block)
    return checksum.hexdigest()


def read_json(path):
    path = safe_path(path)
    require(path.stat().st_size <= MAX_JSON, "bounded metadata JSON required")
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key forbidden")
            result[key] = value
        return result
    with path.open(encoding="utf-8") as stream:
        return json.load(stream, object_pairs_hook=pairs,
                         parse_constant=lambda value: require(False, "nonfinite JSON forbidden"))


def source_snapshot():
    result = {name: digest(ROOT / name) for name in FIXED_SOURCE_SHA}
    require(result == FIXED_SOURCE_SHA, "frozen quota helper/reference/adapter source differs")
    result.update({str(path.relative_to(ROOT)): digest(path) for path in (HERE, TEST)})
    return result


def tree_members(directory):
    """HH_261004 - Inventory every final-tree entry outside timing; aliases, special nodes and unreadable metadata deny admission."""
    directory = safe_path(directory, directory=True)
    files, directories, pending = {}, ["."], [directory]
    while pending:
        current = pending.pop()
        with os.scandir(current) as entries:
            for entry in entries:
                require(not entry.is_symlink(), "sealed final tree contains symlink")
                metadata = entry.stat(follow_symlinks=False)
                relative = str(Path(entry.path).relative_to(directory))
                if stat.S_ISDIR(metadata.st_mode):
                    directories.append(relative)
                    pending.append(Path(entry.path))
                else:
                    require(stat.S_ISREG(metadata.st_mode), "sealed final tree requires regular files only")
                    files[relative] = metadata.st_size
    return dict(files=dict(sorted(files.items())), directories=sorted(directories))


def input_snapshot():
    """HH_261004 - Reproduce the entire original actual-run manifest before and after, including all JPEG integrity hashes outside timing."""
    require(digest(CALLER) == CALLER_SHA, "original full-run manifest SHA differs")
    caller = read_json(CALLER)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("run_root") == str(RUN), "wrong frozen actual run manifest")
    manifest = caller.get("run_files_sha256")
    require(isinstance(manifest, dict) and 0 < len(manifest) <= 10000, "bounded complete original manifest required")
    inventory = tree_members(RUN)
    require(set(inventory["files"]) == set(manifest), "complete original run membership differs")
    for name, expected_sha in manifest.items():
        relative = PurePosixPath(name)
        require(not relative.is_absolute() and relative.parts and all(part not in (".", "..") for part in relative.parts)
                and str(relative) == name, "canonical manifest member required")
        require(isinstance(expected_sha, str) and len(expected_sha) == 64
                and all(character in "0123456789abcdef" for character in expected_sha), "strict manifest SHA required")
        require(digest(RUN / name) == expected_sha, "original run member SHA differs: " + name)
    # HH_261004 - Hashing/metadata inspection may warm OS caches; this is explicitly a final-tree warm counterfactual, not a live prefix.
    require(digest(CALLER) == CALLER_SHA and tree_members(RUN) == inventory, "original inputs changed while verifying manifest")
    episode_files = {name: size for name, size in inventory["files"].items() if name.startswith("episode.partial/")}
    require(episode_files and EPISODE.is_dir(), "complete actual episode required")
    return dict(original_manifest_sha256=CALLER_SHA, full_manifest_file_count=len(manifest),
                full_manifest_SHA_verified=True, full_run_metadata_inventory=inventory,
                episode_regular_file_count=len(episode_files), episode_regular_file_bytes=sum(episode_files.values()),
                episode_final_tree_metadata_only_during_timing=True)


def contract():
    return dict(paired_iterations=PAIRS, timed_calls=2 * PAIRS,
        pair_order="odd pairs AB, even pairs BA; A=exact sealed original Path.rglob, B=complete fd-relative scandir",
        timing_clock="time.perf_counter_ns", timed_scope="one whole final-tree directory_bytes call, no payload reads",
        full_original_manifest_and_source_SHA_verified_before_and_after=True,
        warm_cache_note="Whole-file integrity hashes and inventory run outside timing and may warm filesystem caches.",
        no_live_450_prefix_claim=True, no_payload_decode_or_learning_or_inference=True,
        no_incremental_count_or_membership_cache=True, every_call_full_tree_required=True,
        original_quota_free_floor_12MiB_and_closing_checks_not_activated_or_removed=True,
        all_attempted_calls_retained_including_errors_and_count_mismatches=True,
        no_favorable_order_or_iteration_selection=True, sensor_latency_or_causation_proven=False,
        no_runtime_activation=True, no_CARLA_GPU_SSH_install_training=True,
        image_normalization_activated=False, model_promoted=False, formal_function_approval=False)


def write_new_json(path, document):
    safe_path(path, missing_tail=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n")


def declare():
    require(not PLAN.exists() and not OUTPUT.exists(), "fresh prospective plan/output required")
    document = dict(schema=SCHEMA + ".prospective", provenance="HH_261004 - Seal a metadata-only final-tree comparison before any timed call.",
                    run_root=str(RUN), episode=str(EPISODE), output=str(OUTPUT), original_manifest=str(CALLER),
                    source_sha256=source_snapshot(), input_seal=input_snapshot(), contract=contract())
    write_new_json(PLAN, document)
    return dict(declaration=str(PLAN), declaration_sha256=digest(PLAN), status="DECLARED_NOT_MEASURED")


def paired_walks(functions, directory, expected_bytes, *, clock=time.perf_counter_ns):
    """HH_261004 - Retain all sixty calls in fixed AB/BA order, including failing/count-mismatched calls; exclude no slow tail."""
    require(set(functions) == {"A", "B"}, "exact two fixed accounting implementations required")
    rows = []
    for pair in range(1, PAIRS + 1):
        order = "AB" if pair % 2 else "BA"
        for position, name in enumerate(order, 1):
            started = clock()
            value, error = None, None
            try:
                value = functions[name](directory)
            except Exception as exception:
                error = dict(type=type(exception).__name__, message=str(exception))
            finished = clock()
            rows.append(dict(sequence=len(rows) + 1, pair=pair, pair_order=order, position=position, implementation=name,
                             started_monotonic_ns=started, finished_monotonic_ns=finished, elapsed_ns=finished - started,
                             returned_regular_file_bytes=value, expected_regular_file_bytes=expected_bytes,
                             exact_byte_count_passed=type(value) is int and value == expected_bytes and error is None,
                             clock_passed=type(started) is int and type(finished) is int and finished > started,
                             exception=error))
    return rows


def timing_summary(rows):
    result = {}
    for name in ("A", "B"):
        selected = [row for row in rows if row["implementation"] == name]
        values = sorted(row["elapsed_ns"] / 1e6 for row in selected)
        result[name] = dict(attempted_calls=len(selected), exact_count_calls=sum(row["exact_byte_count_passed"] for row in selected),
                            count_or_call_failures=sum(not row["exact_byte_count_passed"] for row in selected),
                            all_attempted_elapsed_included=True,
                            milliseconds=dict(min=min(values), median=statistics.median(values),
                                              p95_nearest_rank=values[math.ceil(.95 * len(values)) - 1], max=max(values)))
    pairs = []
    for pair in range(1, PAIRS + 1):
        values = {row["implementation"]: row for row in rows if row["pair"] == pair}
        old, new = values["A"]["elapsed_ns"], values["B"]["elapsed_ns"]
        pairs.append(dict(pair=pair, pair_order=values["A"]["pair_order"], old_minus_new_ms=(old - new) / 1e6,
                          old_over_new_ratio=old / new if new > 0 else None,
                          both_exact_count_passed=all(row["exact_byte_count_passed"] for row in values.values())))
    result["every_paired_comparison"] = pairs
    return result


def execute(expected_declaration_sha):
    require(isinstance(expected_declaration_sha, str) and digest(PLAN) == expected_declaration_sha, "prospective declaration SHA differs")
    plan = read_json(PLAN)
    require(plan.get("schema") == SCHEMA + ".prospective" and plan.get("contract") == contract()
            and plan.get("run_root") == str(RUN) and plan.get("episode") == str(EPISODE)
            and plan.get("output") == str(OUTPUT) and plan.get("original_manifest") == str(CALLER), "exact prospective scope differs")
    require(not OUTPUT.exists(), "single create-only benchmark output required")
    sources_before, inputs_before = source_snapshot(), input_snapshot()
    require(sources_before == plan["source_sha256"] and inputs_before == plan["input_seal"], "prospective source/input seal differs")
    specification = importlib.util.spec_from_file_location("sealed_quota_metadata_counterfactual_only", HELPER)
    helper = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(helper)
    functions = dict(A=helper.original_directory_bytes(), B=helper.scandir_directory_bytes)
    safe_path(OUTPUT, missing_tail=True)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    rows = paired_walks(functions, EPISODE, inputs_before["episode_regular_file_bytes"])
    post_errors, inputs_after, sources_after = [], None, None
    # HH_261004 - A post-seal failure must not hide any timed call; preserve its full rows as a failed fresh artifact.
    for name, operation in (("input", input_snapshot), ("source", source_snapshot)):
        try:
            value = operation()
            if name == "input":
                inputs_after = value
            else:
                sources_after = value
        except Exception as exception:
            post_errors.append(dict(check=name, type=type(exception).__name__, message=str(exception)))
    unchanged = not post_errors and inputs_after == inputs_before and sources_after == sources_before
    passed = unchanged and len(rows) == 2 * PAIRS and all(row["exact_byte_count_passed"] and row["clock_passed"] for row in rows)
    report = dict(schema=SCHEMA + ".actual_metadata_only", provenance="HH_261004 - Keep all fixed pairs and errors; do not regrade the actual callback45_v3 FAIL.",
        status="PASS_METADATA_COUNT_PARITY" if passed else "FAIL_METADATA_COUNTERFACTUAL", declaration_sha256=expected_declaration_sha,
        run_root=str(RUN), episode=str(EPISODE), contract=contract(), source_sha256_before=sources_before, source_sha256_after=sources_after,
        input_seal_before=inputs_before, input_seal_after=inputs_after, source_and_input_unchanged=unchanged, post_seal_errors=post_errors,
        all60_timing_rows=rows, timing=timing_summary(rows), actual_callback45_v3_outcome_unchanged=True,
        sensor_latency_or_causation_proven=False, runtime_activated=False, model_promoted=False, formal_function_approval=False)
    write_new_json(OUTPUT / "report.json", report)
    old, new = report["timing"]["A"], report["timing"]["B"]
    readme = f"""<!-- HH_261004 - Explain a whole-final-tree metadata comparison separately from an actual autonomous drive. -->
# 전체 저장 폴더 순회 비교 — 실제 주행 개선은 아직 미검증

상태: {report['status']}. 같은 callback45_v3의 최종 폴더를 고정 30쌍·60회 순회했습니다.
매 호출은 파일 내용을 열지 않고 일반 파일의 논리 바이트 수만 합산합니다.
기존/새 구현은 각각 {old['exact_count_calls']}/30·{new['exact_count_calls']}/30회 동일한 {inputs_before['episode_regular_file_bytes']:,}바이트를 반환했습니다.

| 순회 구현 | median | p95 | max |
| --- | ---: | ---: | ---: |
| 기존 Path.rglob | {old['milliseconds']['median']:.3f}ms | {old['milliseconds']['p95_nearest_rank']:.3f}ms | {old['milliseconds']['max']:.3f}ms |
| 새 전체 scandir | {new['milliseconds']['median']:.3f}ms | {new['milliseconds']['p95_nearest_rank']:.3f}ms | {new['milliseconds']['max']:.3f}ms |

전체 원본 manifest·소스의 측정 전후 일치: {unchanged}. 모든 실패·느린 측정도 [전체 결과](report.json)에 남았습니다.
무결성 검사는 측정 밖에서 원본 파일을 해시하여 OS 캐시를 데울 수 있습니다. 따라서 cold-cache, 주행 중 성장하는 450개 폴더 상태, 실제 센서→계획 지연을 증명하지 않습니다.
기존 callback45_v3의 100ms FAIL은 그대로입니다. 실제 실행기·모델·제어·안전 검사·normalization은 변경하거나 활성화하지 않았습니다.

재현 소스: [벤치마크](../../../../../tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py).
선언: `{PLAN}` · SHA `{expected_declaration_sha}`.
출력은 create-only입니다. 같은 결과 경로로 다시 실행하여 덮어쓸 수 없습니다. 새 측정은 새 revision과 사전 선언이 필요합니다.
"""
    with (OUTPUT / "README.md").open("x", encoding="utf-8") as stream:
        stream.write(readme)
    manifest = dict(schema=SCHEMA + ".artifact_manifest", create_only=True,
                    declaration_sha256=expected_declaration_sha, source_sha256=sources_before,
                    artifact_sha256={name: digest(OUTPUT / name) for name in ("report.json", "README.md")})
    write_new_json(OUTPUT / "artifact_manifest.json", manifest)
    return dict(status=report["status"], attempted_calls=len(rows), exact_count_calls=sum(row["exact_byte_count_passed"] for row in rows),
                source_and_input_unchanged=unchanged, timing=report["timing"], output=str(OUTPUT),
                report_sha256=digest(OUTPUT / "report.json"), artifact_manifest_sha256=digest(OUTPUT / "artifact_manifest.json"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--declare", action="store_true")
    modes.add_argument("--execute", action="store_true")
    parser.add_argument("--expected-declaration-sha256")
    args = parser.parse_args(argv)
    require(bool(args.execute) == bool(args.expected_declaration_sha256), "execute alone requires an explicit prospective SHA")
    result = execute(args.expected_declaration_sha256) if args.execute else declare()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["status"] in ("DECLARED_NOT_MEASURED", "PASS_METADATA_COUNT_PARITY") else 1


if __name__ == "__main__":
    raise SystemExit(main())
