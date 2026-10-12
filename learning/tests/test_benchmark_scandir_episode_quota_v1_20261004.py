"""HH_261004 - Check prospective seals, every AB/BA row and failure retention on synthetic trees without measuring an actual drive."""

import ast
import hashlib
import json
import os
from pathlib import Path

import pytest

from scripts.e2e import benchmark_scandir_episode_quota_v1_20261004 as benchmark


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def synthetic(tmp_path, monkeypatch):
    # HH_261004 - Use only newly generated toy bytes; never read real JPEG payloads or invoke the actual run in CPU tests.
    root = tmp_path / "run"
    root.mkdir()
    for name, content in (("diagnostic.json", b"FAIL preserved"), ("episode.partial/state", b"state"),
                          ("episode.partial/images/.hidden", b"toy pixel-free bytes"),
                          ("episode.partial/images/front/1.jpg", b"not a real image")):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    (root / "empty_directory").mkdir()
    caller = tmp_path / "caller.json"
    manifest = {str(path.relative_to(root)): sha(path) for path in root.rglob("*") if path.is_file()}
    caller.write_text(json.dumps(dict(schema=benchmark.CALLER_SCHEMA, run_root=str(root), run_files_sha256=manifest)))
    monkeypatch.setattr(benchmark, "RUN", root)
    monkeypatch.setattr(benchmark, "EPISODE", root / "episode.partial")
    monkeypatch.setattr(benchmark, "CALLER", caller)
    monkeypatch.setattr(benchmark, "CALLER_SHA", sha(caller))
    monkeypatch.setattr(benchmark, "PLAN", tmp_path / "plans/declaration.json")
    monkeypatch.setattr(benchmark, "OUTPUT", tmp_path / "outputs/new_counterfactual")
    return root, caller


def synthetic_clock():
    counter = iter(range(1, 10000))
    return lambda: next(counter)


def test_exact_30_pairs_ABBA_order_and_all_60_same_synthetic_bytes(synthetic):
    expected = benchmark.input_snapshot()["episode_regular_file_bytes"]
    calls = []
    def function(name):
        def apply(directory):
            assert directory == benchmark.EPISODE
            calls.append(name)
            return expected
        return apply
    rows = benchmark.paired_walks(dict(A=function("A"), B=function("B")), benchmark.EPISODE, expected, clock=synthetic_clock())
    assert len(rows) == len(calls) == 60
    assert calls == list("ABBA" * 15)
    assert [row["sequence"] for row in rows] == list(range(1, 61))
    assert all(row["clock_passed"] and row["exact_byte_count_passed"] for row in rows)
    assert all(row["elapsed_ns"] == 1 for row in rows)
    assert benchmark.timing_summary(rows)["A"]["attempted_calls"] == 30
    assert benchmark.timing_summary(rows)["B"]["exact_count_calls"] == 30


@pytest.mark.parametrize("fault", ["exception", "wrong_count", "boolean_count"])
def test_all_failed_and_slow_rows_retained_not_selected_away(fault):
    calls = []
    def old(directory):
        calls.append("A")
        if fault == "exception":
            raise OSError("synthetic unreadable metadata")
        return False if fault == "boolean_count" else 99
    def new(directory):
        calls.append("B")
        return 10
    rows = benchmark.paired_walks(dict(A=old, B=new), Path("synthetic"), 10, clock=synthetic_clock())
    assert len(rows) == 60 and calls == list("ABBA" * 15)
    assert sum(row["exact_byte_count_passed"] for row in rows) == 30
    summary = benchmark.timing_summary(rows)
    assert summary["A"]["count_or_call_failures"] == 30 and summary["A"]["all_attempted_elapsed_included"]
    assert all(not row["both_exact_count_passed"] for row in summary["every_paired_comparison"])
    assert all(row["exception"] is not None for row in rows if row["implementation"] == "A") == (fault == "exception")


def test_nonpositive_clock_still_preserves_all_rows_but_denies_gate():
    rows = benchmark.paired_walks(dict(A=lambda _: 1, B=lambda _: 1), Path("synthetic"), 1, clock=lambda: 50)
    assert len(rows) == 60 and all(row["exact_byte_count_passed"] for row in rows)
    assert all(not row["clock_passed"] for row in rows)
    assert all(pair["old_over_new_ratio"] is None for pair in benchmark.timing_summary(rows)["every_paired_comparison"])


def test_actual_helper_and_exact_original_on_toy_tree_are_identical(synthetic):
    specification = benchmark.importlib.util.spec_from_file_location("synthetic_test_quota", benchmark.HELPER)
    helper = benchmark.importlib.util.module_from_spec(specification)
    specification.loader.exec_module(helper)
    expected = benchmark.input_snapshot()["episode_regular_file_bytes"]
    assert helper.original_directory_bytes()(benchmark.EPISODE) == helper.scandir_directory_bytes(benchmark.EPISODE) == expected


def test_whole_original_manifest_and_all_hidden_nested_members_verified(synthetic):
    root, _ = synthetic
    snapshot = benchmark.input_snapshot()
    assert snapshot["full_manifest_file_count"] == 4 and snapshot["episode_regular_file_count"] == 3
    assert snapshot["episode_regular_file_bytes"] == sum(path.stat().st_size for path in benchmark.EPISODE.rglob("*") if path.is_file())
    assert "empty_directory" in snapshot["full_run_metadata_inventory"]["directories"]
    assert ".hidden" in " ".join(snapshot["full_run_metadata_inventory"]["files"])
    assert snapshot == benchmark.input_snapshot()
    assert (root / "diagnostic.json").read_bytes() == b"FAIL preserved"


@pytest.mark.parametrize("fault", ["added", "deleted", "changed_same_size", "empty_dir_added", "symlink", "FIFO", "caller_SHA"])
def test_manifest_or_inventory_mutation_rejected(synthetic, fault):
    root, caller = synthetic
    before = benchmark.input_snapshot()
    member = root / "episode.partial/state"
    if fault == "added":
        (root / "extra").write_bytes(b"extra")
    elif fault == "deleted":
        member.unlink()
    elif fault == "changed_same_size":
        member.write_bytes(b"other")
    elif fault == "empty_dir_added":
        (root / "empty_new").mkdir()
        assert benchmark.input_snapshot() != before
        return
    elif fault == "symlink":
        (root / "alias").symlink_to(member)
    elif fault == "FIFO":
        os.mkfifo(root / "fifo")
    else:
        caller.write_text(caller.read_text() + " ")
    with pytest.raises(RuntimeError):
        benchmark.input_snapshot()


@pytest.mark.parametrize("fault", ["schema", "root", "digest_type", "path"])
def test_invalid_original_manifest_schema_members_and_hashes_denied(synthetic, monkeypatch, fault):
    _, caller = synthetic
    document = json.loads(caller.read_text())
    if fault == "schema":
        document["schema"] = "old unrelated run"
    elif fault == "root":
        document["run_root"] = "/tmp/unrelated"
    elif fault == "digest_type":
        document["run_files_sha256"]["diagnostic.json"] = 123
    else:
        document["run_files_sha256"]["../escape"] = "0" * 64
    caller.write_text(json.dumps(document))
    monkeypatch.setattr(benchmark, "CALLER_SHA", sha(caller))
    with pytest.raises(RuntimeError):
        benchmark.input_snapshot()


def test_read_json_denies_duplicate_nonfinite_and_overlarge_metadata(tmp_path, monkeypatch):
    path = tmp_path / "bounded.json"
    for text in ('{"a":1,"a":2}', '{"a":NaN}'):
        path.write_text(text)
        with pytest.raises(RuntimeError):
            benchmark.read_json(path)
    monkeypatch.setattr(benchmark, "MAX_JSON", 10)
    path.write_text('{"a":123456789}')
    with pytest.raises(RuntimeError, match="bounded"):
        benchmark.read_json(path)


def test_frozen_source_snapshot_pins_helper_reference_tests_and_adapters():
    sources = benchmark.source_snapshot()
    assert {name: sources[name] for name in benchmark.FIXED_SOURCE_SHA} == benchmark.FIXED_SOURCE_SHA
    assert sources[str(benchmark.TEST.relative_to(benchmark.ROOT))] == sha(benchmark.TEST)
    assert sources[str(benchmark.HERE.relative_to(benchmark.ROOT))] == sha(benchmark.HERE)
    assert len(sources) == 7


def test_changed_frozen_helper_source_SHA_denies_without_measurement(monkeypatch):
    monkeypatch.setattr(benchmark, "FIXED_SOURCE_SHA", {next(iter(benchmark.FIXED_SOURCE_SHA)): "0" * 64})
    with pytest.raises(RuntimeError, match="frozen"):
        benchmark.source_snapshot()


def test_declaration_makes_no_timed_calls_and_is_create_only(synthetic, monkeypatch):
    monkeypatch.setattr(benchmark, "paired_walks", lambda *args, **kwargs: pytest.fail("timing before declaration"))
    declared = benchmark.declare()
    assert declared["status"] == "DECLARED_NOT_MEASURED"
    assert declared["declaration_sha256"] == sha(benchmark.PLAN) and not benchmark.OUTPUT.exists()
    document = benchmark.read_json(benchmark.PLAN)
    assert document["contract"]["timed_calls"] == 60 and document["contract"]["paired_iterations"] == 30
    with pytest.raises(RuntimeError, match="fresh"):
        benchmark.declare()


def test_single_execute_real_implementations_only_on_synthetic_inputs_preserves_all_rows(synthetic):
    seal = benchmark.declare()["declaration_sha256"]
    result = benchmark.execute(seal)
    assert result["status"] == "PASS_METADATA_COUNT_PARITY" and result["attempted_calls"] == result["exact_count_calls"] == 60
    report = benchmark.read_json(benchmark.OUTPUT / "report.json")
    assert report["input_seal_before"] == report["input_seal_after"]
    assert report["source_sha256_before"] == report["source_sha256_after"]
    assert report["runtime_activated"] is report["model_promoted"] is report["formal_function_approval"] is False
    assert report["sensor_latency_or_causation_proven"] is False and report["actual_callback45_v3_outcome_unchanged"] is True
    assert [row["implementation"] for row in report["all60_timing_rows"]] == list("ABBA" * 15)
    assert sha(benchmark.OUTPUT / "report.json") == result["report_sha256"]
    with pytest.raises(RuntimeError, match="single create-only"):
        benchmark.execute(seal)


@pytest.mark.parametrize("fault", ["stale_plan_SHA", "input", "source", "contract", "existing_output"])
def test_execute_denies_before_any_timing_on_invalid_prospective_seal(synthetic, monkeypatch, fault):
    seal = benchmark.declare()["declaration_sha256"]
    monkeypatch.setattr(benchmark, "paired_walks", lambda *args, **kwargs: pytest.fail("unauthorized timing"))
    if fault == "stale_plan_SHA":
        seal = "0" * 64
    elif fault == "input":
        (benchmark.EPISODE / "state").write_bytes(b"other")
    elif fault in ("source", "contract"):
        document = benchmark.read_json(benchmark.PLAN)
        if fault == "source":
            document["source_sha256"][next(iter(document["source_sha256"]))] = "0" * 64
        else:
            document["contract"]["timed_calls"] = 1
        benchmark.PLAN.write_text(json.dumps(document))
        seal = sha(benchmark.PLAN)
    else:
        benchmark.OUTPUT.mkdir(parents=True)
    with pytest.raises(RuntimeError):
        benchmark.execute(seal)


@pytest.mark.parametrize("fault", ["count", "post_input", "post_source"])
def test_measured_failure_receipt_preserves_every_row_and_forbids_PASS(synthetic, monkeypatch, fault):
    seal = benchmark.declare()["declaration_sha256"]
    original = benchmark.paired_walks
    def failed(functions, directory, expected):
        if fault == "count":
            functions = dict(A=lambda _: expected - 1, B=functions["B"])
        rows = original(functions, directory, expected, clock=synthetic_clock())
        if fault == "post_input":
            (benchmark.EPISODE / "state").write_bytes(b"changed after timing")
        elif fault == "post_source":
            monkeypatch.setattr(benchmark, "source_snapshot", lambda: (_ for _ in ()).throw(RuntimeError("synthetic post source fault")))
        return rows
    monkeypatch.setattr(benchmark, "paired_walks", failed)
    result = benchmark.execute(seal)
    report = benchmark.read_json(benchmark.OUTPUT / "report.json")
    assert result["status"] == "FAIL_METADATA_COUNTERFACTUAL" and len(report["all60_timing_rows"]) == 60
    assert report["timing"]["A"]["attempted_calls"] == report["timing"]["B"]["attempted_calls"] == 30
    if fault == "count":
        assert result["exact_count_calls"] == 30 and report["source_and_input_unchanged"]
    else:
        assert result["exact_count_calls"] == 60 and not report["source_and_input_unchanged"] and report["post_seal_errors"]


def test_symlink_ancestor_and_target_cannot_be_used_for_plan_or_output(tmp_path):
    target = tmp_path / "real"
    target.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(target)
    with pytest.raises(RuntimeError, match="symlink"):
        benchmark.write_new_json(alias / "fresh.json", {})
    assert not (target / "fresh.json").exists()


def test_command_modes_require_exact_prospective_SHA_and_never_mutate_runtime(monkeypatch):
    monkeypatch.setattr(benchmark, "declare", lambda: dict(status="DECLARED_NOT_MEASURED"))
    assert benchmark.main(["--declare"]) == 0
    with pytest.raises(RuntimeError):
        benchmark.main(["--declare", "--expected-declaration-sha256", "0" * 64])
    with pytest.raises(RuntimeError):
        benchmark.main(["--execute"])
    with pytest.raises(SystemExit):
        benchmark.main(["--declare", "--execute"])
    tree = ast.parse(benchmark.HERE.read_text())
    imports = [node for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any("torch" in ast.unparse(node) or "carla" in ast.unparse(node) for node in imports)
    scope = benchmark.contract()
    assert scope["no_live_450_prefix_claim"] and scope["every_call_full_tree_required"]
    assert not scope["image_normalization_activated"] and not scope["formal_function_approval"]
    assert scope["no_runtime_activation"] and not scope["sensor_latency_or_causation_proven"]
