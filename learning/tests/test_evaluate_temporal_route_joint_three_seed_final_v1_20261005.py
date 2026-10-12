"""HH_261005 - Test fixed-final evaluation guards using scalar files and genuine ASTs only."""

import argparse
import ast
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shutil
import time
from types import ModuleType, SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/evaluate_temporal_route_joint_three_seed_final_v1_20261005.py"
RECOVERY = ROOT / "tools/e2e/train_temporal_route_recovery_v3_20261001.py"
PAIR = ROOT / "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"


def pure_module():
    # HH_261005 - Execute stdlib definitions only; no production import, Torch, model or dataset access.
    module = ModuleType("hh_final_evaluator_scalar_fixture")
    scope = vars(module)
    scope.update(__file__=str(SOURCE), argparse=argparse, ast=ast, deepcopy=deepcopy,
        datetime=datetime, timezone=timezone, hashlib=hashlib, inspect=inspect, json=json,
        math=math, os=os, Path=Path, shutil=SimpleNamespace(disk_usage=lambda path: SimpleNamespace(free=10*1024**3)),
        time=time, SimpleNamespace=SimpleNamespace)
    nodes = [node for node in ast.parse(SOURCE.read_text()).body
        if isinstance(node, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future, *nodes], type_ignores=[])), str(SOURCE), "exec"), scope)
    return module


@pytest.fixture
def m():
    return pure_module()


def genuine_function(path, name):
    # HH_261005 - Compile the genuine metric definition without invoking any numerical statement.
    node = next(node for node in ast.parse(path.read_text()).body
        if isinstance(node, ast.FunctionDef) and node.name == name)
    scope = {"__file__": str(path), "__name__": "hh_unexecuted_original_metric"}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    return scope[name]


def scalar_metric(dataset):
    records = []
    for index, example in enumerate(dataset.examples):
        records.append(dict(index=index, token=example.token, metric=1.0))
    for row in records:
        row["episode_id"] = "added_after_emit"
    return records


def scalar_intrinsic(model, device, training, audits, low, legacy, strata, holdout, holdout_audits, known):
    metrics = dict(recovery_train=previous.evaluate_joint(model, None, training, device, audits),
        recovery_holdout=previous.evaluate_joint(model, None, holdout, device, holdout_audits),
        v4=previous.evaluate_joint(model, None, low, device),
        legacy=previous.evaluate_legacy_raw(model, legacy, strata, device),
        known599=previous.evaluate_legacy_raw(model, known, strata, device))
    records = []
    for index in strata["curved"]:
        records.append(dict(index=index, token=legacy.examples[index].token, loss=1.0))
    metrics["intrinsic_legacy_TRAIN_only"] = records
    return metrics


def scalar_joint(model, decoder, dataset, device, audits=None):
    records = []
    for index, example in enumerate(dataset.examples):
        records.append(dict(index=index, token=example.token, metric=1.0))
    return records


def scalar_legacy(model, dataset, strata, device):
    records = []
    for index, example in enumerate(dataset.examples):
        records.append(dict(index=index, token=example.token, metric=1.0))
    return records


def zero_append(dataset):
    return []


def two_appends(dataset):
    records = []
    records.append({})
    records.append({})
    return records


def journal_fixture(m, tmp_path, *, reduced=False, clock=None, budget_seconds=7200):
    # HH_261005 - Small count derivative is explicitly synthetic, never an actual full-cohort claim.
    if reduced:
        m.COUNTS = {name: 1 for name in m.COUNTS}
    datasets = {name: SimpleNamespace(examples=[SimpleNamespace(token=name + "_0")])
        for name in m.COUNTS if name != "intrinsic_legacy_TRAIN_only"}
    expected = {endpoint: {name: [(0, "legacy_0" if name == "intrinsic_legacy_TRAIN_only" else name + "_0")]
        for name in m.COUNTS} for endpoint in m.ENDPOINTS}
    journal = m.Journal(tmp_path, expected, clock=clock or (lambda: 0.0), budget_seconds=budget_seconds)
    journal.bind_datasets(datasets)
    return journal, datasets


def authority_fixture(m, contract):
    training = {"sources": {"synthetic_training.py": "1"*64}, "TRAIN_only": True}
    m.TRAIN_CONTRACT_SHA = m.json_sha(training)
    contract["source_sha256"] = dict(contract["source_sha256"], **training["sources"])
    # HH_261005 - Only absolute synthetic names and declared scalar pins are checked; no baseline file is opened here.
    baselines = {}
    for name, expected in m.BASELINE_EXPECTED.items():
        paths = {field: "/scalar_only_baseline/%s/%s" % (name, field) for field in expected}
        baselines[name] = dict(paths, immutable_input_sha256={paths[field]: pin for field, pin in expected.items()})
    return dict(schema=m.AUTH_SCHEMA, authorized=True,
        purpose="fixed_final_known_DEV_research_evaluation_only", contract=contract,
        contract_sha256=m.json_sha(contract), all_three_whole_training_postchecks_PASS=True,
        GPU_physical_index=0, model_promoted=False, vehicle_control_approved=False, TEST_access_authorized=False,
        training_contract=training, completed_seeds={str(seed): {} for seed in m.SEEDS},
        baseline_assets=baselines)


def test_stdlib_top_level_and_no_eager_numeric_or_dataset_imports():
    tree = ast.parse(SOURCE.read_text())
    imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    names = [alias.name if isinstance(node, ast.Import) else node.module for node in imports for alias in node.names]
    assert not any(name.startswith(("torch", "numpy", "portable_e2e", "scripts", "rclpy")) for name in names)


def test_fixed_five_endpoints_original_counts_71_and_no_selection_or_promotion(m):
    m.source_pins = lambda sources: {"synthetic_training.py": "1"*64}
    contract = m.fixed_contract({})
    assert contract["endpoint_order"] == ["common3000", "GOOD6000", "seed_2026100501", "seed_2026100502", "seed_2026100503"]
    assert contract["full_rows_per_endpoint"] == dict(recovery_train=2718, recovery_holdout=604,
        v4=327, legacy=2487, known599=599, intrinsic_legacy_TRAIN_only=328)
    assert contract["complete_record_denominator"] == 35315
    assert contract["screen_count_per_final"] == 71
    assert (contract["known_DEV_total_available"], contract["known_DEV_eligible"], contract["known_DEV_preexisting_exclusions"]) == (665, 599, 66)
    for key in ("best_checkpoint_selection", "automatic_model_selection", "old_baseline_metrics_reused",
        "model_A_B_six_training_runs_performed", "known_DEV_independent", "optimization_authorized",
        "TEST_access_authorized", "simulator_control_authorized", "vehicle_control_approved", "model_promoted"):
        assert contract[key] is False
    assert contract["formal_main_approved"] == contract["formal_sub_approved"] == 0


def test_exact_root_authority_admits_scalar_contract(m):
    m.source_pins = lambda sources: {}
    contract = m.fixed_contract({})
    m.validate_authority(authority_fixture(m, contract), contract)


@pytest.mark.parametrize("name", ["common3000", "GOOD6000"])
@pytest.mark.parametrize("fault", ["omit_checkpoint", "omit_declaration", "missing_checkpoint_path",
    "missing_declaration_path", "wrong_pin", "duplicate_path"])
def test_original_baseline_checkpoint_declaration_and_exact_membership_required(m, name, fault):
    m.source_pins = lambda sources: {}
    contract = m.fixed_contract({}); grant = authority_fixture(m, contract)
    entry = grant["baseline_assets"][name]
    if fault.startswith("omit_"):
        del entry["immutable_input_sha256"][entry[fault.removeprefix("omit_")]]
    elif fault.startswith("missing_"):
        del entry[fault.removeprefix("missing_").removesuffix("_path")]
    elif fault == "wrong_pin":
        entry["immutable_input_sha256"][entry["checkpoint"]] = "0"*64
    else:
        entry["declaration"] = entry["checkpoint"]
        entry["immutable_input_sha256"] = {entry[field]: pin for field, pin in m.BASELINE_EXPECTED[name].items()}
    with pytest.raises(RuntimeError):
        m.validate_authority(grant, contract)


def test_alarm_and_baseline_byte_seals_are_admitted_before_output_and_torch():
    run = next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "run")
    body = ast.unparse(run)
    assert body.index("validate_authority(authority, proposed)") < body.index("output.mkdir()")
    assert body.index("no unrelated alarm may be overwritten") < body.index("output.mkdir()")
    assert body.index("endpoint['immutable_input_sha256']") < body.index("import torch")
    authority = next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "validate_authority")
    assert "verify_baseline_membership(value['baseline_assets'])" in ast.unparse(authority)


@pytest.mark.parametrize("key,value", [("schema", "old_schema"), ("authorized", False),
    ("purpose", "training"), ("contract_sha256", "0"*64), ("all_three_whole_training_postchecks_PASS", False),
    ("GPU_physical_index", True), ("GPU_physical_index", 1), ("model_promoted", True),
    ("vehicle_control_approved", True), ("TEST_access_authorized", True)])
def test_wrong_authority_refused_before_numeric_access(m, key, value):
    m.source_pins = lambda sources: {}
    contract = m.fixed_contract({}); grant = authority_fixture(m, contract); grant[key] = value
    with pytest.raises(RuntimeError):
        m.validate_authority(grant, contract)


@pytest.mark.parametrize("change", ["order", "missing_seed", "extra_seed", "missing_baseline", "training_source", "training_contract"])
def test_selection_missing_endpoint_and_training_binding_mismatch_refused(m, change):
    m.source_pins = lambda sources: {}
    contract = m.fixed_contract({}); grant = authority_fixture(m, contract)
    if change == "order":
        grant["contract"] = deepcopy(contract); grant["contract"]["endpoint_order"].reverse()
        grant["contract_sha256"] = m.json_sha(grant["contract"])
    elif change == "missing_seed": del grant["completed_seeds"][str(m.SEEDS[-1])]
    elif change == "extra_seed": grant["completed_seeds"]["unknown"] = {}
    elif change == "missing_baseline": del grant["baseline_assets"]["common3000"]
    elif change == "training_source": grant["training_contract"]["sources"]["synthetic_training.py"] = "2"*64
    else: grant["training_contract"]["TRAIN_only"] = False
    with pytest.raises(RuntimeError):
        m.validate_authority(grant, contract)


@pytest.mark.parametrize("path,name,stage,dataset_name", [(RECOVERY, "evaluate_joint", "dataset", "dataset"),
    (RECOVERY, "evaluate_legacy_raw", "dataset", "dataset"),
    (PAIR, "evaluate", "intrinsic_legacy_TRAIN_only", "legacy")])
def test_genuine_frozen_metric_single_append_inverse_and_private_globals(m, path, name, stage, dataset_name):
    original = genuine_function(path, name); before = dict(original.__globals__)
    callback = lambda dataset, row, stage: None
    instrumented = m.instrument_records(original, callback, stage=stage)
    assert original.__globals__ == before and "__hh_emit" not in original.__globals__
    assert instrumented.__globals__ is not original.__globals__
    assert instrumented.__globals__["__hh_emit"] is callback
    assert "__hh_emit" in instrumented.__code__.co_names
    # The genuine mathematical body is never called; source-only inverse is enforced inside production.
    tree = ast.parse(inspect.getsource(original))
    assert sum(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name) and node.func.value.id == "records"
        and node.func.attr == "append" for node in ast.walk(tree)) == 1
    assert dataset_name in inspect.getsource(original)


@pytest.mark.parametrize("function", [zero_append, two_appends])
def test_missing_or_extra_metric_append_seam_refused(m, function):
    with pytest.raises(RuntimeError, match="exactly one"):
        m.instrument_records(function, lambda *args: None, stage="dataset")


def test_closure_metric_refused_without_execution(m):
    captured = "not_numerical"
    def closed(dataset):
        records = []
        records.append(captured)
        return records
    with pytest.raises((RuntimeError, IndentationError)):
        m.instrument_records(closed, lambda *args: None, stage="dataset")


def test_scalar_emission_occurs_after_append_before_original_late_row_mutation(m):
    dataset = SimpleNamespace(examples=[SimpleNamespace(token="a"), SimpleNamespace(token="b")])
    observed = []
    function = m.instrument_records(scalar_metric, lambda dataset, row, stage: observed.append(deepcopy(row)), stage="dataset")
    result = function(dataset)
    assert observed == [dict(index=0, token="a", metric=1.0), dict(index=1, token="b", metric=1.0)]
    assert all(row["episode_id"] == "added_after_emit" for row in result)


def test_private_nested_evaluator_reaches_all_cohort_journals_without_original_namespace_mutation(m, tmp_path):
    journal, datasets = journal_fixture(m, tmp_path, reduced=True)
    previous = SimpleNamespace(evaluate_joint=scalar_joint, evaluate_legacy_raw=scalar_legacy)
    # HH_261005 - A private function global is a scalar-only stand-in, not a patched production module.
    outer_scope = dict(scalar_intrinsic.__globals__, previous=previous)
    from types import FunctionType
    outer = FunctionType(scalar_intrinsic.__code__, outer_scope)
    pair = SimpleNamespace(previous=previous, evaluate=outer)
    try:
        function = m.instrumented_evaluator(pair, journal)
        assert pair.previous is previous and previous.evaluate_joint is scalar_joint
        assert function.__globals__["previous"] is not previous
        journal.select(m.ENDPOINTS[0])
        result = function(None, None, datasets["recovery_train"], {}, datasets["v4"], datasets["legacy"],
            {"curved": [0]}, datasets["recovery_holdout"], {}, datasets["known599"])
        assert set(result) == set(m.COUNTS)
        assert all(row["completed"] == 1 for row in journal.receipt()["endpoints"][m.ENDPOINTS[0]].values())
        assert journal.receipt()["all_completed"] is False
    finally: journal.close()


def test_full_original_denominator_retains_all_unexecuted_rows(m, tmp_path):
    journal, datasets = journal_fixture(m, tmp_path)
    try:
        journal.select(m.ENDPOINTS[0]); journal.emit(datasets["known599"], dict(index=0, token="known599_0", total=1.0), "dataset")
        receipt = journal.receipt()
        assert receipt["full_denominator"] == 35315 and receipt["all_completed"] is False
        assert receipt["endpoints"]["common3000"]["known599"] == dict(expected=599, completed=1, UNEXECUTED=598)
        assert receipt["endpoints"]["seed_2026100503"]["known599"]["UNEXECUTED"] == 599
    finally: journal.close()


def test_all_synthetic_small_endpoint_rows_complete_in_fixed_order(m, tmp_path):
    journal, datasets = journal_fixture(m, tmp_path, reduced=True)
    try:
        for endpoint in m.ENDPOINTS:
            journal.select(endpoint)
            for name in m.COUNTS:
                source = datasets["legacy" if name == "intrinsic_legacy_TRAIN_only" else name]
                journal.emit(source, dict(index=0, token="legacy_0" if name == "intrinsic_legacy_TRAIN_only" else name+"_0", value=1.0),
                    name if name == "intrinsic_legacy_TRAIN_only" else "dataset")
        receipt = journal.receipt()
        assert receipt["full_denominator"] == 30 and receipt["all_completed"] is True
        assert all(row["UNEXECUTED"] == 0 for value in receipt["endpoints"].values() for row in value.values())
        assert len((tmp_path / "all_completed_scalar_records.jsonl").read_bytes().splitlines()) == 30
    finally: journal.close()


@pytest.mark.parametrize("case", ["no_select", "wrong_token", "wrong_index", "bool_index", "float_index",
    "foreign_dataset", "foreign_intrinsic", "wrong_stage", "nan", "duplicate", "wrong_order"])
def test_malformed_or_cross_cohort_row_never_counts_as_completed(m, tmp_path, case):
    journal, datasets = journal_fixture(m, tmp_path, reduced=True)
    try:
        if case == "wrong_order":
            with pytest.raises(RuntimeError): journal.select(m.ENDPOINTS[1])
            assert not journal.finished
            return
        if case != "no_select": journal.select(m.ENDPOINTS[0])
        row = dict(index=0, token="legacy_0", value=1.0); dataset = datasets["legacy"]; stage = "dataset"
        if case == "wrong_token": row["token"] = "foreign"
        elif case == "wrong_index": row["index"] = 1
        elif case == "bool_index": row["index"] = False
        elif case == "float_index": row["index"] = 0.0
        elif case == "foreign_dataset": dataset = SimpleNamespace()
        elif case == "foreign_intrinsic": dataset = SimpleNamespace(); stage = "intrinsic_legacy_TRAIN_only"
        elif case == "wrong_stage": stage = "arbitrary_relabel"
        elif case == "nan": row["value"] = float("nan")
        elif case == "duplicate": journal.emit(dataset, row, stage)
        before = deepcopy(journal.finished)
        with pytest.raises((RuntimeError, ValueError)):
            journal.emit(dataset, row, stage)
        assert journal.finished == before
    finally: journal.close()


def test_deadline_rejection_retains_exact_completed_failed_tail(m, tmp_path):
    ticks = iter([0., 0., 7201.]); journal, datasets = journal_fixture(m, tmp_path, clock=lambda: next(ticks))
    try:
        journal.select(m.ENDPOINTS[0])
        with pytest.raises(RuntimeError, match="wall limit"):
            journal.emit(datasets["known599"], dict(index=0, token="known599_0", value=1.0), "dataset")
        assert journal.receipt()["endpoints"]["common3000"]["known599"]["completed"] == 1
        assert len((tmp_path/"all_completed_scalar_records.jsonl").read_bytes().splitlines()) == 1
    finally: journal.close()


def test_next_batch_deadline_refusal_retains_full_denominator_without_running_next_math(m, tmp_path):
    ticks = iter([0., 0., 7201.]); journal, datasets = journal_fixture(m, tmp_path, clock=lambda: next(ticks))
    try:
        journal.select(m.ENDPOINTS[0])
        with pytest.raises(RuntimeError, match="before next batch"):
            journal.guard()
        assert journal.receipt()["full_denominator"] == 35315
        assert journal.receipt()["endpoints"]["common3000"]["known599"]["completed"] == 0
        assert journal.receipt()["last_unretained_actual_candidate"] is None
        assert (tmp_path/"all_completed_scalar_records.jsonl").stat().st_size == 0
    finally: journal.close()


def test_disk_failure_after_completed_row_retains_row_and_remaining_denominator(m, tmp_path):
    journal, datasets = journal_fixture(m, tmp_path)
    try:
        journal.select(m.ENDPOINTS[0]); m.shutil.disk_usage = lambda path: SimpleNamespace(free=1)
        with pytest.raises(RuntimeError, match="floor"):
            journal.emit(datasets["legacy"], dict(index=0, token="legacy_0", value=1.0), "dataset")
        assert journal.receipt()["endpoints"]["common3000"]["legacy"]["completed"] == 1
        assert journal.receipt()["endpoints"]["common3000"]["legacy"]["UNEXECUTED"] == 2486
    finally: journal.close()


def test_journal_quota_refusal_occurs_before_write_and_count(m, tmp_path):
    journal, datasets = journal_fixture(m, tmp_path)
    try:
        journal.select(m.ENDPOINTS[0]); m.MAX_OUTPUT_BYTES = 2
        with pytest.raises(RuntimeError, match="quota"):
            journal.emit(datasets["legacy"], dict(index=0, token="legacy_0"), "dataset")
        assert journal.bytes_written == 0 and (tmp_path/"all_completed_scalar_records.jsonl").stat().st_size == 0
        assert journal.receipt()["endpoints"]["common3000"]["legacy"]["completed"] == 0
        assert journal.receipt()["last_unretained_actual_candidate"] == dict(endpoint="common3000",
            stage="dataset", actual_row_repr=repr(dict(index=0, token="legacy_0")))
        assert "a failing batch may already have executed" in journal.receipt()["UNEXECUTED_record_notice"]
    finally: journal.close()


def test_create_only_journal_and_json_preserve_existing_bytes(m, tmp_path):
    path = tmp_path/"sealed.json"; m.write_new(path, dict(value=1)); original = path.read_bytes()
    with pytest.raises(FileExistsError): m.write_new(path, dict(value=2))
    assert path.read_bytes() == original
    journal, _ = journal_fixture(m, tmp_path); journal.close()
    with pytest.raises(FileExistsError): journal_fixture(m, tmp_path)


def test_whole_output_cap_reserves_closing_bytes_before_new_metric_file(m, tmp_path):
    budget = m.OutputBudget(tmp_path)
    budget.bytes_written = m.MAX_OUTPUT_BYTES - m.CLOSING_BYTES - 1
    with pytest.raises(RuntimeError, match="closing reserve"):
        budget.write_raw(tmp_path/"metric.json", b"12")
    assert not (tmp_path/"metric.json").exists()
    budget.write_raw(tmp_path/"closing.json", b"12", closing=True)
    assert (tmp_path/"closing.json").read_bytes() == b"12"
    assert budget.bytes_written == m.MAX_OUTPUT_BYTES - m.CLOSING_BYTES + 1


def test_whole_output_floor_and_outside_path_are_refused_before_writing(m, tmp_path):
    budget = m.OutputBudget(tmp_path)
    with pytest.raises(RuntimeError, match="output bytes/path"):
        budget.write_raw(tmp_path.parent/"outside.json", b"12")
    m.shutil.disk_usage = lambda path: SimpleNamespace(free=1)
    with pytest.raises(RuntimeError, match="free floor"):
        budget.write_raw(tmp_path/"metric.json", b"12")
    assert budget.bytes_written == 0 and not (tmp_path/"metric.json").exists()


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'[1,2,3]'])
def test_duplicate_nonfinite_and_nonobject_document_rejected(m, tmp_path, raw):
    path = tmp_path/"document.json"; path.write_bytes(raw)
    with pytest.raises(RuntimeError): m.document(path, hashlib.sha256(raw).hexdigest())


def test_document_wrong_sha_symlink_and_boundary_escape_refused(m, tmp_path):
    path = tmp_path/"document.json"; path.write_bytes(b'{"value":1}')
    assert m.document(path, m.digest(path)) == dict(value=1)
    with pytest.raises(RuntimeError): m.document(path, "0"*64)
    alias = tmp_path/"alias.json"; alias.symlink_to(path)
    with pytest.raises(RuntimeError): m.document(alias, m.digest(path))
    with pytest.raises(RuntimeError): m.canonical(path, boundary=tmp_path/"other")


def test_invalid_grant_run_stops_before_torch_models_and_output(m, tmp_path, monkeypatch):
    authority = tmp_path/"authority.json"; authority.write_bytes(b'{"authorized":false}')
    m.fixed_contract = lambda sources: {}; m.WORKSPACE = tmp_path
    with pytest.raises(RuntimeError, match="ROOT evaluation"):
        m.run(["--authority", str(authority), "--authority-sha256", m.digest(authority)])
    assert not (tmp_path/"runs").exists()


def source_fixture(m, tmp_path):
    # HH_261005 - All35 files below are harmless scalar fixture bytes, not actual runtime archives.
    m.ROOT = tmp_path
    old_names = [name for name in m.FROZEN_PINS if name not in (m.LOADER_NAME, m.LOADER_TEST_NAME)]
    training_names = old_names + ["fixture/runtime_%02d.py" % index for index in range(28)]
    names = training_names + [m.SOURCE_NAME, m.TEST_NAME, m.LOADER_NAME, m.LOADER_TEST_NAME]
    pins = {}
    for index, name in enumerate(names):
        path = tmp_path/name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("# HH_261005 - Scalar closure fixture %d.\n" % index).encode())
        pins[name] = m.digest(path)
    m.FROZEN_PINS = {name: pins[name] for name in m.FROZEN_PINS}
    return {name: pins[name] for name in training_names}


def test_source_closure_exact31_plus_four_current_sources(m, tmp_path):
    training = source_fixture(m, tmp_path)
    before = dict(training); actual = m.source_pins(training)
    assert len(training) == 31 and len(actual) == 35 and training == before
    assert all(actual[name] == pin for name, pin in training.items())
    assert all(actual[name] == m.digest(tmp_path/name)
        for name in (m.SOURCE_NAME, m.TEST_NAME, m.LOADER_NAME, m.LOADER_TEST_NAME))


@pytest.mark.parametrize("case", ["count", "malformed_pin", "changed_training", "changed_loader", "symlink", "lexical_escape", "absolute"])
def test_source_count_pin_current_bytes_and_canonical_name_faults_fail_closed(m, tmp_path, case):
    training = source_fixture(m, tmp_path)
    name = "fixture/runtime_00.py"
    if case == "count": del training[name]
    elif case == "malformed_pin": training[name] = "bad"
    elif case == "changed_training": (tmp_path/name).write_bytes(b"changed")
    elif case == "changed_loader": (tmp_path/m.LOADER_NAME).write_bytes(b"changed")
    elif case == "symlink":
        target = tmp_path/"target.py"; target.write_bytes((tmp_path/name).read_bytes())
        (tmp_path/name).unlink(); (tmp_path/name).symlink_to(target)
    elif case == "lexical_escape": training["fixture/../fixture/runtime_00.py"] = training.pop(name)
    else: training[str(tmp_path/name)] = training.pop(name)
    with pytest.raises((RuntimeError, FileNotFoundError)):
        m.source_pins(training)


def test_existing_original_record_calculation_and_screens_are_not_replaced_with_training():
    tree = ast.parse(SOURCE.read_text())
    run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run")
    body = ast.unparse(run)
    assert "loader.load_reviewed_seed_endpoint" in body
    assert "old_loader.load_joint_parent" in body and "old_loader.load_route_joint_research_npz" in body
    assert body.index("loader.load_reviewed_seed_endpoint") < body.index("pair.load_evaluation_only")
    assert "pair.original.endpoint_screens" in body
    assert "pair.train_arm" not in body and "pair.verify_parent" not in body and "pair.verify_final_endpoint" not in body
    assert "pair.main(" not in body and "optimizer.step" not in body
    assert "original71_thresholds_unchanged" in body


def test_real_source_instrumentation_nested_bindings_remain_private_and_failures_retained(m):
    class JournalStandin:
        def emit(self, *args):
            raise AssertionError("genuine numerical evaluator must not execute in scalar AST review")
        def guard(self):
            raise AssertionError("genuine numerical evaluator must not execute in scalar AST review")
    old_joint = genuine_function(RECOVERY, "evaluate_joint")
    old_legacy = genuine_function(RECOVERY, "evaluate_legacy_raw")
    old_pair = genuine_function(PAIR, "evaluate")
    previous = SimpleNamespace(evaluate_joint=old_joint, evaluate_legacy_raw=old_legacy)
    old_pair.__globals__["previous"] = previous
    original_pair_globals = dict(old_pair.__globals__)
    function = m.instrumented_evaluator(SimpleNamespace(previous=previous, evaluate=old_pair), JournalStandin())
    assert old_pair.__globals__ == original_pair_globals
    assert old_joint.__globals__.get("__hh_emit") is None and old_legacy.__globals__.get("__hh_emit") is None
    assert function.__globals__["previous"] is not previous
    assert function.__globals__["previous"].evaluate_joint is not old_joint
    assert function.__globals__["previous"].evaluate_legacy_raw is not old_legacy
    assert "__hh_emit" in function.__code__.co_names
    assert "__hh_guard" in function.__code__.co_names


def test_guard_precedes_scalar_anchor_math_and_failed_guard_leaves_anchor_unexecuted(m):
    dataset = SimpleNamespace(examples=[SimpleNamespace(token="a")]); emitted = []; guards = []
    def guard():
        guards.append("guard")
        raise RuntimeError("synthetic pre-NN guard rejected")
    callback = lambda *args: emitted.append(args)
    function = m.instrument_records(scalar_intrinsic, callback, stage="intrinsic_legacy_TRAIN_only", guard=guard)
    previous = SimpleNamespace(evaluate_joint=lambda *args: [], evaluate_legacy_raw=lambda *args: [])
    function.__globals__["previous"] = previous
    with pytest.raises(RuntimeError, match="pre-NN"):
        function(None, None, dataset, {}, dataset, dataset, {"curved": [0]}, dataset, {}, dataset)
    assert guards == ["guard"] and emitted == []
