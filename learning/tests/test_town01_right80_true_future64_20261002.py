"""HH_261002 - Synthetic all697 RIGHT80 identities, unchanged original nine/strict metrics, retained failures and no actual input reads."""
import ast
import copy
import json
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_right80_true_future64_20261002 as qa
from test_town01_left45_true_future64_20261002 import future


def test_inverse_identity_and_numerical_functions_identical():
    original, current = qa.previous.identity_ast(), qa.identity_ast()
    for name in ("canonical", "payload", "strict_future", "analyze"):
        old = next(n for n in original.body if isinstance(n, ast.FunctionDef) and n.name == name)
        new = next(n for n in current.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(old) == ast.dump(new)
        before, after = getattr(qa.previous, name), getattr(qa, name)
        assert before.__code__.co_code == after.__code__.co_code and before.__code__.co_consts == after.__code__.co_consts
    assert qa.owner is qa.declared_owner and qa.inspect.__globals__["owner"] is qa.owner
    assert qa.main.__globals__["inspect"] is qa.inspect and qa.task is qa.owner.task


def test_exact_current_paths_sources_and_no_right70_requalification():
    assert qa.SCHEMA == "carla.actual_town01_right80_true_future64_diagnostic.20261002.v1"
    assert qa.OUTPUT.name == "town01_low_speed_right80_future64_v1"
    paths = qa.artifact_paths()
    assert len(paths) == len(set(paths)) == 16 and all("town01_low_speed_right80_teacher_v1" in str(p) for p in paths)
    assert qa.FROZEN.items() >= qa.previous.FROZEN.items()
    for module in (qa.owner, qa.task, qa.owner.worker_module, qa.owner.closing):
        key = str(Path(module.__file__).relative_to(qa.ROOT)); assert qa.FROZEN[key] == qa.owner.digest(module.__file__)
    assert qa.previous.OUTPUT.name.endswith("_v2") and qa.previous.owner.task.SCHEMA.endswith(".v2")
    assert qa.task.fixed_contract("v1", "pilot_00")["closing_receipt_reader_schema"] == qa.owner.closing.SCHEMA
    assert (qa.task.ANCHORS, qa.task.NATIVE_COUNT, qa.task.CAMERA_COUNT, qa.task.LAST_FRAME) == (697, 1755, 878, 1472)
    assert qa.task.ROUTE == qa.previous.task.ROUTE and qa.task.ROUTE_SHA == qa.previous.task.ROUTE_SHA
    assert "all697_true_future_windows.jsonl" in qa.main.__code__.co_consts
    assert "all597_true_future_windows.jsonl" not in qa.main.__code__.co_consts


@pytest.mark.parametrize("speed", [0., .02, .1, 2.4])
def test_undefined_stationary_and_defined_bad_curve_are_unchanged(speed):
    data = future(speed=speed, kappa=.3); before = copy.deepcopy(data)
    assert qa.strict_future(data) == qa.previous.strict_future(data) and data == before
    if speed == 0.: assert all(r["metrics"]["xy_curvature"] == [None, False] for r in qa.strict_future(data))
    else: assert any(r["metrics"]["xy_curvature"][1] for r in qa.strict_future(data))


@pytest.mark.parametrize("fault", ["old_schema", "old_output", "old_artifacts", "missing_artifact", "extra_artifact", "wrong_source"])
def test_external_contract_rejected_before_actual_payload(fault):
    sources = {"synthetic.py": "a"*64}
    caller = dict(schema=qa.SCHEMA+".caller_pins", output=str(qa.OUTPUT), source_sha256=sources,
        artifact_sha256={str(p): "b"*64 for p in qa.artifact_paths()})
    if fault == "old_schema": caller["schema"] = qa.previous.SCHEMA+".caller_pins"
    elif fault == "old_output": caller["output"] = str(qa.previous.OUTPUT)
    elif fault == "old_artifacts": caller["artifact_sha256"] = {str(p): "b"*64 for p in qa.previous.artifact_paths()}
    elif fault == "missing_artifact": caller["artifact_sha256"].pop(next(iter(caller["artifact_sha256"])))
    elif fault == "extra_artifact": caller["artifact_sha256"]["/tmp/foreign"] = "c"*64
    else: caller["source_sha256"] = {}
    with mock.patch.object(qa, "source_pins", return_value=sources), \
            mock.patch.object(qa, "payload", return_value=json.dumps(caller).encode()) as read:
        with pytest.raises(RuntimeError): qa.inspect(Path("synthetic-caller"), "d"*64)
    assert read.call_count == 1


def test_all697_failed_windows_and_all44608_true_points_retained():
    values = [future() for _ in range(697)]
    windows = [dict(camera_index=i, failure_reasons=["body_inset_failed"] if i == 696 else [],
        necessary_checks_clear=i != 696) for i in range(697)]
    with mock.patch.object(qa.frozen, "window_checks", return_value=(windows,
            dict(protected=[{"frame": 1472}], outside_protected=[{"frame": 1601}]))):
        rows, defects, _ = qa.analyze([], [], [], [], [], values)
    assert len(rows) == 697 and sum(len(r["strict_defined_geometry_witness"]) for r in rows) == 44608
    assert rows[-1]["necessary_checks_clear"] is False and defects["protected"] == [{"frame": 1472}]
    assert defects["outside_protected"] == [{"frame": 1601}]
    assert all(r["training_data_approved"] is False for r in rows)
    assert all(r["true_future_original_diagnostic"] is original for r, original in zip(rows, values))


def test_actual_output_still_requires_canonical_create_only_and_rechecks():
    node = next(n for n in qa.identity_ast().body if isinstance(n, ast.FunctionDef) and n.name == "main")
    text = ast.unparse(node)
    for exact in ("canonical(OUTPUT)", "not output.exists()", "owner.digest(name) == sha", "exist_ok=False", "allow_nan=False"):
        assert exact in text
    assert qa.source_pins.__globals__["FROZEN"] is qa.FROZEN
