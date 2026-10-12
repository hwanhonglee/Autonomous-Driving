"""HH_261002 - Full347 synthetic JSON-container regression; immutable native QA and generic conversion remain unchanged."""
from __future__ import annotations

import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import qualify_town01_left45_common10_v3_20261002 as admission
from scripts.e2e import prepare_town01_left45_common10_v3_20261002 as convert

# HH_261002 - Reuse only a privately imported synthetic fixture module; no shared V2 module/fixture globals or actual archives change.
_spec = importlib.util.spec_from_file_location("_left_v3_synthetic_fixtures", Path(__file__).with_name("test_town01_left45_common10_admission_20261002.py"))
_fixture_module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_fixture_module)
_fixture_module.admission, _fixture_module.convert = admission, convert
native_manifest, full_evidence = _fixture_module.native_manifest, _fixture_module.full_evidence


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_frozen_parents_and_native_v2_capture_untouched():
    assert digest(admission.baseline.__file__) == admission.PARENT_SHA
    assert digest(convert.baseline.__file__) == admission.PARENT_CONVERTER_SHA
    assert admission.baseline.SCHEMA.endswith(".v2") and admission.SCHEMA.endswith(".v3")
    assert admission.RESULT_SCHEMA.endswith(".v3") and admission.ADAPTER_ID.endswith("_v3")
    assert admission.PREPARED.name.endswith("-v3") and "admission_v3" in str(admission.RECEIPT)
    assert admission.qa.SCHEMA.endswith(".v2") and admission.qa.owner.task is admission.task
    assert admission.task.fixed_contract("v2", "pilot_00")["run_id"] == "v2"
    assert convert.authority is admission and convert.baseline.authority is admission.baseline
    admission.admission_ast(); convert.converter_ast()


def test_one_comparison_plus_six_identities_is_complete_ast_delta():
    original = ast.parse(Path(admission.baseline.__file__).read_bytes()); original.body.pop()
    inverse = copy.deepcopy(admission.admission_ast()); changes = 0
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in admission.IDENTITIES.values():
            node.value = next(old for old, new in admission.IDENTITIES.items() if new == node.value)
        if isinstance(node, ast.Call) and node.args and ast.dump(node.args[0]) == ast.dump(admission.NEW_COMPARISON):
            node.args[0] = copy.deepcopy(admission.OLD_COMPARISON); changes += 1
    assert changes == 1 and ast.dump(inverse) == ast.dump(original)
    assert admission.evidence.__globals__ is vars(admission)
    assert admission.evidence.__code__.co_filename == admission.__file__


def test_json_roundtrip_changes_containers_not_exact_scalar_types_or_values():
    value = {"original_metrics": (None, False, 2**63+31, math.nextafter(1., 2.), -0.0, "x", True),
        "nested": [(1, {"geometry": (0.19999999999999998, False)})]}
    snapshot = copy.deepcopy(value); result = admission.json_containers(value)
    assert value == snapshot and type(result["original_metrics"]) is list and type(result["nested"][0]) is list
    for old, new in zip(value["original_metrics"], result["original_metrics"]):
        assert type(old) is type(new) and old == new
    assert math.copysign(1, result["original_metrics"][4]) == -1
    assert result["nested"][0][1]["geometry"] == [0.19999999999999998, False]


@pytest.mark.parametrize("value", [{1: "key would be cast"}, {"bad": {1, 2}}, {"bad": SimpleNamespace()},
    {"bad": float("nan")}, {"bad": float("inf")}, {"bad": float("-inf")}])
def test_non_json_keys_scalars_or_nonfinite_never_coerced(value):
    with pytest.raises((admission.adapter.AdapterError, ValueError)): admission.json_containers(value)


@pytest.fixture
def tuple_evidence(full_evidence):
    fixture = full_evidence
    # HH_261002 - Actual frozen nine-metric names/tuple form, all64 and all347; source files here are only pytest temporary fixtures.
    metrics = {name: (0.00012345678901234567, False) for name in admission.frozen.raw.targets.METRICS}
    assert len(metrics) == 9
    future = dict(valid_future_points=64, valid_mask=[True]*64, current_recorded_velocity_base_mps=[1.2, .01, 0.],
        diagnostic_future_xy_m=[[.1*(i+1), .0001*i] for i in range(64)],
        diagnostic_target_timestamp_ns=[2**63+31+100_000_000*i for i in range(64)],
        discrete_bound_diagnostic=dict(steps=[dict(index=i, metrics=copy.deepcopy(metrics)) for i in range(64)]))
    for index, row in enumerate(fixture.windows):
        row["true_future_original_diagnostic"] = copy.deepcopy(future)
        row["timestamp_ns"] = 2**63+31+100_000_000*index
    fixture.windows_path.write_text("".join(json.dumps(row, allow_nan=False)+"\n" for row in fixture.windows))
    fixture.authority["QA_windows_sha256"] = digest(fixture.windows_path)
    fixture.authority_path.write_text(json.dumps(fixture.authority)); fixture.authority_sha = digest(fixture.authority_path)
    return fixture


def test_whole347_nested_tuple_metrics_pass_without_native_or_label_change(tuple_evidence):
    fixture = tuple_evidence; raw = fixture.windows_path.read_bytes()
    saved = [json.loads(line) for line in raw.splitlines()]
    assert saved != fixture.windows and saved == admission.json_containers(fixture.windows)
    value = admission.evidence(fixture.authority_path, fixture.authority_sha)
    assert value["selected_camera_anchors"] == 347 and value["true_future_points"] == 22208
    assert len(value["cases"][0]["selected_anchors"]) == 347 and value["run_id"] == "v2" and value["schema"].endswith(".v3")
    assert value["input_sha256"][str(fixture.windows_path)] == hashlib.sha256(raw).hexdigest()
    assert fixture.windows_path.read_bytes() == raw and type(fixture.windows[0]["timestamp_ns"]) is int
    pair = next(iter(fixture.windows[0]["true_future_original_diagnostic"]["discrete_bound_diagnostic"]["steps"][0]["metrics"].values()))
    assert type(pair) is tuple and pair == (0.00012345678901234567, False)
    assert all(not value[key] for key in ("labels_clipped", "labels_or_pixels_transformed", "raw_native_denials_overridden", "vehicle_control_approved", "model_promoted"))


@pytest.mark.parametrize("mutation", ["metric", "reason", "future", "timestamp", "missing", "extra", "reordered"])
def test_any_full_journal_value_or_membership_difference_rejected(tuple_evidence, mutation):
    windows = tuple_evidence.windows
    if mutation == "metric": windows[193]["true_future_original_diagnostic"]["discrete_bound_diagnostic"]["steps"][63]["metrics"]["xy_curvature"] = (.201, True)
    elif mutation == "reason": windows[193]["failure_reasons"].append("not waived")
    elif mutation == "future": windows[193]["true_future_original_diagnostic"]["diagnostic_future_xy_m"][63][1] += .0000001
    elif mutation == "timestamp": windows[193]["timestamp_ns"] += 1
    elif mutation == "missing": windows.pop()
    elif mutation == "extra": windows.append(copy.deepcopy(windows[-1]))
    else: windows[193], windows[194] = windows[194], windows[193]
    with pytest.raises(admission.adapter.AdapterError, match="full immutable347 QA"):
        admission.evidence(tuple_evidence.authority_path, tuple_evidence.authority_sha)


@pytest.mark.parametrize("field,value", [("actual_teacher_capture_passed", False), ("failed_fixed_window_count", 1),
    ("raw_native_denials", ["permanent False"]), ("raw_native_denials_overridden", True)])
def test_original_failure_and_denial_conjunction_not_weakened(tuple_evidence, field, value):
    fixture = tuple_evidence; fixture.summary[field] = value; fixture.summary_path.write_text(json.dumps(fixture.summary))
    fixture.authority["QA_summary_sha256"] = digest(fixture.summary_path); fixture.authority_path.write_text(json.dumps(fixture.authority))
    with pytest.raises(admission.adapter.AdapterError): admission.evidence(fixture.authority_path, digest(fixture.authority_path))


def test_scoped_new_converter_restores_original_providers_and_forbids_nesting():
    old = e2e.qualify_carla_moving_common10; load, write = convert.base._load_native, convert.base._write_json
    with pytest.raises(LookupError):
        with convert.bound_left_authority():
            assert e2e.qualify_carla_moving_common10 is admission
            with pytest.raises(admission.adapter.AdapterError):
                with convert.bound_left_authority(): pass
            raise LookupError("synthetic failure")
    assert e2e.qualify_carla_moving_common10 is old and (convert.base._load_native, convert.base._write_json) == (load, write)
    assert not convert._active and not convert.baseline._active


def test_frozen_prepared_math_and_denial_callbacks_are_same():
    fn = admission.validation_function()
    assert fn.__globals__["COUNT"] == 347 and fn.__globals__["CAMERA"] == 528
    assert fn.__globals__["ADAPTER_ID"] == admission.ADAPTER_ID and fn.__globals__["prepared_receipt"] is admission.prepared_receipt
    old = convert.base._require_native_admission_not_denied
    with convert.bound_left_authority(): assert convert.base._require_native_admission_not_denied is old
    assert admission.frozen.COUNT == 302 and admission.frozen.CAMERA == 483


def test_source_only_contract_pins_frozen_and_fresh_admission_sources():
    value = admission.public_contract()
    assert value["schema"] == admission.SCHEMA and value["selected_camera_anchors"] == 347 and value["true_future_points"] == 22208
    assert value["native_episode"] == str(admission.task.paths("v2", "pilot_00")[0]/"episode")
    assert value["source_sha256"]["tools/e2e/qualify_town01_left45_common10_20261002.py"] == admission.PARENT_SHA
    assert value["source_sha256"]["tools/e2e/prepare_town01_left45_common10_20261002.py"] == admission.PARENT_CONVERTER_SHA
    assert all(name in value["source_sha256"] for name in ("tools/e2e/qualify_town01_left45_common10_v3_20261002.py", "tools/e2e/prepare_town01_left45_common10_v3_20261002.py"))


def test_converter_requires_original_flag_and_exact_new_receipt():
    for args in ([], ["--execute-reviewed-left45-conversion"]*2):
        with pytest.raises(admission.adapter.AdapterError): convert.main(args)
    with pytest.raises(admission.adapter.AdapterError): convert.prepare_dataset(SimpleNamespace())

