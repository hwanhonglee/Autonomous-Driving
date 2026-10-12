"""HH_261002 - Real fresh RIGHT80 public build/parser/preflight chain with synthetic files and unchanged gates."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import town01_low_speed_right80_task_20261002 as task
from scripts.e2e import collect_town01_low_speed_right80_20261002 as worker
from scripts.e2e import run_owned_town01_low_speed_right80_20261002 as owner


def test_inverse_AST_and_private_provider_namespace():
    assert isinstance(task.identity_ast(), ast.Module)
    assert isinstance(worker.identity_ast(), ast.Module)
    assert isinstance(owner.revision_ast(), ast.Module)
    assert worker.task is owner.task is task and owner.worker_module is worker
    assert worker.parse_args.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["task"] is task
    assert owner.build_plan.__globals__["execution_contract"] is owner.execution_contract
    assert owner._originals["MODULE"] == owner.NEW_MODULE
    for name in ("configuration_from_args", "reconstruct"):
        before, after = getattr(task.previous, name), getattr(task, name)
        assert before.__code__.co_code == after.__code__.co_code and before.__code__.co_consts == after.__code__.co_consts
    for name in ("motion_quality", "turn_screen"):
        before, after = getattr(owner.previous, name), getattr(owner, name)
        assert before.__code__.co_code == after.__code__.co_code
        restored = tuple(1400 if type(value) is int and value == 1600 else value for value in after.__code__.co_consts)
        assert before.__code__.co_consts == restored


def test_only_horizon_identity_census_changes_public_contract(capsys):
    before = task.previous.fixed_contract("v2", "pilot_00"); after = task.fixed_contract("v1", "pilot_00")
    allowed = {"schema", "run_id", "output", "declaration_path", "selected_driving_relative_time_ns", "selection_clock",
        "observed_selected_span_aggregate_clock_tolerance_ns", "selection_rule", "scenario", "selected_driving_native_frames",
        "expected_fixed_anchor_count", "expected_native_states", "expected_camera_bundles", "expected_JPEG_count",
        "true_future_point_count", "capture_duration_seconds", "normal_teacher_native_intervals", "closing_receipt_reader_schema"}
    assert set(before) == set(after)
    assert {name for name in before if before[name] != after[name]} <= allowed
    assert after["seed"] == after["fixed_arguments"]["seed"] == 2026102600
    assert after["fixed_arguments"] == before["fixed_arguments"]
    assert (task.FIRST_FRAME, task.LAST_FRAME, task.ANCHORS, task.NATIVE_COUNT, task.CAMERA_COUNT) == (80, 1472, 697, 1755, 878)
    assert after["expected_JPEG_count"] == 5268 and after["true_future_point_count"] == 44608
    assert task.LAST_NS == 73_600_000_000 and after["normal_teacher_native_intervals"] == 1600
    assert after["capture_duration_seconds"] + after["fixed_arguments"]["stationary_warmup_sec"] + after["fixed_arguments"]["stationary_tail_sec"] == 87.7
    assert task.ROUTE == task.previous.ROUTE and task.ROUTE_SHA == task.previous.ROUTE_SHA
    assert owner.main(["--mode", "contract"]) == 0
    public = json.loads(capsys.readouterr().out)
    assert public["closing_receipt_reader_schema"] == owner.closing.SCHEMA
    assert owner.NEW_MODULE in public["worker_command"] and public["run_id"] == "v1"
    assert public["training_admission_state"] == "PENDING_QA_NOT_AUTHORIZED"


@pytest.mark.parametrize("run", ["v2", "v3", "../v1"])
def test_old_or_unknown_namespace_rejected(run):
    with pytest.raises(ValueError): task.fixed_contract(run, "pilot_00")


def synthetic_plan(tmp_path):
    route, mapping, calibration = [tmp_path/name for name in ("capture_route.json", "mapping.yaml", "calibration.yaml")]
    route.write_text('{"schema_version":1,"town":"Town01","scenario":"right","coordinate_reference":"base_link",'
        '"start_carla_transform":{},"goal_carla_transform":{},"route":[{"x":0,"y":0},{"x":200,"y":0}]}')
    mapping.write_text("synthetic mapping\n"); calibration.write_text("synthetic calibration\n")
    return route, mapping, calibration, {route:task.sha(route), mapping:task.sha(mapping), calibration:task.sha(calibration), owner.XODR:owner.ASSETS[owner.XODR]}


@pytest.mark.parametrize("fault", [None, "missing_reader", "wrong_reader", "wrong_history", "unknown", "old_counts", "old_schema"])
def test_REAL_build_sealed_plan_REAL_parser(tmp_path, fault):
    route, mapping, calibration, assets = synthetic_plan(tmp_path)
    with mock.patch.object(task, "ROOT", tmp_path), mock.patch.object(task, "ROUTE", route), mock.patch.object(task, "ROUTE_SHA", task.sha(route)), \
            mock.patch.dict(owner._originals, dict(MAPPING=mapping, CALIBRATION=calibration, ASSETS=assets, runtime_sources=lambda:{})), \
            mock.patch.object(owner.lifecycle, "external_sources", return_value={}):
        plan = owner.build_plan("v1", "pilot_00")
        if fault == "missing_reader": plan.pop("closing_receipt_reader_schema")
        elif fault == "wrong_reader": plan["closing_receipt_reader_schema"] = "foreign"
        elif fault == "wrong_history": plan["original_LEFT_and_failed_RIGHT_candidates_not_requalified"] = False
        elif fault == "unknown": plan["undeclared_field"] = True
        elif fault == "old_counts": plan["expected_fixed_anchor_count"] = 597
        elif fault == "old_schema": plan["schema"] = task.previous.SCHEMA
        _, path = task.paths("v1", "pilot_00"); path.parent.mkdir(parents=True); path.write_bytes(task.encoded(plan))
        argv = owner.capture_argv("v1", "pilot_00", task.sha(path))
        if fault is None:
            parsed = worker.parse_args(argv)
            assert task.configuration_from_args(parsed) == plan
            assert parsed.recovery_run_id == "v1" and parsed.seed == 2026102600
            assert parsed.max_duration_sec == 90.
        else:
            with pytest.raises((ValueError, SystemExit)): worker.parse_args(argv)


def test_REAL_build_verify_parse_and_owner_preflight(tmp_path):
    route, mapping, calibration, assets = synthetic_plan(tmp_path)
    xodr = tmp_path/"synthetic.xodr"; xodr.write_text("synthetic map bytes\n")
    assets.pop(owner.XODR); assets[xodr] = task.sha(xodr)
    expected_api = str(owner.EGG)+"/carla/__init__.py\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n"
    original_parse = worker.parse_args
    with mock.patch.object(task,"ROOT",tmp_path), mock.patch.object(task,"ROUTE",route), mock.patch.object(task,"ROUTE_SHA",task.sha(route)), \
            mock.patch.dict(owner._originals,dict(ROOT=tmp_path,MAPPING=mapping,CALIBRATION=calibration,XODR=xodr,ASSETS=assets,
                runtime_sources=lambda:{},disk_check=lambda:{"free_bytes":12*1024**3},sys=SimpleNamespace(version_info=(3,10),executable=owner.sys.executable))), \
            mock.patch.object(owner.lifecycle,"external_sources",return_value={}), \
            mock.patch.object(owner.lifecycle.lifecycle,"_assert_no_other_carla") as no_server, \
            mock.patch.object(owner.lifecycle,"ports_free") as no_ports, mock.patch.object(owner.lifecycle,"environment",return_value={}), \
            mock.patch.object(owner.subprocess,"run",return_value=SimpleNamespace(stdout=expected_api)) as api, \
            mock.patch.object(worker,"parse_args",wraps=original_parse) as parser:
        plan = owner.build_plan("v1","pilot_00"); _, path = task.paths("v1","pilot_00")
        path.parent.mkdir(parents=True); path.write_bytes(task.encoded(plan)); sha = task.sha(path)
        verified, result = owner.preflight("v1",sha)
        assert verified == plan and result["status"] == "PREFLIGHT_PASS_NO_SIMULATOR" and result["simulator_started"] is False
        assert result["declaration_sha256"] == sha
        parser.assert_called_once_with(owner.capture_argv("v1","pilot_00",sha))
        no_server.assert_called_once_with(); no_ports.assert_called_once_with(owner.PORT); api.assert_called_once()


def test_original_collector_AST_only_cleanup_identity_change():
    old, new = worker.previous.capture_ast(), worker.capture_ast()
    for node in ast.walk(new):
        if isinstance(node, ast.Constant) and node.value == "right80_post_capture_cleanup_full_brake":
            node.value = "right70_post_capture_cleanup_full_brake"
    assert ast.dump(old) == ast.dump(new)
    assert task.previous.SCHEMA.endswith(".v2") and task.previous.paths("v2")[0].name.endswith("teacher_v2")
    assert owner.lifecycle.independent_audit is owner.original_ack
