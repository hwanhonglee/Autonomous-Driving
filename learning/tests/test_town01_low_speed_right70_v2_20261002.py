"""HH_261002 - Real private build→strict parser chain with synthetic files; no ownership/server/raw capture or field-guard mocks."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import town01_low_speed_right70_task_v2_20261002 as task
from scripts.e2e import collect_town01_low_speed_right70_v2_20261002 as worker
from scripts.e2e import run_owned_town01_low_speed_right70_v2_20261002 as owner


def test_three_inverse_identity_and_original_configuration_predicates():
    assert isinstance(task.identity_ast(),ast.Module)
    assert isinstance(worker.identity_ast(),ast.Module)
    assert isinstance(owner.revision_ast(),ast.Module)
    old=task.previous.configuration_from_args
    assert task.configuration_from_args.__code__.co_code==old.__code__.co_code
    assert task.configuration_from_args.__code__.co_consts==old.__code__.co_consts
    assert task.MEMBERSHIP_AST_SUBSTITUTIONS==task.previous.MEMBERSHIP_AST_SUBSTITUTIONS


def test_public_contract_v2_two_fixed_authenticated_fields_and_no_physical_delta(capsys):
    before=task.previous.fixed_contract("v1","pilot_00");after=task.fixed_contract("v2","pilot_00")
    assert set(after)==set(before)|{"closing_receipt_reader_schema","original_LEFT_and_failed_RIGHT_candidates_not_requalified"}
    for key in before:
        if key not in ("schema","run_id","output","declaration_path"):assert after[key]==before[key],key
    assert owner.main(["--mode","contract"])==0
    public=json.loads(capsys.readouterr().out)
    assert public["schema"]==task.SCHEMA and public["run_id"]=="v2" and owner.NEW_MODULE in public["worker_command"]
    assert public["seed"]==public["fixed_arguments"]["seed"]==2026102600
    assert public["expected_fixed_anchor_count"]==597 and public["expected_native_states"]==1555 and public["expected_camera_bundles"]==778
    assert public["closing_receipt_reader_schema"]==owner.closing.SCHEMA


@pytest.mark.parametrize("run",["v1","v3","../v2"])
def test_old_or_unknown_version_rejected(run):
    with pytest.raises(ValueError):task.fixed_contract(run,"pilot_00")


def synthetic_plan(tmp_path):
    route,mapping,calibration=[tmp_path/name for name in ("capture_route.json","mapping.yaml","calibration.yaml")]
    route.write_text('{"schema_version":1,"town":"Town01","scenario":"right","coordinate_reference":"base_link",'
        '"start_carla_transform":{},"goal_carla_transform":{},"route":[{"x":0,"y":0},{"x":100,"y":0}]}')
    mapping.write_text("synthetic mapping\n");calibration.write_text("synthetic calibration\n")
    sha=task.sha(route);assets={route:sha,mapping:task.sha(mapping),calibration:task.sha(calibration),owner.XODR:owner.ASSETS[owner.XODR]}
    return route,mapping,calibration,assets


@pytest.mark.parametrize("fault",[None,"missing_reader","wrong_reader","wrong_failed_history","unknown_key","changed_fixed","v1_fixed"])
def test_REAL_build_plan_to_REAL_parse_args_authenticates_every_field(tmp_path,fault):
    route,mapping,calibration,assets=synthetic_plan(tmp_path)
    # HH_261002 - Only file locations and unrelated source/API inventories are synthetic; both actual fixed_contract and exact field/value parser guards execute.
    with mock.patch.object(task,"ROOT",tmp_path),mock.patch.object(task,"ROUTE",route),mock.patch.object(task,"ROUTE_SHA",task.sha(route)), \
            mock.patch.dict(owner._originals,dict(MAPPING=mapping,CALIBRATION=calibration,ASSETS=assets,runtime_sources=lambda:{})), \
            mock.patch.object(owner.lifecycle,"external_sources",return_value={}):
        plan=owner.build_plan("v2","pilot_00")
        assert "closing_receipt_reader_schema" in plan and "original_LEFT_and_failed_RIGHT_candidates_not_requalified" in plan
        assert owner.NEW_MODULE in plan["worker_command"]
        if fault=="missing_reader":plan.pop("closing_receipt_reader_schema")
        elif fault=="wrong_reader":plan["closing_receipt_reader_schema"]="foreign"
        elif fault=="wrong_failed_history":plan["original_LEFT_and_failed_RIGHT_candidates_not_requalified"]=False
        elif fault=="unknown_key":plan["unreviewed_owner_field"]=True
        elif fault=="changed_fixed":plan["expected_fixed_anchor_count"]=596
        elif fault=="v1_fixed":plan["schema"]=task.previous.SCHEMA
        _,path=task.paths("v2","pilot_00");path.parent.mkdir(parents=True);path.write_bytes(task.encoded(plan))
        argv=owner.capture_argv("v2","pilot_00",task.sha(path))
        if fault is None:
            parsed=worker.parse_args(argv)
            assert task.configuration_from_args(parsed)==plan
            assert parsed.recovery_run_id=="v2" and parsed.seed==2026102600
        else:
            with pytest.raises((ValueError,SystemExit)):worker.parse_args(argv)


def test_current_dynamic_providers_and_original_modules_not_mutated():
    assert worker.task is task and owner.task is task and owner.worker_module is worker
    assert worker.parse_args.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["task"] is task
    assert owner.build_plan.__globals__["execution_contract"] is owner.execution_contract
    assert owner._originals["MODULE"]==owner.NEW_MODULE
    assert task.previous.SCHEMA.endswith(".v1") and task.previous.paths("v1")[0].name.endswith("teacher_v1")


def test_REAL_build_plan_verify_case_parse_args_and_preflight_complete_chain(tmp_path):
    route,mapping,calibration,assets=synthetic_plan(tmp_path)
    xodr=tmp_path/"synthetic.xodr";xodr.write_text("synthetic API-independent map bytes\n")
    assets.pop(owner.XODR);assets[xodr]=task.sha(xodr)
    expected_api=str(owner.EGG)+"/carla/__init__.py\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n"
    original_parse=worker.parse_args
    # HH_261002 - Only process/port/disk/API discovery is mocked; the real owner build, exact sealed-file verification and actual parser all execute.
    with mock.patch.object(task,"ROOT",tmp_path),mock.patch.object(task,"ROUTE",route),mock.patch.object(task,"ROUTE_SHA",task.sha(route)), \
            mock.patch.dict(owner._originals,dict(ROOT=tmp_path,MAPPING=mapping,CALIBRATION=calibration,XODR=xodr,ASSETS=assets,
                runtime_sources=lambda:{},disk_check=lambda:{"free_bytes":12*1024**3},
                sys=SimpleNamespace(version_info=(3,10),executable=owner.sys.executable))), \
            mock.patch.object(owner.lifecycle,"external_sources",return_value={}), \
            mock.patch.object(owner.lifecycle.lifecycle,"_assert_no_other_carla") as no_server, \
            mock.patch.object(owner.lifecycle,"ports_free") as no_ports, \
            mock.patch.object(owner.lifecycle,"environment",return_value={}), \
            mock.patch.object(owner.subprocess,"run",return_value=SimpleNamespace(stdout=expected_api)) as api_probe, \
            mock.patch.object(worker,"parse_args",wraps=original_parse) as actual_parser:
        plan=owner.build_plan("v2","pilot_00")
        _,path=task.paths("v2","pilot_00");path.parent.mkdir(parents=True);path.write_bytes(task.encoded(plan))
        expected=task.sha(path)
        verified,result=owner.preflight("v2",expected)
        assert verified==plan and result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
        assert result["simulator_started"] is False and result["declaration_sha256"]==expected
        actual_parser.assert_called_once_with(owner.capture_argv("v2","pilot_00",expected))
        no_server.assert_called_once_with();no_ports.assert_called_once_with(owner.PORT);api_probe.assert_called_once()
        assert actual_parser.call_args.args[0][1]=="v2"


def test_original_control_and_coast_AST_unchanged():
    assert ast.dump(worker.capture_ast())==ast.dump(worker.previous.capture_ast())
    for name in ("motion_quality","turn_screen","teacher_command_audit"):
        before=getattr(owner.previous,name);after=getattr(owner,name)
        assert before.__code__.co_code==after.__code__.co_code and before.__code__.co_consts==after.__code__.co_consts
