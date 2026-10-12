"""HH_261002 - Synthetic source/owner/artifact closures only; never declare or launch an actual simulator/model."""
import copy
from contextlib import ExitStack
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
from scripts.e2e import research_temporal_intrinsic_town07_resource_v2_20261002 as new


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))


def fixture(tmp_path,monkeypatch,count=1):
    workspace=tmp_path/"workspace";workspace.mkdir()
    monkeypatch.setattr(new,"ROOT",workspace)
    monkeypatch.setattr(new.protocol,"ROOT",workspace)
    external=tmp_path/"CARLA";monkeypatch.setattr(new.protocol,"CARLA_ROOT",external)
    code=workspace/"code.py";code.write_text("# synthetic code\n")
    asset=workspace/"route.json";write(asset,{"synthetic":"route"})
    api=external/"api.py";api.parent.mkdir();api.write_text("# synthetic API\n")
    root=new.pair_root();root.mkdir(parents=True)
    arm_pins={};all_rows=[]
    for arm in new.previous.ARMS:
        cases=[]
        for index in range(3):
            path=root/(arm+"_trial_%02d_plan.json"%index);folder=root/arm/("trial_%02d"%index)
            plan=dict(schema=new.SCHEMA,arm=arm,trial_index=index,output=str(folder),
                source_sha256={"code.py":new.digest(code)},external_source_sha256={str(api):new.digest(api)},
                asset_sha256={str(asset):new.digest(asset)},exact_normalized_tensor_archive_retained=False)
            write(path,plan);pin=new.digest(path)
            cases.append(dict(trial_index=index,declaration_path=str(path),declaration_sha256=pin))
            if len(all_rows)<count:
                folder.mkdir(parents=True);(folder/"declaration.json").write_bytes(path.read_bytes())
                owned=dict(status="CAPTURE_RETURNED_NONZERO_RETAINED",declaration_sha256=pin,
                    cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True,
                    port_release={"original_TCP_UDP_bind_checks_passed":True})
                write(folder/"owner_result.json",owned)
                write(folder/"diagnostic_report.json",dict(schema=new.SCHEMA+".diagnostic_report",arm=arm,original_owner=owned,
                    ownership_passed=True,status="FAIL_BOUNDED_INTRINSIC_PROTECT",
                    joint_motion_body_policy_wall100ms_passed=False,exact_normalized_tensor_archive_retained=False))
                process=dict(pid=123,start_ticks=456)
                write(folder/"owner_started.json",dict(declaration_sha256=pin,owner=process,server=process,socket_owner=process))
                (folder/"server.log").write_bytes(b"synthetic launched server")
                (folder/"collector.log").write_bytes(b"synthetic failed capture")
                (folder/"source").mkdir();(folder/"source/code.py").write_bytes(code.read_bytes())
                (folder/"source/route.json").write_bytes(asset.read_bytes())
                (folder/"source_external").mkdir();(folder/"source_external/api.py").write_bytes(api.read_bytes())
                episode=folder/"episode.partial";episode.mkdir();(episode/"raw.bin").write_bytes(b"true synthetic raw")
                all_rows.append(dict(arm=arm,trial_index=index,case_plan_sha256=pin,
                    owner_result_sha256=new.digest(folder/"owner_result.json"),
                    diagnostic_report_sha256=new.digest(folder/"diagnostic_report.json"),
                    artifact_inventory=new.inventory(folder),episode_bytes=new.base.directory_bytes(episode),sampled_exact_input_bytes=0))
        batch=root/(arm+"_batch_plan.json")
        write(batch,dict(schema=new.SCHEMA+".batch",prospective_trial_count=3,trials=cases))
        arm_pins[arm]=new.digest(batch)
    pair=dict(schema=new.SCHEMA+".paired_batch",arm_order=list(new.previous.ARMS),physical_case_count=6,trial_count_per_arm=3,
        completed_pair_authority_sha256="a"*64,arm_batch_sha256=arm_pins,automatic_retry=False,outcome_selected_retry=False,
        model_promoted=False,vehicle_control_approved=False)
    pair_path=root/"paired_batch_plan.json";write(pair_path,pair)
    scope=SimpleNamespace(authority_sha="a"*64,assets={})
    monkeypatch.setattr(new.previous,"_scope",scope);monkeypatch.setattr(new.previous,"_pair_sha",new.digest(pair_path))
    resource=new.authority_root()/"closed_prefix.json"
    value=dict(schema=new.AUTH_SCHEMA,resource_accounting_authorized=True,paired_batch_sha256=new.digest(pair_path),
        completed_pair_authority_sha256="a"*64,closed_failed_cases_retained=True,model_promoted=False,
        vehicle_control_approved=False,closed_cases=all_rows)
    write(resource,value)
    return resource,new.digest(resource),value,root


@pytest.mark.parametrize("count",range(7))
def test_exact_remaining_formula_failures_still_three_plus_three(tmp_path,monkeypatch,count):
    path,pin,value,root=fixture(tmp_path,monkeypatch,count)
    with new.activated("preflight" if count<6 else "summary",count,path,pin):
        expected=new.base.FLOOR+(6-count)*new.base.ARM_RESERVE+new.base.SHARED_RESERVE
        report=new.disk_check(expected)
        assert report["prospective_trials"]==6 and report["remaining_write_trials"]==6-count
        assert report["authenticated_closed_trial_count"]==count and report["all_six_failures_and_successes_remain_in_denominator"]
        assert all(row["diagnostic_status"].startswith("FAIL") for row in report["closed_case_evidence"])
        with pytest.raises(Exception,match="reserve unavailable"):new.disk_check(expected-1)


def test_DECLARE_always_fullSIX_even_if_closed_receipt_supplied(tmp_path,monkeypatch):
    path,pin,value,root=fixture(tmp_path,monkeypatch,3)
    with new.activated("declare",None,path,pin):
        expected=new.base.FLOOR+6*new.base.ARM_RESERVE+new.base.SHARED_RESERVE
        report=new.disk_check(expected)
        assert report["remaining_write_trials"]==6 and report["authenticated_closed_trial_count"]==0


def test_no_authority_reservesSIX_and_cannot_skip_trial(tmp_path,monkeypatch):
    path,pin,value,root=fixture(tmp_path,monkeypatch,0)
    with new.activated("preflight",0,None,None):assert new.disk_check(100*1024**3)["remaining_write_trials"]==6
    with new.activated("execute",1,None,None):
        with pytest.raises(Exception,match="completed prefix"):new.disk_check(100*1024**3)


@pytest.mark.parametrize("fault",["cleanup","ports","source","ownership","not_executed","artifact_mutation","missing_inventory",
    "path_escape","other_case","case_order","duplicate_case","quota","sample","future_plan","pair_pin","authority_pin","source_archive",
    "diagnostic_schema","diagnostic_arm","diagnostic_owner"])
def test_unproven_or_changed_or_nonprefix_closure_failclosed(tmp_path,monkeypatch,fault):
    path,pin,value,root=fixture(tmp_path,monkeypatch,2)
    row=value["closed_cases"][0];folder=root/"control/trial_00"
    owner_path=folder/"owner_result.json";owner=json.loads(owner_path.read_text())
    if fault=="cleanup":owner["cleanup_errors"]=["not closed"]
    if fault=="ports":owner["port_release"]["original_TCP_UDP_bind_checks_passed"]=False
    if fault=="source":owner["source_hashes_match"]=False
    if fault=="not_executed":owner["status"]="NOT_EXECUTED"
    if fault in ("cleanup","ports","source","not_executed"):
        write(owner_path,owner);row["owner_result_sha256"]=new.digest(owner_path);row["artifact_inventory"]=new.inventory(folder)
    if fault=="ownership":
        p=folder/"diagnostic_report.json";diagnostic=json.loads(p.read_text());diagnostic["ownership_passed"]=False
        write(p,diagnostic);row["diagnostic_report_sha256"]=new.digest(p);row["artifact_inventory"]=new.inventory(folder)
    if fault.startswith("diagnostic_"):
        p=folder/"diagnostic_report.json";diagnostic=json.loads(p.read_text())
        key=fault.removeprefix("diagnostic_")
        diagnostic[{"schema":"schema","arm":"arm","owner":"original_owner"}[key]]="foreign" if key!="owner" else {}
        write(p,diagnostic);row["diagnostic_report_sha256"]=new.digest(p);row["artifact_inventory"]=new.inventory(folder)
    if fault=="artifact_mutation":(folder/"episode.partial/raw.bin").write_bytes(b"changed")
    if fault=="missing_inventory":row["artifact_inventory"].pop("collector.log")
    if fault=="path_escape":row["artifact_inventory"]["../escape"]={"sha256":"a"*64,"bytes":1}
    if fault=="other_case":row["arm"]="intrinsic_curve"
    if fault=="case_order":value["closed_cases"].reverse()
    if fault=="duplicate_case":value["closed_cases"][1]=copy.deepcopy(row)
    if fault=="quota":row["episode_bytes"]=new.base.EPISODE_QUOTA+1
    if fault=="sample":row["sampled_exact_input_bytes"]=1
    if fault=="future_plan":(root/"intrinsic_curve_trial_02_plan.json").write_bytes(b"unknown future changed")
    if fault=="pair_pin":value["paired_batch_sha256"]="b"*64
    if fault=="source_archive":(folder/"source/code.py").write_bytes(b"changed frozen source archive")
    write(path,value);pin=new.digest(path)
    if fault=="authority_pin":pin="0"*64
    with new.activated("preflight",2,path,pin):
        with pytest.raises(Exception):new.disk_check(100*1024**3)


@pytest.mark.parametrize("fail",[False,True])
def test_all_scopealiases_restore_including_originalsixreserve(fail):
    before={name:getattr(new.previous,name) for name in new._original}
    try:
        with new.activated("declare",None,None,None):
            assert new.previous.SCHEMA==new.SCHEMA and new.previous.MODULE==new.MODULE
            assert new.previous.disk_check is new.disk_check
            assert new.previous.pair_root()==new.pair_root()
            if fail:raise RuntimeError("synthetic failure")
    except RuntimeError:
        assert fail
    assert before=={name:getattr(new.previous,name) for name in new._original}
    assert not new._active and new._resource_path is None and new._mode is None


def test_same_fixed_numeric_controller_gates_only_resource_metadata(monkeypatch):
    original=dict(known_total_optimizer_updates=18200,controller_target_cap_mps=2.4,total_capture_seconds=45,
        sensor_and_ACK_screen={"native_state_count":900,"camera_bundle_count":450},hard_policy_deadline_seconds=.1)
    monkeypatch.setattr(new,"_fixed",lambda *args:copy.deepcopy(original))
    revised=new.fixed_contract("v1",True)
    assert all(revised[k]==value for k,value in original.items()) and len(revised["fixed_physical_execution_order"])==6
    assert revised["closed_FAILED_cases_retained_in_three_plus_three_denominator"]


def test_cli_onlystrips_resourceproof_preserves_all_parentarguments(monkeypatch,tmp_path):
    seen=[]
    monkeypatch.setattr(new.previous,"main",lambda argv:seen.append((argv,new._mode,new._target)) or 0)
    args=["--mode","execute","--arm","intrinsic_curve","--trial","1","--completed-pair-authority","a.json",
        "--completed-pair-authority-sha256","a"*64,"--paired-batch-sha256","b"*64]
    path=tmp_path/"ROOT_receipt.json"
    assert new.main(args+["--closed-cases-authority",str(path),"--closed-cases-authority-sha256","c"*64])==0
    assert seen==[(args,"execute",4)] and new._resource_path is None


@pytest.mark.parametrize("trial",range(3))
def test_REAL_full_parent_scope_plan_argv_validator_preflight(tmp_path,monkeypatch,trial):
    # HH_261002 - Keep original owned preflight/control bindings; replace only external model/process RPCs with synthetic fixtures.
    owner=new.previous;live=owner.live
    pins={name+"_sha256":str(i)*64 for i,name in enumerate(("weights","metadata","checkpoint","declaration"),1)}
    authority=tmp_path/"authority.json";run=tmp_path/"run"
    description=dict(endpoints={arm:dict(pins) for arm in owner.ARMS},source_sha256={},arm_order=list(owner.ARMS))
    monkeypatch.setattr(new,"pair_root",lambda:tmp_path/"protect_v2")
    monkeypatch.setattr(new,"_sources",lambda:{})
    monkeypatch.setattr(owner.paired,"authority",lambda *a:(authority,description,run))
    monkeypatch.setattr(owner.paired,"verify_exports",lambda *a:{})
    monkeypatch.setattr(owner.paired,"evidence_assets",lambda *a:{})
    expected=new.base.FLOOR+6*new.base.ARM_RESERVE+new.base.SHARED_RESERVE
    monkeypatch.setattr(new.shutil,"disk_usage",lambda *a:SimpleNamespace(free=expected))
    before=(owner.numeric.MODULE,owner.numeric.SCHEMA,owner.numeric.WEIGHTS,owner.numeric._exports,
        owner.base.live,owner.protocol.live,owner.signed.live,owner.paired._scope,owner.protocol.disk_check)
    with new.activated("preflight",0,None,None),owner.activated(authority,"a"*64,"control"),owner.protocol.scoped_trial(trial):
        config=owner.base.fixed_contract("v1",True)
        assert config["schema"]==new.SCHEMA and config["arm"]=="control"
        assert config["worker_command"][config["worker_command"].index("-m")+1]==new.MODULE
        assert config["training_source_count"]==29 and config["prospective_physical_case_count"]==6
        assert config["controller_target_cap_mps"]==2.4 and config["total_capture_seconds"]==45.
        assert config["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.5
        assert config["sensor_and_ACK_screen"]["native_state_count"]==900
        assert config["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
        args=live.frozen.parse_args(owner.base.capture_argv("v1",True,"a"*64))
        live.frozen.validate_capture(args,live.collector.load_route(args.route_file))
        assert args.seed==live.PROFILES[trial]["seed"] and args.agent_initialization=="after_bootstrap"
        assert owner.base.paths("v1")[0]==new.pair_root()/"control"/("trial_%02d"%trial)
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(owner.base,"digest",return_value="a"*64))
            stack.enter_context(mock.patch.object(owner.base,"runtime_sources",return_value={"synthetic":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base,"external_sources",return_value={"api":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base,"ASSETS",{p:"a"*64 for p in owner.base.ASSETS}))
            plan=owner.base.build_plan("v1",True)
            stack.enter_context(mock.patch.object(owner.base,"verify_plan",return_value=plan))
            stack.enter_context(mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"))
            stack.enter_context(mock.patch.object(owner.base,"ports_free"))
            imported=SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
            stack.enter_context(mock.patch.object(owner.base.subprocess,"run",return_value=imported))
            stack.enter_context(mock.patch.object(owner.base,"load_npz",return_value=SimpleNamespace(_research_temporal_checkpoint_sha256="3"*64)))
            report=owner.base.preflight("v1","a"*64)[1]
            assert report["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
            assert report["disk"]["required_bytes"]==expected and report["disk"]["remaining_write_trials"]==6
    assert before==(owner.numeric.MODULE,owner.numeric.SCHEMA,owner.numeric.WEIGHTS,owner.numeric._exports,
        owner.base.live,owner.protocol.live,owner.signed.live,owner.paired._scope,owner.protocol.disk_check)


def test_full_archive_assets_reserve_count_not_only_episode_bytes(tmp_path,monkeypatch):
    path,pin,value,root=fixture(tmp_path,monkeypatch,2)
    asset=new.ROOT/"additional_pinned_asset.bin";asset.write_bytes(b"synthetic archive"*100)
    new.previous._scope.assets={asset:new.digest(asset)}
    with new.activated("preflight",2,path,pin):
        wanted=new.base.FLOOR+4*(new.base.ARM_RESERVE+asset.stat().st_size)+new.base.SHARED_RESERVE
        report=new.disk_check(wanted)
        assert report["remaining_source_asset_archive_reserve_bytes"]==4*asset.stat().st_size
        with pytest.raises(Exception,match="reserve unavailable"):new.disk_check(wanted-1)
