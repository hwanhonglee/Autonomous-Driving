"""HH_261002 - Test the actual CPU dispatch boundaries with mocks only; never SSH, load arrays, or initialize CUDA."""
import ast
import base64
import json
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest
from scripts.e2e import dispatch_map_sign_other_margin_CPU_prepare_20261002 as dispatch


def test_real_contract_exact12_and_only_TWO_queries():
    value=dispatch.contract()
    assert value["source_count"]==len(value["source_sha256"])==12
    assert value["allowed_remote_runner_modes"]==["runtime-contract","schedule-contract"]
    assert value["required_environment"]["CUDA_VISIBLE_DEVICES"]==""
    assert not value["CUDA_initialization_allowed"] and not value["optimizer_allowed"]
    assert not value["fit_or_validate_only_invocation"] and not value["DEV_HOLDOUT_payload_access"]
    assert value["dispatcher_sha256"]==dispatch.digest(Path(dispatch.__file__))
    assert all(Path(row["local_path"]).is_file() and dispatch.digest(Path(row["local_path"]))==row["sha256"]
        for row in value["source_sha256"].values())


def fixture(tmp_path,monkeypatch,fault=None):
    monkeypatch.setattr(dispatch,"PLAN",tmp_path)
    monkeypatch.setattr(dispatch,"RECEIPT",tmp_path/"receipts")
    value=dict(schema=dispatch.SCHEMA,source_sha256={"source.py":dict(local_path=str(tmp_path/"source.py"),sha256="a"*64)},
        fit_contract=dict(schema="synthetic",loss_contract={"scope":"synthetic"}))
    monkeypatch.setattr(dispatch,"contract",lambda:value)
    authority=dict(schema=dispatch.AUTH_SCHEMA,dispatch_contract=value,CPU_preparation_authorized=True,
        remote_source_transfer_authorized=True,remote_contract_execution_authorized=True,
        fit_authorized=False,CUDA_initialization_authorized=False,TRAIN_payload_access_authorized=False,
        DEV_HOLDOUT_payload_access_authorized=False,package_installation_authorized=False)
    if fault in authority:authority[fault]=not authority[fault]
    if fault=="contract":authority["dispatch_contract"]={}
    path=tmp_path/"ROOT_authority.json";path.write_text(json.dumps(authority))
    pin=dispatch.digest(path)
    if fault=="pin":pin="0"*64
    if fault=="duplicate":path.write_text('{"schema":1,"schema":2}');pin=dispatch.digest(path)
    return value,path,pin


def mock_remote(value,mode,fault=None):
    pins={name:row["sha256"] for name,row in value["source_sha256"].items()}
    result=dict(mode=mode,source_sha256_before=pins,source_sha256_after=pins,
        training_output_absent=True,fit_invoked=False,optimizer_updates_authorized=0,
        CUDA_initialized_before=False if mode!="source-check" else None,
        CUDA_initialized_after=False if mode!="source-check" else None,result=None)
    if mode=="runtime-contract":result["result"]={"python_prefix":"synthetic"}
    if mode=="schedule-contract":result["result"]=dict(schema="synthetic.prospective_schedule",
        source_sha256=pins,no_CUDA_or_optimizer_or_output=True,fit_authorized=False,
        equal_global_proof_sha256="b"*64,loss_contract_sha256=dispatch._document_sha(value["fit_contract"]["loss_contract"]))
    if fault=="CUDA":result["CUDA_initialized_after"]=True
    if fault=="source":result["source_sha256_after"]={}
    if fault=="run":result["training_output_absent"]=False
    if fault=="optimizer":result["optimizer_updates_authorized"]=True  # bool must not masquerade as int0.
    if fault=="schedule":result["result"]["loss_contract_sha256"]="0"*64
    return result


def fake_process(monkeypatch,value,*,fault_stage=None,rc=0,remote_fault=None,timeout=False):
    calls=[]
    def run(argv,**kwargs):
        assert kwargs==dict(capture_output=True,timeout=45 if "mkdir --" in argv[-1]
            else 90 if argv[0]=="rsync" else 300)
        calls.append(argv)
        if argv[0]=="rsync" or "mkdir --" in argv[-1]:return SimpleNamespace(stdout=b"",stderr=b"",returncode=0)
        mode=shlex.split(argv[-1])[-1]
        if fault_stage==mode and timeout:
            raise dispatch.subprocess.TimeoutExpired(argv,kwargs["timeout"],output=b"partial",stderr=b"timeout")
        output=json.dumps(mock_remote(value,mode,remote_fault if fault_stage==mode else None)).encode()
        return SimpleNamespace(stdout=output,stderr=b"retained diagnostic",returncode=rc if fault_stage==mode else 0)
    monkeypatch.setattr(dispatch.subprocess,"run",run)
    return calls


def test_mock_exact_mkdir_rsync_TWO_queries_sourcechecks_and_createonly(tmp_path,monkeypatch):
    value,path,pin=fixture(tmp_path,monkeypatch);calls=fake_process(monkeypatch,value)
    assert dispatch.execute(path,pin)==0
    assert len(calls)==6
    assert calls[0][:len(dispatch.SSH)]==dispatch.SSH and "mkdir -- "+str(dispatch.SOURCE) in calls[0][-1]
    transfer=calls[1]
    assert transfer[:3]==["rsync","-rt","--ignore-existing"] and "--delete" not in transfer
    assert transfer[-1]==dispatch.SSH[-1]+":"+str(dispatch.SOURCE)+"/"
    assert [shlex.split(call[-1])[-1] for call in calls[2:]]==["source-check","runtime-contract","schedule-contract","source-check"]
    assert all("--validate-only" not in shlex.split(call[-1]) and "fit" not in shlex.split(call[-1]) for call in calls[2:])
    report=json.loads((dispatch.RECEIPT/"summary.json").read_text())
    assert report["status"]=="CPU_CONTRACTS_VERIFIED_NO_FIT" and report["CUDA_initialized"] is False
    assert not report["optimizer_started"] and not report["fit_executed"]
    before=(dispatch.RECEIPT/"summary.json").read_bytes()
    with pytest.raises(ValueError,match="create-only"):dispatch.execute(path,pin)
    assert (dispatch.RECEIPT/"summary.json").read_bytes()==before and len(calls)==6


@pytest.mark.parametrize("fault",["CPU_preparation_authorized","remote_source_transfer_authorized",
    "remote_contract_execution_authorized","fit_authorized","CUDA_initialization_authorized",
    "TRAIN_payload_access_authorized","DEV_HOLDOUT_payload_access_authorized","package_installation_authorized",
    "contract","pin","duplicate"])
def test_no_default_or_expanded_or_wrong_authority(tmp_path,monkeypatch,fault):
    value,path,pin=fixture(tmp_path,monkeypatch,fault)
    calls=fake_process(monkeypatch,value)
    with pytest.raises(Exception):dispatch.execute(path,pin)
    assert calls==[] and not dispatch.RECEIPT.exists()


@pytest.mark.parametrize("fault",["CUDA","source","run","optimizer","schedule"])
def test_failure_keepsraw_and_still_remote_source_postchecks(tmp_path,monkeypatch,fault):
    value,path,pin=fixture(tmp_path,monkeypatch)
    mode="schedule-contract" if fault=="schedule" else "runtime-contract"
    calls=fake_process(monkeypatch,value,fault_stage=mode,remote_fault=fault)
    assert dispatch.execute(path,pin)==1
    report=json.loads((dispatch.RECEIPT/"summary.json").read_text())
    assert report["status"]=="FAILED_RETAINED" and report["failure"] and report["final_source_check"]
    assert shlex.split(calls[-1][-1])[-1]=="source-check"
    assert (dispatch.RECEIPT/(mode.replace("-","_")+"_stdout.txt")).is_file()
    assert not report["fit_executed"]


@pytest.mark.parametrize("timeout",[False,True])
def test_remote_nonzero_timeout_originalstdout_retained(tmp_path,monkeypatch,timeout):
    value,path,pin=fixture(tmp_path,monkeypatch)
    calls=fake_process(monkeypatch,value,fault_stage="runtime-contract",rc=9,timeout=timeout)
    assert dispatch.execute(path,pin)==1 and shlex.split(calls[-1][-1])[-1]=="source-check"
    command=json.loads((dispatch.RECEIPT/"runtime_contract_command.json").read_text())
    assert command["returncode"] is (None if timeout else 9)
    assert (dispatch.RECEIPT/"runtime_contract_stderr.txt").read_bytes()==(b"timeout" if timeout else b"retained diagnostic")


@pytest.mark.parametrize("mode",["fit","validate-only","contract","","schedule-contract --mode fit"])
def test_remote_mode_cannot_smuggle_fit(mode):
    with pytest.raises(ValueError,match="queries"):dispatch.remote_command({"source_sha256":{}},mode)


def test_remote_program_readonly_sameprocess_CPU_guard_no_file_write():
    tree=ast.parse(dispatch.REMOTE_READ_ONLY)
    attributes=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(attributes)&{"write_text","write_bytes","open","mkdir","makedirs","save","savez","step","backward"}
    assert "torch.cuda._lazy_init=deny_cuda" in dispatch.REMOTE_READ_ONLY
    assert "after_cuda=torch.cuda.is_initialized()" in dispatch.REMOTE_READ_ONLY
    assert "runner.main([\"--mode\",mode])" in dispatch.REMOTE_READ_ONLY
    assert "no fit or validate-only dispatch permitted" in dispatch.REMOTE_READ_ONLY


def test_exact_payload_and_unset_environment_contains_no_fit_arguments():
    value=dispatch.contract();command=dispatch.remote_command(value,"runtime-contract")
    words=shlex.split(command[-1]);payload=json.loads(base64.b64decode(words[-2]))
    assert payload["source"]==str(dispatch.SOURCE) and payload["run"]==str(dispatch.RUN)
    assert len(payload["source_sha256"])==12 and words[-1]=="runtime-contract"
    assert words[:9]==["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP"]
    assert "CUDA_VISIBLE_DEVICES=" in words and "PYTHONNOUSERSITE=1" in words
    assert "--validate-only" not in words and "--authorization" not in words
