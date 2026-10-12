"""HH_261002 - Prove separate FIT authority and fixed command with synthetic receipts/subprocesses only."""
import ast
import json
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest
from scripts.e2e import run_prepared_map_sign_other_margin_repair_20261002 as fit


def fixture(tmp_path,monkeypatch,fault=None):
    prepared=tmp_path/"prepared";prepared.mkdir()
    monkeypatch.setattr(fit,"PREPARED",prepared);monkeypatch.setattr(fit,"PLAN",tmp_path)
    monkeypatch.setattr(fit,"OUTPUT",tmp_path/"actual_fit")
    source={"synthetic.py":dict(local_path=str(tmp_path/"synthetic.py"),sha256="a"*64)}
    value=dict(source_sha256=source,fit_contract=dict(schema="synthetic",training_authorized=False))
    monkeypatch.setattr(fit.CPU,"contract",lambda:value)
    pins={k:v["sha256"] for k,v in source.items()}
    runtime=dict(mode="runtime-contract",result={"providers":"synthetic"},CUDA_initialized_after=False)
    schedule=dict(mode="schedule-contract",result=dict(equal_global_proof_sha256="b"*64,loss_contract_sha256="c"*64))
    post=dict(mode="source-check",source_sha256_after=pins)
    summary=dict(status="CPU_CONTRACTS_VERIFIED_NO_FIT",source_sha256=source,CUDA_initialized=False,
        optimizer_started=False,fit_executed=False,runtime_contract=runtime,schedule_contract=schedule,final_source_check=post)
    for name,data in (("runtime_contract_stdout.txt",runtime),("schedule_contract_stdout.txt",schedule),
        ("final_source_check_stdout.txt",post),("summary.json",summary),("declaration.json",dict(dispatch_contract=value))):
        (prepared/name).write_text(json.dumps(data))
    authority=tmp_path/"ROOT_fit_authority.json"
    auth=dict(schema="synthetic.authorization",source_sha256=pins,contract=value["fit_contract"],research_fit_authorized=True,
        native_denials_preserved=True,DEV_or_HOLDOUT_used_for_fit=False,automatic_model_promotion=False,
        vehicle_control_approved=False,reviewer="ROOT",equal_global_proof_sha256="b"*64,loss_contract_sha256="c"*64,
        runtime_identity=runtime["result"])
    if fault in auth:
        auth[fault]=not auth[fault] if type(auth[fault]) is bool else "wrong"
    if fault=="reviewer":auth["reviewer"]=" "
    authority.write_text(json.dumps(auth))
    args=SimpleNamespace(authority=authority,authority_sha256=fit.digest(authority),
        expected_CPU_declaration_sha256=fit.digest(prepared/"declaration.json"),expected_CPU_summary_sha256=fit.digest(prepared/"summary.json"))
    if fault=="summary_pin":args.expected_CPU_summary_sha256="0"*64
    if fault=="authority_pin":args.authority_sha256="0"*64
    if fault=="raw":(prepared/"runtime_contract_stdout.txt").write_text('{}')
    if fault=="duplicate":authority.write_text('{"schema":1,"schema":2}');args.authority_sha256=fit.digest(authority)
    return args,value,auth,pins


def arguments(args):
    return ["--mode","execute","--expected-CPU-declaration-sha256",args.expected_CPU_declaration_sha256,
        "--expected-CPU-summary-sha256",args.expected_CPU_summary_sha256,"--authority",str(args.authority),
        "--authority-sha256",args.authority_sha256]


def fake_processes(monkeypatch,args,pins,*,fault=None):
    calls=[]
    def run(argv,**kwargs):
        calls.append(argv)
        if "--validate-only" in argv[-1]:
            value=dict(status="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT",source_sha256=pins,
                authorization_sha256=args.authority_sha256,training_authorized=True)
            if fault=="validate":value["status"]="WRONG"
        elif "--ignore-existing" in argv or "mkdir -p" in argv[-1]:value={}
        else:
            after=shlex.split(argv[-1])[-1]=="after"
            value=dict(source_sha256=pins,authority_sha256=args.authority_sha256,after_fit=after)
            if fault=="source" and not after:value["source_sha256"]={}
        return SimpleNamespace(returncode=0,stdout=json.dumps(value).encode(),stderr=b"original diagnostic")
    def popen(argv,*,stdout,stderr):
        calls.append(argv);stdout.write(b'{"remote":"actual mocked stdout"}\n');stderr.write(b"retained stderr")
        return SimpleNamespace(pid=12345,wait=lambda:7 if fault=="fit" else 0)
    monkeypatch.setattr(fit.subprocess,"run",run);monkeypatch.setattr(fit.subprocess,"Popen",popen)
    return calls


def test_mock_complete_transfer_then_validate_then_fixed_fit(tmp_path,monkeypatch):
    args,value,auth,pins=fixture(tmp_path,monkeypatch);calls=fake_processes(monkeypatch,args,pins)
    assert fit.main(arguments(args))==0
    assert calls[1][:3]==["rsync","-rt","--ignore-existing"] and calls[1][-1].endswith(str(fit.REMOTE_AUTH))
    assert len(calls)==6 and "--validate-only" in calls[3][-1] and "--validate-only" not in calls[4][-1]
    assert "--mode fit" in calls[4][-1]
    assert json.loads((fit.OUTPUT/"started.json").read_text())["SSH_PID"]==12345
    final=json.loads((fit.OUTPUT/"source_postcheck.json").read_text())
    assert final["all27_results_not_inferred_from_exit_code"] and not final["model_promoted"]
    assert (fit.OUTPUT/"actual_fit_stdout.jsonl").read_bytes().startswith(b'{"remote"')
    before=(fit.OUTPUT/"declaration.json").read_bytes()
    with pytest.raises(ValueError,match="fresh local"):fit.main(arguments(args))
    assert (fit.OUTPUT/"declaration.json").read_bytes()==before and len(calls)==6


@pytest.mark.parametrize("fault",["research_fit_authorized","native_denials_preserved","DEV_or_HOLDOUT_used_for_fit",
    "automatic_model_promotion","vehicle_control_approved","schema","source_sha256","contract","reviewer",
    "equal_global_proof_sha256","loss_contract_sha256","runtime_identity","summary_pin","authority_pin","raw","duplicate"])
def test_no_wrong_or_expanded_or_unmatched_FIT_authority(tmp_path,monkeypatch,fault):
    args,value,auth,pins=fixture(tmp_path,monkeypatch,fault);calls=fake_processes(monkeypatch,args,pins)
    with pytest.raises(Exception):fit.main(arguments(args))
    assert calls==[] and not fit.OUTPUT.exists()


@pytest.mark.parametrize("fault",["source","validate","fit"])
def test_errors_no_late_fakePASS_or_overwritten_partial(tmp_path,monkeypatch,fault):
    args,value,auth,pins=fixture(tmp_path,monkeypatch);calls=fake_processes(monkeypatch,args,pins,fault=fault)
    with pytest.raises(ValueError):fit.main(arguments(args))
    report=json.loads((fit.OUTPUT/"failure_receipt.json").read_text())
    assert report["status"]=="FAILED_PARTIAL_RETAINED" and not report["foreign_processes_killed"]
    assert not (fit.OUTPUT/"source_postcheck.json").exists()
    if fault!="fit":assert not (fit.OUTPUT/"started.json").exists()
    else:
        assert json.loads((fit.OUTPUT/"completion.json").read_text())["returncode"]==7
        assert (fit.OUTPUT/"actual_fit_stderr.txt").read_bytes()==b"retained stderr"


@pytest.mark.parametrize("timeout",[False,True])
def test_stage_raw_nonzero_and_timeout_preserved(tmp_path,monkeypatch,timeout):
    monkeypatch.setattr(fit,"OUTPUT",tmp_path)
    def run(*args,**kwargs):
        if timeout:raise fit.subprocess.TimeoutExpired(args,10,output=b"partial out",stderr=b"partial err")
        return SimpleNamespace(returncode=4,stdout=b"partial out",stderr=b"partial err")
    monkeypatch.setattr(fit.subprocess,"run",run)
    with pytest.raises(ValueError,match="raw retained"):fit._stage("test",["mock"],10)
    assert (tmp_path/"test_stdout.txt").read_bytes()==b"partial out"
    assert json.loads((tmp_path/"test_command.json").read_text())["returncode"]==(None if timeout else 4)


def test_command_exact_private_identity_OMP16_and_validation_single_flag():
    actual=fit.command("a"*64);validate=fit.command("a"*64,validate=True)
    assert validate==actual+" --validate-only" and actual.startswith("cd "+str(fit.SOURCE)+" && env -u PYTHONPATH")
    assert "CUDA_VISIBLE_DEVICES="+fit.GPU in actual and "OMP_NUM_THREADS=16" in actual and "MKL_NUM_THREADS=16" in actual
    assert str(fit.VENV)+" -B "+str(fit.SOURCE/"train_map_sign_other_margin_repair_20261002.py") in actual
    assert "--authorization "+str(fit.REMOTE_AUTH) in actual and "--mode fit" in actual
    with pytest.raises(ValueError):fit.command("wrong")


def test_remote_exact_ZERO_driver_allowance_and_no_torch_or_writes():
    tree=ast.parse(fit.REMOTE_CHECK)
    imports=[a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
    assert "torch" not in imports and "int(row[2])==0" in fit.REMOTE_CHECK and "not apps.strip()" in fit.REMOTE_CHECK
    calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(calls)&{"write_text","write_bytes","open","mkdir","step","backward"}
