"""HH_261002 - Transfer exactly12 sources and query two CPU contracts only after an external ROOT authority."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time

from . import prepare_map_sign_other_margin_repair_20261002 as preparation

ROOT, PLAN, SOURCE, VENV = preparation.ROOT, preparation.PLAN, preparation.SOURCE, preparation.VENV
WORKSPACE, NAME = preparation.WORKSPACE, preparation.NAME
RUN = WORKSPACE/"runs/diagnostics"/NAME
RECEIPT = ROOT/"storage/experiments/training/2026-10-02/map_sign_other_margin_CPU_prepare_dispatch_v1"
SCHEMA = "portable_e2e.map_sign_other_margin_actual_CPU_preparation_dispatch.20261002.v1"
AUTH_SCHEMA = SCHEMA+".external_ROOT_authority"
SSH = ["ssh","-T","-p","6000","-i","/home/a/.ssh/pro6000_ed25519_v2",
    "-oBatchMode=yes","-oConnectTimeout=15","pro6000@220.90.5.14"]
MODES = ("runtime-contract","schedule-contract")
require, digest, read_json = preparation.require, preparation.digest, preparation.read_json

# HH_261002 - This remote Python is read-only: directory creation/transfer use explicit mkdir/rsync, not hidden file writes.
REMOTE_READ_ONLY = r'''
import base64, contextlib, hashlib, importlib, io, json, os, pathlib, sys
def require(ok,message):
    if not ok: raise ValueError(message)
value=json.loads(base64.b64decode(sys.argv[1]));mode=sys.argv[2]
source=pathlib.Path(value["source"]);run=pathlib.Path(value["run"])
workspace=pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e")
require(source==workspace/"source_worktrees/map_sign_other_margin_repair_20261002_v1"
    and run==workspace/"runs/diagnostics/map_sign_other_margin_repair_20261002_v1", "exact private paths required")
require(sys.flags.optimize==0 and pathlib.Path(sys.prefix)==workspace/"venvs/py312", "existing private unoptimized Python required")
for path in (workspace,source.parent,source):
    require(path.is_dir() and not path.is_symlink() and path.resolve()==path,"canonical private directory required")
require(os.environ.get("CUDA_VISIBLE_DEVICES")=="" and os.environ.get("PYTHONNOUSERSITE")=="1"
    and os.environ.get("PYTHONDONTWRITEBYTECODE")=="1", "CPU-only import environment required")
require(not any(name in os.environ for name in ("PYTHONPATH","PYTHONHOME","PYTHONUSERBASE","PYTHONSTARTUP")),
    "ambient Python providers must be unset")
expected=value["source_sha256"]
def snapshot():
    require(set(p.name for p in source.iterdir())==set(expected) and len(expected)==12,"exact12 flat files required")
    got={}
    for name,sha in expected.items():
        path=source/name
        require(path.is_file() and not path.is_symlink() and path.resolve()==path and path.stat().st_size<=2*1024**2,
            "bounded canonical source required")
        got[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    require(got==expected,"remote source changed")
    require(not run.exists() and not run.is_symlink(),"training output must remain absent")
    return got
before=snapshot();result=None;before_cuda=None;after_cuda=None
require(mode in ("source-check","runtime-contract","schedule-contract"),"no fit or validate-only dispatch permitted")
if mode!="source-check":
    os.chdir(source);sys.path.insert(0,str(source))
    runner=importlib.import_module("train_map_sign_other_margin_repair_20261002")
    require(pathlib.Path(runner.__file__).resolve()==source/"train_map_sign_other_margin_repair_20261002.py",
        "runner import escaped sealed flat package")
    import torch
    before_cuda=torch.cuda.is_initialized()
    require(before_cuda is False,"CUDA already initialized")
    def deny_cuda(*args,**kwargs): raise ValueError("CPU preparation forbids CUDA initialization")
    saved_lazy=torch.cuda._lazy_init;torch.cuda._lazy_init=deny_cuda
    try:
        stream=io.StringIO()
        with contextlib.redirect_stdout(stream):
            rc=runner.main(["--mode",mode])
        require(rc==0,"CPU contract entry failed")
        result=json.loads(stream.getvalue(),parse_constant=lambda text: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
        after_cuda=torch.cuda.is_initialized()
        require(after_cuda is False,"CUDA initialized during CPU contract")
    finally: torch.cuda._lazy_init=saved_lazy
after=snapshot()
print(json.dumps(dict(mode=mode,result=result,source_sha256_before=before,source_sha256_after=after,
    CUDA_initialized_before=before_cuda,CUDA_initialized_after=after_cuda,training_output_absent=True,
    fit_invoked=False,optimizer_updates_authorized=0,DEV_HOLDOUT_payload_access_authorized=False),allow_nan=False))
'''


def _sha(value):
    return type(value) is str and len(value)==64 and all(c in "0123456789abcdef" for c in value)


def _document_sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def _environment():
    # HH_261002 - The GPU remains invisible even though the later, separately authorized fit permits only physical GPU0.
    return ["env",*[part for key in preparation.contract()["required_unset_environment"] for part in ("-u",key)],
        "CUDA_VISIBLE_DEVICES=","PYTHONNOUSERSITE=1","PYTHONDONTWRITEBYTECODE=1",
        "OMP_NUM_THREADS=1","MKL_NUM_THREADS=1","CUBLAS_WORKSPACE_CONFIG=:4096:8"]


def contract():
    prepared=preparation.contract()
    require(len(prepared["source_sha256"])==12 and prepared["remote_source"]==str(SOURCE),"exact12 prepared sources required")
    return dict(schema=SCHEMA,source_sha256=prepared["source_sha256"],fit_contract=prepared["fit_contract"],
        command_only_preparation_sha256=digest(Path(preparation.__file__)),dispatcher_sha256=digest(Path(__file__)),
        remote_read_only_program_sha256=hashlib.sha256(REMOTE_READ_ONLY.encode()).hexdigest(),
        remote_host=SSH[-1],SSH_argv=SSH,remote_source=str(SOURCE),private_python=str(VENV),remote_training_output=str(RUN),
        local_receipt=str(RECEIPT),allowed_remote_runner_modes=list(MODES),source_count=12,
        source_creation="exclusive_mkdir_then_rsync_ignore_existing_and_exact_source_census",
        required_environment=dict(CUDA_VISIBLE_DEVICES="",PYTHONNOUSERSITE="1",PYTHONDONTWRITEBYTECODE="1",
            OMP_NUM_THREADS="1",MKL_NUM_THREADS="1",CUBLAS_WORKSPACE_CONFIG=":4096:8"),
        required_unset_environment=prepared["required_unset_environment"],
        TRAIN_schedule_metadata_only=True,TRAIN_RGB_and_target_arrays_read=False,DEV_HOLDOUT_payload_access=False,
        fit_or_validate_only_invocation=False,CUDA_initialization_allowed=False,optimizer_allowed=False,
        package_installation_allowed=False,model_promoted=False,vehicle_control_approved=False)


def _write(path,value):
    with path.open("x") as stream: json.dump(value,stream,indent=2,allow_nan=False);stream.write("\n")


def _run(stage,argv,*,timeout=300):
    started=time.time_ns()
    try:
        result=subprocess.run(argv,capture_output=True,timeout=timeout)
        stdout,stderr,rc=result.stdout,result.stderr,result.returncode
    except subprocess.TimeoutExpired as error:
        stdout,stderr,rc=error.stdout or b"",error.stderr or b"",None
    for suffix,data in (("stdout",stdout),("stderr",stderr)):
        with (RECEIPT/(stage+"_"+suffix+".txt")).open("xb") as stream:stream.write(data)
    _write(RECEIPT/(stage+"_command.json"),dict(argv=argv,returncode=rc,started_wall_time_ns=started,
        ended_wall_time_ns=time.time_ns(),timeout_seconds=timeout,stdout_sha256=hashlib.sha256(stdout).hexdigest(),
        stderr_sha256=hashlib.sha256(stderr).hexdigest()))
    require(rc==0,"CPU preparation stage failed; raw output retained: "+stage)
    require(len(stdout)<=8*1024**2,"bounded CPU-contract stdout required")
    return stdout


def remote_command(value,mode):
    require(mode in (*MODES,"source-check"),"only source/runtime/schedule queries allowed")
    pins={name:row["sha256"] for name,row in value["source_sha256"].items()}
    payload=base64.b64encode(json.dumps(dict(source=str(SOURCE),run=str(RUN),source_sha256=pins),sort_keys=True).encode()).decode()
    return SSH+[shlex.join(_environment()+[str(VENV),"-B","-c",REMOTE_READ_ONLY,payload,mode])]


def _remote(stage,value,mode):
    result=json.loads(_run(stage,remote_command(value,mode)),
        parse_constant=lambda text: (_ for _ in ()).throw(ValueError("nonfinite remote JSON")))
    pins={name:row["sha256"] for name,row in value["source_sha256"].items()}
    require(result.get("mode")==mode and result.get("source_sha256_before")==result.get("source_sha256_after")==pins
        and result.get("training_output_absent") is True and result.get("fit_invoked") is False
        and type(result.get("optimizer_updates_authorized")) is int and result["optimizer_updates_authorized"]==0,
        "remote source/no-training witness differs")
    if mode in MODES:
        require(result.get("CUDA_initialized_before") is False and result.get("CUDA_initialized_after") is False,
            "actual CPU process must remain CUDA-uninitialized")
    return result


def execute(authority,authority_sha):
    value=contract();path=Path(authority)
    require(_sha(authority_sha) and path==path.resolve() and path.is_relative_to(PLAN)
        and not path.is_symlink() and digest(path)==authority_sha,"externally pinned canonical ROOT CPU authority required")
    authorized=read_json(path)
    require(authorized.get("schema")==AUTH_SCHEMA and authorized.get("dispatch_contract")==value
        and all(authorized.get(key) is True for key in ("CPU_preparation_authorized","remote_source_transfer_authorized",
            "remote_contract_execution_authorized")) and all(authorized.get(key) is False for key in
            ("fit_authorized","CUDA_initialization_authorized","TRAIN_payload_access_authorized",
             "DEV_HOLDOUT_payload_access_authorized","package_installation_authorized")),
        "exact separate remote CPU-only authority required; no fit authorization")
    require(not RECEIPT.exists() and not RECEIPT.is_symlink(),"create-only local dispatch receipts required")
    RECEIPT.mkdir(parents=True,exist_ok=False)
    _write(RECEIPT/"declaration.json",dict(dispatch_contract=value,external_ROOT_authority_sha256=authority_sha))
    created=False;failure=None;runtime=None;schedule=None;post=None
    try:
        # HH_261002 - Shell performs only exact directory checks and exclusive mkdir; rsync never overwrites a source.
        setup=" && ".join(["test -d "+shlex.quote(str(WORKSPACE)),
            *["test ! -L "+shlex.quote(str(p)) for p in (WORKSPACE,SOURCE.parent,SOURCE)],
            "test -d "+shlex.quote(str(SOURCE.parent)),"test ! -e "+shlex.quote(str(RUN)),
            "test ! -L "+shlex.quote(str(RUN)),"umask 077","mkdir -- "+shlex.quote(str(SOURCE))])
        _run("create_fresh_source",SSH+[setup],timeout=45);created=True
        sources=[row["local_path"] for _,row in sorted(value["source_sha256"].items())]
        require(contract()==value,"local source contract changed before transfer")
        transport=shlex.join(SSH[:-1])
        _run("transfer_sources",["rsync","-rt","--ignore-existing","--chmod=Fu=rw,Fgo=,Du=rwx,Dgo=",
            "-e",transport,"--",*sources,SSH[-1]+":"+str(SOURCE)+"/"],timeout=90)
        _remote("copied_source_check",value,"source-check")
        runtime=_remote("runtime_contract",value,"runtime-contract")
        schedule=_remote("schedule_contract",value,"schedule-contract")
        wanted={name:row["sha256"] for name,row in value["source_sha256"].items()}
        got=schedule["result"]
        require(got.get("schema")==value["fit_contract"]["schema"]+".prospective_schedule"
            and got.get("source_sha256")==wanted and got.get("no_CUDA_or_optimizer_or_output") is True
            and got.get("fit_authorized") is False and _sha(got.get("equal_global_proof_sha256"))
            and got.get("loss_contract_sha256")==_document_sha(value["fit_contract"]["loss_contract"]),
            "schedule/loss/source commitment differs")
        require(contract()==value and digest(path)==authority_sha,"local authority/sources changed during preparation")
    except Exception as error:failure=type(error).__name__+": "+str(error)
    finally:
        if created:
            try:post=_remote("final_source_check",value,"source-check")
            except Exception as error:failure=(failure+"; " if failure else "")+"postcheck "+type(error).__name__+": "+str(error)
    _write(RECEIPT/"summary.json",dict(schema=SCHEMA,status="FAILED_RETAINED" if failure else "CPU_CONTRACTS_VERIFIED_NO_FIT",
        failure=failure,source_sha256=value["source_sha256"],runtime_contract=runtime,schedule_contract=schedule,
        final_source_check=post,remote_source_created=created,external_ROOT_authority_sha256=authority_sha,
        CUDA_initialized=False if runtime and schedule else None,optimizer_started=False,fit_executed=False,
        DEV_HOLDOUT_payload_access_authorized=False,package_installation_performed=False))
    print(json.dumps(dict(status="FAILED_RETAINED" if failure else "CPU_CONTRACTS_VERIFIED_NO_FIT",failure=failure,
        receipt=str(RECEIPT),summary_sha256=digest(RECEIPT/"summary.json")),allow_nan=False))
    return 1 if failure else 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","execute"),default="contract")
    parser.add_argument("--authority",type=Path);parser.add_argument("--authority-sha256")
    args=parser.parse_args(argv)
    if args.mode=="contract":print(json.dumps(contract(),indent=2,allow_nan=False));return 0
    require(args.authority is not None,"external ROOT authority is required")
    return execute(args.authority,args.authority_sha256)


if __name__=="__main__":raise SystemExit(main())
