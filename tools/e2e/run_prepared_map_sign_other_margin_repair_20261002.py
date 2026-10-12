"""HH_261002 - ROOT-only separate FIT authority transfer, TRAIN-only validation and fixed4000 A/B; no install/promotion."""
from __future__ import annotations

import argparse
import base64
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

from . import dispatch_map_sign_other_margin_CPU_prepare_20261002 as CPU

ROOT,PLAN,WORKSPACE,SOURCE,RUN,VENV,SSH = CPU.ROOT,CPU.PLAN,CPU.WORKSPACE,CPU.SOURCE,CPU.RUN,CPU.VENV,CPU.SSH
PREPARED=CPU.RECEIPT
OUTPUT=ROOT/"storage/experiments/training/2026-10-02/map_sign_other_margin_fit_dispatch_v1"
REMOTE_PLAN=WORKSPACE/"plans"/CPU.NAME
REMOTE_AUTH=REMOTE_PLAN/"authorization.json"
GPU="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
SCHEMA="portable_e2e.map_sign_other_margin_explicit_ROOT_fit_dispatch.20261002.v1"
CPU_SOURCE_SHA="da858057ad93d71abea29fbfd3f67ab73b07288ef14cc7a078b8d5946251fbb5"
require,digest,read_json,write_new=CPU.require,CPU.digest,CPU.read_json,CPU._write

# HH_261002 - Match the frozen inner GPU idle predicate (0MiB, not a guessed driver allowance); supplement empty compute-apps.
REMOTE_CHECK=r'''
import base64,hashlib,json,pathlib,subprocess,sys
def require(ok,message):
    if not ok:raise ValueError(message)
value=json.loads(base64.b64decode(sys.argv[1]));after=sys.argv[2]=="after"
workspace=pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e")
source=workspace/"source_worktrees/map_sign_other_margin_repair_20261002_v1"
run=workspace/"runs/diagnostics/map_sign_other_margin_repair_20261002_v1"
authority=workspace/"plans/map_sign_other_margin_repair_20261002_v1/authorization.json"
require(sys.flags.optimize==0 and pathlib.Path(sys.prefix)==workspace/"venvs/py312","private unoptimized Python required")
for path in (source,authority,*source.parents,*authority.parents):require(not path.is_symlink(),"canonical owned sources/authority required")
pins=value["source_sha256"]
require(len(pins)==12 and set(p.name for p in source.iterdir())==set(pins),"exact12 flat sources required")
for name,sha in pins.items():
    path=source/name
    require(path.is_file() and not path.is_symlink() and path.resolve()==path
        and hashlib.sha256(path.read_bytes()).hexdigest()==sha,"source bytes changed")
require(authority.is_file() and hashlib.sha256(authority.read_bytes()).hexdigest()==value["authority_sha256"],"fit authority bytes changed")
require(not run.is_symlink() and (after or not run.exists()),"fresh owned output required before fit")
gpu=None
if not after:
    row=subprocess.check_output(["nvidia-smi","-i","0","--query-gpu=index,uuid,memory.used,utilization.gpu",
        "--format=csv,noheader,nounits"],text=True,timeout=20).strip().split(",")
    apps=subprocess.check_output(["nvidia-smi","-i","0","--query-compute-apps=gpu_uuid,pid",
        "--format=csv,noheader,nounits"],text=True,timeout=20)
    require(len(row)==4 and row[0].strip()=="0" and row[1].strip()=="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
        and int(row[2])==0 and int(row[3])==0 and not apps.strip(),"original exact0MiB/util0/physicalGPU0/empty-apps required")
    gpu=dict(physical_index=0,uuid=row[1].strip(),memory_used_MiB=0,utilization_percent=0,compute_processes=0)
print(json.dumps(dict(source_sha256=pins,authority_sha256=value["authority_sha256"],after_fit=after,gpu_idle=gpu,
    CUDA_initialized=False,original_inner_GPU0_lease_and_preflight_unchanged=True)))
'''


def command(authority_sha,*,validate=False):
    require(CPU._sha(authority_sha),"external FIT authority SHA required")
    args=["env",*[part for key in CPU.preparation.contract()["required_unset_environment"] for part in ("-u",key)],
        "CUDA_VISIBLE_DEVICES="+GPU,"CUBLAS_WORKSPACE_CONFIG=:4096:8","PYTHONNOUSERSITE=1","PYTHONDONTWRITEBYTECODE=1",
        "OMP_NUM_THREADS=16","MKL_NUM_THREADS=16",str(VENV),"-B",str(SOURCE/"train_map_sign_other_margin_repair_20261002.py"),
        "--mode","fit","--authorization",str(REMOTE_AUTH),"--authorization-sha256",authority_sha]
    if validate:args.append("--validate-only")
    return "cd "+shlex.quote(str(SOURCE))+" && "+shlex.join(args)


def identities(args):
    require(digest(Path(CPU.__file__))==CPU_SOURCE_SHA,"frozen reviewed CPU dispatcher changed")
    for pin in (args.expected_CPU_declaration_sha256,args.expected_CPU_summary_sha256,args.authority_sha256):
        require(CPU._sha(pin),"all external lower-case SHA256 pins required")
    require(digest(PREPARED/"declaration.json")==args.expected_CPU_declaration_sha256
        and digest(PREPARED/"summary.json")==args.expected_CPU_summary_sha256,"exact completed CPU preparation bytes required")
    declared=read_json(PREPARED/"declaration.json");summary=read_json(PREPARED/"summary.json")
    value=declared["dispatch_contract"]
    require(value==CPU.contract() and summary.get("status")=="CPU_CONTRACTS_VERIFIED_NO_FIT"
        and summary.get("source_sha256")==value["source_sha256"] and summary.get("CUDA_initialized") is False
        and summary.get("optimizer_started") is False and summary.get("fit_executed") is False,
        "complete original CPU source/runtime/schedule preparation required")
    for role in ("runtime_contract","schedule_contract","final_source_check"):
        require(summary[role]==read_json(PREPARED/(role+"_stdout.txt")),"full CPU raw/result receipt differs")
    path=args.authority
    require(path==path.resolve() and path.is_relative_to(PLAN) and not path.is_symlink()
        and digest(path)==args.authority_sha256,"canonical externally pinned separate FIT authority required")
    auth=read_json(path);pins={k:v["sha256"] for k,v in value["source_sha256"].items()}
    require(auth.get("schema")==value["fit_contract"]["schema"]+".authorization"
        and auth.get("source_sha256")==pins and auth.get("contract")==value["fit_contract"]
        and auth.get("research_fit_authorized") is True and auth.get("native_denials_preserved") is True
        and all(auth.get(k) is False for k in ("DEV_or_HOLDOUT_used_for_fit","automatic_model_promotion","vehicle_control_approved"))
        and type(auth.get("reviewer")) is str and len(auth["reviewer"].strip())>=3,
        "exact separately reviewed paired4000 fit scope required")
    schedule=summary["schedule_contract"]["result"]
    require(auth.get("equal_global_proof_sha256")==schedule["equal_global_proof_sha256"]
        and auth.get("loss_contract_sha256")==schedule["loss_contract_sha256"]
        and auth.get("runtime_identity")==summary["runtime_contract"]["result"],"exact actual runtime/schedule/loss commitments required")
    return value,auth,pins


def _stage(stage,argv,timeout):
    try:
        result=subprocess.run(argv,capture_output=True,timeout=timeout)
        stdout,stderr,rc=result.stdout,result.stderr,result.returncode
    except subprocess.TimeoutExpired as error:
        stdout,stderr,rc=error.stdout or b"",error.stderr or b"",None
    for suffix,data in (("stdout",stdout),("stderr",stderr)):
        with (OUTPUT/(stage+"_"+suffix+".txt")).open("xb") as stream:stream.write(data)
    write_new(OUTPUT/(stage+"_command.json"),dict(argv=argv,returncode=rc,timeout_seconds=timeout))
    require(rc==0,"owned fit dispatch stage failed; original raw retained: "+stage)
    return stdout


def _remote(pins,authority_sha,*,after=False):
    encoded=base64.b64encode(json.dumps(dict(source_sha256=pins,authority_sha256=authority_sha)).encode()).decode()
    environment=["env",*[part for key in CPU.preparation.contract()["required_unset_environment"] for part in ("-u",key)],
        "CUDA_VISIBLE_DEVICES=","PYTHONNOUSERSITE=1","PYTHONDONTWRITEBYTECODE=1",str(VENV),"-B"]
    raw=_stage("remote_after" if after else "remote_before",SSH+[shlex.join(environment+["-c",REMOTE_CHECK,encoded,"after" if after else "before"])],90)
    result=json.loads(raw)
    require(result.get("source_sha256")==pins and result.get("authority_sha256")==authority_sha
        and result.get("after_fit") is after,"remote source/FIT authority witness differs")
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("execute",),required=True)
    parser.add_argument("--expected-CPU-declaration-sha256",dest="expected_CPU_declaration_sha256",required=True)
    parser.add_argument("--expected-CPU-summary-sha256",dest="expected_CPU_summary_sha256",required=True)
    parser.add_argument("--authority",type=Path,required=True);parser.add_argument("--authority-sha256",required=True)
    args=parser.parse_args(argv);value,auth,pins=identities(args);own_sha=digest(Path(__file__))
    require(not OUTPUT.exists() and not any(p.is_symlink() for p in (OUTPUT,*OUTPUT.parents)),"fresh local fit receipts required")
    OUTPUT.mkdir(parents=True,exist_ok=False)
    write_new(OUTPUT/"declaration.json",dict(schema=SCHEMA,dispatcher_sha256=own_sha,source_sha256=pins,
        authority_sha256=args.authority_sha256,CPU_declaration_sha256=args.expected_CPU_declaration_sha256,
        CPU_summary_sha256=args.expected_CPU_summary_sha256,validate_command=command(args.authority_sha256,validate=True),
        fit_command=command(args.authority_sha256),original_GPU_preflight_exact_zero_MiB=True,model_promoted=False))
    (OUTPUT/"source").mkdir()
    for provider in (Path(__file__),Path(CPU.__file__)):
        with (OUTPUT/"source"/provider.name).open("xb") as stream:stream.write(provider.read_bytes())
    try:
        mkdir=" && ".join(["test -d "+shlex.quote(str(REMOTE_PLAN.parent)),
            "test ! -L "+shlex.quote(str(REMOTE_PLAN.parent)),"test ! -L "+shlex.quote(str(REMOTE_PLAN)),
            "mkdir -p -- "+shlex.quote(str(REMOTE_PLAN)),"test ! -e "+shlex.quote(str(REMOTE_AUTH)),
            "test ! -L "+shlex.quote(str(REMOTE_AUTH))])
        _stage("create_fit_authority_directory",SSH+[mkdir],45)
        _stage("transfer_fit_authority",["rsync","-rt","--ignore-existing","--chmod=Fu=rw,Fgo=",
            "-e",shlex.join(SSH[:-1]),"--",str(args.authority),SSH[-1]+":"+str(REMOTE_AUTH)],90)
        _remote(pins,args.authority_sha256)
        checked=json.loads(_stage("TRAIN_CPU_validation",SSH+[command(args.authority_sha256,validate=True)],600))
        require(checked.get("status")=="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT" and checked.get("source_sha256")==pins
            and checked.get("authorization_sha256")==args.authority_sha256 and checked.get("training_authorized") is True,
            "actual authenticated TRAIN-only preflight required before optimizer")
        identities(args);require(digest(Path(__file__))==own_sha,"fit dispatcher changed before launch")
        with (OUTPUT/"actual_fit_stdout.jsonl").open("xb") as stdout,(OUTPUT/"actual_fit_stderr.txt").open("xb") as stderr:
            process=subprocess.Popen(SSH+[command(args.authority_sha256)],stdout=stdout,stderr=stderr)
            started=dict(status="OWNED_SSH_FIT_COMMAND_STARTED_OPTIMIZER_NOT_YET_PROVEN",SSH_PID=process.pid,
                started_UTC=datetime.now(timezone.utc).isoformat(),fixed_updates_per_arm=4000,remote_output=str(RUN),physical_GPU=0)
            write_new(OUTPUT/"started.json",started);print(json.dumps(started),flush=True);rc=process.wait()
        write_new(OUTPUT/"completion.json",dict(status="SSH_EXITED_RESULTS_NOT_PROMOTED",returncode=rc,
            stdout_sha256=digest(OUTPUT/"actual_fit_stdout.jsonl"),stderr_sha256=digest(OUTPUT/"actual_fit_stderr.txt")))
        require(rc==0,"actual paired fit failed; raw stdout/stderr and remote partial evidence retained")
        _remote(pins,args.authority_sha256,after=True);identities(args)
        require(digest(Path(__file__))==digest(OUTPUT/"source"/Path(__file__).name)==own_sha
            and digest(OUTPUT/"source"/Path(CPU.__file__).name)==CPU_SOURCE_SHA,"fit dispatcher/source archive changed after fit")
        write_new(OUTPUT/"source_postcheck.json",dict(all_sources_and_authority_unchanged=True,
            original_GPU0_lease_and_inner_exact0_preflight_unchanged=True,all27_results_not_inferred_from_exit_code=True,model_promoted=False))
        return 0
    except Exception as error:
        write_new(OUTPUT/"failure_receipt.json",dict(status="FAILED_PARTIAL_RETAINED",error=type(error).__name__+": "+str(error),
            packages_installed=False,foreign_processes_killed=False,model_promoted=False));raise


if __name__=="__main__":raise SystemExit(main())
