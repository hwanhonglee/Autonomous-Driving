"""HH_261002 - Print/seal a CPU-only preparation command contract; never SSH, initialize CUDA, fit, or install anything."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT/"storage/experiments/plans/2026-10-02/map_sign_other_margin_repair_v1"
AUTH_SCHEMA = "portable_e2e.map_sign_other_margin_CPU_preparation_authority.20261002.v1"
SCHEMA = "portable_e2e.map_sign_other_margin_CPU_preparation.20261002.v1"
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
NAME = "map_sign_other_margin_repair_20261002_v1"
SOURCE, VENV = WORKSPACE/"source_worktrees"/NAME, WORKSPACE/"venvs/py312/bin/python"


def require(condition, message):
    if not condition: raise ValueError(message)


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 1024**2, "bounded canonical JSON file required")
    def pairs(items):
        require(len({k for k,v in items})==len(items), "duplicate JSON field")
        return dict(items)
    return json.loads(path.read_text(), object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON")))


def contract():
    # HH_261002 - Import only declaration helpers: the old/new losses import Torch only when a tensor loss is actually called.
    from . import train_map_sign_other_margin_repair_20261002 as runner
    pins = {}
    for name in sorted(runner.SOURCE_NAMES):
        matches = (ROOT/"tools/e2e"/name, ROOT/"learning/portable_e2e"/name)
        paths = [p for p in matches if p.is_file()]
        require(len(paths)==1 and not paths[0].is_symlink(), "one canonical local source provider required: "+name)
        pins[name]=dict(local_path=str(paths[0]),sha256=digest(paths[0]))
    require(len(pins)==12 and all(pins[name]["sha256"]==sha for name,sha in runner.FROZEN_SOURCE_SHA.items()),
        "all ten frozen TRAIN providers must remain exact")
    commands = []
    for stage, args in (("runtime_identity",["--mode","runtime-contract"]),
        ("schedule_and_source_identity",["--mode","schedule-contract"]),
        ("TRAIN_only_CPU_preflight",["--mode","fit","--authorization",str(WORKSPACE/"plans"/NAME/"authorization.json"),
            "--authorization-sha256","<SEPARATELY_PINNED_ROOT_FIT_AUTHORITY_SHA>","--validate-only"])):
        command=[str(VENV),"-B",str(SOURCE/Path(runner.__file__).name),*args]
        commands.append(dict(stage=stage,argv=command,shell=shlex.join(command),must_run_in=str(SOURCE),
            physical_GPU0_lease_entered=False,CUDA_initialization_allowed=False,optimizer_allowed=False))
    return dict(schema=SCHEMA,actual_host="pro6000",private_workspace=str(WORKSPACE),private_python=str(VENV),
        remote_source=str(SOURCE),fit_contract=runner.prepared_contract(),source_sha256=pins,
        preparation_dispatcher_sha256=digest(Path(__file__)),commands=commands,
        required_environment=dict(CUDA_VISIBLE_DEVICES=runner.previous.mining.GPU_UUID,CUBLAS_WORKSPACE_CONFIG=":4096:8",
            PYTHONNOUSERSITE="1",PYTHONDONTWRITEBYTECODE="1",OMP_NUM_THREADS="16",MKL_NUM_THREADS="16"),
        required_unset_environment=["PYTHONPATH","PYTHONHOME","PYTHONUSERBASE","PYTHONSTARTUP"],
        only_physical_GPU0_allowed=True,actual_GPU0_preflight_and_lease_required_for_later_ROOT_fit=True,
        source_transfer_performed=False,remote_command_executed=False,all_commands_not_executed=True,
        package_installation_allowed=False,CPU_prepare_only=True,fit_authorized=False,
        no_DEV_or_HOLDOUT_payload_access=True,model_promoted=False,vehicle_control_approved=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","seal"),default="contract")
    parser.add_argument("--authority",type=Path);parser.add_argument("--authority-sha256")
    args=parser.parse_args(argv); value=contract()
    if args.mode=="contract": print(json.dumps(value,indent=2,allow_nan=False));return 0
    require(args.authority is not None and type(args.authority_sha256) is str and len(args.authority_sha256)==64
        and all(c in "0123456789abcdef" for c in args.authority_sha256),"external ROOT CPU-preparation authority required")
    path=args.authority.resolve()
    require(path==args.authority and path.is_relative_to(PLAN) and not path.is_symlink()
        and digest(path)==args.authority_sha256,"exact new preparation authority bytes/path required")
    authority=read_json(path)
    require(authority.get("schema")==AUTH_SCHEMA and authority.get("CPU_preparation_contract")==value
        and authority.get("CPU_preparation_authorized") is True and authority.get("fit_authorized") is False
        and authority.get("remote_execution_authorized") is False,"scope must remain CPU-plan-only, not actual training")
    output=PLAN/"prepared_CPU_command_contract.json"
    require(not output.exists() and not output.is_symlink(),"new command receipt must not overwrite")
    require(contract()==value,"local sources changed during seal")
    with output.open("x") as stream:
        json.dump(dict(**value,external_CPU_preparation_authority_sha256=args.authority_sha256),stream,indent=2,allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(prepared_path=str(output),prepared_sha256=digest(output),remote_executed=False,fit_authorized=False)))
    return 0


if __name__=="__main__":raise SystemExit(main())
