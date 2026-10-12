"""HH_261002 - Prepare ONLY a fresh reviewed private source clone and CPU validation; never start a fit."""
from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as training

ROOT=Path(__file__).resolve().parents[2]
REMOTE=Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE=REMOTE/"source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"
RUN=REMOTE/"runs/training/temporal_intrinsic_curve_pair_20261002_v1"
PYTHON=REMOTE/"venvs/py312/bin/python"
PARENT=REMOTE/"runs/training/temporal_route_joint_20261002_v1"
WARM=REMOTE/"runs/training/temporal_route_recovery_v3_20261001_v1/exports/step_003000"
BACKBONE=REMOTE/"runs/training/temporal_v2_balanced_replay_20261001_v1/exports/step_003000"
PREPARED=Path("/home/pro6000/personal/hwanhong/dataset/prepared")
RECOVERY=PREPARED/"town07-signed-recovery-train-20261001-v4"
HOLDOUT=PREPARED/"town07-signed-recovery-holdout-20261001-v4"
PARENT_HISTORY_SHA="3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
SSH=["ssh","-T","-oBatchMode=yes","-oConnectTimeout=15","pro6000-training"]

# HH_261002 - Same reviewed tar-stream/regular-file/xb mechanism, scoped to this NEW29-file source+authority bundle.
REMOTE_COPY=r'''
import base64, hashlib, json, os, pathlib, sys, tarfile
destination=pathlib.Path(sys.argv[1]); expected=json.loads(base64.b64decode(sys.argv[2]))
workspace=pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e").resolve(strict=True)
assert destination==workspace/"source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"
assert not any(p.is_symlink() for p in (destination,*destination.parents))
assert destination.parent.is_dir() and not destination.exists()
assert len(expected)==29 and "authorization.json" in expected
assert (workspace/"venvs/py312/bin/python").is_file()
os.mkdir(destination); seen=set()
with tarfile.open(fileobj=sys.stdin.buffer,mode="r|") as archive:
    for member in archive:
        name=pathlib.PurePosixPath(member.name)
        assert member.isfile() and not name.is_absolute() and ".." not in name.parts
        assert str(name) in expected and str(name) not in seen and member.size<=2*1024**2
        data=archive.extractfile(member).read()
        assert hashlib.sha256(data).hexdigest()==expected[str(name)]
        path=destination/name;path.parent.mkdir(parents=True,exist_ok=True)
        assert not any(p.is_symlink() for p in (path,*path.parents))
        with path.open("xb") as stream:stream.write(data)
        seen.add(str(name))
assert seen==set(expected)
assert all(hashlib.sha256((destination/name).read_bytes()).hexdigest()==sha for name,sha in expected.items())
assert not (workspace/"runs/training/temporal_intrinsic_curve_pair_20261002_v1").exists()
receipt=dict(status="EXACT28_SOURCE_PLUS_AUTH_CREATE_ONLY_COPIED",source=str(destination),file_sha256=expected,
    source_file_count=28,authorization_file_count=1,packages_installed=False,existing_files_overwritten=False)
with (destination/"transfer_receipt.json").open("x") as stream:json.dump(receipt,stream,indent=2,sort_keys=True)
print(json.dumps(receipt,sort_keys=True))
'''


def command(auth_sha,*,validate):
    # HH_261002 - Remove inherited Python injection before private-venv startup. GPU visibility is never a numeric ordinal alias.
    environment=["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP",
        "PYTHONNOUSERSITE=1","OMP_NUM_THREADS=4","CUBLAS_WORKSPACE_CONFIG=:4096:8",
        "CUDA_VISIBLE_DEVICES="+("" if validate else training.original.GPU_UUID)]
    arguments=[str(PYTHON),"-B",str(SOURCE/"tools/workspace_python.py"),"-m","scripts.e2e.train_temporal_intrinsic_curve_pair_20261002","--",
        "--mode","validate-only" if validate else "fit","--parent-run",str(PARENT),
        "--parent-history-sha256",PARENT_HISTORY_SHA,"--warm-parent-weights",str(WARM/"model_state.npz"),
        "--warm-parent-metadata",str(WARM/"metadata.json"),"--backbone-parent-weights",str(BACKBONE/"model_state.npz"),
        "--backbone-parent-metadata",str(BACKBONE/"metadata.json"),"--recovery-train",str(RECOVERY),
        "--recovery-conversion",str(RECOVERY/"moving_conversion_qualification.json"),"--recovery-holdout",str(HOLDOUT),
        "--holdout-conversion",str(HOLDOUT/"moving_conversion_qualification.json"),"--output",str(RUN),
        "--authorization",str(SOURCE/"authorization.json"),"--authorization-sha256",auth_sha,"--loader-workers","2"]
    return shlex.join(environment+arguments)


def package(sources,authorization,auth_sha):
    payload=io.BytesIO();files=dict(sources,**{"authorization.json":auth_sha})
    with tarfile.open(fileobj=payload,mode="w") as archive:
        for name,sha in sorted(files.items()):
            path=authorization if name=="authorization.json" else ROOT/name
            data=path.read_bytes();training.require(hashlib.sha256(data).hexdigest()==sha,"source/auth bytes changed while packaging")
            training.require(not path.is_symlink() and len(data)<=2*1024**2,"bounded regular source/auth required")
            info=tarfile.TarInfo(name=name);info.size,info.mode=len(data),0o600;archive.addfile(info,io.BytesIO(data))
    return payload.getvalue(),files


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("prepare",),required=True)
    parser.add_argument("--authorization",type=Path,required=True);parser.add_argument("--authorization-sha256",required=True)
    parser.add_argument("--expected-runner-sha256",required=True);parser.add_argument("--expected-loss-sha256",required=True)
    parser.add_argument("--receipt",type=Path,required=True)
    args=parser.parse_args(argv)
    training.require(not args.receipt.exists() and not any(p.is_symlink() for p in (args.receipt,*args.receipt.parents))
        and args.receipt.resolve().is_relative_to(ROOT/"storage/experiments/training/2026-10-02"),"fresh canonical dispatch receipt required")
    training.require(not any(p.is_symlink() for p in (args.authorization,*args.authorization.parents)),"regular original root authorization required")
    contract=training.contract();authority=training.validate_authorization(args.authorization,args.authorization_sha256,contract)
    training.authorize_parent_history(authority,PARENT_HISTORY_SHA);sources=contract["sources"]
    training.require(len(sources)==28 and sources["tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"]==args.expected_runner_sha256
        and sources["learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"]==args.expected_loss_sha256,
        "exact externally reviewed runner/loss/all28 source pins required")
    payload,files=package(sources,args.authorization,args.authorization_sha256)
    args.receipt.mkdir(parents=True,exist_ok=False)
    declaration=dict(comment="HH_261002 - Fresh reviewed canonical transfer and TRAIN-only CPU preflight; NO optimizer dispatch.",
        dispatch_source_sha256=training.digest(Path(__file__)),source_sha256=sources,authorization_sha256=args.authorization_sha256,
        parent_four_actual_sha256=training.PARENT,parent_actual6000_history_sha256=PARENT_HISTORY_SHA,
        remote_source=str(SOURCE),remote_run=str(RUN),existing_private_venv=str(PYTHON),packages_installed=False,
        optimizer_started=False,physical_GPU_allowed=training.original.GPU_UUID,
        validation_command=command(args.authorization_sha256,validate=True),
        proposed_fit_command_requires_separate_root_execution=command(args.authorization_sha256,validate=False))
    training.write_new(args.receipt/"declaration.json",declaration)
    for name in ("tools/e2e/dispatch_temporal_intrinsic_curve_pair_20261002.py",):
        path=args.receipt/"source"/name;path.parent.mkdir(parents=True)
        with path.open("xb") as stream:stream.write((ROOT/name).read_bytes())
    try:
        encoded=base64.b64encode(json.dumps(files,sort_keys=True).encode()).decode()
        transfer=subprocess.run(SSH+[shlex.join(["python3","-c",REMOTE_COPY,str(SOURCE),encoded])],input=payload,capture_output=True,timeout=120)
        for name,data in (("transfer_stdout.json",transfer.stdout),("transfer_stderr.txt",transfer.stderr)):
            with (args.receipt/name).open("xb") as stream:stream.write(data)
        training.require(transfer.returncode==0,"create-only transfer failed; original stdout/stderr/partial source retained")
        copied=json.loads(transfer.stdout);training.require(copied["file_sha256"]==files,"copied source/auth receipt differs")
        validation=subprocess.run(SSH+[command(args.authorization_sha256,validate=True)],capture_output=True,timeout=1800)
        for name,data in (("remote_validation_stdout.json",validation.stdout),("remote_validation_stderr.txt",validation.stderr)):
            with (args.receipt/name).open("xb") as stream:stream.write(data)
        training.require(validation.returncode==0,"CPU preflight failed; NO training started; partial receipts retained")
        checked=json.loads(validation.stdout)
        training.require(checked["status"]=="VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT" and checked["protocol"]==contract
            and checked["source_sha256"]==sources and checked["parent_history_sha256"]==PARENT_HISTORY_SHA
            and checked["trainable_state_tensor_count"]==54 and checked["frozen_state_tensor_count"]==47
            and checked["holdout_payload_loaded"] is False and checked["model_promoted"] is False,"remote CPU protocol/membership/source differs")
        training.require(training.source_snapshot()==sources and training.digest(args.authorization)==args.authorization_sha256
            and training.digest(Path(__file__))==declaration["dispatch_source_sha256"],"local source/auth changed during preparation")
        training.write_new(args.receipt/"summary.json",dict(status="PRIVATE_REMOTE_TRAIN_ONLY_CPU_PREFLIGHT_PASS_NO_FIT",
            optimizer_started=False,source_file_count=28,authorization_file_count=1,model_promoted=False,
            remote_validation_sha256=training.digest(args.receipt/"remote_validation_stdout.json"),
            proposed_fit_command_requires_separate_root_execution=declaration["proposed_fit_command_requires_separate_root_execution"]))
    except Exception as error:
        training.write_new(args.receipt/"failure_receipt.json",dict(status="PREPARATION_FAILED_NO_FIT_PARTIAL_RETAINED",
            error=type(error).__name__+": "+str(error),optimizer_started=False,packages_installed=False));raise
    print(json.dumps(dict(status="PRIVATE_REMOTE_TRAIN_ONLY_CPU_PREFLIGHT_PASS_NO_FIT",receipt=str(args.receipt)),indent=2))
    return 0


if __name__=="__main__":raise SystemExit(main())
