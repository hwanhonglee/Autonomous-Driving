"""HH_261002 - Execute only the exact reviewed prepared fit; retain raw SSH stdout/stderr and never install/promote."""
from __future__ import annotations
import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PREPARED=ROOT/"storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_dispatch_v1"
REMOTE=Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE=REMOTE/"source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"
RUN=REMOTE/"runs/training/temporal_intrinsic_curve_pair_20261002_v1"
GPU="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
RUNNER_SHA="3da40c323c69b1d3e1a64b77f68955b4a8c169a895b5698db312f86d4b23c0c8"
LOSS_SHA="051a07c032c9335083b2e004711f58460e7e412aecb98e362c061e0a9a8ae314"
PREPARE_SHA="d50f8ac2530503df4ab4a3647a84b1b2bdfbfe9bdb8904711a5efaf5b6210be5"
HISTORY_SHA="3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
PARENT_PINS={"weights_sha256":"b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    "metadata_sha256":"a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    "checkpoint_sha256":"43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    "declaration_sha256":"0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5"}
SSH=["ssh","-T","-oBatchMode=yes","-oConnectTimeout=15","pro6000-training"]


def require(value,message):
    if not value:raise ValueError(message)


def digest(path):
    path=Path(path);require(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)),"canonical regular pinned input required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path,value):
    with path.open("x") as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False)


def read_pinned(path,sha):
    require(digest(path)==sha,"external prepared receipt differs")
    payload=Path(path).read_bytes();require(hashlib.sha256(payload).hexdigest()==sha,"receipt changed while read")
    def unique(pairs):
        result={}
        for key,value in pairs:require(key not in result,"duplicate JSON key");result[key]=value
        return result
    def invalid(value):raise ValueError("non-finite JSON: "+value)
    return json.loads(payload,object_pairs_hook=unique,parse_constant=invalid)


def expected_command(authority):
    # HH_261002 - A command string in a receipt is not executable authority unless its complete argv matches this fixed private protocol.
    warm=REMOTE/"runs/training/temporal_route_recovery_v3_20261001_v1/exports/step_003000"
    backbone=REMOTE/"runs/training/temporal_v2_balanced_replay_20261001_v1/exports/step_003000"
    prepared=Path("/home/pro6000/personal/hwanhong/dataset/prepared")
    training=prepared/"town07-signed-recovery-train-20261001-v4";holdout=prepared/"town07-signed-recovery-holdout-20261001-v4"
    return ["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP",
        "PYTHONNOUSERSITE=1","OMP_NUM_THREADS=4","CUBLAS_WORKSPACE_CONFIG=:4096:8","CUDA_VISIBLE_DEVICES="+GPU,
        str(REMOTE/"venvs/py312/bin/python"),"-B",str(SOURCE/"tools/workspace_python.py"),"-m",
        "scripts.e2e.train_temporal_intrinsic_curve_pair_20261002","--","--mode","fit",
        "--parent-run",str(REMOTE/"runs/training/temporal_route_joint_20261002_v1"),"--parent-history-sha256",HISTORY_SHA,
        "--warm-parent-weights",str(warm/"model_state.npz"),"--warm-parent-metadata",str(warm/"metadata.json"),
        "--backbone-parent-weights",str(backbone/"model_state.npz"),"--backbone-parent-metadata",str(backbone/"metadata.json"),
        "--recovery-train",str(training),"--recovery-conversion",str(training/"moving_conversion_qualification.json"),
        "--recovery-holdout",str(holdout),"--holdout-conversion",str(holdout/"moving_conversion_qualification.json"),
        "--output",str(RUN),"--authorization",str(SOURCE/"authorization.json"),"--authorization-sha256",authority,"--loader-workers","2"]


REMOTE_PREFLIGHT=r'''
import base64,hashlib,json,pathlib,subprocess,sys
if sys.flags.optimize:raise RuntimeError("preflight assertions must not be disabled")
def gpu0_idle_read_only():
    row=subprocess.check_output(["nvidia-smi","-i","0","--query-gpu=uuid,memory.used,utilization.gpu","--format=csv,noheader,nounits"],text=True).strip().split(",")
    processes=subprocess.check_output(["nvidia-smi","-i","0","--query-compute-apps=gpu_uuid,pid","--format=csv,noheader,nounits"],text=True)
    assert len(row)==3 and row[0].strip()=="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    memory=int(row[1]);utilization=int(row[2])
    assert 0<=memory<=16 and utilization==0 and not processes.strip()
    return dict(memory_used_MiB=memory,utilization_percent=utilization,gpu0_compute_process_count=0,maximum_driver_residual_MiB=16)
source=pathlib.Path(sys.argv[1]);run=pathlib.Path(sys.argv[2]);pins=json.loads(base64.b64decode(sys.argv[3]))
assert source==pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/temporal_intrinsic_curve_pair_20261002_v1")
assert run==pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e/runs/training/temporal_intrinsic_curve_pair_20261002_v1")
assert not run.exists() and not any(p.is_symlink() for p in (source,run,*source.parents,*run.parents))
assert len(pins)==29
for name,sha in pins.items():
    relative=pathlib.PurePosixPath(name);assert not relative.is_absolute() and ".." not in relative.parts
    path=source/relative;assert path.is_file() and not any(p.is_symlink() for p in (path,*path.parents))
    assert hashlib.sha256(path.read_bytes()).hexdigest()==sha
idle=gpu0_idle_read_only()
print(json.dumps(dict(status="EXACT_SOURCE_AUTH_GPU0_IDLE_NO_CUDA_PREFLIGHT_PASS",physical_gpu=0,source_files=28,gpu0_idle=idle)))
'''


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("fit",),required=True)
    for role in ("prepared-declaration","prepared-summary","authority"):parser.add_argument("--expected-"+role+"-sha256",required=True)
    parser.add_argument("--output",type=Path,required=True);args=parser.parse_args(argv)
    require(all(len(value)==64 and all(c in "0123456789abcdef" for c in value) for value in
        (args.expected_prepared_declaration_sha256,args.expected_prepared_summary_sha256,args.expected_authority_sha256)),"exact lower-case SHA256 required")
    require(not args.output.exists() and not any(p.is_symlink() for p in (args.output,*args.output.parents))
        and args.output.resolve().is_relative_to(ROOT/"storage/experiments/training/2026-10-02")
        and not args.output.resolve().is_relative_to(PREPARED),"fresh local launch receipt outside frozen preparation required")
    declaration=read_pinned(PREPARED/"declaration.json",args.expected_prepared_declaration_sha256)
    summary=read_pinned(PREPARED/"summary.json",args.expected_prepared_summary_sha256)
    sources=declaration["source_sha256"]
    require(summary["status"]=="PRIVATE_REMOTE_TRAIN_ONLY_CPU_PREFLIGHT_PASS_NO_FIT" and summary["optimizer_started"] is False
        and digest(PREPARED/"remote_validation_stdout.json")==summary["remote_validation_sha256"]
        and declaration["authorization_sha256"]==args.expected_authority_sha256 and len(sources)==28
        and sources["tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"]==RUNNER_SHA
        and sources["learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"]==LOSS_SHA
        and declaration["dispatch_source_sha256"]==PREPARE_SHA and declaration["parent_actual6000_history_sha256"]==HISTORY_SHA
        and declaration["parent_four_actual_sha256"]==PARENT_PINS and declaration["physical_GPU_allowed"]==GPU
        and declaration["existing_private_venv"]==str(REMOTE/"venvs/py312/bin/python")
        and declaration["optimizer_started"] is False and declaration["packages_installed"] is False
        and declaration["remote_source"]==str(SOURCE) and declaration["remote_run"]==str(RUN),"exact completed CPU preparation required")
    require(all(not Path(name).is_absolute() and ".." not in Path(name).parts for name in sources),"relative canonical source keys required")
    require(all(digest(ROOT/name)==sha for name,sha in sources.items()),"frozen28 local source bytes changed")
    command=declaration["proposed_fit_command_requires_separate_root_execution"]
    require(shlex.split(command)==expected_command(args.expected_authority_sha256)
        and command==summary["proposed_fit_command_requires_separate_root_execution"],"no arbitrary command or GPU/parent substitution")
    own_sha=digest(Path(__file__))
    def recheck():
        require(digest(PREPARED/"declaration.json")==args.expected_prepared_declaration_sha256
            and digest(PREPARED/"summary.json")==args.expected_prepared_summary_sha256
            and digest(PREPARED/"remote_validation_stdout.json")==summary["remote_validation_sha256"]
            and digest(Path(__file__))==own_sha and all(digest(ROOT/name)==sha for name,sha in sources.items()),"prepared/frozen source changed")
    args.output.mkdir(parents=True,exist_ok=False)
    write_new(args.output/"declaration.json",dict(mode="explicit_root_fit_dispatch",source_sha256=own_sha,
        prepared_declaration_sha256=args.expected_prepared_declaration_sha256,prepared_summary_sha256=args.expected_prepared_summary_sha256,
        authority_sha256=args.expected_authority_sha256,command=command,source_files=sources,model_promoted=False))
    for role in ("source",):
        archive=args.output/role/Path(__file__).name;archive.parent.mkdir()
        with archive.open("xb") as stream:stream.write(Path(__file__).read_bytes())
    try:
        pins=dict(sources,**{"authorization.json":args.expected_authority_sha256})
        encoded=base64.b64encode(json.dumps(pins,sort_keys=True).encode()).decode()
        preflight=subprocess.run(SSH+[shlex.join(["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP",
            "PYTHONNOUSERSITE=1","python3","-B","-c",REMOTE_PREFLIGHT,str(SOURCE),str(RUN),encoded])],capture_output=True,timeout=90)
        for name,value in (("preflight_stdout.json",preflight.stdout),("preflight_stderr.txt",preflight.stderr)):
            with (args.output/name).open("xb") as stream:stream.write(value)
        require(preflight.returncode==0,"remote read-only source/GPU0 idle check failed; no fit launched")
        recheck()
        with (args.output/"actual_fit_stdout.jsonl").open("xb") as out,(args.output/"actual_fit_stderr.txt").open("xb") as err:
            process=subprocess.Popen(SSH+[command],stdout=out,stderr=err)
            started=dict(status="ACTUAL_OWNED_SSH_FIT_STARTED",started_UTC=datetime.now(timezone.utc).isoformat(),SSH_PID=process.pid,
                remote_run=str(RUN),fixed_updates_per_arm=6000,physical_GPU=0,optimizer_start_not_inferred_until_remote_started_receipt=True)
            write_new(args.output/"started.json",started);print(json.dumps(started),flush=True);code=process.wait()
        write_new(args.output/"completion.json",dict(status="SSH_COMMAND_EXITED_RESULTS_NOT_PROMOTED",exit_code=code,
            stdout_sha256=digest(args.output/"actual_fit_stdout.jsonl"),stderr_sha256=digest(args.output/"actual_fit_stderr.txt"),model_promoted=False))
        require(code==0,"actual fit command failed; original stdout/stderr/partial remote run retained")
        recheck()
        write_new(args.output/"source_postcheck.json",dict(prepared_and_frozen_sources_unchanged=True,model_promoted=False))
        return 0
    except Exception as error:
        write_new(args.output/"failure_receipt.json",dict(status="DISPATCH_FAILED_PARTIAL_RETAINED_NOT_PROMOTED",error=type(error).__name__+": "+str(error),
            foreign_processes_killed=False,packages_installed=False,model_promoted=False));raise


if __name__=="__main__":raise SystemExit(main())
