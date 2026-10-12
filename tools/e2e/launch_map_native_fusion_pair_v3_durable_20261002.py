"""HH_261002 - Launch exactly one authorized private V3 fit with detached, create-only regular-file logs.

This source-only helper never connects over SSH, installs, retries, resumes,
selects GPU1 or kills any process. A launch receipt is not evidence of updates.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

NAME = "map_native_fusion_pair_20261002_v3"
SCHEMA = "portable_e2e.map_native_fusion_detached_launch.20261002.v3"
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = WORKSPACE / "source_worktrees" / NAME
VENV = WORKSPACE / "venvs/py312"
PYTHON = VENV / "bin/python"
TRAINER = SOURCE / "train_map_native_fusion_pair_v3_20261002.py"
TRAIN_AUTHORITY = WORKSPACE / "plans/training" / NAME / "ROOT_fixed20k_structure_pair_authorization.json"
HELPER = WORKSPACE / "plans/launches" / NAME / Path(__file__).name
LAUNCH_AUTHORITY = HELPER.parent / "ROOT_detached_launch_authorization.json"
LOGS = WORKSPACE / "runs/launches" / NAME
OUTPUT = WORKSPACE / "runs/diagnostics" / NAME
LEASE = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
ENVIRONMENT = dict(CUDA_VISIBLE_DEVICES=GPU_UUID, CUBLAS_WORKSPACE_CONFIG=":4096:8",
    PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="16", MKL_NUM_THREADS="16")


def require(value, message):
    if not value: raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(v in "0123456789abcdef" for v in value)


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path.resolve() == path
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical unaliased path required")
    require(path.is_relative_to(WORKSPACE), "path escapes private portable workspace")
    return path


def digest(path):
    canonical(path); require(path.is_file(), "regular pinned file required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_pinned(path, expected):
    require(sha(expected) and digest(path) == expected, "external bytes differ")
    return json.loads(path.read_bytes())


def contract():
    return dict(schema=SCHEMA, study_id=NAME, source=str(SOURCE), trainer=str(TRAINER),
        trainer_authority=str(TRAIN_AUTHORITY), helper=str(HELPER), launch_authority=str(LAUNCH_AUTHORITY),
        logs=str(LOGS), output=str(OUTPUT), existing_private_venv=str(VENV), physical_GPU_index=0, GPU_UUID=GPU_UUID,
        environment=dict(ENVIRONMENT), loader_workers=2, attempts=1,
        both_fixed20000_arms_fresh_parent_and_optimizer=True, resume=False, automatic_retry=False,
        detached_session=True, stdin="DEVNULL", stdout="create_only_regular_file", stderr="create_only_regular_file",
        original_GPU0_training_lease_required=True, trainer_rechecks_lease_and_GPU=True,
        main_capability_approved=False, model_promoted=False, vehicle_control_approved=False)


def write_new(path, value):
    canonical(path)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")


def process_generation(pid):
    # HH_261002 - Record kernel start ticks rather than treating a reused PID as the owned child.
    text = Path(f"/proc/{pid}/stat").read_text()
    return int(text[text.rfind(")") + 2:].split()[19])


def source_and_authority(value):
    # HH_261002 - Read source/contract/provider descriptors only; the trainer itself authenticates actual TRAIN and CUDA.
    pins = value.get("source_sha256")
    require(type(pins) is dict and len(pins) == 15 and TRAINER.name in pins
        and all(type(name) is str and Path(name).name == name and sha(pin) for name, pin in pins.items()),
        "fifteen external flat source pins required before imports")
    require(all(digest(SOURCE / name) == pin for name, pin in pins.items()),
        "source bytes changed before import")
    sys.path.insert(0, str(SOURCE))
    import train_map_native_fusion_pair_v3_20261002 as revision
    require(Path(revision.__file__) == TRAINER and revision.SOURCE == SOURCE and revision.OUTPUT == OUTPUT,
        "trainer imports escaped fresh identity")
    revision.verify_frozen()
    source = revision.source_snapshot(SOURCE)
    authority = read_pinned(TRAIN_AUTHORITY, value["trainer_authorization_sha256"])
    proof = revision.protocol.schedule_proof(revision.protocol.schedule())
    revision.protocol.validate_authorization(authority, source, proof, 2)
    require(value.get("source_sha256") == source and len(source) == 15,
        "external full fifteen-source launch pins differ")
    require(revision.protocol.screens.runtime_identity() == authority["runtime_identity"],
        "current private binary providers differ")
    return source


def lease_and_GPU_idle():
    canonical(LEASE); require(LEASE.is_file(), "existing original GPU0 lease file required")
    descriptor = os.open(LEASE, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-gpu=index,uuid,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip().splitlines()
        apps = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-compute-apps=pid,gpu_uuid,used_memory",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip()
        fields = [v.strip() for v in rows[0].split(",")] if len(rows) == 1 else []
        require(len(fields) == 4 and fields[:2] == ["0", GPU_UUID] and fields[2].isdecimal()
            and 0 <= int(fields[2]) <= 16 and fields[3] == "0" and not apps,
            "GPU0 must have exact UUID, no compute processes/utilization, and only driver residue")
        return dict(physical_index=0, uuid=GPU_UUID, memory_used_MiB=int(fields[2]), utilization=0, compute_processes=0)
    finally:
        os.close(descriptor)


def environment_check():
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == VENV and Path.cwd() == SOURCE
        and sys.flags.optimize == 0 and Path(__file__).absolute() == HELPER,
        "exact remote host, existing private venv, source cwd, and pinned private helper required")
    for path in (WORKSPACE, SOURCE, VENV, HELPER, TRAINER, TRAIN_AUTHORITY, LAUNCH_AUTHORITY, LOGS.parent, OUTPUT.parent):
        canonical(path); require(path.exists(), "required private path missing")
    canonical(LOGS); canonical(OUTPUT)
    require(not LOGS.exists() and not OUTPUT.exists(), "fresh logs and trainer output only; no retry/resume/overwrite")
    require(all(os.environ.get(k) == v for k, v in ENVIRONMENT.items())
        and not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")),
        "strict approved inherited environment required")


def launch(args):
    environment_check()
    require(sha(args.helper_sha256) and digest(HELPER) == args.helper_sha256, "ROOT-pinned launcher source changed")
    value = read_pinned(LAUNCH_AUTHORITY, args.authorization_sha256)
    require(type(value) is dict and value.get("schema") == SCHEMA + ".authorization"
        and value.get("contract") == contract() and value.get("helper_sha256") == args.helper_sha256
        and sha(value.get("trainer_authorization_sha256")) and value.get("detached_launch_authorized") is True
        and type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3
        and value.get("automatic_model_promotion") is False and value.get("vehicle_control_approved") is False,
        "exact external ROOT launch scope required")
    source = source_and_authority(value)
    gpu = lease_and_GPU_idle()
    require(digest(HELPER) == args.helper_sha256 and digest(LAUNCH_AUTHORITY) == args.authorization_sha256
        and digest(TRAIN_AUTHORITY) == value["trainer_authorization_sha256"], "launch authority changed before fork")
    command = [str(PYTHON), "-B", "-u", str(TRAINER), "--mode", "fit", "--loader-workers", "2",
        "--authorization", str(TRAIN_AUTHORITY), "--authorization-sha256", value["trainer_authorization_sha256"]]
    if args.validate_only:
        return dict(status="DETACHED_LAUNCH_PREFLIGHT_ONLY_NO_PROCESS_NO_TRAIN_NO_OUTPUT", contract=contract(),
            source_sha256=source, GPU_idle=gpu, command=command)
    LOGS.mkdir(mode=0o700)
    write_new(LOGS / "launch_plan.json", dict(contract=contract(), source_sha256=source,
        helper_sha256=args.helper_sha256, launch_authorization_sha256=args.authorization_sha256,
        trainer_authorization_sha256=value["trainer_authorization_sha256"], command=command, GPU_idle=gpu,
        created_at_utc=datetime.now(timezone.utc).isoformat(), actual_optimizer_updates_claimed=0))
    # HH_261002 - Real local regular files outlive SSH stdout; never pass PIPE, terminal or inherited stdin to the fit.
    with (LOGS / "stdout.log").open("xb", buffering=0) as stdout, (LOGS / "stderr.log").open("xb", buffering=0) as stderr:
        require(stat.S_ISREG(os.fstat(stdout.fileno()).st_mode) and stat.S_ISREG(os.fstat(stderr.fileno()).st_mode),
            "durable output descriptors must be regular files")
        process = subprocess.Popen(command, cwd=SOURCE, env=dict(os.environ), stdin=subprocess.DEVNULL,
            stdout=stdout, stderr=stderr, start_new_session=True, close_fds=True)
    # HH_261002 - Even an immediate child exit retains launch/log evidence; missing generation never proves a running owned fit.
    generation, generation_error = None, None
    try:
        generation = dict(process_start_ticks=process_generation(process.pid),
            boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip())
    except (OSError, ValueError) as error:
        generation_error = dict(error_type=type(error).__name__, error=str(error))
    value = dict(status="DETACHED_PROCESS_STARTED_NOT_YET_TRAINING_PROOF", pid=process.pid,
        process_start_ticks=None if generation is None else generation["process_start_ticks"],
        boot_id=None if generation is None else generation["boot_id"],
        owned_process_generation_confirmed=generation is not None, generation_read_error=generation_error,
        created_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=source, command=command, stdin="DEVNULL", stdout=str(LOGS / "stdout.log"),
        stderr=str(LOGS / "stderr.log"), session_detached=True, actual_optimizer_updates_claimed=0)
    write_new(LOGS / "launch_receipt.json", value)
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "launch"), default="contract")
    parser.add_argument("--authorization-sha256"); parser.add_argument("--helper-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    if args.mode == "contract":
        require(args.authorization_sha256 is None and args.helper_sha256 is None and not args.validate_only,
            "query cannot accept execution authority")
        result = contract()
    else: result = launch(args)
    print(json.dumps(result, indent=2, allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
