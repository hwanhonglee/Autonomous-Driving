"""HH_261004 - Run one CPU admission check and one detached GPU0 TRAIN-only pilot.

Only existing private venv/source/assets are used. No install, SSH, retry,
simulator control, foreign process termination, or automatic promotion.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

W = Path("/home/pro6000/personal/hwanhong/portable_e2e")
PERSONAL = W.parent
NAME = "temporal_retention_pair_pilot_20261004_v2"
SOURCE = W / "source_worktrees" / NAME
PLAN = W / "plans/training" / NAME
AUTH = PLAN / "ROOT_TRAIN_only_authorization.json"
CPU = PLAN / "actual_TRAIN_CPU_preflight.json"
OUTPUT = W / "runs/training" / NAME
LOGS = W / "runs/launches" / NAME
PYTHON = W / "venvs/py312/bin/python"
GPU = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
ENV = dict(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="4", MKL_NUM_THREADS="4",
    OPENBLAS_NUM_THREADS="1", CUBLAS_WORKSPACE_CONFIG=":4096:8")


def require(value, message):
    if not value: raise ValueError(message)


def canonical(path):
    require(path.is_absolute() and path.is_relative_to(PERSONAL) and path.resolve() == path
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical private path only")
    return path


def digest(path):
    canonical(path); require(path.is_file(), "regular private file required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON denied"); result[key] = value
    return result


def pinned(path, expected):
    canonical(path); require(path.is_file(), "regular pinned JSON required")
    data = path.read_bytes()
    require(len(expected) == 64 and hashlib.sha256(data).hexdigest() == expected, "external SHA mismatch")
    return json.loads(data, object_pairs_hook=unique,
        parse_constant=lambda _: require(False, "nonfinite JSON denied"))


def write_new(path, value):
    canonical(path)
    with path.open("x") as stream: json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)


def command(authority, mode, authority_sha):
    require(set(authority["asset_arguments"]) == {"parent-run", "warm-parent-weights", "warm-parent-metadata",
        "backbone-parent-weights", "backbone-parent-metadata", "recovery-train", "recovery-conversion", "left-train", "right-train"},
        "exact TRAIN-only asset arguments required")
    result = [str(PYTHON), "-B", "-u", str(SOURCE / "tools/workspace_python.py"), "-m",
        "scripts.e2e.train_temporal_retention_pair_pilot_v2_20261004", "--", "--mode", mode]
    for key, value in authority["asset_arguments"].items():
        canonical(Path(value)); require(Path(value).exists(), "original private TRAIN asset absent")
        result += ["--" + key, value]
    return result + ["--left-conversion-sha256", authority["left_conversion_sha256"], "--output", str(OUTPUT),
        "--pilot-authority", str(AUTH), "--pilot-authority-sha256", authority_sha]


def verify_sources(authority):
    sources = authority["contract"]["sources"]
    require(len(sources) == 34 and authority["contract"]["study_id"] == NAME, "fresh V2 source closure required")
    for name, expected in sources.items():
        require(not Path(name).is_absolute() and ".." not in Path(name).parts, "relative source path required")
        require(digest(SOURCE / name) == expected, "source changed")


def idle_gpu0():
    # HH_261004 - Recheck the original ownership lease and physical GPU0 immediately before launch.
    lease = canonical(W / "runs/campaigns/.gpu0_training.lock")
    descriptor = os.open(lease, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        row = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-gpu=index,uuid,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip().split(",")
        apps = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-compute-apps=pid,gpu_uuid,used_memory",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip()
        row = [v.strip() for v in row]
        require(len(row) == 4 and row[:2] == ["0", GPU] and row[2].isdecimal()
            and int(row[2]) <= 16 and row[3] == "0" and not apps, "physical GPU0 not idle; do not interfere")
        return dict(physical_GPU=0, UUID=GPU, memory_MiB=int(row[2]), compute_processes=0)
    finally: os.close(descriptor)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("preflight", "launch"), required=True)
    parser.add_argument("--authority-sha256", required=True)
    parser.add_argument("--cpu-sha256")
    args = parser.parse_args(argv)
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == W / "venvs/py312"
        and Path.cwd() == SOURCE and sys.flags.optimize == 0, "existing remote private venv and source cwd only")
    canonical(OUTPUT); canonical(LOGS)
    require(not OUTPUT.exists() and not LOGS.exists(), "one fresh attempt only")
    authority = pinned(AUTH, args.authority_sha256); verify_sources(authority)
    require(authority.get("authorized") is True and type(authority.get("physical_GPU_index")) is int
        and authority["physical_GPU_index"] == 0 and authority.get("model_promoted") is False
        and authority.get("vehicle_control_approved") is False, "explicit research-only GPU0 authority required")
    helper_sha = digest(Path(__file__).absolute())
    require(authority.get("launcher_sha256") == helper_sha, "externally sealed launcher bytes required")
    environment = dict(os.environ, **ENV, CUDA_VISIBLE_DEVICES="" if args.mode == "preflight" else GPU)
    if args.mode == "preflight":
        require(not CPU.exists(), "CPU preflight receipt create-only")
        completed = subprocess.run(command(authority, "validate-only", args.authority_sha256), env=environment,
            cwd=SOURCE, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=600)
        require(completed.returncode == 0, "CPU admission failed: " + completed.stderr[-6000:])
        value = json.loads(completed.stdout, object_pairs_hook=unique)
        require(value["status"] == "PASS_QUALIFIED20_TRAIN_CPU_ONLY", "actual CPU admission receipt required")
        verify_sources(authority); require(digest(AUTH) == args.authority_sha256, "authority changed during preflight")
        write_new(CPU, dict(value, helper_sha256=helper_sha, authority_sha256=args.authority_sha256,
            source_sha256=authority["contract"]["sources"], UTC=datetime.now(timezone.utc).isoformat(), CUDA_initialized=False))
        print(json.dumps(dict(status=value["status"], receipt=str(CPU), receipt_sha256=digest(CPU),
            sources=34, updates_authorized_per_arm=1000, actual_optimizer_updates=0))); return 0
    require(args.cpu_sha256 is not None, "externally reviewed CPU receipt SHA required")
    cpu = pinned(CPU, args.cpu_sha256)
    require(cpu["status"] == "PASS_QUALIFIED20_TRAIN_CPU_ONLY" and cpu["helper_sha256"] == helper_sha
        and cpu["authority_sha256"] == args.authority_sha256 and cpu["source_sha256"] == authority["contract"]["sources"]
        and cpu["declaration"]["schema"] == authority["contract"]["schema"], "current admitted CPU identity required")
    for name, expected in cpu["declaration"]["immutable_input_sha256"].items():
        require(digest(Path(name)) == expected, "qualified original asset changed after CPU admission")
    gpu = idle_gpu0(); verify_sources(authority)
    LOGS.mkdir(mode=0o700, exist_ok=False)
    with (LOGS / "stdout.log").open("x") as out, (LOGS / "stderr.log").open("x") as err:
        child = subprocess.Popen(command(authority, "fit", args.authority_sha256), cwd=SOURCE, env=environment,
            stdin=subprocess.DEVNULL, stdout=out, stderr=err, start_new_session=True, close_fds=True)
    raw = Path(f"/proc/{child.pid}/stat").read_text()
    receipt = dict(status="LAUNCHED_NOT_EVIDENCE_OF_TRAINING", PID=child.pid,
        process_start_ticks=int(raw[raw.rfind(")") + 2:].split()[19]),
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), UTC=datetime.now(timezone.utc).isoformat(),
        GPU=gpu, authority_sha256=args.authority_sha256, cpu_sha256=args.cpu_sha256, helper_sha256=helper_sha,
        model_promoted=False, vehicle_control_approved=False)
    write_new(LOGS / "launch_receipt.json", receipt); print(json.dumps(receipt)); return 0


if __name__ == "__main__": raise SystemExit(main())
