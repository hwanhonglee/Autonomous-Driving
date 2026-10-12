"""HH_261001 - Copy the reviewed minimal trainer into a fresh personal worktree and validate without CUDA."""

from __future__ import annotations

import base64
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

from scripts.e2e import train_temporal_v2_balanced_replay_20261001 as training


ROOT = Path(__file__).resolve().parents[2]
RECEIPT = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_dispatch_v1"
REMOTE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = REMOTE / "source_worktrees/temporal_v2_balanced_replay_20261001_v1"
RUN = REMOTE / "runs/training/temporal_v2_balanced_replay_20261001_v1"
PARENT = REMOTE / "runs/diagnostics/temporal_v2_raw_bootstrap_20261001_v1/raw_geometry_speed_checkpoint.pt"
PYTHON = REMOTE / "venvs/py312/bin/python"
TRAINER_SHA = "a1e3ad4ed9302b02701c7e3aa56de88afacf6681b275c1bc6986f79ebfd79103"
SSH = ["ssh", "-T", "-p", "6000", "-i", "/home/a/.ssh/pro6000_ed25519_v2",
       "-oBatchMode=yes", "-oConnectTimeout=15", "pro6000@220.90.5.14"]

# HH_261001 - The transport writes only new regular files beneath the exact personal worktree, never installs packages.
REMOTE_COPY = r'''
import base64, hashlib, json, os, pathlib, sys, tarfile
destination = pathlib.Path(sys.argv[1])
expected = json.loads(base64.b64decode(sys.argv[2]))
workspace = pathlib.Path("/home/pro6000/personal/hwanhong/portable_e2e").resolve(strict=True)
assert destination == workspace / "source_worktrees/temporal_v2_balanced_replay_20261001_v1"
assert destination.parent.is_dir() and not destination.parent.is_symlink()
assert not destination.exists() and not destination.is_symlink()
assert len(expected) == 17
os.mkdir(destination)
seen = set()
with tarfile.open(fileobj=sys.stdin.buffer, mode="r|") as archive:
    for member in archive:
        name = pathlib.PurePosixPath(member.name)
        assert member.isfile() and not name.is_absolute() and ".." not in name.parts
        assert str(name) in expected and str(name) not in seen and member.size <= 2*1024**2
        data = archive.extractfile(member).read()
        assert hashlib.sha256(data).hexdigest() == expected[str(name)]
        path = destination / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream: stream.write(data)
        seen.add(str(name))
assert seen == set(expected)
assert all(hashlib.sha256((destination/name).read_bytes()).hexdigest()==sha for name,sha in expected.items())
receipt = dict(status="EXACT_MINIMAL_PERSONAL_SOURCE_COPIED", source=str(destination), source_sha256=expected,
    source_file_count=len(seen), packages_installed=False, existing_files_overwritten=False)
with (destination / "transfer_receipt.json").open("x") as stream:
    json.dump(receipt, stream, indent=2, sort_keys=True)
print(json.dumps(receipt, sort_keys=True))
'''


def command(*, validate):
    # HH_261001 - CPU validation and actual training use the same argv; only the explicit CUDA visibility differs.
    environment = ["env", "PYTHONNOUSERSITE=1", "OMP_NUM_THREADS=4", "CUBLAS_WORKSPACE_CONFIG=:4096:8",
                   "CUDA_VISIBLE_DEVICES=" + ("" if validate else training.GPU_UUID)]
    arguments = [str(PYTHON), "-B", str(SOURCE / "tools/workspace_python.py"), "-m",
                 "scripts.e2e.train_temporal_v2_balanced_replay_20261001", "--",
                 "--parent-checkpoint", str(PARENT), "--output", str(RUN), "--loader-workers", "2"]
    if validate:
        arguments.append("--validate-only")
    return shlex.join(environment + arguments)


def main():
    training.require(not RECEIPT.exists(), "create-only dispatch receipt required")
    sources = training.source_snapshot()
    training.require(len(sources) == 17 and sources["tools/e2e/train_temporal_v2_balanced_replay_20261001.py"] == TRAINER_SHA,
                     "reviewed frozen minimal source closure required")
    payload = io.BytesIO()
    with tarfile.open(fileobj=payload, mode="w") as archive:
        for name in sorted(sources):
            path = ROOT / name
            data = path.read_bytes()
            training.require(training.digest(path) == sources[name], "source changed while packaging")
            info = tarfile.TarInfo(name=name)
            info.size, info.mode = len(data), 0o600
            archive.addfile(info, io.BytesIO(data))
    encoded = base64.b64encode(json.dumps(sources, sort_keys=True).encode()).decode()
    RECEIPT.mkdir(parents=True, exist_ok=False)
    training.write_new(RECEIPT / "declaration.json", dict(comment="HH_261001 - Reviewed balanced replay source transfer; validation only.",
        source_sha256=sources, remote_source=str(SOURCE), remote_run=str(RUN), parent_checkpoint_sha256=training.PARENT_SHA,
        existing_personal_venv=str(PYTHON), physical_GPU_allowed=training.GPU_UUID,
        packages_installed=False, external_files_overwritten=False, optimizer_started=False,
        validation_command=command(validate=True), proposed_training_command=command(validate=False)))
    transfer = subprocess.run(SSH + [shlex.join(["python3", "-c", REMOTE_COPY, str(SOURCE), encoded])],
                              input=payload.getvalue(), capture_output=True, timeout=90)
    with (RECEIPT / "transfer_stdout.json").open("xb") as stream:
        stream.write(transfer.stdout)
    with (RECEIPT / "transfer_stderr.txt").open("xb") as stream:
        stream.write(transfer.stderr)
    training.require(transfer.returncode == 0, "remote create-only source transfer failed; retain original outputs")
    copied = json.loads(transfer.stdout)
    training.require(copied["source_sha256"] == sources, "remote copied source identity differs")
    validation = subprocess.run(SSH + [command(validate=True)], capture_output=True, timeout=600)
    with (RECEIPT / "remote_validation_stdout.json").open("xb") as stream:
        stream.write(validation.stdout)
    with (RECEIPT / "remote_validation_stderr.txt").open("xb") as stream:
        stream.write(validation.stderr)
    training.require(validation.returncode == 0, "remote CPU validation failed; no GPU launch authorized by this receipt")
    checked = json.loads(validation.stdout)
    training.require(checked["status"] == "VALIDATED_NO_GPU_OR_OUTPUT" and checked["source_sha256"] == sources
                     and checked["requested_optimizer_updates"] == 3000
                     and checked["training_episode_count"] == 9 and checked["vehicle_control_approved"] is False,
                     "remote declared cohort/closure/scope differs")
    training.write_new(RECEIPT / "summary.json", dict(status="REMOTE_CPU_PREFLIGHT_PASSED_NO_TRAINING_STARTED",
        source_file_count=len(sources), remote_source=str(SOURCE), remote_run=str(RUN),
        remote_validation_sha256=training.digest(RECEIPT / "remote_validation_stdout.json"),
        parent_checkpoint_sha256=training.PARENT_SHA, optimizer_started=False,
        proposed_training_command=command(validate=False), packages_installed=False))
    print(json.dumps(dict(status="REMOTE_CPU_PREFLIGHT_PASSED_NO_TRAINING_STARTED", output=str(RECEIPT),
                          proposed_training_command=command(validate=False)), indent=2))


if __name__ == "__main__":
    main()
