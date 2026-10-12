"""HH_260906 - Retry only the CPU environment contract with a fresh one-shot C run."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


# HH_260906 - Tests can replace WORKSPACE in memory; the production CLI accepts no redirect.
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
GPU0_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
PACKAGE_SHA256 = "e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d"
TRAIN_SHA256 = "1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440"
DEV_SHA256 = "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"
PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
B_SOURCE_SHA256 = "86e583aacf654d78ce913a95ba537e0025317aa5a737aefacc2162aa7e2adceb"
B_CHECKPOINT_SHA256 = "12c63095e1a6c401e7772ddf9441c727e3e96ee523623995f1368bf818826fff"
SOURCE_SHA256 = {
    "train_map_town_rehearsal_c.py": "e1808cb77df8bc7c28d5019eb46c2d40d7e756d2f45ce59d01b43d944ab04de2",
    "map_expansion_contract.py": "6dd294564231ceb6fdee8351be5ad27efdf954f06da5b64a8754b9d8dd2fa190",
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}


def digest(path: Path) -> str:
    """HH_260906 - Require a regular file and hash all bytes before execution."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def require_hash(path: Path, expected: str):
    if digest(path) != expected:
        raise ValueError(f"SHA256 mismatch: {path}")


def paths() -> dict[str, Path]:
    """HH_260906 - Bind fresh source/run/log/receipt under the personal workspace."""
    runs = WORKSPACE / "runs/diagnostics"
    return {
        "source": WORKSPACE / "source_worktrees/map_town_rehearsal_20260927_v2",
        "frozen": WORKSPACE / "source_worktrees/moving_recovery_k_20260922_v1",
        "data": WORKSPACE.parent / "dataset/map_generalization_20260927_v1",
        "parent": runs / "map_expanded_20260926_ai_v2",
        "reference_b": runs / "map_generalization_20260927_v1",
        "output": runs / "map_town_rehearsal_20260927_v2",
        "log": runs / "mapping_town_rehearsal_20260927_v2.launch.log",
        "receipt": runs / "mapping_town_rehearsal_20260927_v2.exit.json",
    }


def prepare_environment() -> dict[str, str]:
    """HH_260906 - Bind GPU0 UUID for deterministic CPU preflight without allocating CUDA."""
    env = dict(os.environ)
    for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE"):
        env.pop(name, None)
    env.update(CUDA_VISIBLE_DEVICES=GPU0_UUID,
               CUBLAS_WORKSPACE_CONFIG=":4096:8", PYTHONNOUSERSITE="1",
               PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="16", MKL_NUM_THREADS="16")
    return env


def preflight(spec: dict[str, Path]) -> dict:
    """HH_260906 - Refuse changed inputs or any existing C output before a log is opened."""
    if Path(sys.prefix).absolute() != WORKSPACE / "venvs/py312":
        raise ValueError("personal py312 venv required")
    if WORKSPACE.is_symlink() or not WORKSPACE.is_dir():
        raise ValueError("personal workspace must be a directory, not a symlink")
    for name in ("source", "frozen", "data", "parent", "reference_b"):
        item = spec[name]
        if item.is_symlink() or not item.is_dir():
            raise ValueError(f"personal {name} directory is absent or symlinked")
    for name in ("output", "log", "receipt"):
        if spec[name].exists() or spec[name].is_symlink():
            raise ValueError("fresh output, log and exit receipt required")
    if spec["output"].parent.is_symlink() or not spec["output"].parent.is_dir():
        raise ValueError("personal diagnostics parent required")
    source_hashes = {}
    for filename, expected in SOURCE_SHA256.items():
        require_hash(spec["source"] / filename, expected)
        source_hashes[filename] = expected
    for file, expected in ((spec["data"] / "manifest.json", PACKAGE_SHA256),
                           (spec["data"] / "train.npz", TRAIN_SHA256),
                           (spec["data"] / "dev.npz", DEV_SHA256),
                           (spec["parent"] / "data_manifest.json", PARENT_MANIFEST_SHA256),
                           (spec["parent"] / "checkpoint.pt", PARENT_CHECKPOINT_SHA256),
                           (spec["reference_b"] / "data_manifest.json", PACKAGE_SHA256),
                           (spec["reference_b"] / "checkpoint.pt", B_CHECKPOINT_SHA256),
                           (spec["reference_b"] / "source/train_map_native_ah.py", B_SOURCE_SHA256)):
        require_hash(file, expected)
    package = json.loads((spec["data"] / "manifest.json").read_text())
    parent = json.loads((spec["parent"] / "summary.json").read_text())
    parent_started = json.loads((spec["parent"] / "started.json").read_text())
    b_summary = json.loads((spec["reference_b"] / "summary.json").read_text())
    b_started = json.loads((spec["reference_b"] / "started.json").read_text())
    if (package.get("parent_manifest_sha256") != PARENT_MANIFEST_SHA256
            or package.get("splits", {}).get("train", {}).get("count") != 913
            or package.get("splits", {}).get("dev", {}).get("count") != 118
            or package["splits"]["train"].get("sha256") != TRAIN_SHA256
            or package["splits"]["dev"].get("sha256") != DEV_SHA256
            or parent.get("checkpoint_sha256") != PARENT_CHECKPOINT_SHA256
            or parent_started.get("data_manifest_sha256") != PARENT_MANIFEST_SHA256
            or b_summary.get("checkpoint_sha256") != B_CHECKPOINT_SHA256
            or b_started.get("data_manifest_sha256") != PACKAGE_SHA256
            or b_started.get("source_sha256") != B_SOURCE_SHA256):
        raise ValueError("Town03 package, parent or B reference metadata changed")
    return {"package_sha256": PACKAGE_SHA256, "train_sha256": TRAIN_SHA256,
            "dev_sha256": DEV_SHA256, "parent_manifest_sha256": PARENT_MANIFEST_SHA256,
            "parent_checkpoint_sha256": PARENT_CHECKPOINT_SHA256,
            "b_source_sha256": B_SOURCE_SHA256, "b_checkpoint_sha256": B_CHECKPOINT_SHA256,
            "source_sha256": source_hashes, "runner_sha256": digest(Path(__file__))}


def check_gpu0_idle() -> dict:
    """HH_260906 - Read-only UUID/idle gate; no GPU1 fallback or process termination."""
    rows = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu",
         "--format=csv,noheader,nounits"], timeout=20, text=True).splitlines()
    matches = [[value.strip() for value in row.split(",")] for row in rows if GPU0_UUID in row]
    if len(matches) != 1 or len(matches[0]) != 4 or matches[0][0] != "0":
        raise ValueError("approved UUID is not exactly physical GPU0")
    _, uuid, memory, utilization = matches[0]
    if uuid != GPU0_UUID or int(memory) != 0 or int(utilization) != 0:
        raise ValueError("GPU0 is occupied; no training or retry")
    return {"physical_index": 0, "uuid": uuid, "memory_used_mib": 0,
            "utilization_percent": 0}


def command(spec: dict[str, Path]) -> list[str]:
    """HH_260906 - Exactly one fresh 2000-step C arm; never resume B weights."""
    return [sys.executable, str(spec["source"] / "train_map_town_rehearsal_c.py"),
            "--frozen-repo", str(spec["frozen"]), "--data", str(spec["data"]),
            "--parent", str(spec["parent"]), "--reference-b", str(spec["reference_b"]),
            "--output", str(spec["output"])]


def run() -> int:
    """HH_260906 - CPU validate once, check idle GPU0 once, train once, write terminal receipt."""
    spec = paths()
    identity = preflight(spec)
    argv = command(spec)
    phase = "cpu_validate"
    returncode = 1
    gpu = None
    error = None
    with spec["log"].open("x") as log:
        try:
            log.write("CPU_VALIDATE_ONLY\n")
            log.flush()
            validation = subprocess.run(argv + ["--validate-only"], cwd=spec["frozen"],
                                        env=prepare_environment(), stdout=log,
                                        stderr=subprocess.STDOUT, check=False)
            returncode = validation.returncode
            log.flush()
            if returncode != 0:
                raise ValueError("CPU validation failed")
            if "TOWN_REHEARSAL_CPU_PREFLIGHT_PASS" not in spec["log"].read_text():
                raise ValueError("CPU validation success marker absent")
            phase = "gpu0_preflight"
            gpu = check_gpu0_idle()
            phase = "train_once"
            log.write("TRAIN_ONCE_2000_STEPS\n")
            log.flush()
            training = subprocess.run(argv, cwd=spec["frozen"],
                                      env=prepare_environment(), stdout=log,
                                      stderr=subprocess.STDOUT, check=False)
            returncode = training.returncode
            if returncode != 0:
                error = "trainer exited nonzero"
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            returncode = returncode if phase == "cpu_validate" and returncode != 0 else 1
            log.write(f"RUNNER_ABORT {error}\n")
            log.flush()
    with spec["receipt"].open("x") as stream:
        json.dump({"schema": "portable_e2e.map_town_rehearsal_runner_exit.v2",
                   "completed_at_utc": datetime.now(timezone.utc).isoformat(),
                   "phase": phase, "returncode": returncode, "error": error,
                   "command": argv, "validate_only_command": argv + ["--validate-only"],
                   "gpu0_preflight": gpu, "input_identity": identity,
                   "log": str(spec["log"]), "output": str(spec["output"]),
                   "validation_run_count": 1,
                   "optimization_run_count": 1 if phase == "train_once" else 0,
                   "automatic_retry": False, "automatic_promotion": False,
                   "model_approved": False}, stream, indent=2)
    return returncode


if __name__ == "__main__":
    raise SystemExit(run())
