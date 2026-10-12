"""HH_260906 - Launch one source-bound Town03 map generalization pilot on personal GPU0."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


# HH_260906 - Production paths are fixed; tests may replace WORKSPACE without exposing a CLI redirect.
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
GPU0_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
PACKAGE_SHA256 = "e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d"
TRAIN_SHA256 = "1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440"
DEV_SHA256 = "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"
PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
SOURCE_SHA256 = {
    "train_map_native_ah.py": "86e583aacf654d78ce913a95ba537e0025317aa5a737aefacc2162aa7e2adceb",
    "map_expansion_contract.py": "6dd294564231ceb6fdee8351be5ad27efdf954f06da5b64a8754b9d8dd2fa190",
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}


def digest(path: Path) -> str:
    """HH_260906 - Hash regular source/data bytes without accepting symlink substitutions."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def require_hash(path: Path, expected: str) -> None:
    if digest(path) != expected:
        raise ValueError(f"SHA256 mismatch: {path}")


def paths() -> dict[str, Path]:
    """HH_260906 - Derive only the declared personal source, dataset, parent and fresh run paths."""
    runs = WORKSPACE / "runs/diagnostics"
    return {
        "source": WORKSPACE / "source_worktrees/map_generalization_20260927_v1",
        "frozen": WORKSPACE / "source_worktrees/moving_recovery_k_20260922_v1",
        "data": WORKSPACE.parent / "dataset/map_generalization_20260927_v1",
        "parent": runs / "map_expanded_20260926_ai_v2",
        "output": runs / "map_generalization_20260927_v1",
        "log": runs / "mapping_generalization_20260927_v1.launch.log",
        "receipt": runs / "mapping_generalization_20260927_v1.exit.json",
    }


def prepare_environment() -> dict[str, str]:
    """HH_260906 - Bind one physical UUID and prevent inherited Python/package paths."""
    env = dict(os.environ)
    for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE"):
        env.pop(name, None)
    env.update(CUDA_VISIBLE_DEVICES=GPU0_UUID, CUBLAS_WORKSPACE_CONFIG=":4096:8",
               PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
               OMP_NUM_THREADS="16", MKL_NUM_THREADS="16")
    return env


def preflight(spec: dict[str, Path]) -> dict:
    """HH_260906 - Fail before log creation if the fixed pilot's personal inputs differ."""
    if Path(sys.prefix).absolute() != WORKSPACE / "venvs/py312":
        raise ValueError("personal py312 venv required")
    if WORKSPACE.is_symlink() or not WORKSPACE.is_dir():
        raise ValueError("personal workspace must be a directory, not a symlink")
    for name in ("source", "frozen", "data", "parent"):
        item = spec[name]
        if item.is_symlink() or not item.is_dir():
            raise ValueError(f"personal {name} directory is absent or symlinked")
    if (spec["output"].exists() or spec["output"].is_symlink()
            or spec["log"].exists() or spec["log"].is_symlink()
            or spec["receipt"].exists() or spec["receipt"].is_symlink()):
        raise ValueError("fresh output, log and exit receipt required")
    if not spec["output"].parent.is_dir() or spec["output"].parent.is_symlink():
        raise ValueError("personal diagnostics parent required")
    source_hashes = {}
    for filename, expected in SOURCE_SHA256.items():
        item = spec["source"] / filename
        require_hash(item, expected)
        source_hashes[filename] = expected
    require_hash(spec["data"] / "manifest.json", PACKAGE_SHA256)
    require_hash(spec["data"] / "train.npz", TRAIN_SHA256)
    require_hash(spec["data"] / "dev.npz", DEV_SHA256)
    require_hash(spec["parent"] / "data_manifest.json", PARENT_MANIFEST_SHA256)
    require_hash(spec["parent"] / "checkpoint.pt", PARENT_CHECKPOINT_SHA256)
    package = json.loads((spec["data"] / "manifest.json").read_text())
    parent = json.loads((spec["parent"] / "summary.json").read_text())
    started = json.loads((spec["parent"] / "started.json").read_text())
    if (package.get("parent_manifest_sha256") != PARENT_MANIFEST_SHA256
            or package.get("splits", {}).get("train", {}).get("count") != 913
            or package.get("splits", {}).get("dev", {}).get("count") != 118
            or package["splits"]["train"].get("sha256") != TRAIN_SHA256
            or package["splits"]["dev"].get("sha256") != DEV_SHA256
            or parent.get("checkpoint_sha256") != PARENT_CHECKPOINT_SHA256
            or started.get("data_manifest_sha256") != PARENT_MANIFEST_SHA256):
        raise ValueError("Town03 package or parent metadata differs from the frozen pilot")
    return {"package_sha256": PACKAGE_SHA256, "train_sha256": TRAIN_SHA256,
            "dev_sha256": DEV_SHA256, "parent_manifest_sha256": PARENT_MANIFEST_SHA256,
            "parent_checkpoint_sha256": PARENT_CHECKPOINT_SHA256,
            "source_sha256": source_hashes, "runner_sha256": digest(Path(__file__))}


def check_gpu0_idle() -> dict:
    """HH_260906 - Read GPU state only; never clear another user's process or fall back to GPU1."""
    rows = subprocess.check_output([
        "nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu",
        "--format=csv,noheader,nounits"], timeout=20, text=True).splitlines()
    selected = [[value.strip() for value in row.split(",")] for row in rows if GPU0_UUID in row]
    if len(selected) != 1 or len(selected[0]) != 4 or selected[0][0] != "0":
        raise ValueError("approved UUID is not exactly physical GPU0")
    _, uuid, memory, utilization = selected[0]
    if uuid != GPU0_UUID or int(memory) != 0 or int(utilization) != 0:
        raise ValueError("GPU0 is occupied; no training or retry")
    return {"physical_index": 0, "uuid": uuid, "memory_used_mib": 0,
            "utilization_percent": 0}


def command(spec: dict[str, Path]) -> list[str]:
    """HH_260906 - Reuse the frozen native trainer for exactly one 2000-step expanded-TRAIN arm."""
    return [sys.executable, str(spec["source"] / "train_map_native_ah.py"),
            "--frozen-repo", str(spec["frozen"]), "--data", str(spec["data"]),
            "--parent", str(spec["parent"]), "--output", str(spec["output"]),
            "--expanded-train"]


def run() -> int:
    """HH_260906 - CPU validate once, optimize once, and record a terminal receipt without retry."""
    spec = paths()
    identity = preflight(spec)
    env = prepare_environment()
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
                                        env=env, stdout=log, stderr=subprocess.STDOUT,
                                        check=False)
            returncode = validation.returncode
            log.flush()
            if returncode != 0:
                raise ValueError("CPU validation failed")
            if "NATIVE_MAPPING_CPU_PREFLIGHT_PASS" not in spec["log"].read_text():
                raise ValueError("CPU validation success marker absent")
            phase = "gpu0_preflight"
            gpu = check_gpu0_idle()
            phase = "train_once"
            log.write("TRAIN_ONCE_2000_STEPS\n")
            log.flush()
            training = subprocess.run(argv, cwd=spec["frozen"], env=env,
                                      stdout=log, stderr=subprocess.STDOUT, check=False)
            returncode = training.returncode
            if returncode != 0:
                error = "trainer exited nonzero"
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            returncode = returncode if phase == "cpu_validate" and returncode != 0 else 1
            log.write(f"RUNNER_ABORT {error}\n")
            log.flush()
    # HH_260906 - The receipt is written after every logged outcome, including failed CPU/GPU preflight.
    with spec["receipt"].open("x") as stream:
        json.dump({"schema": "portable_e2e.map_generalization_runner_exit.v1",
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
