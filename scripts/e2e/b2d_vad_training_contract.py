#!/usr/bin/env python3
"""Reproducible bootstrap contract for public Bench2Drive VAD training."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlparse


OFFICIAL_REPOSITORY = "https://github.com/Thinklab-SJTU/Bench2DriveZoo.git"
OFFICIAL_REPOSITORY_SLUG = "Thinklab-SJTU/Bench2DriveZoo"
DEFAULT_REPOSITORY_REF = "uniad/vad"
MANIFEST_NAME = "b2d_vad_training_manifest.json"

PRIVATE_TINY_CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
PUBLIC_B2D_CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)
PUBLIC_FROM_PRIVATE_INDICES = tuple(
    PRIVATE_TINY_CAMERA_ORDER.index(name) for name in PUBLIC_B2D_CAMERA_ORDER
)

DEPTH_CONTRACTS = {
    "legacy": {
        "extension": ".png",
        "reader": "cv2.imread",
        "legacy_prepare_b2d_compatible": True,
        "requires_versioned_adapter": False,
    },
    "v0.0.4": {
        "extension": ".npz",
        "reader": "numpy.load",
        "legacy_prepare_b2d_compatible": False,
        "requires_versioned_adapter": True,
    },
}

STORAGE_TARGETS_GIB = {"smoke": 20}

DEFERRED_STORAGE_PLANS = {
    "base_with_working_copy": {
        "status": "deferred_external_storage",
        "future_capacity_estimate_gib": 1000,
        "active_capacity_check": False,
        "automatic_download": False,
    },
    "full_and_custom": {
        "status": "excluded_current_scope",
        "published_full_download_tb": 4,
        "future_working_capacity_estimate_gib": 8000,
        "active_capacity_check": False,
        "automatic_download": False,
    },
}

LICENSE_CONTRACT = {
    "github_source": "CC-BY-NC-ND-4.0",
    "huggingface_metadata": "Apache-2.0",
    "conflict": True,
    "policy": (
        "Treat the GitHub CC-BY-NC-ND-4.0 terms as controlling until the "
        "rights holder clarifies the conflicting Hugging Face metadata."
    ),
    "commercial_use": "blocked_pending_rights_holder_clearance",
    "adapted_checkpoint_distribution": "blocked_pending_rights_holder_clearance",
}


class ContractError(RuntimeError):
    """Raised when the training bootstrap contract cannot be satisfied."""


def _run(
    command: Sequence[str],
    cwd: Optional[Path] = None,
    env: Optional[Mapping[str, str]] = None,
    check: bool = False,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(command),
        cwd=str(cwd) if cwd else None,
        env=dict(env) if env else None,
        check=check,
        text=True,
        capture_output=True,
    )


def _version_tuple(value: str) -> Tuple[int, ...]:
    match = re.search(r"(\d+)(?:\.(\d+))?(?:\.(\d+))?", value)
    if not match:
        return ()
    return tuple(int(item) for item in match.groups(default="0"))


def validate_camera_adapter(
    source_order: Sequence[str] = PRIVATE_TINY_CAMERA_ORDER,
    target_order: Sequence[str] = PUBLIC_B2D_CAMERA_ORDER,
    indices: Sequence[int] = PUBLIC_FROM_PRIVATE_INDICES,
) -> Dict[str, Any]:
    if len(source_order) != 6 or len(target_order) != 6 or len(indices) != 6:
        raise ContractError("camera adapter must describe exactly six cameras")
    if len(set(source_order)) != 6 or len(set(target_order)) != 6:
        raise ContractError("camera orders must contain six unique camera names")
    if set(source_order) != set(target_order):
        raise ContractError("private and public camera orders must contain the same cameras")
    if sorted(indices) != list(range(6)):
        raise ContractError("camera adapter indices must be a permutation of 0..5")
    resolved = tuple(source_order[index] for index in indices)
    if resolved != tuple(target_order):
        raise ContractError(
            "camera adapter does not transform the private Tiny order into public B2D order"
        )
    return {
        "pass": True,
        "source_order": list(source_order),
        "target_order": list(target_order),
        "target_from_source_indices": list(indices),
    }


def validate_depth_files(version: str, paths: Iterable[Path]) -> Dict[str, Any]:
    if version not in DEPTH_CONTRACTS:
        raise ContractError(
            "depth version must be one of: " + ", ".join(sorted(DEPTH_CONTRACTS))
        )
    expected = DEPTH_CONTRACTS[version]["extension"]
    observed = sorted({Path(path).suffix.lower() for path in paths})
    if not observed:
        raise ContractError("at least one depth file is required to validate the format")
    unexpected = [suffix for suffix in observed if suffix != expected]
    if unexpected:
        raise ContractError(
            "depth files for {} must use {}; observed {}".format(
                version, expected, ", ".join(observed)
            )
        )
    return {
        "pass": True,
        "version": version,
        "observed_extensions": observed,
        **DEPTH_CONTRACTS[version],
    }


def detect_depth_layout(dataset_root: Path, sample_limit: int = 200) -> Dict[str, Any]:
    if not dataset_root.exists():
        return {
            "present": False,
            "layout": "unknown",
            "sample_count": 0,
            "extensions": [],
        }
    extensions = set()
    count = 0
    for path in dataset_root.rglob("*"):
        if not path.is_file() or "depth_" not in str(path.parent):
            continue
        suffix = path.suffix.lower()
        if suffix in (".png", ".npz"):
            extensions.add(suffix)
            count += 1
        if count >= sample_limit:
            break
    if extensions == {".png"}:
        layout = "legacy"
    elif extensions == {".npz"}:
        layout = "v0.0.4"
    elif extensions:
        layout = "mixed_invalid"
    else:
        layout = "unknown"
    return {
        "present": True,
        "layout": layout,
        "sample_count": count,
        "extensions": sorted(extensions),
    }


def training_recommendation(vram_gib: Optional[float]) -> Dict[str, Any]:
    if vram_gib is None:
        return {
            "profile": "cpu_or_unknown",
            "full_e2e_supported": False,
            "recommendation": "planning-head-only after a measured CUDA memory probe",
        }
    if vram_gib < 16:
        return {
            "profile": "planning-head-only",
            "full_e2e_supported": False,
            "recommendation": (
                "cache/freeze backbone and BEV features; do not promise full VAD backward "
                "on this GPU"
            ),
        }
    if vram_gib < 24:
        return {
            "profile": "planning-head-only-or-reduced-smoke",
            "full_e2e_supported": False,
            "recommendation": "measure one-batch peak memory before changing model geometry",
        }
    return {
        "profile": "full-e2e-memory-probe",
        "full_e2e_supported": None,
        "recommendation": (
            "the upstream project publishes no minimum VRAM; validate one full backward pass"
        ),
    }


def storage_report(path: Path) -> Dict[str, Any]:
    anchor = path
    while not anchor.exists() and anchor != anchor.parent:
        anchor = anchor.parent
    usage = shutil.disk_usage(str(anchor))
    free_gib = usage.free / (1024 ** 3)
    smoke_ready = free_gib >= STORAGE_TARGETS_GIB["smoke"]
    return {
        "path": str(path),
        "filesystem_anchor": str(anchor),
        "free_gib": round(free_gib, 2),
        "active_scope": {
            "profile": "smoke",
            "minimum_free_gib": STORAGE_TARGETS_GIB["smoke"],
            "ready": smoke_ready,
        },
        "smoke_ready": smoke_ready,
        "capacity_planning": {
            "status": "deferred_until_external_storage_root_is_selected",
            "external_storage_root": None,
            "profiles": DEFERRED_STORAGE_PLANS,
        },
    }


def gpu_report() -> Dict[str, Any]:
    executable = shutil.which("nvidia-smi")
    if not executable:
        return {"present": False, "devices": [], "maximum_vram_gib": None}
    result = _run(
        [
            executable,
            "--query-gpu=name,memory.total,driver_version",
            "--format=csv,noheader,nounits",
        ]
    )
    devices = []
    for line in result.stdout.splitlines() if result.returncode == 0 else []:
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 3:
            continue
        try:
            memory_mib = float(parts[1])
        except ValueError:
            continue
        devices.append(
            {
                "name": parts[0],
                "memory_mib": memory_mib,
                "memory_gib": round(memory_mib / 1024, 2),
                "driver_version": parts[2],
            }
        )
    maximum = max((device["memory_gib"] for device in devices), default=None)
    return {
        "present": bool(devices),
        "devices": devices,
        "maximum_vram_gib": maximum,
        "query_error": result.stderr.strip() if result.returncode else "",
    }


def cuda_report(prefix: Optional[Path] = None) -> Dict[str, Any]:
    prefix_nvcc = prefix / "bin/nvcc" if prefix else None
    executable = (
        str(prefix_nvcc)
        if prefix_nvcc is not None and prefix_nvcc.is_file()
        else shutil.which("nvcc")
    )
    if not executable:
        return {"nvcc_present": False, "version": None, "official_11_8": False}
    result = _run([executable, "--version"])
    match = re.search(r"release\s+([0-9.]+)", result.stdout + result.stderr)
    version = match.group(1) if match else None
    return {
        "nvcc_present": result.returncode == 0,
        "path": executable,
        "version": version,
        "official_11_8": bool(version and _version_tuple(version)[:2] == (11, 8)),
    }


def _probe_python(executable: Path) -> Dict[str, Any]:
    if not executable.is_file():
        return {"present": False, "path": str(executable)}
    code = (
        "import json,sys; out={'version':list(sys.version_info[:3])}; "
        "\ntry:\n import torch; out['torch']=torch.__version__; "
        "out['torch_cuda']=torch.version.cuda; out['cuda_available']=torch.cuda.is_available()"
        "\nexcept Exception as e: out['torch_error']=type(e).__name__+': '+str(e)"
        "\ntry:\n import mmcv; out['mmcv']=getattr(mmcv,'__version__','installed')"
        "\nexcept Exception as e: out['mmcv_error']=type(e).__name__+': '+str(e)"
        "\nprint(json.dumps(out))"
    )
    result = _run([str(executable), "-c", code])
    if result.returncode != 0:
        return {
            "present": True,
            "path": str(executable),
            "probe_error": result.stderr.strip() or result.stdout.strip(),
        }
    try:
        data = json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        data = {"probe_error": "python probe did not return JSON"}
    data.update({"present": True, "path": str(executable)})
    data["python_3_8"] = tuple(data.get("version", ()))[:2] == (3, 8)
    return data


def environment_report(prefix: Path) -> Dict[str, Any]:
    prefix_python = prefix / "bin/python"
    conda = shutil.which("conda")
    python38 = shutil.which("python3.8")
    probe = _probe_python(prefix_python)
    return {
        "prefix": str(prefix),
        "prefix_exists": prefix.exists(),
        "prefix_python": probe,
        "conda": conda,
        "system_python3_8": python38,
        "bootstrap_available": bool(conda or python38),
        "system_python_untouched": True,
    }


def is_official_remote(value: str) -> bool:
    remote = value.strip().rstrip("/")
    if remote.endswith(".git"):
        remote = remote[:-4]
    if remote.startswith("git@github.com:"):
        remote = "https://github.com/" + remote.split(":", 1)[1]
    parsed = urlparse(remote)
    return (
        parsed.scheme in ("http", "https")
        and parsed.hostname == "github.com"
        and parsed.path.strip("/").lower() == OFFICIAL_REPOSITORY_SLUG.lower()
    )


def repository_report(repo_dir: Path) -> Dict[str, Any]:
    if not (repo_dir / ".git").is_dir():
        return {"present": False, "path": str(repo_dir)}
    remote = _run(["git", "remote", "get-url", "origin"], cwd=repo_dir)
    commit = _run(["git", "rev-parse", "HEAD"], cwd=repo_dir)
    branch = _run(["git", "branch", "--show-current"], cwd=repo_dir)
    dirty = _run(["git", "status", "--porcelain"], cwd=repo_dir)
    remote_value = remote.stdout.strip()
    official = is_official_remote(remote_value)
    return {
        "present": True,
        "path": str(repo_dir),
        "origin": remote_value,
        "official_origin": official,
        "branch": branch.stdout.strip(),
        "commit": commit.stdout.strip(),
        "dirty": bool(dirty.stdout.strip()),
    }


def load_manifest(prefix: Path) -> Dict[str, Any]:
    path = prefix / MANIFEST_NAME
    if not path.is_file():
        return {"present": False, "path": str(path)}
    try:
        content = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"present": True, "path": str(path), "valid": False, "error": str(error)}
    errors = []
    document = content if isinstance(content, dict) else {}
    repository = document.get("repository", {})
    large_artifacts = document.get("large_artifacts", {})
    if document.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if not is_official_remote(str(repository.get("url", ""))):
        errors.append("repository.url must name the official Bench2DriveZoo repository")
    if not re.fullmatch(r"[0-9a-f]{40}", str(repository.get("commit", ""))):
        errors.append("repository.commit must be an exact 40-character lowercase commit")
    if not str(repository.get("ref", "")).strip():
        errors.append("repository.ref must record the installed branch or ref")
    if document.get("python_requirement") != "3.8":
        errors.append("python_requirement must be 3.8")
    if document.get("cuda_requirement") != "11.8":
        errors.append("cuda_requirement must be 11.8")
    if document.get("environment_prefix") != str(prefix.resolve()):
        errors.append("environment_prefix does not match the checked prefix")
    if large_artifacts != {"dataset_downloaded": False, "checkpoint_downloaded": False}:
        errors.append("manifest must not claim automatic dataset or checkpoint downloads")
    return {
        "present": True,
        "path": str(path),
        "valid": not errors,
        "errors": errors,
        "content": document,
    }


def build_check_report(prefix: Path, repo_dir: Path, dataset_root: Path) -> Dict[str, Any]:
    environment = environment_report(prefix)
    repository = repository_report(repo_dir)
    storage = storage_report(prefix)
    gpu = gpu_report()
    depth = detect_depth_layout(dataset_root)
    manifest = load_manifest(prefix)
    cuda = cuda_report(prefix)
    camera = validate_camera_adapter()
    recommendation = training_recommendation(gpu["maximum_vram_gib"])
    blocked = []
    warnings = []

    prefix_python = environment["prefix_python"]
    if not prefix_python.get("present"):
        blocked.append(
            "training prefix has no Python; install requires conda or a Python 3.8 interpreter"
        )
    elif not prefix_python.get("python_3_8"):
        blocked.append("training prefix must use Python 3.8")
    else:
        if prefix_python.get("torch_error") or not prefix_python.get("torch"):
            blocked.append("training prefix does not contain PyTorch")
        elif not prefix_python.get("cuda_available"):
            blocked.append("training prefix PyTorch cannot access CUDA")
        if prefix_python.get("mmcv_error") or not prefix_python.get("mmcv"):
            blocked.append("training prefix does not contain the integrated Bench2DriveZoo mmcv")
        if not cuda.get("official_11_8"):
            blocked.append("training prefix does not expose the official CUDA 11.8 nvcc")
    if not environment["bootstrap_available"] and not prefix_python.get("present"):
        blocked.append("neither conda nor python3.8 is available to create the isolated prefix")
    if not repository.get("present"):
        blocked.append("official Bench2DriveZoo repository is not present in the isolated prefix")
    elif not repository.get("official_origin"):
        blocked.append("Bench2DriveZoo origin is not the official Thinklab-SJTU repository")
    elif repository.get("dirty"):
        blocked.append("Bench2DriveZoo checkout is dirty; provenance is not reproducible")
    if not storage["smoke_ready"]:
        blocked.append("less than 20 GiB is free; even the Mini smoke profile is unsafe")
    if not gpu["present"]:
        blocked.append("no NVIDIA GPU was detected")
    if recommendation["profile"] == "planning-head-only":
        warnings.append("GPU VRAM is below 16 GiB; use planning-head-only training")
    if depth["layout"] == "mixed_invalid":
        blocked.append("dataset mixes legacy PNG and v0.0.4 NPZ depth without a version boundary")
    elif depth["layout"] == "v0.0.4":
        warnings.append("v0.0.4 NPZ depth requires the versioned adapter; legacy prepare_B2D.py is incompatible")
    if not manifest.get("present"):
        blocked.append("installation manifest is absent; branch and commit provenance is unknown")
    elif not manifest.get("valid"):
        blocked.append("installation manifest is invalid")
    elif repository.get("present"):
        recorded = manifest["content"].get("repository", {}).get("commit")
        if recorded != repository.get("commit"):
            blocked.append("repository HEAD does not match the manifest commit")

    return {
        "schema_version": 1,
        "mode": "check",
        "status": "BLOCKED" if blocked else "READY",
        "blocking_reasons": blocked,
        "warnings": warnings,
        "environment": environment,
        "gpu": gpu,
        "cuda": cuda,
        "training": recommendation,
        "storage": storage,
        "repository": repository,
        "manifest": manifest,
        "dataset": {"root": str(dataset_root), "depth": depth},
        "contracts": {
            "camera_adapter": camera,
            "depth_formats": DEPTH_CONTRACTS,
            "frame_rate_hz": 10,
            "future_step_s": 0.5,
            "future_steps": 6,
        },
        "license": LICENSE_CONTRACT,
        "download_policy": {
            "dataset_auto_download": False,
            "checkpoint_auto_download": False,
            "repository_download_only_in_install_mode": True,
        },
    }


def _write_manifest(
    prefix: Path,
    repo: Mapping[str, Any],
    repo_ref: str,
    torch_version: str,
) -> Path:
    if not repo.get("official_origin") or not re.fullmatch(
        r"[0-9a-f]{40}", str(repo.get("commit", ""))
    ):
        raise ContractError("cannot write a manifest for a non-official or unresolved repository")
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "environment_prefix": str(prefix.resolve()),
        "python_requirement": "3.8",
        "cuda_requirement": "11.8",
        "torch_version": torch_version,
        "repository": {
            "url": OFFICIAL_REPOSITORY,
            "ref": repo_ref,
            "commit": repo["commit"],
        },
        "contracts": {
            "private_camera_order": list(PRIVATE_TINY_CAMERA_ORDER),
            "public_camera_order": list(PUBLIC_B2D_CAMERA_ORDER),
            "public_from_private_indices": list(PUBLIC_FROM_PRIVATE_INDICES),
            "depth_formats": DEPTH_CONTRACTS,
        },
        "license": LICENSE_CONTRACT,
        "large_artifacts": {
            "dataset_downloaded": False,
            "checkpoint_downloaded": False,
        },
    }
    path = prefix / MANIFEST_NAME
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def install_environment(args: argparse.Namespace) -> Dict[str, Any]:
    prefix = args.env_prefix.resolve()
    repo_dir = args.repo_dir.resolve()
    if not args.accept_noncommercial_license:
        raise ContractError(
            "--install requires --accept-noncommercial-license because GitHub declares "
            "CC-BY-NC-ND-4.0"
        )
    conda = shutil.which("conda")
    python38 = shutil.which("python3.8")
    existing_python = _probe_python(prefix / "bin/python")
    if not conda and not python38 and not existing_python.get("python_3_8"):
        raise ContractError(
            "Python 3.8/conda is unavailable; refusing to mutate system Python or create an "
            "unsupported environment"
        )
    if prefix.exists() and not (prefix / "bin/python").is_file():
        raise ContractError("environment prefix exists but is not a Python environment: " + str(prefix))
    if existing_python.get("present") and not existing_python.get("python_3_8"):
        raise ContractError("existing environment prefix must use Python 3.8")
    if storage_report(prefix)["smoke_ready"] is not True:
        raise ContractError("less than 20 GiB is free; refusing even a Mini bootstrap")
    if repo_dir.exists():
        repo = repository_report(repo_dir)
        if not repo.get("official_origin"):
            raise ContractError("existing repository is not the official Bench2DriveZoo checkout")
        if repo.get("dirty"):
            raise ContractError("existing Bench2DriveZoo checkout is dirty")

    commands = []
    if not prefix.exists():
        if conda:
            commands.append([conda, "create", "-y", "--prefix", str(prefix), "python=3.8"])
        else:
            commands.append([python38, "-m", "venv", str(prefix)])
    if conda:
        commands.append(
            [
                conda,
                "install",
                "-y",
                "--prefix",
                str(prefix),
                "-c",
                "nvidia/label/cuda-11.8.0",
                "cuda-toolkit",
            ]
        )
    elif not (prefix / "bin/nvcc").is_file() and not Path(
        "/usr/local/cuda-11.8/bin/nvcc"
    ).is_file():
        raise ContractError(
            "python3.8 exists but CUDA 11.8 nvcc does not; use conda so CUDA stays inside the prefix"
        )

    for command in commands:
        _run(command, check=True)

    if repo_dir.exists():
        _run(["git", "fetch", "origin", args.repo_ref], cwd=repo_dir, check=True)
        _run(["git", "checkout", args.repo_commit or "FETCH_HEAD"], cwd=repo_dir, check=True)
    else:
        repo_dir.parent.mkdir(parents=True, exist_ok=True)
        _run(
            [
                "git",
                "clone",
                "--branch",
                args.repo_ref,
                "--single-branch",
                OFFICIAL_REPOSITORY,
                str(repo_dir),
            ],
            check=True,
        )
        if args.repo_commit:
            _run(["git", "checkout", args.repo_commit], cwd=repo_dir, check=True)

    python = prefix / "bin/python"
    if not python.is_file():
        raise ContractError("isolated Python was not created: " + str(python))
    pip_environment = os.environ.copy()
    pip_environment["PATH"] = str(prefix / "bin") + os.pathsep + pip_environment.get("PATH", "")
    pip_environment["CC"] = shutil.which("gcc-9") or "gcc"
    pip_environment["CXX"] = shutil.which("g++-9") or "g++"
    if conda:
        pip_environment["CUDA_HOME"] = str(prefix)
    elif (prefix / "bin/nvcc").is_file():
        pip_environment["CUDA_HOME"] = str(prefix)
    else:
        pip_environment["CUDA_HOME"] = "/usr/local/cuda-11.8"

    torch_spec = "torch=={}".format(args.torch_version)
    torchvision_spec = "torchvision=={}".format(args.torchvision_version)
    torchaudio_spec = "torchaudio=={}".format(args.torchaudio_version)
    _run(
        [str(python), "-m", "pip", "install", "pip==23.3.2", "setuptools", "wheel"],
        env=pip_environment,
        check=True,
    )
    _run(
        [
            str(python),
            "-m",
            "pip",
            "install",
            torch_spec,
            torchvision_spec,
            torchaudio_spec,
            "--index-url",
            "https://download.pytorch.org/whl/cu118",
        ],
        env=pip_environment,
        check=True,
    )
    _run(
        [str(python), "-m", "pip", "install", "-v", "-e", str(repo_dir)],
        cwd=repo_dir,
        env=pip_environment,
        check=True,
    )

    repo = repository_report(repo_dir)
    manifest = _write_manifest(prefix, repo, args.repo_ref, args.torch_version)
    return {
        "status": "INSTALLED",
        "environment_prefix": str(prefix),
        "repository": repo,
        "manifest": str(manifest),
        "dataset_downloaded": False,
        "checkpoint_downloaded": False,
    }


def _parser() -> argparse.ArgumentParser:
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="read-only prerequisite check (default)")
    modes.add_argument("--install", action="store_true", help="explicitly install the isolated stack")
    parser.add_argument(
        "--env-prefix",
        type=Path,
        default=workspace / ".venv-b2d",
        help="isolated environment prefix; system Python is never modified",
    )
    parser.add_argument("--repo-dir", type=Path, help="Bench2DriveZoo checkout directory")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=workspace / "data/bench2drive",
        help="optional existing dataset root to inspect; it is never downloaded",
    )
    parser.add_argument("--repo-ref", default=DEFAULT_REPOSITORY_REF)
    parser.add_argument("--repo-commit", help="optional exact commit to check out")
    parser.add_argument("--accept-noncommercial-license", action="store_true")
    parser.add_argument("--torch-version", default="2.1.2")
    parser.add_argument("--torchvision-version", default="0.16.2")
    parser.add_argument("--torchaudio-version", default="2.1.2")
    parser.add_argument("--json-output", type=Path, help="explicitly write the report to this path")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    args.env_prefix = args.env_prefix.expanduser()
    if args.repo_dir is None:
        args.repo_dir = args.env_prefix / "src/Bench2DriveZoo"
    else:
        args.repo_dir = args.repo_dir.expanduser()
    args.dataset_root = args.dataset_root.expanduser()
    try:
        if args.install:
            report = install_environment(args)
            exit_code = 0
        else:
            report = build_check_report(args.env_prefix, args.repo_dir, args.dataset_root)
            exit_code = 0 if report["status"] == "READY" else 2
    except (ContractError, subprocess.CalledProcessError) as error:
        detail = str(error)
        if isinstance(error, subprocess.CalledProcessError):
            detail = (error.stderr or error.stdout or detail).strip()
        report = {
            "schema_version": 1,
            "mode": "install" if args.install else "check",
            "status": "BLOCKED",
            "blocking_reasons": [detail],
            "dataset_downloaded": False,
            "checkpoint_downloaded": False,
        }
        exit_code = 2
    output = json.dumps(report, indent=2, sort_keys=True)
    print(output)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
