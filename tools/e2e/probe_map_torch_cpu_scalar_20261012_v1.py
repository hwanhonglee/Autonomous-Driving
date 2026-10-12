"""HH_261012 - Probe existing CPU tensor conversion on 256 fictional byte values, without data or learning."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "storage/experiments/plans/2026-10-12/map_torch_cpu_scalar_probe_v1"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
RSS, FLOOR, RESERVE = 512 * 1024**2, 2 * 1024**3, 64 * 1024**2
SCHEMA = "portable_e2e.map_torch_cpu_scalar_probe.20261012.v1"


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode() + b"\n"


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(64 * 1024), b""):
            result.update(part)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args()
    require(args.execute and sys.flags.dont_write_bytecode and not sys.flags.optimize,
            "explicit clean assertion-enabled non-bytecode probe required")
    for key, expected in {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1",
                          "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                          "OPENBLAS_NUM_THREADS": "1"}.items():
        require(os.environ.get(key) == expected, "clean existing CPU environment required")
    require(os.sched_getaffinity(0) == {3}, "declared CPU3 affinity required")
    plan_raw, authority_raw = (PLAN / "ROOT_plan.json").read_bytes(), (PLAN / "ROOT_authority.json").read_bytes()
    require(hashlib.sha256(plan_raw).hexdigest() == args.plan_sha256
            and hashlib.sha256(authority_raw).hexdigest() == args.authority_sha256, "external RAW declaration pins differ")
    plan, authority = json.loads(plan_raw), json.loads(authority_raw)
    require(plan["schema"] == SCHEMA and plan["source_sha256"] == digest(Path(__file__).resolve())
            and authority == {"schema": SCHEMA + ".ROOT_authority", "plan_sha256": args.plan_sha256,
            "Torch_CPU_scalar256_authorized": True, "archive_arrays_authorized": False,
            "model_authorized": False, "learning_authorized": False, "GPU_authorized": False},
            "fresh scalar-only authority required")
    require(plan["limits"] == {"VmHWM_bytes": RSS, "deadline_seconds": 60, "floor_bytes": FLOOR,
            "reserve_bytes": RESERVE, "stdout_bytes": 64 * 1024, "scalar_count": 256},
            "prospective limits differ")
    require(LOCK.resolve() == LOCK, "canonical existing runtime lease required")
    with LOCK.open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        generation = os.fstat(lease.fileno())
        owner_pid, started = os.getpid(), time.monotonic()
        maximum_hwm = 0

        def guard():
            nonlocal maximum_hwm
            info, current = LOCK.stat(), os.fstat(lease.fileno())
            require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1
                    and (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
                    == (generation.st_dev, generation.st_ino, generation.st_size, generation.st_mtime_ns, generation.st_ctime_ns)
                    == (current.st_dev, current.st_ino, current.st_size, current.st_mtime_ns, current.st_ctime_ns)
                    and os.getpid() == owner_pid, "held lease or owner generation changed")
            status = Path("/proc/self/status").read_text()
            hwm = next(int(line.split()[1]) * 1024 for line in status.splitlines() if line.startswith("VmHWM:"))
            maximum_hwm = max(maximum_hwm, hwm)
            fs = os.statvfs(ROOT)
            require(maximum_hwm <= RSS and time.monotonic() - started < 60
                    and fs.f_bavail * fs.f_frsize >= FLOOR + RESERVE, "unchanged probe budget exceeded")

        try:
            guard()
            origins = {name: importlib.util.find_spec(name).origin for name in ("numpy", "torch")}
            require(origins == plan["expected_origins"], "existing selected package origins changed")
            before = {path: digest(Path(path)) for path in origins.values()}
            import numpy as np
            import torch
            guard()
            require(not torch.cuda.is_initialized(), "CUDA must remain uninitialized")
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
            require(torch.get_num_threads() == torch.get_num_interop_threads() == 1, "one own-process Torch thread required")
            with torch.no_grad():
                byte_values = np.arange(256, dtype=np.uint8)
                numpy_normalized = byte_values.astype(np.float32) / np.float32(255)
                original_torch_expression = torch.tensor(byte_values.copy(), dtype=torch.float32, device="cpu") / 255
                copied = torch.from_numpy(numpy_normalized).clone()
                require(copied.device.type == original_torch_expression.device.type == "cpu"
                        and copied.dtype == torch.float32 and not copied.requires_grad,
                        "CPU float32 gradient-free scalar tensors required")
                require(copied.data_ptr() != numpy_normalized.ctypes.data
                        and np.array_equal(copied.numpy().view(np.uint32), numpy_normalized.view(np.uint32)),
                        "owned normalized-host copy must preserve every bit")
                mismatch = np.flatnonzero(numpy_normalized.view(np.uint32)
                                          != original_torch_expression.numpy().view(np.uint32))
                copied[0] = 1
                require(numpy_normalized[0] == 0 and byte_values[0] == 0, "tensor copy modified host/native source")
                guard()
                result = dict(schema=SCHEMA + ".result", status="COMPLETE_SCALAR_PROBE_NOT_BATCH_OR_TRAINER",
                    Torch_version=torch.__version__, NumPy_version=np.__version__, origins=origins,
                    normalized_host_copy_bit_equal=True, copied_tensor_source_alias=False,
                    original_Torch_expression_bit_equal_to_NumPy=not bool(mismatch.size),
                    mismatched_scalar_count=int(mismatch.size), mismatched_byte_values=mismatch.tolist(),
                    maximum_absolute_difference=float(np.max(np.abs(numpy_normalized - original_torch_expression.numpy()))),
                    selected_origin_source_sha256=before, selected_provider_complete_transitive_closure_claimed=False,
                    actual_archive_reads=0, actual_batches=0, model_forwards=0, backward=0,
                    optimizer_updates=0, CUDA_initialized=False, GPU=0, NN=0,
                    Torch_threads=torch.get_num_threads(), Torch_interop_threads=torch.get_num_interop_threads())
            require(not torch.cuda.is_initialized()
                    and before == {path: digest(Path(path)) for path in origins.values()}, "selected sources or CUDA state changed")
            guard()
            result.update(elapsed_seconds=time.monotonic() - started, sampled_maximum_VmHWM_bytes=maximum_hwm,
                          sampled_guards_not_OS_containment=True, ROOT_plan_RAW_sha256=args.plan_sha256,
                          ROOT_authority_RAW_sha256=args.authority_sha256)
            raw = encoded(result)
            require(len(raw) <= 64 * 1024, "bounded scalar stdout exceeded")
            sys.stdout.buffer.write(raw)
            return 0
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
