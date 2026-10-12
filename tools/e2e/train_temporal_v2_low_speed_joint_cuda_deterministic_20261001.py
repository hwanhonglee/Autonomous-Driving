"""HH_261001 - Version the strict-CUDA interpolation fix without editing the failed v1 fit or frozen math."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path

# HH_261001 - The revision entrypoint also reconstructs virtual namespaces in spawned CPU data workers.
if __name__ == "__mp_main__":
    import importlib.util
    _workspace_entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _workspace_spec = importlib.util.spec_from_file_location("hh_joint_revision_worker_workspace", _workspace_entry)
    _workspace_module = importlib.util.module_from_spec(_workspace_spec)
    _workspace_spec.loader.exec_module(_workspace_module)
    _workspace_module.configure(Path(__file__).resolve().parents[2])

import torch
from torch.nn import functional as F

from portable_e2e.deterministic_linear_interpolation_v2 import (
    KERNEL_ID, DeterministicPathTimeDecoderV2, fixed_linear_interpolation)
from scripts.e2e import train_temporal_v2_low_speed_joint_20261001 as frozen


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_v2_low_speed_joint_cuda_deterministic.20261001.v2"
EXPORT_SCHEMA = "portable_e2e.temporal_v2_low_speed_joint_cuda_deterministic_export.20261001.v2"
FROZEN_TRAINER_SHA = "f047739b639586c5998d8d117bd6d0330940ee28c9c5eadad34ce0a57340178d"
FROZEN_DECODER_SHA = "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7"
FAILED_V1 = frozen.WORKSPACE / "runs/training/temporal_v2_low_speed_joint_20261001_v1"
require, digest = frozen.require, frozen.digest


def verify_frozen_dependencies():
    require(digest(Path(frozen.__file__).resolve()) == FROZEN_TRAINER_SHA, "failed-v1 trainer bytes changed")
    require(digest(ROOT / "learning/portable_e2e/path_time_decoder_v2.py") == FROZEN_DECODER_SHA,
            "frozen decoder bytes changed")


def revision_sources():
    paths = (Path(__file__), ROOT / "learning/portable_e2e/deterministic_linear_interpolation_v2.py")
    return {str(path.relative_to(ROOT)): digest(path) for path in paths}


def failed_run_reference():
    # HH_261001 - Retain the observed before-fit/empty-history evidence; do not turn it into a performance result.
    require(FAILED_V1.is_dir(), "retained failed-v1 run is required before a new numerical revision")
    require((FAILED_V1 / "history.jsonl").stat().st_size == 0
            and not (FAILED_V1 / "summary.json").exists()
            and not list((FAILED_V1 / "checkpoints").glob("*.pt")),
            "expected failed-before-first-journal/checkpoint v1 evidence differs")
    return dict(path=str(FAILED_V1), observed_failure="strict CUDA native linear interpolation backward unsupported",
        history_rows=0, optimizer_updates_recorded=0, no_checkpoint_created=True,
        declaration_sha256=digest(FAILED_V1 / "declaration.json"),
        started_sha256=digest(FAILED_V1 / "started.json"),
        before_train_evaluation_sha256=digest(FAILED_V1 / "train_joint_before.json"),
        before_known_dev_sha256=digest(FAILED_V1 / "known_dev_raw_before.json"),
        evidence_scope="Retained run files plus root-observed exception; empty journal alone does not attest every possible optimizer path.")


@contextmanager
def revised_training_scope(reference=None):
    """HH_261001 - Bind only the new decoder kernel, receipt schema, source archive and declared numerical revision."""
    verify_frozen_dependencies()
    previous_decoder, previous_schema = frozen.PathTimeDecoderV2, frozen.SCHEMA
    previous_snapshot, previous_write = frozen.source_snapshot, frozen.write_new
    previous_tf32 = torch.backends.cuda.matmul.allow_tf32
    revisions = revision_sources()

    def snapshot():
        # HH_261001 - The independently evolving live runtime is not imported by this fit and is not a training dependency.
        values = previous_snapshot()
        values.pop("learning/portable_e2e/temporal_simulation_research_v2_20261001.py", None)
        return dict(values, **revisions)

    def write(path, value):
        if isinstance(value, dict) and Path(path).name in ("declaration.json", "metadata.json", "summary.json"):
            value = dict(value, numerical_revision=dict(kernel_id=KERNEL_ID,
                source_sha256=revisions, strict_deterministic_algorithms=True,
                tf32_matrix_multiplication=False, frozen_decoder_sha256=FROZEN_DECODER_SHA,
                frozen_training_orchestrator_sha256=FROZEN_TRAINER_SHA,
                interpolation_math="same 33-to-257 align_corners=True weights implemented as dense GEMM",
                finite_precision_bitwise_equivalence_claimed=False,
                unused_live_runtime_excluded_from_training_source_snapshot=True), failed_v1_reference=reference)
            if Path(path).name == "metadata.json":
                value["schema"] = EXPORT_SCHEMA
        return previous_write(path, value)

    frozen.PathTimeDecoderV2 = DeterministicPathTimeDecoderV2
    frozen.SCHEMA = SCHEMA
    frozen.source_snapshot, frozen.write_new = snapshot, write
    torch.backends.cuda.matmul.allow_tf32 = False
    try:
        yield
    finally:
        frozen.PathTimeDecoderV2, frozen.SCHEMA = previous_decoder, previous_schema
        frozen.source_snapshot, frozen.write_new = previous_snapshot, previous_write
        torch.backends.cuda.matmul.allow_tf32 = previous_tf32


def cuda_kernel_smoke(output):
    """HH_261001 - Test strict GPU0 forward/backward determinism before another actual optimizer fit."""
    verify_frozen_dependencies()
    require(not output.exists() and not output.is_symlink(), "create-only kernel smoke result required")
    require(output.resolve().is_relative_to((frozen.WORKSPACE / "runs/diagnostics").resolve()),
            "smoke output must remain in personal diagnostics")
    with frozen.previous.gpu0_lease():
        gpu = frozen.previous.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "deterministic cuBLAS environment required")
        old_tf32 = torch.backends.cuda.matmul.allow_tf32
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.use_deterministic_algorithms(True)
        torch.set_num_threads(1)
        generator = torch.Generator().manual_seed(2026100103)
        records = []
        try:
            for dtype in (torch.float32, torch.float64):
                source = torch.randn(4, 1, 33, generator=generator, dtype=dtype)
                weights = torch.randn(4, 1, 257, generator=generator, dtype=dtype)
                cpu = source.clone().requires_grad_()
                baseline = F.interpolate(cpu, size=257, mode="linear", align_corners=True)
                (baseline*weights).sum().backward()
                previous_result, previous_gradient = None, None
                for repetition in range(3):
                    value = source.cuda().requires_grad_()
                    result = fixed_linear_interpolation(value)
                    (result*weights.cuda()).sum().backward()
                    torch.cuda.synchronize()
                    tolerance = 1e-6 if dtype == torch.float32 else 1e-12
                    gradient_tolerance = 1e-5 if dtype == torch.float32 else 1e-12
                    require(torch.allclose(result.cpu(), baseline.detach(), atol=tolerance, rtol=tolerance),
                            "fixed CUDA interpolation differs from frozen CPU forward")
                    require(torch.allclose(value.grad.cpu(), cpu.grad, atol=gradient_tolerance, rtol=gradient_tolerance),
                            "fixed CUDA interpolation differs from frozen CPU backward")
                    if previous_result is not None:
                        require(torch.equal(previous_result, result) and torch.equal(previous_gradient, value.grad),
                                "strict CUDA interpolation repeats are not bitwise deterministic")
                    previous_result, previous_gradient = result.detach().clone(), value.grad.detach().clone()
                    records.append(dict(dtype=str(dtype), repetition=repetition,
                        maximum_forward_absolute_difference=float((result.cpu()-baseline.detach()).abs().max()),
                        maximum_gradient_absolute_difference=float((value.grad.cpu()-cpu.grad).abs().max()),
                        forward_absolute_and_relative_tolerance=tolerance,
                        gradient_absolute_and_relative_tolerance=gradient_tolerance,
                        repeated_output_and_gradient_exact=True))
            # HH_261001 - Cover the entire unchanged retimer backward, not only the replaced interpolation primitive.
            curv = torch.full((4, 33), .005, device="cuda", requires_grad=True)
            preferred = torch.full((4, 33), 2.3, device="cuda", requires_grad=True)
            current = torch.full((4,), 2.484, device="cuda")
            decoder = DeterministicPathTimeDecoderV2().cuda()
            decoded = decoder(curv, preferred, current, torch.zeros(4, dtype=torch.long, device="cuda"),
                              torch.zeros(4, device="cuda"), torch.full((4,), 64., device="cuda"))
            (decoded["xy"].square().mean()+decoded["instantaneous_speed"].square().mean()).backward()
            torch.cuda.synchronize()
            require(bool(torch.isfinite(curv.grad).all()) and bool(torch.isfinite(preferred.grad).all()),
                    "whole strict-CUDA decoder backward gradient is nonfinite")
            require(torch.equal(decoded["instantaneous_speed"][:, 0], current), "measured v0 changed")
            record = dict(schema="portable_e2e.deterministic_interpolation_cuda_smoke.20261001.v1",
                status="STRICT_GPU0_KERNEL_AND_FULL_DECODER_BACKWARD_PASS", gpu=gpu, kernel_id=KERNEL_ID,
                interpolation=records, whole_decoder_backward_finite=True, observed_v0_preserved=True,
                decoder_dispatch_receipt=decoder.last_interpolation_receipt,
                strict_deterministic_algorithms=True, tf32_matrix_multiplication=False,
                revision_source_sha256=revision_sources(), numerical_bitwise_cpu_cuda_equivalence_claimed=False,
                optimizer_updates=0, training_run_performed=False, vehicle_control_approved=False)
            output.parent.mkdir(parents=True, exist_ok=True)
            frozen.write_new(output, record)
            print(json.dumps(record, allow_nan=False), flush=True)
        finally:
            torch.backends.cuda.matmul.allow_tf32 = old_tf32
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False, add_help=False)
    parser.add_argument("--cuda-smoke-only", action="store_true")
    parser.add_argument("--smoke-output", type=Path)
    options, remaining = parser.parse_known_args(argv)
    if options.cuda_smoke_only:
        require(options.smoke_output is not None and not remaining, "smoke mode accepts only a fresh output path")
        return cuda_kernel_smoke(options.smoke_output)
    require(options.smoke_output is None, "smoke-output is only legal with smoke mode")
    verify_frozen_dependencies()
    if remaining == ["--help"]:
        return frozen.main(remaining)
    reference = failed_run_reference()
    with revised_training_scope(reference):
        return frozen.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
