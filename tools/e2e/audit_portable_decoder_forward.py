#!/usr/bin/env python3
"""HH_260906 - Replay only the frozen decoder forward from retained oracle latents on CPU; never optimize or approve data."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import torch

from scripts.e2e import audit_portable_decoder_probe as base

ROOT = Path(__file__).resolve().parents[2]
BASE_SHA = "515ddf40ca9fb2f67bfef7e902c76fa6cdd3f3d62726bbff80e082de97550dcd"
PROBE_SUMMARY_SHA = "cfe03aaabbd0a7f55016c0b4b72fe29a5ef6596ce3c692427a6e758c13338226"
OUTPUT_HASH_MANIFEST_SHA = "d204b1038effb0d5dad171b5db97a8df0e78242f2652d5ddb01c596ff0e9dc51"
TOLERANCE = {"absolute": 1e-5, "relative": 1e-5}
require = base.require


def cpu_guard():
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and not torch.cuda.is_initialized(),
        "forward audit requires hidden CUDA and an uninitialized CUDA runtime")
    torch.set_num_threads(4)


def source_pins():
    require(base.sha(ROOT / "scripts/e2e/audit_portable_decoder_probe.py") == BASE_SHA,
        "released independent auditor source changed")
    return {**base.source_pins(), "scripts/e2e/audit_portable_decoder_forward.py": base.sha(Path(__file__))}


def finite(value):
    require(isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value),
        "nonfinite or nonnumeric forward input/output")
    return value


def matrix(values, count, width):
    require(isinstance(values, list) and len(values) == count and all(isinstance(p, list) and len(p) == width for p in values),
        "forward tensor shape differs from frozen decoder ABI")
    for row in values:
        for value in row: finite(value)


def compare_values(recorded_xy, recorded_speed, replay_xy, replay_speed):
    # HH_260906 - Report every discrepancy; a CPU/GPU difference is not silently accepted or assigned a cause.
    matrix(recorded_xy, 64, 2); matrix(replay_xy, 64, 2)
    require(isinstance(recorded_speed, list) and isinstance(replay_speed, list)
        and len(recorded_speed) == len(replay_speed) == 64, "forward speed shape differs")
    mismatch, maxima = [], {"x_m": 0., "y_m": 0., "speed_mps": 0.}
    for point in range(64):
        for name, recorded, replay in (("x_m", recorded_xy[point][0], replay_xy[point][0]),
            ("y_m", recorded_xy[point][1], replay_xy[point][1]),
            ("speed_mps", recorded_speed[point], replay_speed[point])):
            finite(recorded); finite(replay)
            difference = abs(recorded - replay)
            maxima[name] = max(maxima[name], difference)
            if not math.isclose(recorded, replay, abs_tol=TOLERANCE["absolute"], rel_tol=TOLERANCE["relative"]):
                mismatch.append({"point_index": point, "component": name, "recorded_gpu_value": recorded,
                    "replayed_cpu_value": replay, "absolute_difference": difference})
    return {"status": "MATCH_WITH_PREDECLARED_TOLERANCE" if not mismatch else "UNVERIFIED",
        "compared_scalar_count": 192, "mismatch_scalar_count": len(mismatch),
        "maximum_absolute_difference_by_component": maxima, "mismatches": mismatch}


def replay_batch(inputs, recorded, config):
    # HH_260906 - Construct no learned module or optimizer; only exact float32 ABI tensors and a configuration shell exist.
    require(0 < len(inputs) == len(recorded) <= 256, "forward batch size differs")
    n = len(inputs)
    history = torch.zeros(n, 10, len(base.model.FEATURE_NAMES), dtype=torch.float32)
    route = torch.zeros(n, 128, 2, dtype=torch.float32)
    mask = torch.zeros(n, 128, dtype=torch.bool)
    latent_rows = []
    for index, (item, row) in enumerate(zip(inputs, recorded)):
        require(item["valid_mask"] == [True] * 64 and len(item["route_xy"]) in range(2, 129), "mask or route denominator differs")
        matrix(item["route_xy"], len(item["route_xy"]), 2)
        history[index, -1, base.model.FEATURE_NAMES.index("velocity_x_mps")] = finite(item["current_vx_mps"])
        route[index, :len(item["route_xy"])]=torch.tensor(item["route_xy"], dtype=torch.float32)
        mask[index, :len(item["route_xy"])]=True
        require(len(row["candidates"]) == 6, "fixed initializations omitted")
        candidates = []
        for k, candidate in enumerate(row["candidates"]):
            require(candidate["initialization_index"] == k and candidate["numeric_status"] == "FINITE", "candidate identity/status differs")
            matrix(candidate["optimized_raw_latents"], 64, 2)
            candidates.append(candidate["optimized_raw_latents"])
        latent_rows.append(candidates)
    latents = torch.tensor(latent_rows, dtype=torch.float32)
    require(all(bool(torch.isfinite(t).all()) and not t.requires_grad for t in (history, route, latents)), "float32 ABI overflow or gradients enabled")
    before = hashlib.sha256(latents.numpy().tobytes()).hexdigest()
    with torch.no_grad():
        xy, speed = base.model.PerspectiveTrajectoryModel._decode_physical_v1(SimpleNamespace(config=config), latents, history, route, mask)
    require(tuple(xy.shape) == (n, 6, 64, 2) and tuple(speed.shape) == (n, 6, 64)
        and not xy.requires_grad and not speed.requires_grad and bool(torch.isfinite(xy).all()) and bool(torch.isfinite(speed).all()),
        "decoder forward output shape/finite/no-grad contract failed")
    require(before == hashlib.sha256(latents.numpy().tobytes()).hexdigest(), "retained optimized latents mutated during forward")
    return xy.tolist(), speed.tolist(), {"optimized_latent_float32_sha256": before,
        "cpu_xy_float32_sha256": hashlib.sha256(xy.numpy().tobytes()).hexdigest(),
        "cpu_speed_float32_sha256": hashlib.sha256(speed.numpy().tobytes()).hexdigest()}


def audit(owned_root, input_root):
    cpu_guard(); sources = source_pins()
    input_pins, original_pins = {}, {}
    manifest = base.read(input_root, "manifest.json", input_pins)
    require(input_pins["manifest.json"]["sha256"] == base.INPUT_SHA, "unreviewed frozen input manifest")
    inputs = base.read(input_root, "inputs.jsonl", input_pins, lines=True)
    require(input_pins["inputs.jsonl"]["sha256"] == manifest["inputs_sha256"], "frozen input payload SHA differs")
    owner_started = base.read(owned_root, "owner_started.json", original_pins)
    owner_result = base.read(owned_root, "owner_result.json", original_pins)
    summary = base.read(owned_root, "probe/summary.json", original_pins)
    require(original_pins["probe/summary.json"]["sha256"] == PROBE_SUMMARY_SHA == owner_result["probe_summary_sha256"],
        "unreviewed original GPU result")
    require(owner_started["plan"]["source_commit"] == base.COMMIT and owner_started["plan"]["probe_sha256"] == base.PROBE_SHA
        and owner_started["plan"]["input_manifest_sha256"] == base.INPUT_SHA
        and owner_result["status"] == summary["status"] == "COMPLETE_NOT_ADMITTED" and owner_result["returncode"] == 0
        and owner_result["child_exited"] is True and owner_result["source_and_plan_postcheck_pass"] is True,
        "original completed source/owner proof differs")
    require(summary["source_identity"] == manifest["source_identity"] and summary["plan"] == manifest["plan"]
        and summary["source_and_input_postcheck_pass"] is True, "original source/input postcheck differs")
    historical = base.historical_source_proof(summary["source_identity"])
    started = base.read(owned_root, "probe/started.json", original_pins)
    require(started["input_pins"] == input_pins and started["source_identity"] == summary["source_identity"], "original startup input/source binding differs")
    data = base.read(owned_root, "probe/per_anchor.jsonl", original_pins, lines=True)
    sums_path = base.contract._safe_file(Path(owned_root), "probe/SHA256SUMS", "original GPU hash manifest")
    require(base.sha(sums_path) == OUTPUT_HASH_MANIFEST_SHA, "unreviewed original output hash manifest")
    hashes = [line.split("  ") for line in sums_path.read_text().splitlines()]
    require(len(hashes) == 4 and {name for _, name in hashes} == {"started.json", "summary.json", "per_anchor.jsonl", "optimization_history.jsonl"},
        "original GPU hash manifest inventory differs")
    for expected, name in hashes:
        path = base.contract._safe_file(Path(owned_root), f"probe/{name}", "original GPU output")
        require(base.sha(path) == expected, "original GPU output SHA differs")
        original_pins[f"probe/{name}"] = {"sha256": expected, "size_bytes": path.stat().st_size}
    original_pins["probe/SHA256SUMS"] = {"sha256": base.sha(sums_path), "size_bytes": sums_path.stat().st_size}
    require(len(inputs) == len(data) == 1337 and [r["anchor_index"] for r in inputs] == list(range(1337))
        and [r["anchor_index"] for r in data] == list(range(1337)), "full 1337 anchor denominator differs")
    config = base.model.ModelConfig.from_mapping(manifest["source_identity"]["model_config"])
    require(config.to_dict() == base.model.ModelConfig(model_id=base.model.PHYSICAL_MODEL_ID, maximum_step_m=1.).to_dict(), "unreviewed physical decoder config")
    rows, batches = [], []
    for start in range(0, 1337, 256):
        batch, detail = inputs[start:start + 256], data[start:start + 256]
        xy, speed, tensor_pins = replay_batch(batch, detail, config)
        batches.append({"batch_start_anchor_index": start, "batch_size": len(batch), **tensor_pins})
        for index, (item, original) in enumerate(zip(batch, detail)):
            require(item["sample_id"] == original["sample_id"] and item["trial_id"] == original["trial_id"]
                and item["capture_phase"] == original["capture_phase"] and item["original_raw_violating_step_counts"] == original["original_raw_violating_step_counts"],
                "original sample/failure identity changed")
            values = []
            for k, candidate in enumerate(original["candidates"]):
                values.append({"initialization_index": k, "original_runtime_gate": candidate["output_runtime_gate"],
                    **compare_values(candidate["decoder_xy"], candidate["decoder_speed"], xy[index][k], speed[index][k])})
            rows.append({"sample_id": item["sample_id"], "anchor_index": item["anchor_index"], "trial_id": item["trial_id"],
                "capture_phase": item["capture_phase"], "original_raw_violating_step_counts": item["original_raw_violating_step_counts"], "candidates": values})
    base.recheck(owned_root, original_pins); base.recheck(input_root, input_pins)
    require(source_pins() == sources, "forward audit source changed")
    cpu_guard()
    candidates = [c for row in rows for c in row["candidates"]]
    mismatches = sum(c["mismatch_scalar_count"] for c in candidates)
    result = {"schema": "portable_e2e.independent_decoder_final_forward.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "MATCHED_NOT_ADMITTED" if not mismatches else "UNVERIFIED_NOT_ADMITTED", "torch_version": torch.__version__,
        "device": "cpu", "torch_threads": torch.get_num_threads(), "cuda_visible_devices": os.environ["CUDA_VISIBLE_DEVICES"],
        "comparison_tolerance": TOLERANCE, "anchor_count": len(rows), "candidate_count": len(candidates),
        "point_count": len(candidates) * 64, "compared_scalar_count": len(candidates) * 192, "mismatch_scalar_count": mismatches,
        "mismatch_candidate_count": sum(c["mismatch_scalar_count"] > 0 for c in candidates),
        "maximum_absolute_difference_by_component": {name: max(c["maximum_absolute_difference_by_component"][name] for c in candidates)
            for name in ("x_m", "y_m", "speed_mps")},
        "original_runtime_gate_counts": dict(Counter(c["original_runtime_gate"]["status"] for c in candidates)),
        "original_raw_curvature_failed_anchor_count": sum(bool(r["original_raw_violating_step_counts"]["xy_curvature"]) for r in rows),
        "batches": batches, "source_sha256": sources, "historical_execution_source_proof": historical,
        "input_pins": input_pins, "original_gpu_output_pins": original_pins, "source_and_input_postcheck_pass": True,
        "scope": {"no_grad": True, "model_parameters_instantiated": False, "optimizer_instantiated": False,
            "optimization_replayed": False, "training_data_approved": False, "runtime_gate_changed": False,
            "original_gpu_outputs_replaced": False, "jpeg_or_raw_capture_payload_read": False},
        "interpretation": "Forward-only arithmetic comparison of every retained latent. Numerical mismatch remains UNVERIFIED; device/version alone does not establish its cause. Original GPU verdicts are copied, not reclassified. A match proves neither causal driving performance nor data admission."}
    return result, rows


def write_result(output, summary, rows):
    require(not output.exists() and not output.is_symlink(), "new-only forward audit output required")
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    with (output / "per_anchor_forward.jsonl").open("x") as stream:
        for row in rows: stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text("".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir()) if path.is_file()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--owned-root", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    require(not args.output_dir.exists() and not args.output_dir.is_symlink() and all(not args.output_dir.resolve().is_relative_to(p.resolve())
        for p in (args.owned_root, args.input_root)), "new output must remain outside original evidence")
    result = audit(args.owned_root, args.input_root)
    write_result(args.output_dir, *result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
