#!/usr/bin/env python3
"""HH_260906 - Measure future-aware decoder approximation without training a causal model or replacing labels."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import signal
from types import SimpleNamespace
import time

import torch

from portable_e2e import model, runtime_contract as runtime
from portable_e2e.torch_dataset import _linear_route_samples
from scripts.e2e import audit_carla_raw_pre_admission as raw

ROOT = Path(__file__).resolve().parents[2]
MODEL_SHA = "b72c0fcbaf558254a3e7b02aa90406ed157724a07f63d0ef9f46d0d444f92fc4"
RUNTIME_SHA = "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3"
ASSIGNED_GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
CONFIG = model.ModelConfig(model_id=model.PHYSICAL_MODEL_ID, maximum_step_m=1.0)
PLAN = {"iterations": 512, "batch_size": 256, "candidate_count": 6, "future_points": 64,
    "optimizer": "Adam", "learning_rate": .05, "betas": [.9, .999], "epsilon": 1e-8, "weight_decay": 0.0,
    "initialization": "candidate0=zeros; candidates1..5=Normal(0,0.03), CPU generator seed 20260908+global_anchor_index",
    "initialization_seed": 20260908, "initialization_std": .03, "dtype": "float32",
    "objective": "mean_t ||predicted_XY-target_XY||^2 + 1.0 second^2 * mean_t (predicted_speed-target_speed)^2",
    "objective_reduction_for_backward": "sum over independent anchors and six initializations",
    "speed_squared_error_weight_s2": 1.0, "early_stopping": False, "scheduler": False,
    "gradient_clipping": False, "best_iteration_replacement": False, "convergence_window": 64,
    "convergence_absolute_change_warning_m2": 1e-6, "convergence_relative_change_warning": .01,
    "descriptive_max_xy_error_thresholds_m": [.001, .01, .1]}
EXPECTED_COUNTS = {"run_001": 671, "run_002": 666}
SCOPE = {"causal_model_training": False, "model_parameters_instantiated": False, "model_parameter_gradients": False,
    "future_target_used_by_optimizer": True, "test_payload_read": False, "jpeg_payload_read": False,
    "common10_dataset_written": False, "raw_label_replacement": False, "raw_failure_filtering": False,
    "training_data_approved": False, "runtime_model_checkpoint": False, "model_promotion": False,
    "global_optimum_certified": False, "new_admission_threshold": False}
require = raw.require


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def source_identity():
    # HH_260906 - Pin exact decoder/gate and all reused extraction/tensorization formulas, independently of a Git cleanliness claim.
    require(raw.scalar.sha(ROOT / "portable_e2e/model.py") == MODEL_SHA, "unreviewed physical decoder source")
    require(raw.scalar.sha(ROOT / "portable_e2e/runtime_contract.py") == RUNTIME_SHA, "unreviewed runtime gate source")
    functions = (model.PerspectiveTrajectoryModel._decode_physical_v1, runtime.validate_and_select_trajectory,
        raw.measured_timeline, raw.future_state, raw.adapter._causal_state, raw.adapter._interpolate_state,
        raw.adapter._relative_xy_yaw, raw.contract._canonical_route_in_base, _linear_route_samples,
        raw.scalar.sha, raw.scalar.require)
    files = {Path(__file__).resolve(), raw.contract.DEFAULT_CONTRACT_PATH.resolve(), ROOT / "portable_e2e/dataset.py"}
    files.update(Path(inspect.getsourcefile(f)).resolve() for f in functions)
    return {"files": {str(p.relative_to(ROOT)): raw.scalar.sha(p) for p in sorted(files)},
        "functions": {f"{f.__module__}.{f.__name__}": raw.digest(inspect.getsource(f).encode()) for f in functions},
        "model_config": CONFIG.to_dict(), "model_config_sha256": raw.digest(encoded(CONFIG.to_dict())),
        "runtime_gate": asdict(runtime.RuntimeGateConfig()), "runtime_gate_id": runtime.RUNTIME_GATE_ID}


def read_bytes(root, name, ledger):
    path = raw.contract._safe_file(Path(root), name, "probe evidence")
    payload = raw.contract._read_regular_file_bounded(path, 256 * 1024 * 1024, "probe evidence")
    ledger[name] = {"sha256": raw.digest(payload), "size_bytes": len(payload)}
    return payload


def fresh(output, roots):
    output = Path(output)
    require(not output.exists() and not output.is_symlink()
        and all(not output.resolve().is_relative_to(Path(p).resolve()) for p in roots), "new output must be outside immutable inputs")
    return output


def recheck(root, ledger):
    for name, pin in ledger.items():
        require(raw.scalar.sha(raw.contract._safe_file(Path(root), name, "probe recheck")) == pin["sha256"], "probe input bytes changed")


def write_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def write_sums(output):
    with (output / "SHA256SUMS").open("x") as stream:
        for path in sorted(output.iterdir()):
            if path.is_file() and path.name != "SHA256SUMS":
                stream.write(f"{raw.scalar.sha(path)}  {path.name}\n")


def extract_anchor(timeline, camera, route, original):
    # HH_260906 - Only the final actual vx is consumed by the exact decoder; no fabricated full-model history is claimed.
    stamp = raw.adapter._timestamp_ns(camera["timestamp"], "probe anchor")
    ego_ns, ego = raw.adapter._causal_state(timeline, stamp)
    require(ego_ns == stamp and original["anchor_timestamp_ns"] == stamp and original["frame"] == camera["frame"]
        and original["capture_phase"] == camera["capture_phase"] and original["valid_points"] == 64
        and original["valid_mask"] == [True] * 64 and original["invalid_reasons"] == [None] * 64,
        "full original anchor binding or mask changed")
    yaw = float(ego["yaw"])
    canonical, _, _ = raw.contract._canonical_route_in_base(tuple((p["x"], p["y"]) for p in route["route"]),
        position_m=(ego["x"], ego["y"], ego["z"]), orientation_xyzw=[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)],
        contract=raw.contract.load_contract(), context="decoder-only oracle probe")
    route_points = _linear_route_samples(canonical, limit=128)
    stamps = [t for t, _ in timeline]
    xy, speed, target_yaw = [], [], []
    for i in range(64):
        target_stamp = stamp + (i + 1) * 100_000_000
        future, _ = raw.future_state(timeline, stamps, target_stamp)
        require(future is not None, "previously full future is now missing; no interpolation over gaps")
        point, angle = raw.adapter._relative_xy_yaw(ego, future)
        target_speed = math.hypot(future["vx"], future["vy"])
        prior_step = original["diagnostic"]["steps"][i]
        require(prior_step["target_timestamp_ns"] == target_stamp
            and abs(prior_step["metrics"]["speed_limit"][0] - target_speed) < 1e-10, "reconstructed target differs from original diagnostic")
        xy.append(point); speed.append(target_speed); target_yaw.append(angle)
    violations = {name: sum(step["metrics"][name][1] for step in original["diagnostic"]["steps"])
        for name in original["diagnostic"]["steps"][0]["metrics"]}
    return {"frame": camera["frame"], "anchor_timestamp_ns": stamp, "capture_phase": camera["capture_phase"],
        "current_vx_mps": ego["vx"], "route_xy": route_points, "target_xy": xy, "target_speed": speed,
        "target_yaw": target_yaw, "valid_mask": [True] * 64, "original_raw_violating_step_counts": violations}


def extract(diagnostic_root, campaign_root, output, expected_summary_sha, expected_script_sha):
    require(raw.scalar.sha(Path(__file__)) == expected_script_sha, "probe script SHA mismatch")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and not torch.cuda.is_initialized(),
        "CPU extraction requires CUDA hidden and uninitialized")
    diagnostic_root, campaign_root = Path(diagnostic_root), Path(campaign_root)
    output = fresh(output, [diagnostic_root, campaign_root])
    sources, diagnostic_ledger = source_identity(), {}
    payload = read_bytes(diagnostic_root, "summary.json", diagnostic_ledger)
    require(raw.digest(payload) == expected_summary_sha, "raw diagnostic summary SHA mismatch")
    summary = json.loads(payload)
    require(summary["schema"] == raw.SCHEMA and summary["status"] == "DIAGNOSED_NOT_ADMITTED"
        and summary["trial_count"] == 2 and summary["total_jpeg_count"] == 8802, "wrong frozen v4 raw diagnostic corpus")
    for function in (raw.adapter._causal_state, raw.adapter._interpolate_state, raw.adapter._relative_xy_yaw,
                     raw.contract._canonical_route_in_base):
        name = f"{function.__module__}.{function.__name__}"
        require(summary["source_identity"]["functions"][name] == sources["functions"][name], "historical extraction formula changed")
    require(summary["source_identity"]["files"]["scripts/e2e/audit_carla_raw_pre_admission.py"]
        == sources["files"]["scripts/e2e/audit_carla_raw_pre_admission.py"], "historical raw timeline helper changed")
    sums = read_bytes(diagnostic_root, "SHA256SUMS", diagnostic_ledger).decode().splitlines()
    expected = {line.split("  ")[1]: line.split("  ")[0] for line in sums}
    require(expected.get("summary.json") == expected_summary_sha, "original diagnostic hash manifest mismatch")
    anchor_bytes = read_bytes(diagnostic_root, "future_anchor_audit.jsonl", diagnostic_ledger)
    require(raw.digest(anchor_bytes) == expected.get("future_anchor_audit.jsonl"), "original future ledger SHA mismatch")
    prior = [json.loads(line) for line in anchor_bytes.splitlines()]
    require(len(prior) == 1467 and len({(r["trial"], r["frame"]) for r in prior}) == len(prior), "original anchor denominator changed")
    prior_by_frame = {(r["trial"], r["frame"]): r for r in prior}
    rows, trial_ledgers = [], {}
    require([t["trial_id"] for t in summary["trials"]] == list(EXPECTED_COUNTS), "v4 trials omitted, repeated or reordered")
    for trial in summary["trials"]:
        name = trial["trial_id"]
        require(trial.get("reviewed_protocol") == "comfortable_v4_acknowledged_batch"
            and trial["training_data_approved"] is False and trial["original_episode_directory"] == "episode",
            "only the two reviewed unapproved v4 captures are in scope")
        root, ledger = campaign_root / "town07_straight_calibration" / name, {}
        for item in trial["source_manifest"]:
            require(raw.digest(read_bytes(root, item["path"], ledger)) == item["sha256"], "original capture metadata SHA mismatch")
        require(ledger["provenance/portable_e2e/model.py"]["sha256"] == MODEL_SHA
            and ledger["provenance/portable_e2e/runtime_contract.py"]["sha256"] == RUNTIME_SHA, "capture bounds source differs from probe")
        states = [json.loads(line) for line in read_bytes(root, "episode/states.jsonl", ledger).splitlines()]
        cameras = [json.loads(line) for line in read_bytes(root, "episode/camera_frames.jsonl", ledger).splitlines()]
        route = json.loads(read_bytes(root, "episode/route.json", ledger))
        timeline = raw.measured_timeline(states)
        count = 0
        for camera in cameras:
            original = prior_by_frame[(name, camera["frame"])]
            if camera["capture_phase"] == "stationary_tail":
                require(original["disposition"] == "tail_label_context_only", "tail policy changed")
                continue
            require(original["disposition"] == "full_64_point_anchor", "no incomplete or failed anchor may be filtered")
            row = extract_anchor(timeline, camera, route, original)
            row.update(anchor_index=len(rows), trial_id=name, sample_id=f"{name}:{camera['frame']}")
            rows.append(row); count += 1
        require(count == EXPECTED_COUNTS[name], "full v4 anchor denominator mismatch")
        require(Counter(r["capture_phase"] for r in rows if r["trial_id"] == name)
            == {"stationary_warmup": 35, "driving": count - 35}, "original warmup policy changed")
        trial_ledgers[name] = ledger
    require(len(rows) == 1337 and source_identity() == sources and not torch.cuda.is_initialized(),
        "extraction source, full denominator or CPU-only scope changed")
    for name, ledger in trial_ledgers.items():
        recheck(campaign_root / "town07_straight_calibration" / name, ledger)
    recheck(diagnostic_root, diagnostic_ledger)
    output.mkdir(parents=True, exist_ok=False)
    with (output / "inputs.jsonl").open("x") as stream:
        for row in rows:
            stream.write(encoded(row).decode() + "\n")
    manifest = {"schema": "portable_e2e.decoder_oracle_inputs.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "DIAGNOSTIC_INPUTS_NOT_TRAINING_DATA", "input_count": 1337, "counts_by_trial": EXPECTED_COUNTS,
        "inputs_sha256": raw.scalar.sha(output / "inputs.jsonl"), "source_identity": sources,
        "original_diagnostic_pins": diagnostic_ledger, "original_capture_metadata_pins": trial_ledgers,
        "plan": PLAN, "scope": SCOPE, "history_notice": "Zero-filled decoder-only [10,13] ABI stub with actual final vx. All other history channels are unused by the exact decoder; this is not a causal full-model input.",
        "interpretation": "All original warmup/driving futures and raw failure counts retained. Tail is label context only. No images or Common10 dataset are exported."}
    write_json(output / "manifest.json", manifest); write_sums(output)
    return manifest


def tensor_batch(rows, device):
    require(rows and len(rows) <= 256, "probe batch must contain 1..256 anchors")
    history = torch.zeros(len(rows), 10, len(model.FEATURE_NAMES), dtype=torch.float32, device=device)
    route = torch.zeros(len(rows), 128, 2, dtype=torch.float32, device=device)
    mask = torch.zeros(len(rows), 128, dtype=torch.bool, device=device)
    for i, row in enumerate(rows):
        require(row["valid_mask"] == [True] * 64 and len(row["route_xy"]) in range(2, 129), "decoder input mask/route invalid")
        history[i, -1, model.FEATURE_NAMES.index("velocity_x_mps")] = raw.adapter._number(row["current_vx_mps"], "actual vx")
        route[i, :len(row["route_xy"])] = torch.tensor(row["route_xy"], device=device)
        mask[i, :len(row["route_xy"])] = True
    target_xy = torch.tensor([r["target_xy"] for r in rows], dtype=torch.float32, device=device)
    target_speed = torch.tensor([r["target_speed"] for r in rows], dtype=torch.float32, device=device)
    require(tuple(target_xy.shape) == (len(rows), 64, 2) and tuple(target_speed.shape) == (len(rows), 64)
        and all(bool(torch.isfinite(t).all()) for t in (history, route, target_xy, target_speed)), "nonfinite or malformed decoder inputs")
    return history, route, mask, target_xy, target_speed


def decode(latent, history, route, mask):
    # HH_260906 - Call the existing implementation unbound with only its frozen configuration; no learned module exists.
    return model.PerspectiveTrajectoryModel._decode_physical_v1(SimpleNamespace(config=CONFIG), latent, history, route, mask)


def objective(xy, speed, target_xy, target_speed):
    return (xy - target_xy[:, None]).square().sum(-1).mean(-1) + (speed - target_speed[:, None]).square().mean(-1)


def initial_latents(indices):
    values = []
    for index in indices:
        require(type(index) is int and index >= 0, "invalid global anchor index")
        generator = torch.Generator(device="cpu").manual_seed(PLAN["initialization_seed"] + index)
        row = torch.zeros(6, 64, 2, dtype=torch.float32)
        row[1:] = torch.randn((5, 64, 2), generator=generator, dtype=torch.float32) * PLAN["initialization_std"]
        values.append(row)
    return torch.stack(values)


def optimize_batch(rows, device, stop, progress, *, _test_iterations=None):
    # HH_260906 - The private iteration hook is for tiny unit fixtures; the CLI has no iteration or initialization override.
    iterations = 512 if _test_iterations is None else _test_iterations
    require(type(iterations) is int and 1 <= iterations <= 512, "invalid fixed/test iteration budget")
    history, route, mask, target_xy, target_speed = tensor_batch(rows, device)
    latent = initial_latents([r["anchor_index"] for r in rows]).to(device).requires_grad_(True)
    initial_sha = raw.digest(latent.detach().cpu().numpy().tobytes())
    optimizer = torch.optim.Adam([latent], lr=.05, betas=(.9, .999), eps=1e-8, weight_decay=0.)
    completed, failure, initial_values, window_values = 0, None, None, None
    for step in range(iterations + 1):
        if stop():
            failure = "deadline_or_signal"; break
        xy, speed = decode(latent, history, route, mask)
        losses = objective(xy, speed, target_xy, target_speed)
        if not bool(torch.isfinite(losses).all()):
            failure = "nonfinite_objective"; break
        numeric = losses.detach().cpu()
        initial_values = numeric.clone() if initial_values is None else initial_values
        if step == max(0, iterations - 64):
            window_values = numeric.clone()
        progress({"iteration": step, "mean_objective_by_initialization": numeric.mean(0).tolist(),
            "maximum_objective": float(numeric.max()), "optimizer_steps_completed": completed})
        if step == iterations:
            break
        optimizer.zero_grad()
        losses.sum().backward()
        if latent.grad is None or not bool(torch.isfinite(latent.grad).all()):
            failure = "nonfinite_or_missing_latent_gradient"; break
        optimizer.step(); completed += 1
    with torch.no_grad():
        xy, speed = decode(latent, history, route, mask)
        final = objective(xy, speed, target_xy, target_speed)
    finite = all(bool(torch.isfinite(v).all()) for v in (latent, xy, speed, final))
    return {"status": "COMPLETE" if failure is None and completed == iterations and finite else "PARTIAL_OR_FAILED",
        "failure": failure or (None if finite else "nonfinite_terminal_values"), "iterations_completed": completed,
        "initial_latent_sha256": initial_sha, "initial_objective": initial_values,
        "window_objective": window_values, "final_objective": final.detach().cpu(),
        "latent": latent.detach().cpu(), "xy": xy.detach().cpu(), "speed": speed.detach().cpu(), "terminal_values_finite": finite}


def candidate_reports(rows, result):
    reports = []
    xy, speed = result["xy"], result["speed"]
    target_xy = torch.tensor([r["target_xy"] for r in rows], dtype=torch.float32)
    target_speed = torch.tensor([r["target_speed"] for r in rows], dtype=torch.float32)
    errors = torch.linalg.norm(xy - target_xy[:, None], dim=-1)
    speed_error = (speed - target_speed[:, None]).abs()
    for i, row in enumerate(rows):
        candidates = []
        for k in range(6):
            finite = all(bool(torch.isfinite(v).all()) for v in
                (result["latent"][i, k], xy[i, k], speed[i, k], result["final_objective"][i, k]))
            if not finite:
                candidates.append({"initialization_index": k, "numeric_status": "NONFINITE_UNAVAILABLE",
                    **{key: None for key in ("initial_objective_m2", "final_objective_m2", "ade_m", "fde_m", "maximum_xy_error_m",
                        "speed_rmse_mps", "maximum_speed_error_mps", "last_window_objective_m2", "last_window_absolute_change_m2",
                        "xy_error_by_step_m", "speed_absolute_error_by_step_mps", "optimized_raw_latents", "decoder_xy", "decoder_speed")},
                    "convergence_warning": True, "output_runtime_gate": {"status": "NOT_EVALUATED_NONFINITE", "reason": "Nonfinite terminal values are not repaired or omitted."},
                    "terminal_tensor_byte_sha256": raw.digest(b"".join(v.numpy().tobytes() for v in (result["latent"][i, k], xy[i, k], speed[i, k]))),
                    "nonfinite_scalar_counts": {name: int((~torch.isfinite(v)).sum()) for name, v in
                        (("latent", result["latent"][i, k]), ("xy", xy[i, k]), ("speed", speed[i, k]), ("objective", result["final_objective"][i, k]))}})
                continue
            logits = [float(j == k) for j in range(6)]
            try:
                runtime.validate_and_select_trajectory(xy[i].tolist(), speed[i].tolist(), logits,
                    current_speed_mps=row["current_vx_mps"])
                gate = {"status": "PASS", "reason": None}
            except raw.contract.ContractError as error:
                gate = {"status": "FAIL", "reason": str(error)}
            initial = None if result["initial_objective"] is None else float(result["initial_objective"][i, k])
            end = float(result["final_objective"][i, k])
            window = None if result["window_objective"] is None else float(result["window_objective"][i, k])
            change = None if window is None else abs(end - window)
            candidates.append({"initialization_index": k, "numeric_status": "FINITE", "initial_objective_m2": initial, "final_objective_m2": end,
                "last_window_objective_m2": window, "last_window_absolute_change_m2": change,
                "convergence_warning": window is None or (change > 1e-6 and change / max(abs(window), 1e-12) > .01),
                "ade_m": float(errors[i, k].mean()), "fde_m": float(errors[i, k, -1]),
                "maximum_xy_error_m": float(errors[i, k].max()),
                "speed_rmse_mps": float(speed_error[i, k].square().mean().sqrt()),
                "maximum_speed_error_mps": float(speed_error[i, k].max()),
                "xy_error_by_step_m": errors[i, k].tolist(), "speed_absolute_error_by_step_mps": speed_error[i, k].tolist(),
                "output_runtime_gate": gate, "optimized_raw_latents": result["latent"][i, k].tolist(),
                "decoder_xy": xy[i, k].tolist(), "decoder_speed": speed[i, k].tolist()})
        finite_choices = [k for k in range(6) if candidates[k]["final_objective_m2"] is not None]
        reports.append({"sample_id": row["sample_id"], "anchor_index": row["anchor_index"], "trial_id": row["trial_id"],
            "capture_phase": row["capture_phase"], "original_raw_violating_step_counts": row["original_raw_violating_step_counts"],
            "candidates": candidates, "oracle_objective_minimum_initialization_index":
                min(finite_choices, key=lambda k: candidates[k]["final_objective_m2"]) if finite_choices else None,
            "iterations_completed": result["iterations_completed"], "optimization_status": result["status"]})
    return reports


def distribution(values):
    ordered = sorted(values)
    def percentile(q):
        p = (len(ordered) - 1) * q
        lower = int(p)
        return ordered[lower] + (ordered[min(lower + 1, len(ordered) - 1)] - ordered[lower]) * (p - lower)
    return {"count": len(values), "mean": sum(values) / len(values), "min": ordered[0], "p50": percentile(.5),
        "p90": percentile(.9), "p99": percentile(.99), "max": ordered[-1]} if values else {"count": 0}


def summarize_reports(reports):
    groups = {f"initialization_{k}": [r["candidates"][k] for r in reports] for k in range(6)}
    groups["future_aware_best_final_objective_of_six"] = [r["candidates"][r["oracle_objective_minimum_initialization_index"]]
        for r in reports if r["oracle_objective_minimum_initialization_index"] is not None]
    return {name: {"original_reported_anchor_denominator": len(reports), "included_candidate_count": len(candidates),
        "unavailable_all_initializations_anchor_count": len(reports) - len(candidates),
        "metrics": {key: {**distribution([c[key] for c in candidates if c[key] is not None]),
        "missing_count": sum(c[key] is None for c in candidates)} for key in
        ("initial_objective_m2", "final_objective_m2", "ade_m", "fde_m", "maximum_xy_error_m", "speed_rmse_mps", "maximum_speed_error_mps")},
        "runtime_gate_counts": dict(Counter(c["output_runtime_gate"]["status"] for c in candidates)),
        "convergence_warning_count": sum(c["convergence_warning"] for c in candidates),
        "descriptive_max_xy_error_coverage": {str(t): {"within_count": sum(c["maximum_xy_error_m"] is not None and c["maximum_xy_error_m"] <= t for c in candidates),
            "unavailable_count": sum(c["maximum_xy_error_m"] is None for c in candidates),
        "denominator": len(candidates)} for t in (.001, .01, .1)}} for name, candidates in groups.items()}


def normalized_gpu_uuid(value):
    text = value.hex() if isinstance(value, bytes) else str(value).lower()
    if text.startswith("gpu-"):
        text = text[4:]
    text = text.replace("-", "")
    require(len(text) == 32 and all(c in "0123456789abcdef" for c in text), "unverifiable CUDA device UUID")
    return text


def persisted_reports(output):
    # HH_260906 - A failed write can leave a partial final line; count actual complete records and retain the tail unchanged.
    path = output / "per_anchor.jsonl"
    if not path.exists():
        return [], {"complete_json_line_count": 0, "incomplete_tail_bytes": 0, "detail_file_created": False}
    payload = raw.contract._read_regular_file_bounded(path, 256 * 1024 * 1024, "persisted probe details")
    rows, tail = [], 0
    lines = payload.splitlines(keepends=True)
    for index, line in enumerate(lines):
        if not line.endswith(b"\n"):
            require(index == len(lines) - 1, "incomplete internal detail line")
            tail = len(line); break
        record = json.loads(line)
        require(record["anchor_index"] == len(rows), "persisted detail rows are missing, duplicated or reordered")
        rows.append(record)
    return rows, {"complete_json_line_count": len(rows), "incomplete_tail_bytes": tail, "detail_file_created": True}


def run_probe(input_root, output, expected_manifest_sha, expected_script_sha, device, max_wall_seconds):
    require(raw.scalar.sha(Path(__file__)) == expected_script_sha, "probe script SHA mismatch")
    require(not isinstance(max_wall_seconds, bool) and math.isfinite(max_wall_seconds) and max_wall_seconds > 0, "finite positive wall budget required")
    require(device in ("cpu", "cuda:0"), "only CPU or the single assigned GPU0 is allowed")
    if device == "cuda:0":
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == ASSIGNED_GPU_UUID
            and torch.cuda.is_available() and torch.cuda.device_count() == 1,
            "CUDA probe requires only the assigned physical GPU0 UUID visible")
        properties = torch.cuda.get_device_properties(0)
        require(normalized_gpu_uuid(getattr(properties, "uuid", None)) == normalized_gpu_uuid(ASSIGNED_GPU_UUID),
            "visible logical cuda:0 is not the assigned physical GPU0 UUID")
    else:
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "CPU diagnostics must hide CUDA explicitly")
    torch.set_num_threads(4)
    output, sources, ledger = fresh(output, [input_root]), source_identity(), {}
    manifest_bytes = read_bytes(input_root, "manifest.json", ledger)
    require(raw.digest(manifest_bytes) == expected_manifest_sha, "probe input manifest SHA mismatch")
    manifest = json.loads(manifest_bytes)
    require(manifest["schema"] == "portable_e2e.decoder_oracle_inputs.v1" and manifest["input_count"] == 1337
        and manifest["counts_by_trial"] == EXPECTED_COUNTS and manifest["plan"] == PLAN and manifest["scope"] == SCOPE
        and manifest["source_identity"] == sources, "input provenance/configuration differs from frozen probe")
    data = read_bytes(input_root, "inputs.jsonl", ledger)
    require(raw.digest(data) == manifest["inputs_sha256"], "probe tensor input SHA mismatch")
    rows = [json.loads(line) for line in data.splitlines()]
    require(len(rows) == 1337 and [r["anchor_index"] for r in rows] == list(range(1337))
        and len({r["sample_id"] for r in rows}) == 1337 and Counter(r["trial_id"] for r in rows) == EXPECTED_COUNTS,
        "probe input sample denominator changed")
    output.mkdir(parents=True, exist_ok=False)
    started, stop_state, old_handlers = time.monotonic(), {"signal": None}, {}
    def stop_signal(number, _frame):
        stop_state["signal"] = signal.Signals(number).name
    for number in (signal.SIGINT, signal.SIGTERM):
        old_handlers[number] = signal.signal(number, stop_signal)
    reports, batch_reports, failure, current_batch = [], [], None, None
    try:
        write_json(output / "started.json", {"schema": "portable_e2e.decoder_oracle_execution.v1", "status": "RUNNING",
            "started_at_utc": datetime.now(timezone.utc).isoformat(), "source_identity": sources, "input_pins": ledger,
            "plan": PLAN, "scope": SCOPE, "max_wall_seconds": max_wall_seconds, "device": device,
            "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"), "torch_version": torch.__version__,
            "assigned_gpu_uuid": ASSIGNED_GPU_UUID if device == "cuda:0" else None,
            "visible_device_uuid_verified": device == "cuda:0",
            "wall_budget_notice": "Cooperative iteration deadline; final decode, gate, serialization and provenance postchecks need margin. An external hard process timeout remains necessary."})
        with (output / "optimization_history.jsonl").open("x") as history_file, (output / "per_anchor.jsonl").open("x") as detail_file:
            for start in range(0, len(rows), 256):
                if stop_state["signal"] or time.monotonic() - started >= max_wall_seconds:
                    failure = "deadline_or_signal_before_batch"; break
                batch = rows[start:start + 256]
                current_batch = {"batch_start_anchor_index": start, "batch_size": len(batch), "iterations_completed": 0}
                def progress(record):
                    current_batch["iterations_completed"] = record["optimizer_steps_completed"]
                    history_file.write(encoded({"batch_start_anchor_index": start, **record}).decode() + "\n")
                    history_file.flush()
                result = optimize_batch(batch, device,
                    lambda: bool(stop_state["signal"]) or time.monotonic() - started >= max_wall_seconds, progress)
                detail = candidate_reports(batch, result)
                for row in detail:
                    detail_file.write(encoded(row).decode() + "\n")
                    detail_file.flush(); reports.append(row)
                batch_reports.append({key: result[key] for key in ("status", "failure", "iterations_completed", "initial_latent_sha256", "terminal_values_finite")}
                    | {"batch_start_anchor_index": start, "batch_size": len(batch), "reported_anchor_count": len(detail),
                       "unreported_anchor_indices": [] if detail else [r["anchor_index"] for r in batch]})
                current_batch = None
                if result["status"] != "COMPLETE":
                    failure = result["failure"] or "incomplete_batch"; break
    except (Exception, KeyboardInterrupt) as error:
        failure = f"{type(error).__name__}: {error}"
        if current_batch is not None:
            batch_reports.append({**current_batch, "status": "FAILED", "failure": failure, "reported_anchor_count": 0,
                "unreported_anchor_indices": list(range(current_batch["batch_start_anchor_index"],
                    current_batch["batch_start_anchor_index"] + current_batch["batch_size"])),
                "notice": "Last completed iteration is from the append-only progress journal; no final predictions could be produced for this batch."})
    finally:
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
    try:
        reports, detail_integrity = persisted_reports(output)
        if detail_integrity["incomplete_tail_bytes"]:
            failure = failure or "incomplete_final_detail_line"
        for batch in batch_reports:
            indices = set(range(batch["batch_start_anchor_index"], batch["batch_start_anchor_index"] + batch["batch_size"]))
            present = {r["anchor_index"] for r in reports} & indices
            batch["reported_anchor_count"] = len(present)
            batch["unreported_anchor_indices"] = sorted(indices - present)
    except Exception as error:
        detail_integrity, failure = {"failure": str(error)}, f"detail_postcheck: {error}"
    try:
        recheck(input_root, ledger)
        require(source_identity() == sources, "probe source changed during optimization")
        immutable = True
    except Exception as error:
        immutable, failure = False, f"source_or_input_postcheck: {error}"
    complete = failure is None and immutable and len(reports) == 1337 and len(batch_reports) == 6 and all(b["status"] == "COMPLETE" for b in batch_reports)
    summary = {"schema": "portable_e2e.decoder_oracle_probe.v1", "status": "COMPLETE_NOT_ADMITTED" if complete else "PARTIAL_OR_FAILED_NOT_ADMITTED",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(), "elapsed_wall_seconds": time.monotonic() - started,
        "expected_anchor_count": 1337, "reported_anchor_count": len(reports), "unreported_anchor_count": 1337 - len(reports),
        "completed_batches": sum(b["status"] == "COMPLETE" for b in batch_reports),
        "batches": batch_reports, "failure": failure, "received_signal": stop_state["signal"], "source_and_input_postcheck_pass": immutable,
        "detail_publication_integrity": detail_integrity,
        "source_identity": sources, "input_pins": ledger, "plan": PLAN, "scope": SCOPE,
        "residual_summaries": summarize_reports(reports),
        "reported_anchor_counts_by_optimization_status": dict(Counter(r["optimization_status"] for r in reports)),
        "interpretation": "Future-aware numerical optimization of independent latent controls, not causal learned performance or a certified global optimum/lower bound. Each initialization and every raw failure is retained; best-of-six is an oracle descriptive summary, not model promotion or admission. Current labels and gates are unchanged."}
    write_json(output / "summary.json", summary); write_sums(output)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    commands = parser.add_subparsers(dest="command", required=True)
    extract_parser = commands.add_parser("extract", allow_abbrev=False)
    extract_parser.add_argument("--diagnostic-root", type=Path, required=True)
    extract_parser.add_argument("--raw-campaign-root", type=Path, required=True)
    extract_parser.add_argument("--expected-summary-sha256", required=True)
    probe_parser = commands.add_parser("probe", allow_abbrev=False)
    probe_parser.add_argument("--input-root", type=Path, required=True)
    probe_parser.add_argument("--expected-manifest-sha256", required=True)
    probe_parser.add_argument("--device", choices=("cpu", "cuda:0"), required=True)
    probe_parser.add_argument("--max-wall-seconds", type=float, required=True)
    for child in (extract_parser, probe_parser):
        child.add_argument("--output-dir", type=Path, required=True)
        child.add_argument("--expected-script-sha256", required=True)
    args = parser.parse_args(argv)
    if args.command == "extract":
        extract(args.diagnostic_root, args.raw_campaign_root, args.output_dir, args.expected_summary_sha256, args.expected_script_sha256)
        return 0
    result = run_probe(args.input_root, args.output_dir, args.expected_manifest_sha256, args.expected_script_sha256, args.device, args.max_wall_seconds)
    return 0 if result["status"] == "COMPLETE_NOT_ADMITTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
