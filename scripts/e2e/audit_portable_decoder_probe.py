#!/usr/bin/env python3
"""HH_260906 - Independently audit the fixed future-aware decoder probe without optimization or data admission."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
from types import SimpleNamespace

import torch

from portable_e2e import contract, model, runtime_contract as gate
from scripts.e2e import summarize_carla_goal_stop_trials as evidence

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "3535d972fb2d70ed68459394c4445493280eb2b0"
PROBE_SHA = "dce8ea20a42c16a3b5591f2547bcd6ff95a331be0f367273ca11570ac84cbf8a"
OWNER_SHA = "7bce5d13dba2302e03927e0113f6cbf7097898e85da3d8f4b1aa39953b6db396"
SHARED_OWNER_SHA = "743d25ba10a1fbb856535ed2711fab5f0ffbe89ba990718a860fb9119ccbd18a"
INPUT_SHA = "9b5a6bfc57d5916a8461be42957c3963a4d4a223cb284a6785e19b9720431fe8"
MODEL_SHA = "b72c0fcbaf558254a3e7b02aa90406ed157724a07f63d0ef9f46d0d444f92fc4"
GATE_SHA = "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
TOLERANCE = {"absolute": 1e-5, "relative": 1e-5}
require = evidence.require


def sha(path):
    return evidence.sha(Path(path))


def close(observed, expected):
    require(isinstance(observed, (int, float)) and not isinstance(observed, bool) and math.isfinite(observed)
        and math.isclose(observed, expected, abs_tol=1e-5, rel_tol=1e-5), "stored numerical metric differs from independent reconstruction")


def f32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


def initial_objective_check(observed, reconstructed):
    # HH_260906 - Preserve cross-device disagreements without relaxing the preregistered tolerance or claiming their cause.
    require(all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in (observed, reconstructed)), "nonfinite initial objective")
    matches = math.isclose(observed, reconstructed, abs_tol=TOLERANCE["absolute"], rel_tol=TOLERANCE["relative"])
    return {"status": "CPU_FORWARD_MATCH_WITH_PREDECLARED_TOLERANCE" if matches else "INITIAL_FORWARD_UNVERIFIED",
        "recorded_gpu_objective_m2": observed, "reconstructed_cpu_objective_m2": reconstructed,
        "absolute_difference_m2": abs(observed - reconstructed), "comparison_tolerance": TOLERANCE}


def read(root, relative, ledger, *, lines=False):
    path = contract._safe_file(Path(root), relative, "independent probe input")
    data = contract._read_regular_file_bounded(path, 256 * 1024 * 1024, "independent probe input")
    ledger[relative] = {"sha256": hashlib.sha256(data).hexdigest(), "size_bytes": len(data)}
    return [json.loads(line) for line in data.splitlines()] if lines else json.loads(data)


def recheck(root, ledger):
    for path, pin in ledger.items():
        require(sha(Path(root) / path) == pin["sha256"], "audit input changed during verification")


def source_pins():
    require(sha(ROOT / "portable_e2e/model.py") == MODEL_SHA and sha(ROOT / "portable_e2e/runtime_contract.py") == GATE_SHA,
        "unreviewed decoder or gate in independent audit")
    return {path: sha(ROOT / path) for path in ("scripts/e2e/audit_portable_decoder_probe.py", "portable_e2e/model.py",
        "portable_e2e/runtime_contract.py", "portable_e2e/contract.py", "portable_e2e/dataset.py", "scripts/e2e/summarize_carla_goal_stop_trials.py")}


def historical_source_proof(identity):
    # HH_260906 - Legitimate later reader fixes must not rewrite historical execution provenance; Git blob reads cannot fetch.
    expected_paths = {"scripts/e2e/probe_portable_decoder_representability.py", "portable_e2e/model.py", "portable_e2e/runtime_contract.py",
        "portable_e2e/contract.py", "portable_e2e/dataset.py", "portable_e2e/torch_dataset.py",
        "scripts/e2e/audit_carla_raw_pre_admission.py", "scripts/e2e/prepare_carla_common10_dataset.py",
        "scripts/e2e/summarize_carla_goal_stop_trials.py", "portable_e2e/config/common_10hz_v1.contract.json"}
    require(set(identity["files"]) == expected_paths, "historical execution source inventory differs")
    env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    for path, expected in identity["files"].items():
        contents = subprocess.check_output(["git", "-c", "protocol.allow=never", "show", f"{COMMIT}:{path}"], cwd=ROOT, env=env, timeout=10)
        require(hashlib.sha256(contents).hexdigest() == expected, "historical executed source differs from exact recorded Git blob")
    for path, expected in (("scripts/e2e/run_owned_portable_decoder_probe.py", OWNER_SHA),
        ("scripts/e2e/run_portable_training_campaign.py", SHARED_OWNER_SHA)):
        contents = subprocess.check_output(["git", "-c", "protocol.allow=never", "show", f"{COMMIT}:{path}"], cwd=ROOT, env=env, timeout=10)
        require(hashlib.sha256(contents).hexdigest() == expected, "historical owned execution source differs")
    return {"source_commit": COMMIT, "verified_file_count": len(expected_paths), "source_sha256": identity["files"],
        "owner_source_sha256": {"scripts/e2e/run_owned_portable_decoder_probe.py": OWNER_SHA,
            "scripts/e2e/run_portable_training_campaign.py": SHARED_OWNER_SHA},
        "network_fetch_allowed": False, "current_reader_sha_may_differ_from_historical_execution": True}


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    def q(fraction):
        index = (len(values) - 1) * fraction
        lo = int(index)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)
    return {"count": len(values), "mean": sum(values) / len(values), "min": values[0],
        "p50": q(.5), "p90": q(.9), "p99": q(.99), "max": values[-1]}


def verify_inputs(input_root, raw_root):
    # HH_260906 - Verify all metadata and independently reconstruct ten declared phase examples, without image reads.
    ledger, raw_ledgers = {}, {}
    manifest = read(input_root, "manifest.json", ledger)
    require(ledger["manifest.json"]["sha256"] == INPUT_SHA, "wrong frozen decoder input manifest")
    rows = read(input_root, "inputs.jsonl", ledger, lines=True)
    require(ledger["inputs.jsonl"]["sha256"] == manifest["inputs_sha256"]
        and manifest["source_identity"]["files"]["scripts/e2e/probe_portable_decoder_representability.py"] == PROBE_SHA,
        "input or extractor source binding mismatch")
    require(len(rows) == 1337 and [r["anchor_index"] for r in rows] == list(range(1337))
        and len({r["sample_id"] for r in rows}) == 1337 and all(r["valid_mask"] == [True] * 64 for r in rows), "input denominator or mask changed")
    examples, error, cohorts, contexts = [], 0.0, {}, {}
    for name, count in (("run_001", 671), ("run_002", 666)):
        group = [r for r in rows if r["trial_id"] == name]
        require(len(group) == count and Counter(r["capture_phase"] for r in group) == {"stationary_warmup": 35, "driving": count - 35}, "warmup/driving denominator changed")
        root, pins = Path(raw_root) / "town07_straight_calibration" / name, manifest["original_capture_metadata_pins"][name]
        for path, pin in pins.items():
            require(not path.lower().endswith((".jpg", ".jpeg")) and sha(root / path) == pin["sha256"], "original non-image metadata binding mismatch")
        states = read(root, "episode/states.jsonl", {}, lines=True)
        stamps = [round(s["timestamp"] * 1e9) for s in states]
        by_frame = {s["frame"]: s for s in states}
        for row in group:
            native = by_frame[row["frame"]]
            require(row["anchor_timestamp_ns"] == round(native["timestamp"] * 1e9) and row["current_vx_mps"] == native["vx"], "input current state differs from raw")
            tags = ["all", row["capture_phase"]]
            if row["capture_phase"] == "driving" and 7.8 <= math.hypot(native["vx"], native["vy"]) <= 8.2:
                tags.append("cruise_7p8_to_8p2_mps")
            if native["goal_stop"].get("pilot_state") == "coast_low":
                tags.append("governor_coast_low")
            tags.append("raw_curvature_failed" if row["original_raw_violating_step_counts"]["xy_curvature"] else "raw_curvature_clear")
            cohorts[row["sample_id"]] = tags
            contexts[row["sample_id"]] = {"frame": row["frame"], "current_vx_mps": native["vx"],
                "current_planar_speed_mps": math.hypot(native["vx"], native["vy"]),
                "pilot_state": native["goal_stop"].get("pilot_state"), "route_progress_m": native["route_progress_m"]}
        for index in (0, 34, 35, count // 2, count - 1):
            row = group[index]; anchor = by_frame[row["frame"]]
            for k in range(64):
                stamp = row["anchor_timestamp_ns"] + (k + 1) * 100_000_000
                j = bisect_right(stamps, stamp)
                if stamps[j - 1] == stamp:
                    future = states[j - 1]
                else:
                    require(0 < j < len(stamps), "future lacks a native bracket")
                    a, b = states[j - 1], states[j]
                    require(b["frame"] == a["frame"] + 1, "future bracket crosses a native gap")
                    ratio = (stamp - stamps[j - 1]) / (stamps[j] - stamps[j - 1])
                    future = {key: a[key] + ratio * (b[key] - a[key]) for key in ("x", "y", "vx", "vy")}
                    delta = math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
                    angle = a["yaw"] + ratio * delta
                    future["yaw"] = math.atan2(math.sin(angle), math.cos(angle))
                c, s = math.cos(anchor["yaw"]), math.sin(anchor["yaw"])
                dx, dy = future["x"] - anchor["x"], future["y"] - anchor["y"]
                actual = [c * dx + s * dy, -s * dx + c * dy, math.hypot(future["vx"], future["vy"]),
                    math.atan2(math.sin(future["yaw"] - anchor["yaw"]), math.cos(future["yaw"] - anchor["yaw"]))]
                expected = [*row["target_xy"][k], row["target_speed"][k], row["target_yaw"][k]]
                error = max(error, *(abs(a - b) for a, b in zip(actual, expected)))
            examples.append({"trial": name, "within_trial_index": index, "frame": row["frame"], "phase": row["capture_phase"]})
        raw_ledgers[name] = pins
    require(error <= 1e-12, "independent raw future reconstruction differs")
    require(sum("raw_curvature_failed" in tags for tags in cohorts.values()) == 299, "original curvature failure denominator changed")
    result = {"status": "VERIFIED", "input_count": 1337, "metadata_sha_count": sum(len(v) for v in raw_ledgers.values()),
        "reconstructed_anchor_count": 10, "reconstructed_point_count": 640, "maximum_reconstruction_difference": error,
        "phase_examples": examples, "cohort_counts": dict(Counter(tag for tags in cohorts.values() for tag in tags)),
        "cohort_notice": "Cohorts overlap; cruise is actual anchor planar speed within [7.8,8.2] m/s while driving, coast is the recorded governor state. No sample is excluded.",
        "jpeg_payload_read": False, "model_optimization": False, "pins": ledger, "original_metadata_pins": raw_ledgers,
        "anchor_contexts": contexts}
    return result, rows, cohorts, manifest


def candidate_metrics(item, candidate):
    # HH_260906 - GPU outputs are float32 values; targets are rounded to the producer's tensor precision before independent arithmetic.
    xy, speed = candidate["decoder_xy"], candidate["decoder_speed"]
    require(len(xy) == len(speed) == 64 and all(len(p) == 2 for p in xy), "candidate point denominator differs")
    target = [[f32(v) for v in p] for p in item["target_xy"]]
    target_speed = [f32(v) for v in item["target_speed"]]
    errors = [math.dist(a, b) for a, b in zip(xy, target)]
    speed_errors = [abs(a - b) for a, b in zip(speed, target_speed)]
    require(all(math.isfinite(v) for v in errors + speed_errors), "nonfinite candidate cannot claim complete numerical evidence")
    result = {"ade_m": sum(errors) / 64, "fde_m": errors[-1], "maximum_xy_error_m": max(errors),
        "speed_rmse_mps": math.sqrt(sum(v * v for v in speed_errors) / 64), "maximum_speed_error_mps": max(speed_errors),
        "final_objective_m2": sum(v * v for v in errors) / 64 + sum(v * v for v in speed_errors) / 64}
    for key, value in result.items():
        close(candidate[key], value)
    require(len(candidate["xy_error_by_step_m"]) == len(candidate["speed_absolute_error_by_step_mps"]) == 64, "residual point denominator differs")
    for a, b in zip(candidate["xy_error_by_step_m"], errors): close(a, b)
    for a, b in zip(candidate["speed_absolute_error_by_step_mps"], speed_errors): close(a, b)
    return result


def validate_latents(candidate):
    values = candidate["optimized_raw_latents"]
    require(isinstance(values, list) and len(values) == 64 and all(isinstance(p, list) and len(p) == 2
        and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in p) for p in values),
        "optimized latent record incomplete or nonfinite")


def check_history(history, batches):
    require(len(history) == 6 * 513 and len(batches) == 6, "fixed optimization history denominator differs")
    for b, start in enumerate(range(0, 1337, 256)):
        block = history[b * 513:(b + 1) * 513]
        require([r["iteration"] for r in block] == list(range(513))
            and all(r["batch_start_anchor_index"] == start and r["optimizer_steps_completed"] == r["iteration"] for r in block), "history missing, duplicated or reordered")
        require(batches[b]["batch_start_anchor_index"] == start and batches[b]["batch_size"] == min(256, 1337 - start)
            and batches[b]["iterations_completed"] == 512 and batches[b]["status"] == "COMPLETE", "batch budget differs")
        require(all(len(r["mean_objective_by_initialization"]) == 6
            and all(math.isfinite(v) and v >= 0 for v in r["mean_objective_by_initialization"])
            and math.isfinite(r["maximum_objective"]) for r in block), "nonfinite or missing history values")


def stop_extent_witness(item, candidate):
    # HH_260906 - Reproduce the unchanged terminal extent condition, not a proposed exception or automatic candidate substitution.
    speeds, points = candidate["decoder_speed"], candidate["decoder_xy"]
    previous_point, previous_speed, extent, increases = (0., 0.), max(0., item["current_vx_mps"]), 0.0, []
    for index, (point, speed) in enumerate(zip(points, speeds)):
        extent += math.dist(previous_point, point)
        if speed > previous_speed + 1e-9:
            increases.append({"point_index": index, "entry_speed_mps": previous_speed, "predicted_speed_mps": speed})
        previous_point, previous_speed = point, speed
    cfg = gate.RuntimeGateConfig()
    dynamic_stop = not increases and speeds[-1] <= cfg.stationary_claim_speed_epsilon_mps
    raw_future = {}
    if "target_speed" in item and "target_xy" in item:
        previous = (0., 0.); raw_extent = 0.
        for point in item["target_xy"]:
            raw_extent += math.dist(previous, point); previous = point
        raw_future = {"raw_future_maximum_planar_speed_mps": max(item["target_speed"]),
            "raw_future_final_planar_speed_mps": item["target_speed"][-1], "raw_future_path_extent_m": raw_extent}
    return {"current_vx_mps": item["current_vx_mps"], "maximum_predicted_speed_mps": max(speeds), "final_predicted_speed_mps": speeds[-1],
        **raw_future,
        "integrated_speed_distance_m": sum(speeds) * .1, "predicted_path_extent_m": extent,
        "maximum_predicted_radius_m": max(math.hypot(*p) for p in points),
        "minimum_planar_extent_m": cfg.minimum_planar_extent_m, "stationary_speed_tolerance_mps": cfg.stationary_speed_tolerance_mps,
        "stationary_claim_speed_epsilon_mps": cfg.stationary_claim_speed_epsilon_mps,
        "monotonic_nonincreasing_speed": not increases, "speed_increase_count": len(increases), "speed_increases": increases,
        "dynamically_consistent_stop": dynamic_stop,
        "insufficient_extent_condition": extent < cfg.minimum_planar_extent_m and max(speeds) > cfg.stationary_speed_tolerance_mps and not dynamic_stop}


def reconstruct_initial(batch, expected_sha, config):
    # HH_260906 - Forward-only CPU verification; a different-version RNG hash is UNVERIFIED, never silently accepted.
    latents = []
    for item in batch:
        rng = torch.Generator(device="cpu").manual_seed(20260908 + item["anchor_index"])
        values = torch.zeros(6, 64, 2, dtype=torch.float32)
        values[1:] = torch.randn((5, 64, 2), generator=rng, dtype=torch.float32) * .03
        latents.append(values)
    initial = torch.stack(latents)
    actual_sha = hashlib.sha256(initial.numpy().tobytes()).hexdigest()
    if actual_sha != expected_sha:
        return {"status": "UNVERIFIED_INITIALIZATION_VERSION_DIFFERENCE", "local_sha256": actual_sha,
            "recorded_sha256": expected_sha}, None
    history = torch.zeros(len(batch), 10, len(model.FEATURE_NAMES))
    route = torch.zeros(len(batch), 128, 2)
    mask = torch.zeros(len(batch), 128, dtype=torch.bool)
    for i, item in enumerate(batch):
        history[i, -1, 1] = item["current_vx_mps"]
        route[i, :len(item["route_xy"])]=torch.tensor(item["route_xy"], dtype=torch.float32)
        mask[i, :len(item["route_xy"])]=True
    with torch.no_grad():
        xy, speed = model.PerspectiveTrajectoryModel._decode_physical_v1(SimpleNamespace(config=config), initial, history, route, mask)
    return {"status": "INITIAL_LATENT_SHA_MATCH", "sha256": actual_sha,
        "cpu_initial_forward_only": True, "torch_version": torch.__version__}, (xy.tolist(), speed.tolist())


def summarize(rows, cohorts):
    result = {}
    for cohort in sorted({tag for tags in cohorts.values() for tag in tags}):
        subset = [r for r in rows if cohort in cohorts[r["sample_id"]]]
        groups = {f"initialization_{k}": [r["candidates"][k] for r in subset] for k in range(6)}
        groups["future_aware_best_objective_of_six"] = [r["candidates"][r["best_objective_index"]] for r in subset]
        result[cohort] = {name: {"anchor_count": len(values), "metrics": {key: distribution([v[key] for v in values])
            for key in ("ade_m", "fde_m", "maximum_xy_error_m", "speed_rmse_mps", "maximum_speed_error_mps", "final_objective_m2")},
            "runtime_gate_counts": dict(Counter(v["runtime_gate_status"] for v in values)),
            "convergence_warning_count": sum(v["convergence_warning"] for v in values),
            "descriptive_max_xy_error_coverage": {str(t): {"within_count": sum(v["maximum_xy_error_m"] <= t for v in values), "denominator": len(values)} for t in (.001, .01, .1)}}
            for name, values in groups.items()}
    return result


def audit(owned_root, input_root, raw_root):
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and not torch.cuda.is_initialized(), "independent audit must remain CPU-only")
    torch.set_num_threads(4)
    sources, ledger = source_pins(), {}
    verification, inputs, cohorts, manifest = verify_inputs(input_root, raw_root)
    started = read(owned_root, "owner_started.json", ledger)
    owner = read(owned_root, "owner_result.json", ledger)
    plan = started["plan"]
    require(plan["source_commit"] == COMMIT and plan["probe_sha256"] == PROBE_SHA
        and plan["runner_sha256"] == OWNER_SHA and plan["shared_runner_sha256"] == SHARED_OWNER_SHA
        and plan["input_manifest_sha256"] == INPUT_SHA and plan["cooperative_wall_seconds"] == 3600
        and plan["outer_wall_seconds"] == 3900 and plan["gpu_uuid"] == GPU_UUID
        and started["gpu0_idle_checked"] is True, "unreviewed owned execution plan")
    child = read(owned_root, "owner_child.json", ledger)
    require(type(child["pid"]) is int and child["pid"] > 1 and child["pgid"] == child["pid"], "owned process identity incomplete")
    execution = read(owned_root, "probe/started.json", ledger)
    require(execution["device"] == "cuda:0" and execution["cuda_visible_devices"] == GPU_UUID
        and execution["assigned_gpu_uuid"] == GPU_UUID and execution["visible_device_uuid_verified"] is True
        and execution["scope"] == manifest["scope"] and execution["max_wall_seconds"] == 3600,
        "recorded device or optimization-only execution scope differs")
    require(owner["child_exited"] is True and owner["source_and_plan_postcheck_pass"] is True
        and owner["training_data_approved"] is False and owner["causal_model_training"] is False, "owner cleanup/source/scope incomplete")
    report = read(owned_root, "probe/summary.json", ledger)
    require(ledger["probe/summary.json"]["sha256"] == owner["probe_summary_sha256"], "owner summary SHA mismatch")
    require(report["plan"] == manifest["plan"] and report["scope"] == manifest["scope"]
        and report["source_identity"] == manifest["source_identity"], "frozen probe plan/source/scope differs from input")
    require(execution["plan"] == report["plan"] and execution["source_identity"] == report["source_identity"]
        and execution["input_pins"] == verification["pins"], "started execution input/source/plan differs from frozen result")
    historical = historical_source_proof(report["source_identity"])
    detail = read(owned_root, "probe/per_anchor.jsonl", ledger, lines=True)
    history = read(owned_root, "probe/optimization_history.jsonl", ledger, lines=True)
    sums_path = Path(owned_root) / "probe/SHA256SUMS"
    require(sums_path.is_file() and not sums_path.is_symlink(), "probe hash manifest missing")
    named_outputs = []
    for line in sums_path.read_text().splitlines():
        expected, name = line.split("  ")
        require(Path(name).name == name and sha(Path(owned_root) / "probe" / name) == expected, "probe output hash manifest mismatch")
        named_outputs.append(name)
    require(len(named_outputs) == 4 and set(named_outputs) == {"started.json", "summary.json", "per_anchor.jsonl", "optimization_history.jsonl"},
        "probe output hash manifest omits or repeats a required artifact")
    ledger["probe/SHA256SUMS"] = {"sha256": sha(sums_path), "size_bytes": sums_path.stat().st_size}
    require(owner["status"] == report["status"] == "COMPLETE_NOT_ADMITTED" and owner["returncode"] == 0
        and report["source_and_input_postcheck_pass"] is True and report["reported_anchor_count"] == 1337
        and report["unreported_anchor_count"] == 0 and report["detail_publication_integrity"]["incomplete_tail_bytes"] == 0,
        "PARTIAL/FAILED input retained privately; cannot publish a complete representability comparison")
    check_history(history, report["batches"])
    require(len(detail) == 1337 and [r["anchor_index"] for r in detail] == list(range(1337)), "result anchor denominator differs")
    reconstructed, initial_checks, gate_failures = [], [], []
    cfg = model.ModelConfig.from_mapping(manifest["source_identity"]["model_config"])
    require(cfg.to_dict() == model.ModelConfig(model_id=model.PHYSICAL_MODEL_ID, maximum_step_m=1.).to_dict(), "nondefault physical decoder config")
    for b, start in enumerate(range(0, 1337, 256)):
        batch = inputs[start:start + 256]
        initial_proof, initial = reconstruct_initial(batch, report["batches"][b]["initial_latent_sha256"], cfg)
        initial_checks.append(initial_proof)
        for offset, item in enumerate(batch):
            stored = detail[start + offset]
            require(stored["sample_id"] == item["sample_id"] and stored["trial_id"] == item["trial_id"]
                and stored["capture_phase"] == item["capture_phase"] and stored["original_raw_violating_step_counts"] == item["original_raw_violating_step_counts"]
                and stored["iterations_completed"] == 512 and stored["optimization_status"] == "COMPLETE"
                and len(stored["candidates"]) == 6, "result identity, original failures or fixed budget changed")
            candidates = []
            for k, candidate in enumerate(stored["candidates"]):
                require(candidate["initialization_index"] == k and candidate["numeric_status"] == "FINITE", "initialization missing, reordered or nonfinite")
                validate_latents(candidate)
                metrics = candidate_metrics(item, candidate)
                logits = [float(j == k) for j in range(6)]
                try:
                    gate.validate_and_select_trajectory([c["decoder_xy"] for c in stored["candidates"]],
                        [c["decoder_speed"] for c in stored["candidates"]], logits, current_speed_mps=item["current_vx_mps"])
                    gate_status, reason = "PASS", None
                except model.ContractError as error:
                    gate_status, reason = "FAIL", str(error)
                require(candidate["output_runtime_gate"] == {"status": gate_status, "reason": reason}, "stored runtime verdict differs")
                if gate_status == "FAIL":
                    witness = stop_extent_witness(item, candidate) if reason == "selected trajectory has insufficient planar extent" else None
                    require(witness is None or witness["insufficient_extent_condition"] is True, "extent gate witness differs")
                    gate_failures.append({"sample_id": item["sample_id"], "anchor_index": item["anchor_index"],
                        "initialization_index": k, "reason": reason, "raw_anchor_context": verification["anchor_contexts"][item["sample_id"]],
                        "stop_extent_witness": witness})
                window, end = candidate["last_window_objective_m2"], candidate["final_objective_m2"]
                change = abs(end - window)
                warning = change > 1e-6 and change / max(abs(window), 1e-12) > .01
                close(candidate["last_window_absolute_change_m2"], change)
                require(candidate["convergence_warning"] is warning, "convergence warning differs")
                initial_check = {"status": "INITIAL_LATENT_SHA_UNVERIFIED"}
                if initial is not None:
                    xy0, speed0 = initial[0][offset][k], initial[1][offset][k]
                    target = [[f32(v) for v in p] for p in item["target_xy"]]
                    initial_objective = sum(math.dist(a, target[j]) ** 2 + (speed0[j] - f32(item["target_speed"][j])) ** 2 for j, a in enumerate(xy0)) / 64
                    initial_check = initial_objective_check(candidate["initial_objective_m2"], initial_objective)
                candidates.append({**metrics, "initialization_index": k, "runtime_gate_status": gate_status,
                    "runtime_gate_reason": reason, "convergence_warning": warning, "initial_objective_verification": initial_check})
            best = min(range(6), key=lambda k: stored["candidates"][k]["final_objective_m2"])
            require(best == stored["oracle_objective_minimum_initialization_index"], "stored best-of-six index differs")
            reconstructed.append({"sample_id": item["sample_id"], "anchor_index": item["anchor_index"], "trial_id": item["trial_id"],
                "cohorts": cohorts[item["sample_id"]], "original_raw_violating_step_counts": item["original_raw_violating_step_counts"],
                "best_objective_index": best, "candidate_runtime_pass_count": sum(c["runtime_gate_status"] == "PASS" for c in candidates),
                "best_objective_runtime_gate_status": candidates[best]["runtime_gate_status"], "candidates": candidates})
        for k in range(6):
            for iteration, field in ((0, "initial_objective_m2"), (448, "last_window_objective_m2"), (512, "final_objective_m2")):
                expected = sum(r["candidates"][k][field] for r in detail[start:start + len(batch)]) / len(batch)
                close(history[b * 513 + iteration]["mean_objective_by_initialization"][k], expected)
    initial_discrepancies = [{"anchor_index": r["anchor_index"], "initialization_index": c["initialization_index"],
        **c["initial_objective_verification"]} for r in reconstructed for c in r["candidates"]
        if c["initial_objective_verification"]["status"] != "CPU_FORWARD_MATCH_WITH_PREDECLARED_TOLERANCE"]
    initial_verified = not initial_discrepancies and all(p["status"] == "INITIAL_LATENT_SHA_MATCH" for p in initial_checks)
    recheck(owned_root, ledger); recheck(input_root, verification["pins"])
    for name, pins in verification["original_metadata_pins"].items():
        recheck(Path(raw_root) / "town07_straight_calibration" / name, pins)
    require(source_pins() == sources and not torch.cuda.is_initialized(), "audit source/CPU scope changed")
    summary = {"schema": "portable_e2e.independent_decoder_oracle_audit.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "VERIFIED_NOT_ADMITTED" if initial_verified else "INITIALIZATION_UNVERIFIED_NOT_ADMITTED",
        "executed_probe_commit": COMMIT, "executed_probe_sha256": PROBE_SHA, "auditor_source_sha256": sources,
        "auditor_source_notice": "Executed later from the local working-tree script; its recorded SHA is authoritative. The auditor is not claimed to belong to the executed probe commit.",
        "executed_probe_environment": {"owner_started_at_utc": started["started_at_utc"], "owner_completed_at_utc": owner["completed_at_utc"],
            "elapsed_wall_seconds": owner["elapsed_wall_seconds"], "torch_version": execution["torch_version"], "device": execution["device"]},
        "historical_execution_source_proof": historical,
        "comparison_tolerance": TOLERANCE, "input_verification": verification, "owned_artifact_pins": ledger,
        "anchor_count": 1337, "initialization_count": 6, "candidate_count": 8022, "candidate_point_count": 513408,
        "history_record_count": 3078, "optimizer_step_count": 3072, "initial_forward_checks": initial_checks,
        "initial_forward_unverified_count": len(initial_discrepancies), "initial_forward_unverified_candidates": initial_discrepancies,
        "final_recorded_prediction_errors_and_gates": "VERIFIED",
        "finite_optimized_latent_scalar_count": 1026816,
        "runtime_gate_failure_count": len(gate_failures), "runtime_gate_failures": gate_failures,
        "anchors_with_at_least_one_runtime_passing_candidate": sum(r["candidate_runtime_pass_count"] > 0 for r in reconstructed),
        "anchors_with_all_six_runtime_failures": sum(r["candidate_runtime_pass_count"] == 0 for r in reconstructed),
        "best_objective_runtime_failures": [{"sample_id": r["sample_id"], "anchor_index": r["anchor_index"],
            "best_objective_index": r["best_objective_index"], "candidate_runtime_pass_count": r["candidate_runtime_pass_count"]}
            for r in reconstructed if r["best_objective_runtime_gate_status"] == "FAIL"],
        "cohort_results": summarize(reconstructed, cohorts), "scope": {"new_optimization": False, "model_parameter_training": False,
            "cpu_initial_decoder_forward_only": True, "training_data_approved": False, "labels_replaced": False, "raw_failures_removed": False,
            "jpeg_payload_read": False, "test_payload_read": False, "global_optimum_proven": False},
        "limitations": ["Future-aware best-of-six is an oracle numerical approximation, not causal driving performance, a certified optimum or an admission decision.",
            "Convergence warnings and all fixed initializations remain visible. A small last-window change also cannot certify global optimality.",
            "Initial decoder verification is CPU-only against recorded GPU objectives with predeclared rounding tolerance; final errors and gates are independently recomputed from actual recorded GPU predictions.",
            "Final optimized latent values are retained and checked finite; this audit does not replay the GPU optimization or independently establish its final decoder-forward arithmetic.",
            "Any initial forward discrepancy remains UNVERIFIED with both values; a different device/version alone does not prove its cause.",
            "Cohorts overlap and are descriptive; no raw failure, warmup anchor or future point was filtered."]}
    return summary, reconstructed, history


def write_result(output, summary, rows, history):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "create-only audit output required")
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    with (output / "per_anchor_numeric.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    (output / "input_verification.json").write_text(json.dumps(summary["input_verification"], indent=2) + "\n")
    render(output, summary, rows, history)
    readme = """# 물리 decoder 표현 가능성: 미래를 아는 수치 최적화\n\n<!-- HH_260906 - Keep decoder-only oracle approximation separate from learned performance and data admission. -->\n\n기존 comfortable_v4 두 실행의 워밍업·주행 시작점 1,337개를 모두 사용했습니다. 학습된 모델 없이 기존 물리 decoder 입력만 6개 고정 초기값에서 각각 512회 최적화한 결과입니다. **실제 자율주행 성능이나 학습 데이터 승인이 아닙니다.**\n\n모든 8,022개 후보 / 513,408개 미래 점의 오차·출력 게이트를 독립적으로 다시 계산했습니다. 원본 XY 곡률 실패 299개 시작점은 그대로 남습니다. 각 초기값과 미래를 아는 best-of-six를 구분하며, 1 mm·1 cm·10 cm는 설명용 구간이지 승인 기준이 아닙니다.\n\n![실제 고정 반복 수치 최적화 기록](01_oracle_objective_history.png)\n\n![초기값별 실제 최대 위치 오차 누적분포](02_maximum_xy_error_ecdf.png)\n\n전체 집계와 서로 겹치는 워밍업·실제 순항 속도·coast·원본 곡률 실패 그룹은 [summary.json](summary.json), 모든 후보 수치는 [per_anchor_numeric.jsonl](per_anchor_numeric.jsonl), 원본 상태 640점 대조는 [input_verification.json](input_verification.json)에 있습니다.\n\n마지막 64회 변화량에 따른 수렴 경고를 모두 표시했습니다. 경고가 없더라도 전역 최적해라는 보장은 없습니다. 초기값 CPU 재구성은 사전 선언한 절대·상대 1e-5 허용량으로 실제 GPU 기록과 비교하며, RNG SHA가 다르면 확인 불가로 남깁니다. 원본 입력·출력·코드 해시는 JSON과 [SHA256SUMS](SHA256SUMS)에 연결됩니다.\n\n원본 이미지·학습 데이터셋·모델 체크포인트는 이 폴더에 없습니다. 실제 수치 최적화 실행과 이후 독립 CPU 감사는 서로 다른 작업입니다.\n"""
    (output / "README.md").write_text(readme_report(summary) + readme)
    (output / "SHA256SUMS").write_text("".join(f"{sha(p)}  {p.name}\n" for p in sorted(output.iterdir()) if p.is_file()))


def readme_report(summary):
    # HH_260906 - Publish measured denominators, remaining verification gaps and terminal-stop witnesses without granting approval.
    all_results = summary["cohort_results"]["all"]
    best = all_results["future_aware_best_objective_of_six"]
    table = ["| 고정 초기값 | 평균 ADE (m) | 평균 FDE (m) | 전체 최대 XY 오차 (m) | 평균 속도 RMSE (m/s) | 출력 게이트 PASS/1,337 | 수렴 경고 |",
        "|---|---:|---:|---:|---:|---:|---:|"]
    for name, group in all_results.items():
        m = group["metrics"]
        label = "미래를 아는 best-of-six" if name.startswith("future_") else name
        table.append(f"| {label} | {m['ade_m']['mean']:.6f} | {m['fde_m']['mean']:.6f} | {m['maximum_xy_error_m']['max']:.6f} | "
            f"{m['speed_rmse_mps']['mean']:.6f} | {group['runtime_gate_counts'].get('PASS', 0)} | {group['convergence_warning_count']} |")
    witness_rows = ["| 시작점 (원본 frame) | 선택 초기값 | 원본 vx (m/s) | 예측 최대속도 (m/s) | 6.4초 경로 길이 (m) | 속도 증가 구간 수 | 6개 중 게이트 PASS |",
        "|---|---:|---:|---:|---:|---:|---:|"]
    for failed in summary["best_objective_runtime_failures"]:
        record = next(r for r in summary["runtime_gate_failures"] if r["anchor_index"] == failed["anchor_index"]
            and r["initialization_index"] == failed["best_objective_index"])
        w = record["stop_extent_witness"]
        witness_rows.append(f"| {failed['sample_id']} | {failed['best_objective_index']} | {w['current_vx_mps']:.6f} | "
            f"{w['maximum_predicted_speed_mps']:.6f} | {w['predicted_path_extent_m']:.6f} | {w['speed_increase_count']} | {failed['candidate_runtime_pass_count']} |")
    gap = summary["initial_forward_unverified_candidates"]
    initial_notice = (f"초기 잠재값 SHA는 6개 배치 모두 일치했지만, 초기 목적함수 {len(gap):,}/8,022개는 CPU 재계산이 사전 절대·상대 1e-5 허용량 밖입니다. "
        f"최대 차이는 {max(r['absolute_difference_m2'] for r in gap):.9g} m²입니다. 실제 실행 Torch {summary['executed_probe_environment']['torch_version']} GPU와 감사 Torch "
        f"{summary['initial_forward_checks'][0].get('torch_version', 'unknown')} CPU가 다르지만 이것만으로 원인은 확정하지 않습니다. **초기 전방계산은 확인 미완료**이며 허용량을 넓히지 않았습니다."
        if gap and all("absolute_difference_m2" in r for r in gap) else
        "초기값 및 초기 목적함수의 개별 확인 상태는 summary.json을 따릅니다. 확인 불가 항목을 PASS로 바꾸지 않습니다.")
    return ("# 실제 결과 요약 — 미승인 연구 진단\n\n<!-- HH_260906 - This report preserves incomplete initialization verification and every original failure. -->\n\n"
        f"독립 감사 상태: `{summary['status']}`. 실제 GPU 수치 최적화는 2026-09-08 05:05:03–05:07:27 KST에 완료됐습니다. "
        "학습 모델의 가중치는 만들거나 변경하지 않았으며, 실차·폐루프 주행·10 Hz 인퍼런스 증거가 아닙니다.\n\n"
        "두 원본 실행 671 + 666 = 1,337개 시작점(워밍업 70, 주행 1,267), 각 64개 미래 점을 전부 유지했습니다. "
        "Adam 0.05, 512회, 256개 단위 6배치(마지막 57개), 고정 6초기값입니다. iteration 0을 포함한 실제 기록은 3,078줄입니다.\n\n"
        + "\n".join(table) + "\n\n"
        f"전체 후보 출력 검사: **{8022 - summary['runtime_gate_failure_count']:,} PASS / {summary['runtime_gate_failure_count']} FAIL / 8,022개**. "
        "모든 저장 후보의 513,408점 잔차·목적함수·출력 게이트는 독립 재계산으로 일치했습니다. "
        f"미래를 아는 best-of-six 평균 ADE는 {best['metrics']['ade_m']['mean'] * 1000:.3f} mm지만 "
        f"전체 최대 오차는 {best['metrics']['maximum_xy_error_m']['max'] * 1000:.3f} mm입니다. "
        f"best-of-six의 전체 미래 최대 오차 ≤1 mm는 {best['descriptive_max_xy_error_coverage']['0.001']['within_count']:,}/1,337, "
        f"≤1 cm는 {best['descriptive_max_xy_error_coverage']['0.01']['within_count']:,}/1,337, "
        f"≤10 cm는 {best['descriptive_max_xy_error_coverage']['0.1']['within_count']:,}/1,337입니다. "
        "설명용 구간이며 새 데이터 승인 기준이 아닙니다.\n\n"
        "## 아직 남아 있는 검증 제한\n\n" + initial_notice + "\n\n"
        "최종 저장 GPU 출력의 잔차와 게이트는 검증했지만, 최적화 자체나 최종 잠재값→출력의 GPU 연산 전체를 재실행한 것은 아닙니다. "
        "최종 잠재값 1,026,816개는 모두 보존·유한성 검사했습니다. 원본 곡률 실패 299개 시작점은 그대로 FAIL이며, "
        "미래 정답을 보고 맞춘 궤적이 가까워졌다고 원본 라벨을 교체하거나 승인하지 않습니다.\n\n"
        "## 정지 직전 출력 게이트 실패: 곡률이 아니라 최소 이동거리\n\n"
        "best-of-six 중 아래 4개가 실패했습니다. 모든 34개 후보 실패 이유는 `selected trajectory has insufficient planar extent`입니다. "
        "현재 게이트는 경로 길이 <0.05 m이면서 예측 최대속도 >0.1 m/s이고 단조 감속 정지 예외가 성립하지 않으면 거부합니다. "
        "네 후보 모두 최종 속도는 0이지만 중간 속도가 증가해 예외가 아닙니다. 원본 vx와 planar 속도는 분리 기록했습니다.\n\n"
        + "\n".join(witness_rows) + "\n\n"
        "이 네 시작점의 원본 미래 최대 planar 속도는 각각 0.075952 / 0.072048 / 0.081739 / 0.079579 m/s로 모두 0.1 m/s 미만입니다. "
        "0.1 m/s 초과는 수치 근사 후보에서 발생했으며, 원본 미래 자체의 동일 속도 초과라고 해석하면 안 됩니다. "
        "원본·예측 경로 길이와 각 속도 증가 점은 JSON 증거에 함께 보존했습니다.\n\n"
        "run_002:4011은 6개 초기값 모두 실패합니다. 다른 후보가 통과한 시작점도 자동 대체 선택하지 않았습니다. "
        "이는 그대로 남긴 출력 계약 불일치이며 게이트·속도값·라벨·마스크를 수정하지 않았습니다.\n\n"
        "## 증거와 다음 판단\n\n"
        "코드·입력·원본 메타데이터 해시는 JSON에 연결됩니다. 실행된 코드 10개 및 소유 프로세스 코드 2개는 "
        f"정확한 Git `{COMMIT}`의 로컬 blob으로 검증했으며 fetch는 금지했습니다. 이후 감사 코드와 실행 당시 코드는 구분합니다. "
        "서로 겹치는 순항·coast·원본 곡률 실패 그룹은 요약에 모두 남겼습니다. "
        "결과는 물리 decoder가 원본에 얼마나 가까이 근사할 수 있는지 보는 수치 연구입니다. 관측 가능 입력을 쓰는 학습, 정지 의도 표현, "
        "라벨 정책·불확실성·독립 승인 절차 검토가 별도로 필요합니다.\n\n"
        "재감사에는 원본 비공개/ignored 입력 세 묶음이 필요하며 Git clone만으로 이 원본은 다운로드되지 않습니다. "
        "공개 JSON·PNG의 무결성은 이 폴더에서 `sha256sum -c SHA256SUMS`로 확인할 수 있습니다. "
        "원본이 있는 로컬에서는 CUDA를 숨긴 기존 Python 환경으로 다음 명령을 실행합니다. 새 출력 폴더만 허용합니다.\n\n"
        "```bash\nCUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python -m scripts.e2e.audit_portable_decoder_probe \\\n"
        "  --owned-root artifacts/training/2026-09-08/decoder_representability_v1 \\\n"
        "  --input-root artifacts/training/2026-09-08/decoder_representability_inputs_v1 \\\n"
        "  --raw-campaign-root artifacts/training/2026-09-08/brake_free_goal_stop_v4 \\\n"
        "  --output-dir artifacts/training/2026-09-08/decoder_representability_new_audit\n```\n\n")


def render(output, summary, rows, history):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fig, ax = plt.subplots(figsize=(16, 9), dpi=120)
    sizes = [256] * 5 + [57]
    for k in range(6):
        curve = [sum(history[b * 513 + i]["mean_objective_by_initialization"][k] * sizes[b] for b in range(6)) / 1337 for i in range(513)]
        ax.plot(range(513), curve, label=f"Initialization {k}")
    ax.set(xlabel="Fixed latent-optimization iteration (not model training)", ylabel="All-anchor weighted objective (m²)", yscale="log",
        title="All 1,337 anchors · six fixed initializations · 512 iterations each")
    ax.grid(alpha=.2); ax.legend(loc="upper right")
    fig.text(.5, .015, "Future-aware numerical fitting only; no labels changed, no model/data admission, no certified global optimum.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, .04, 1, 1)); fig.savefig(output / "01_oracle_objective_history.png"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(16, 9), dpi=120)
    for k in range(7):
        values = sorted(r["candidates"][r["best_objective_index"] if k == 6 else k]["maximum_xy_error_m"] for r in rows)
        ax.plot([max(v, 1e-9) for v in values], [(i + 1) / len(values) for i in range(len(values))],
            label="Future-aware best final objective" if k == 6 else f"Initialization {k}", color="black" if k == 6 else None, linewidth=2.5 if k == 6 else 1.2)
    for threshold in (.001, .01, .1): ax.axvline(threshold, color="gray", linestyle=":", linewidth=.8)
    ax.set(xlabel="Maximum XY error across each full 64-point future (m; log scale)", ylabel="Fraction of all 1,337 anchors", xscale="log",
        title="Every fixed initialization retained; descriptive 1 mm / 1 cm / 10 cm lines are not admission thresholds")
    ax.grid(alpha=.2); ax.legend(loc="lower right")
    fig.text(.5, .015, "Original 299 curvature-failed anchors remain included. Best-by-objective does not necessarily minimize maximum XY error.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, .04, 1, 1)); fig.savefig(output / "02_maximum_xy_error_ecdf.png"); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--owned-root", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--raw-campaign-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    require(not args.output_dir.exists() and all(not args.output_dir.resolve().is_relative_to(p.resolve())
        for p in (args.owned_root, args.input_root, args.raw_campaign_root)), "fresh audit output must be outside original inputs")
    result = audit(args.owned_root, args.input_root, args.raw_campaign_root)
    write_result(args.output_dir, *result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
