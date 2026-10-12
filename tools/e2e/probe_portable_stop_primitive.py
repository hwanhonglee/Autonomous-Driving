#!/usr/bin/env python3
"""HH_260906 - Probe synthetic stop geometry on CPU without training, captured data or runtime admission."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal

import torch

from portable_e2e import stop_primitive_research as primitive
from portable_e2e.contract import ContractError
from portable_e2e import runtime_contract as runtime


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.synthetic_stop_primitive_probe.v1"
RUNTIME_SHA = "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3"
SPEEDS = (0., .01, .05, .075, .1, .11, .4, 1., 4., 30. / 3.6)
DURATION_LOGITS = (-20., -4., 0., 4., 20.)
DTYPES = ("float32", "float64")
PROFILE_NAMES = ("straight", "constant_left_20", "constant_right_minus20",
    "alternating_plus_minus20", "sine_amplitude2", "constant_left_0p2")
SOURCE_NAMES = ("portable_e2e/stop_primitive_research.py", "portable_e2e/runtime_contract.py",
    "portable_e2e/contract.py", "portable_e2e/dataset.py", "scripts/e2e/probe_portable_stop_primitive.py")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def write_json(path, value):
    with Path(path).open("x") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + "\n")


def read_json(raw):
    def pairs(entries):
        result = {}
        for key, value in entries:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def nonfinite(_):
        raise ValueError("nonfinite JSON number")
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=nonfinite)


def cpu_guard():
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and not torch.cuda.is_initialized(),
        "probe requires explicitly hidden CUDA and an uninitialized CUDA runtime")


def curvature_profiles():
    # HH_260906 - Fix all six synthetic profiles before execution; none is selected using a measured outcome.
    return [[0.] * 64, [20.] * 64, [-20.] * 64,
        [20. if index % 2 == 0 else -20. for index in range(64)],
        [2. * math.sin(2. * math.pi * index / 63.) for index in range(64)], [.2] * 64]


def source_pins():
    require(sha(ROOT / "portable_e2e/runtime_contract.py") == RUNTIME_SHA,
        "existing runtime source changed; gate substitution is forbidden")
    return {name: {"sha256": sha(ROOT / name), "size_bytes": (ROOT / name).stat().st_size}
        for name in SOURCE_NAMES}


def build_plan():
    """HH_260906 - Return a source-bound declaration only; callers must save and commit it before the actual grid run."""
    expected = {"RESEARCH_ID": "portable_e2e.stop_primitive.research.v1", "TIME_STEP_S": .1,
        "FUTURE_POINTS": 64, "CANDIDATE_COUNT": 6, "MAX_SPEED_MPS": 30. / 3.6,
        "MAX_DECELERATION_MPS2": 2.9, "MAX_CURVATURE_RAD_PER_M": .2,
        "MAX_LATERAL_ACCELERATION_MPS2": 2.8}
    require(all(getattr(primitive, key) == value for key, value in expected.items()),
        "research primitive constants differ from the fixed design")
    return {"schema": SCHEMA + ".plan", "status": "DECLARED_NOT_EXECUTED", "source_pins": source_pins(),
        "research_constants": expected, "current_speed_grid_mps": list(SPEEDS),
        "duration_logits_grid": list(DURATION_LOGITS), "dtypes": list(DTYPES),
        "case_order": "dtype, current speed, duration logit; all ascending as listed",
        "case_count": 100, "candidate_count": 600, "future_point_count": 38400,
        "profiles": [{"candidate_index": i, "name": name, "curvature_logits": values}
            for i, (name, values) in enumerate(zip(PROFILE_NAMES, curvature_profiles()))],
        "batch_size": 1, "device": "cpu", "torch_threads": 4,
        "runtime_gate_id": runtime.RUNTIME_GATE_ID, "runtime_config": asdict(runtime.RuntimeGateConfig()),
        "runtime_selection": "Every slot independently selected by one-hot logits; no alternative winner or relaxed gate.",
        "numerical_comparison": {"reference": "Independent stdlib float64 closed form on dtype-rounded input scalars.",
            "absolute_epsilon_factor": 64, "relative_epsilon_factor": 64,
            "invariant_epsilon_factor": 256, "runtime_tolerances_changed": False},
        "formula": {"duration": "min(6.4, max(0.1,v0/2.9) + sigmoid(z)*(6.4-max(0.1,v0/2.9)))",
            "speed": "v(t)=0 if t>=T else max(0,v0*(1-t/T)); t=(i+1)*0.1",
            "curvature": "tanh(logit)*min(0.2,2.8/max(entry_speed^2,1e-6))",
            "discrete_geometry": "ds=0.1*endpoint_speed; heading=sum(curvature*ds); XY=sum(ds*[cos(heading),sin(heading)])"},
        "scope": {"synthetic_only": True, "stop_decision_assumed_by_caller": True,
            "training_data_approved": False, "model_training": False, "model_parameters": False,
            "optimizer": False, "raw_data_read": False, "runtime_changed": False,
            "live_control": False, "road_or_collision_approval": False}}


def independent_reference(current, duration_logit, profile):
    """HH_260906 - Recompute every discrete point in stdlib arithmetic, not by calling the primitive or copying its output."""
    minimum = max(.1, current / 2.9)
    duration = min(6.4, minimum + (1. / (1. + math.exp(-duration_logit))) * (6.4 - minimum))
    speed, heading, xy = [], [], []
    entry, angle, x, y = current, 0., 0., 0.
    for index, logit in enumerate(profile):
        t = (index + 1) * .1
        v = 0. if t >= duration else max(0., current * (1. - t / duration))
        curvature = math.tanh(logit) * min(.2, 2.8 / max(entry * entry, 1e-6))
        distance = .1 * v
        angle += curvature * distance
        x += distance * math.cos(angle); y += distance * math.sin(angle)
        speed.append(v); heading.append(angle); xy.append([x, y]); entry = v
    return {"stop_duration_s": duration, "speed_mps": speed, "heading_rad": heading, "xy_base_m": xy}


def compare_reference(actual, reference, epsilon):
    differences, maxima = [], {}
    for component in ("stop_duration_s", "speed_mps", "heading_rad", "xy_base_m"):
        def flatten(value):
            if isinstance(value, list):
                return [x for row in value for x in flatten(row)]
            return [value]
        values, expected = flatten(actual[component]), flatten(reference[component])
        require(len(values) == len(expected), "numerical reference shape mismatch")
        for index, (value, target) in enumerate(zip(values, expected)):
            require(math.isfinite(value) and math.isfinite(target), "nonfinite numerical output")
            error = abs(value - target)
            maxima[component] = max(maxima.get(component, 0.), error)
            if not math.isclose(value, target, abs_tol=64 * epsilon, rel_tol=64 * epsilon):
                differences.append({"component": component, "scalar_index": index,
                    "primitive_value": value, "reference_value": target, "absolute_difference": error})
    return {"status": "MATCH" if not differences else "UNVERIFIED", "compared_scalar_count": 257,
        "absolute_tolerance": 64 * epsilon, "relative_tolerance": 64 * epsilon,
        "maximum_absolute_difference": maxima, "mismatch_scalar_count": len(differences), "mismatches": differences}


def invariant_checks(current, actual, epsilon):
    # HH_260906 - Numerical slack belongs only to diagnostic arithmetic; the runtime call below remains unchanged.
    slack = 256 * epsilon
    speeds, xy, headings = actual["speed_mps"], actual["xy_base_m"], actual["heading_rad"]
    entry, previous, angle = current, [0., 0.], 0.
    rates, distances, disagreement = [], [], []
    monotonic, nonnegative, curvature, lateral, stopped_heading = True, True, True, True, True
    for speed, point, heading in zip(speeds, xy, headings):
        step = math.dist(previous, point); ds = .1 * speed
        dh = abs(heading - angle)
        monotonic &= speed <= entry + slack
        nonnegative &= speed >= 0.
        curvature &= dh <= .2 * ds + slack
        lateral &= dh * entry * entry <= 2.8 * ds + slack * max(1., entry * entry)
        stopped_heading &= ds != 0. or heading == angle
        rates.append((speed - entry) / .1); distances.append(step); disagreement.append(abs(step - ds))
        entry, previous, angle = speed, point, heading
    duration = actual["stop_duration_s"]
    flags = {"terminal_speed_exact_zero": speeds[-1] == 0., "nonnegative_speed": nonnegative,
        "monotonic_nonincreasing_with_numeric_slack": monotonic,
        "deceleration_with_numeric_slack": min(rates) >= -2.9 - slack,
        "speed_bound_with_numeric_slack": max(speeds) <= 30. / 3.6 + slack,
        "duration_bounds_with_numeric_slack": max(.1, current / 2.9) - slack <= duration <= 6.4 + slack,
        "discrete_integral_with_numeric_slack": max(disagreement) <= slack * max(1., sum(distances)),
        "curvature_increment_with_numeric_slack": curvature,
        "lateral_increment_with_numeric_slack": lateral, "no_heading_change_after_exact_stop": stopped_heading}
    return {"status": "PASS" if all(flags.values()) else "FAIL", "flags": flags,
        "arithmetic_slack": slack, "minimum_speed_rate_mps2": min(rates),
        "maximum_speed_rate_mps2": max(rates), "discrete_path_distance_m": math.fsum(distances),
        "endpoint_speed_integral_m": math.fsum(speeds) * .1,
        "maximum_step_integral_disagreement_m": max(disagreement),
        "continuous_linear_braking_distance_m_reference_only": .5 * current * duration,
        "continuous_distance_is_not_the_discrete_output_contract": True}


def run_case(dtype_name, requested_speed, duration_logit, case_index):
    cpu_guard(); require(dtype_name in DTYPES, "unsupported probe dtype")
    dtype = getattr(torch, dtype_name)
    current = torch.tensor([requested_speed], dtype=dtype, device="cpu")
    duration = torch.full((1, 6), duration_logit, dtype=dtype, device="cpu")
    profiles = torch.tensor([curvature_profiles()], dtype=dtype, device="cpu")
    inputs = (current, duration, profiles)
    before = [hashlib.sha256(t.numpy().tobytes()).hexdigest() for t in inputs]
    with torch.no_grad():
        result = primitive.decode_stop_primitive(*inputs)
    fields = {name: getattr(result, name) for name in ("xy_base_m", "speed_mps", "heading_rad", "stop_duration_s")}
    shapes = {"xy_base_m": (1,6,64,2), "speed_mps": (1,6,64), "heading_rad": (1,6,64), "stop_duration_s": (1,6)}
    for name, tensor in fields.items():
        require(tuple(tensor.shape) == shapes[name] and tensor.device.type == "cpu" and tensor.dtype == dtype
            and not tensor.requires_grad and bool(torch.isfinite(tensor).all()), "primitive output ABI/finite/no-grad failure")
    require(before == [hashlib.sha256(t.numpy().tobytes()).hexdigest() for t in inputs], "primitive mutated synthetic inputs")
    outputs = {name: tensor[0].tolist() for name, tensor in fields.items()}
    effective = float(current.item()); epsilon = torch.finfo(dtype).eps
    candidates = []
    for index, profile in enumerate(profiles[0].tolist()):
        actual = {name: value[index] for name, value in outputs.items()}
        reference = independent_reference(effective, float(duration[0,index]), profile)
        logits = [float(i == index) for i in range(6)]
        try:
            selected = runtime.validate_and_select_trajectory(outputs["xy_base_m"], outputs["speed_mps"],
                logits, current_speed_mps=effective)
            require(selected.candidate_index == index, "runtime selected a different synthetic candidate")
            gate = {"status": "PASS", "reason": None, "selected_index": selected.candidate_index}
        except ContractError as error:
            gate = {"status": "FAIL", "reason": str(error), "selected_index": index}
        candidates.append({"candidate_index": index, "profile": PROFILE_NAMES[index],
            "runtime_selection_logits": logits, **actual,
            "independent_reference": compare_reference(actual, reference, epsilon),
            "numerical_invariants": invariant_checks(effective, actual, epsilon), "runtime_gate": gate})
    return {"case_index": case_index, "dtype": dtype_name, "dtype_epsilon": epsilon,
        "requested_current_speed_mps": requested_speed, "effective_current_speed_mps": effective,
        "duration_logit": duration_logit, "input_tensor_sha256": before,
        "input_tensors_unchanged": True, "synthetic_only": True, "candidates": candidates}


def summarize(rows):
    candidates = [candidate for row in rows for candidate in row["candidates"]]
    return {"reported_case_count": len(rows), "reported_candidate_count": len(candidates),
        "reported_future_point_count": 64 * len(candidates),
        "compared_scalar_count": sum(c["independent_reference"]["compared_scalar_count"] for c in candidates),
        "runtime_gate_counts": dict(Counter(c["runtime_gate"]["status"] for c in candidates)),
        "runtime_failure_reasons": dict(Counter(c["runtime_gate"]["reason"] for c in candidates if c["runtime_gate"]["status"] == "FAIL")),
        "numerical_reference_counts": dict(Counter(c["independent_reference"]["status"] for c in candidates)),
        "numerical_invariant_counts": dict(Counter(c["numerical_invariants"]["status"] for c in candidates)),
        "runtime_failures": [{"case_index": row["case_index"], "dtype": row["dtype"],
            "requested_current_speed_mps": row["requested_current_speed_mps"], "duration_logit": row["duration_logit"],
            "candidate_index": candidate["candidate_index"], "reason": candidate["runtime_gate"]["reason"]}
            for row in rows for candidate in row["candidates"] if candidate["runtime_gate"]["status"] == "FAIL"]}


def persisted_rows(path):
    # HH_260906 - Reconcile completed JSONL lines after an interrupted write; preserve any incomplete tail without inventing records.
    if not path.exists():
        return [], {"complete_record_count": 0, "unparsed_tail_bytes": 0}
    raw = path.read_bytes(); result, consumed = [], 0
    for line in raw.splitlines(keepends=True):
        if not line.endswith(b"\n"):
            break
        try:
            row = read_json(line)
            require(row["case_index"] == len(result) and len(row["candidates"]) == 6,
                "persisted synthetic row identity differs")
        except (ValueError, KeyError, TypeError):
            break
        result.append(row); consumed += len(line)
    return result, {"complete_record_count": len(result), "unparsed_tail_bytes": len(raw) - consumed}


def run_probe(plan_path, output):
    cpu_guard(); plan_path, output = Path(plan_path), Path(output)
    require(plan_path.is_file() and not plan_path.is_symlink(), "preregistered plan must be a real file")
    require(not output.exists() and not output.is_symlink()
        and not output.resolve().is_relative_to((ROOT / "datasets").resolve()), "output must be fresh and outside datasets")
    plan_bytes = plan_path.read_bytes(); plan = read_json(plan_bytes)
    require(canonical(plan) == canonical(build_plan()), "preregistered plan or source differs from the fixed grid")
    output.mkdir(parents=True, exist_ok=False)
    status = {"schema": SCHEMA, "status": "RUNNING", "started_utc": datetime.now(timezone.utc).isoformat(),
        "plan_sha256": hashlib.sha256(plan_bytes).hexdigest(), "source_pins": plan["source_pins"],
        "torch_version": torch.__version__, "device": "cpu", "cuda_visible_devices": "",
        "scope": plan["scope"], "interpretation": "Synthetic geometry only. STOP is assumed, not learned. Runtime failures remain; neither numeric matches nor runtime PASS approves data, a policy, road tracking or collision avoidance."}
    interrupted = {}
    handlers = {name: signal.getsignal(name) for name in (signal.SIGINT, signal.SIGTERM)}
    threads = torch.get_num_threads()
    rows, error = [], None
    try:
        for name in handlers:
            signal.signal(name, lambda number, _: interrupted.setdefault("signal", number))
        torch.set_num_threads(4)
        (output / "plan.json").write_bytes(plan_bytes)
        write_json(output / "started.json", status)
        for name, pin in plan["source_pins"].items():
            path = output / "source_archive" / name; path.parent.mkdir(parents=True, exist_ok=True)
            data = (ROOT / name).read_bytes(); require(hashlib.sha256(data).hexdigest() == pin["sha256"], "source changed before archive")
            with path.open("xb") as stream: stream.write(data)
        with (output / "rows.jsonl").open("x") as stream:
            for dtype in DTYPES:
                for speed in SPEEDS:
                    for logit in DURATION_LOGITS:
                        require(not interrupted, "synthetic probe interrupted by signal")
                        row = run_case(dtype, speed, logit, len(rows))
                        stream.write(canonical(row) + "\n"); stream.flush(); rows.append(row)
        require(not interrupted and len(rows) == 100, "incomplete synthetic grid")
        require(plan_path.read_bytes() == plan_bytes and source_pins() == plan["source_pins"], "plan/source changed during probe")
        cpu_guard()
        status.update(status="COMPLETE_NOT_ADMITTED", source_and_plan_postcheck_pass=True)
    except BaseException as caught:
        error = caught
        status.update(status="FAILED_PARTIAL_NOT_ADMITTED", error_type=type(caught).__name__, error=str(caught))
        try:
            status["source_and_plan_postcheck_pass"] = plan_path.read_bytes() == plan_bytes and source_pins() == plan["source_pins"]
        except Exception:
            status["source_and_plan_postcheck_pass"] = False
    finally:
        try:
            recorded, ledger = persisted_rows(output / "rows.jsonl")
            if status["status"] == "COMPLETE_NOT_ADMITTED" and (ledger["unparsed_tail_bytes"]
                    or canonical(recorded) != canonical(rows)):
                error = ValueError("persisted row ledger differs from completed grid")
                status.update(status="FAILED_PARTIAL_NOT_ADMITTED", error_type="ValueError", error=str(error))
            rows = recorded
            status.update(finished_utc=datetime.now(timezone.utc).isoformat(), interruption=interrupted,
                torch_threads=4, persisted_row_ledger=ledger, **summarize(rows))
            write_json(output / "summary.json", status)
            with (output / "SHA256SUMS").open("x") as stream:
                for path in sorted(output.rglob("*")):
                    if path.is_file() and path.name != "SHA256SUMS":
                        stream.write(f"{sha(path)}  {path.relative_to(output).as_posix()}\n")
        finally:
            torch.set_num_threads(threads)
            for name, handler in handlers.items(): signal.signal(name, handler)
    if error is not None:
        raise error
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    run_probe(args.plan, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
