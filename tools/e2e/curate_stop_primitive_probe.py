#!/usr/bin/env python3
"""HH_260906 - Publish authenticated synthetic STOP numbers without executing a model or changing a gate."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import struct


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.synthetic_stop_primitive_probe.v1"
PUBLIC_SCHEMA = "portable_e2e.synthetic_stop_primitive_publication.v1"
RUNTIME_SHA = "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3"
SPEEDS = (0., .01, .05, .075, .1, .11, .4, 1., 4., 30. / 3.6)
DURATION_LOGITS = (-20., -4., 0., 4., 20.)
DTYPES = ("float32", "float64")
PROFILE_NAMES = ("straight", "constant_left_20", "constant_right_minus20",
    "alternating_plus_minus20", "sine_amplitude2", "constant_left_0p2")
SOURCE_NAMES = ("portable_e2e/stop_primitive_research.py", "portable_e2e/runtime_contract.py",
    "portable_e2e/contract.py", "portable_e2e/dataset.py", "scripts/e2e/probe_portable_stop_primitive.py")
INPUT_NAMES = frozenset(("plan.json", "started.json", "summary.json", "rows.jsonl",
    *("source_archive/" + name for name in SOURCE_NAMES)))
SCOPE = {"synthetic_only": True, "stop_decision_assumed_by_caller": True,
    "training_data_approved": False, "model_training": False, "model_parameters": False,
    "optimizer": False, "raw_data_read": False, "runtime_changed": False,
    "live_control": False, "road_or_collision_approval": False}
CONSTANTS = {"RESEARCH_ID": "portable_e2e.stop_primitive.research.v1", "TIME_STEP_S": .1,
    "FUTURE_POINTS": 64, "CANDIDATE_COUNT": 6, "MAX_SPEED_MPS": 30. / 3.6,
    "MAX_DECELERATION_MPS2": 2.9, "MAX_CURVATURE_RAD_PER_M": .2,
    "MAX_LATERAL_ACCELERATION_MPS2": 2.8}
RUNTIME_CONFIG = {"candidate_count": 6, "future_points": 64, "timestep_s": .1,
    "maximum_speed_mps": 30. / 3.6, "maximum_step_m": 1., "maximum_abs_x_m": 60.,
    "maximum_abs_y_m": 60., "maximum_heading_step_rad": .75, "maximum_curvature_rad_per_m": .5,
    "maximum_lateral_acceleration_mps2": 3., "heading_minimum_step_m": .01,
    "maximum_backward_step_m": .002, "maximum_speed_disagreement_mps": .25,
    "maximum_integrated_distance_disagreement_m": .02,
    "maximum_integrated_distance_disagreement_ratio": .05, "maximum_acceleration_mps2": 3.,
    "maximum_deceleration_mps2": 6., "minimum_planar_extent_m": .05,
    "stationary_speed_tolerance_mps": .1, "stationary_claim_speed_epsilon_mps": .0001,
    "current_speed_reverse_jitter_tolerance_mps": .1, "maximum_stationary_radius_m": .05,
    "maximum_stationary_extent_m": .15, "minimum_low_speed_progress_ratio": .5,
    "maximum_first_point_distance_m": 1., "minimum_first_point_x_m": -.25}
FLAG_NAMES = frozenset(("terminal_speed_exact_zero", "nonnegative_speed",
    "monotonic_nonincreasing_with_numeric_slack", "deceleration_with_numeric_slack",
    "speed_bound_with_numeric_slack", "duration_bounds_with_numeric_slack",
    "discrete_integral_with_numeric_slack", "curvature_increment_with_numeric_slack",
    "lateral_increment_with_numeric_slack", "no_heading_change_after_exact_stop"))
COMPONENTS = ("stop_duration_s", "speed_mps", "heading_rad", "xy_base_m")
ROW_KEYS = frozenset(("case_index", "dtype", "dtype_epsilon", "requested_current_speed_mps",
    "effective_current_speed_mps", "duration_logit", "input_tensor_sha256",
    "input_tensors_unchanged", "synthetic_only", "candidates"))
CANDIDATE_KEYS = frozenset(("candidate_index", "profile", "runtime_selection_logits",
    *COMPONENTS, "independent_reference", "numerical_invariants", "runtime_gate"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def valid_sha(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def read_json(raw):
    # HH_260906 - Reject duplicate keys and nonfinite numbers before trusting any diagnostic status.
    def pairs(entries):
        result = {}
        for key, value in entries:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def invalid(_):
        raise ValueError("nonfinite JSON number")
    result = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)
    def walk(value):
        if isinstance(value, dict):
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)
        elif type(value) in (int, float):
            require(math.isfinite(value), "nonfinite JSON number")
    walk(result)
    return result


def no_symlinks(path):
    path = Path(path).absolute()
    require(all(not item.is_symlink() for item in (path, *path.parents)), "symlink paths are forbidden")


def checked_output(output, input_root):
    output = Path(output).absolute()
    no_symlinks(output)
    require(not output.exists(), "output must be new; overwrite is forbidden")
    resolved = output.resolve()
    forbidden = (Path(input_root).absolute(), Path(input_root).resolve(),
        ROOT / "datasets", (ROOT / "datasets").resolve())
    require(all(not resolved.is_relative_to(path) for path in forbidden), "output overlaps protected inputs/datasets")
    return output


def read_inventory(input_root, expected_summary_sha256, expected_checksums_sha256):
    require(valid_sha(expected_summary_sha256) and valid_sha(expected_checksums_sha256),
        "explicit lowercase summary and checksum SHA256 pins are required")
    input_root = Path(input_root).absolute()
    no_symlinks(input_root)
    require(input_root.is_dir(), "input directory is missing")
    files = {}
    for path in input_root.rglob("*"):
        require(not path.is_symlink(), "input symlink is forbidden")
        require(path.is_dir() or path.is_file(), "input contains a nonregular entry")
        if path.is_file():
            files[path.relative_to(input_root).as_posix()] = path
    require(set(files) == INPUT_NAMES | {"SHA256SUMS"}, "input inventory must contain exactly nine artifacts and SHA256SUMS")
    raw = {name: path.read_bytes() for name, path in files.items()}
    require(digest(raw["SHA256SUMS"]) == expected_checksums_sha256, "checksum manifest SHA mismatch")
    require(digest(raw["summary.json"]) == expected_summary_sha256, "summary SHA mismatch")
    ledger = {}
    for line in raw["SHA256SUMS"].decode("ascii").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, "malformed checksum manifest")
        checksum, name = match.groups()
        require(name in INPUT_NAMES and name not in ledger, "unexpected or duplicate checksum path")
        require(digest(raw[name]) == checksum, "input artifact SHA mismatch")
        ledger[name] = checksum
    require(set(ledger) == INPUT_NAMES, "incomplete checksum manifest")
    return raw


def expected_cases():
    return list(itertools.product(DTYPES, SPEEDS, DURATION_LOGITS))


def validate_plan(plan, summary, started, raw):
    require(plan["schema"] == SCHEMA + ".plan" and plan["status"] == "DECLARED_NOT_EXECUTED", "unsupported plan")
    require(summary["schema"] == SCHEMA and summary["status"] == "COMPLETE_NOT_ADMITTED", "probe is not complete")
    require(started["schema"] == SCHEMA and started["status"] == "RUNNING", "invalid initial execution marker")
    count = len(expected_cases())
    require(plan["current_speed_grid_mps"] == list(SPEEDS) and plan["duration_logits_grid"] == list(DURATION_LOGITS)
        and plan["dtypes"] == list(DTYPES), "grid changed")
    require(plan["case_order"] == "dtype, current speed, duration logit; all ascending as listed", "case order changed")
    require((plan["case_count"], plan["candidate_count"], plan["future_point_count"]) == (count, count * 6, count * 384),
        "declared denominator changed")
    require(canonical(plan["research_constants"]) == canonical(CONSTANTS), "research constants changed")
    require(canonical(plan["runtime_config"]) == canonical(RUNTIME_CONFIG)
        and plan["runtime_gate_id"] == "portable_e2e.runtime_geometry_gate.v8", "runtime gate/config changed")
    require(plan["batch_size"] == 1 and plan["device"] == "cpu" and plan["torch_threads"] == 4, "execution plan changed")
    profiles = [[0.] * 64, [20.] * 64, [-20.] * 64,
        [20. if i % 2 == 0 else -20. for i in range(64)],
        [2. * math.sin(2. * math.pi * i / 63.) for i in range(64)], [.2] * 64]
    require(plan["profiles"] == [{"candidate_index": i, "name": name, "curvature_logits": values}
        for i, (name, values) in enumerate(zip(PROFILE_NAMES, profiles))], "fixed candidate profiles changed")
    comparison = plan["numerical_comparison"]
    require(comparison == {"reference": "Independent stdlib float64 closed form on dtype-rounded input scalars.",
        "absolute_epsilon_factor": 64, "relative_epsilon_factor": 64,
        "invariant_epsilon_factor": 256, "runtime_tolerances_changed": False}, "numerical comparison changed")
    for document in (plan, summary, started):
        require(canonical(document["scope"]) == canonical(SCOPE), "synthetic/no-admission scope changed")
        require(document["source_pins"] == plan["source_pins"], "execution source identities differ")
    require(set(plan["source_pins"]) == set(SOURCE_NAMES), "source inventory changed")
    for name, pin in plan["source_pins"].items():
        source = raw["source_archive/" + name]
        require(pin == {"sha256": digest(source), "size_bytes": len(source)}, "archived source identity mismatch")
    require(plan["source_pins"]["portable_e2e/runtime_contract.py"]["sha256"] == RUNTIME_SHA,
        "archived runtime is not the reviewed unchanged gate")
    for document in (summary, started):
        require(document["plan_sha256"] == digest(raw["plan.json"]), "executed plan identity mismatch")
        require(document["device"] == "cpu" and document["cuda_visible_devices"] == "",
            "execution was not the declared CPU probe")
    require(summary["torch_threads"] == 4, "final execution thread count changed")
    require(summary["source_and_plan_postcheck_pass"] is True and summary["interruption"] == {}, "source postcheck or interruption failure")
    require(summary["persisted_row_ledger"] == {"complete_record_count": count, "unparsed_tail_bytes": 0}, "incomplete persisted rows")
    require(summary["started_utc"] == started["started_utc"], "execution start mismatch")
    start, end = (datetime.fromisoformat(summary[name]) for name in ("started_utc", "finished_utc"))
    require(start.utcoffset() is not None and end.utcoffset() is not None and end >= start, "invalid execution time order")


def validate_candidate(candidate, index, epsilon):
    require(set(candidate) == CANDIDATE_KEYS and type(candidate["candidate_index"]) is int
        and candidate["candidate_index"] == index and candidate["profile"] == PROFILE_NAMES[index], "candidate slot/profile changed")
    require(candidate["runtime_selection_logits"] == [float(i == index) for i in range(6)], "every candidate must have its own selection request")
    require(finite(candidate["stop_duration_s"]), "nonfinite stop duration")
    for component in ("speed_mps", "heading_rad"):
        require(isinstance(candidate[component], list) and len(candidate[component]) == 64
            and all(finite(x) for x in candidate[component]), "invalid 64-point scalar trajectory")
    require(isinstance(candidate["xy_base_m"], list) and len(candidate["xy_base_m"]) == 64
        and all(isinstance(point, list) and len(point) == 2 and all(finite(x) for x in point)
            for point in candidate["xy_base_m"]), "invalid 64-point XY trajectory")
    gate = candidate["runtime_gate"]
    require(set(gate) == {"status", "reason", "selected_index"} and type(gate["selected_index"]) is int
        and gate["selected_index"] == index and gate["status"] in ("PASS", "FAIL"), "invalid runtime gate record")
    require((gate["status"] == "PASS" and gate["reason"] is None) or
        (gate["status"] == "FAIL" and isinstance(gate["reason"], str) and bool(gate["reason"])), "gate reason/status contradiction")
    reference = candidate["independent_reference"]
    require(reference["status"] in ("MATCH", "UNVERIFIED") and reference["compared_scalar_count"] == 257,
        "invalid numerical comparison count/status")
    require(reference["absolute_tolerance"] == 64 * epsilon and reference["relative_tolerance"] == 64 * epsilon,
        "numerical comparison tolerance changed")
    require(set(reference["maximum_absolute_difference"]) == set(COMPONENTS)
        and all(finite(value) and value >= 0. for value in reference["maximum_absolute_difference"].values()), "invalid numeric error maxima")
    require(isinstance(reference["mismatches"], list) and type(reference["mismatch_scalar_count"]) is int
        and reference["mismatch_scalar_count"] == len(reference["mismatches"])
        and (reference["status"] == "MATCH") == (not reference["mismatches"]), "numerical mismatch ledger contradiction")
    for item in reference["mismatches"]:
        require(set(item) == {"component", "scalar_index", "primitive_value", "reference_value", "absolute_difference"}
            and item["component"] in COMPONENTS and type(item["scalar_index"]) is int
            and 0 <= item["scalar_index"] < {"stop_duration_s": 1, "xy_base_m": 128}.get(item["component"], 64)
            and all(finite(item[name]) for name in ("primitive_value", "reference_value", "absolute_difference"))
            and item["absolute_difference"] >= 0, "invalid retained numerical mismatch")
    invariant = candidate["numerical_invariants"]
    require(set(invariant["flags"]) == FLAG_NAMES and all(type(flag) is bool for flag in invariant["flags"].values()), "invalid invariant flags")
    require(invariant["status"] == ("PASS" if all(invariant["flags"].values()) else "FAIL")
        and invariant["arithmetic_slack"] == 256 * epsilon
        and invariant["continuous_distance_is_not_the_discrete_output_contract"] is True, "invariant status/tolerance contradiction")
    for name in ("minimum_speed_rate_mps2", "maximum_speed_rate_mps2", "discrete_path_distance_m",
            "endpoint_speed_integral_m", "maximum_step_integral_disagreement_m",
            "continuous_linear_braking_distance_m_reference_only"):
        require(finite(invariant[name]), "nonfinite invariant measurement")


def recount(rows):
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
            "candidate_index": c["candidate_index"], "reason": c["runtime_gate"]["reason"]}
            for row in rows for c in row["candidates"] if c["runtime_gate"]["status"] == "FAIL"]}


def authenticate(input_root, expected_summary_sha256, expected_checksums_sha256):
    raw = read_inventory(input_root, expected_summary_sha256, expected_checksums_sha256)
    plan, summary, started = (read_json(raw[name]) for name in ("plan.json", "summary.json", "started.json"))
    validate_plan(plan, summary, started, raw)
    require(raw["rows.jsonl"].endswith(b"\n"), "partial final JSONL record")
    rows = [read_json(line) for line in raw["rows.jsonl"].splitlines()]
    expected = expected_cases()
    require(len(rows) == len(expected), "not all declared cases were recorded")
    for index, (row, (dtype, speed, logit)) in enumerate(zip(rows, expected)):
        require(set(row) == ROW_KEYS and type(row["case_index"]) is int and row["case_index"] == index,
            "case identity/fields changed")
        require(row["dtype"] == dtype and finite(row["requested_current_speed_mps"])
            and row["requested_current_speed_mps"] == speed and finite(row["duration_logit"])
            and row["duration_logit"] == logit, "fixed case order/grid changed")
        code, epsilon = ("f", 2. ** -23) if dtype == "float32" else ("d", 2. ** -52)
        require(row["dtype_epsilon"] == epsilon and row["effective_current_speed_mps"] == struct.unpack(code, struct.pack(code, speed))[0],
            "dtype-rounded input/epsilon changed")
        require(row["synthetic_only"] is True and row["input_tensors_unchanged"] is True
            and isinstance(row["input_tensor_sha256"], list) and len(row["input_tensor_sha256"]) == 3
            and all(valid_sha(value) for value in row["input_tensor_sha256"]), "synthetic input identity/immutability missing")
        require(isinstance(row["candidates"], list) and len(row["candidates"]) == 6, "all six candidates must be retained")
        for slot, candidate in enumerate(row["candidates"]):
            validate_candidate(candidate, slot, epsilon)
    counts = recount(rows)
    require(all(canonical(summary[key]) == canonical(value) for key, value in counts.items()), "summary differs from full row ledger")
    for name in ("summary.json", "rows.jsonl"):
        public_safe(raw[name])
    return {"raw": raw, "plan": plan, "summary": summary, "rows": rows, "recount": counts}


def public_safe(raw):
    require(not any(token in raw for token in (b"/home/", b"/root/", b"/tmp/", b"-----BEGIN ", b"ssh://"))
        and re.search(rb"(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])", raw) is None,
        "public output contains a private-path/key/endpoint-like token")


def numerical_notes(rows):
    result = {}
    for dtype in DTYPES:
        entries = [(row, c) for row in rows if row["dtype"] == dtype for c in row["candidates"]]
        result[dtype] = {"candidate_count": len(entries),
            "minimum_recorded_speed_rate_mps2": min(c["numerical_invariants"]["minimum_speed_rate_mps2"] for _, c in entries),
            "maximum_recorded_reference_error": {name: max(c["independent_reference"]["maximum_absolute_difference"][name]
                for _, c in entries) for name in COMPONENTS},
            "maximum_continuous_reference_minus_endpoint_integral_m": max(
                .5 * row["effective_current_speed_mps"] * c["stop_duration_s"] - .1 * math.fsum(c["speed_mps"])
                for row, c in entries)}
    return result


def plot_selection(rows):
    indexed = {(row["dtype"], row["requested_current_speed_mps"], row["duration_logit"]): row for row in rows}
    speed_rows = [indexed[dtype, speed, logit] for dtype in DTYPES for speed in (.4, 4.) for logit in DURATION_LOGITS]
    xy_rows = [indexed[dtype, 4., 0.] for dtype in DTYPES]
    return speed_rows, xy_rows


def render(rows, output):
    # HH_260906 - Curves use stored numbers; outcome-independent conditions include both dtypes and every requested duration.
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    matplotlib_init = Path(matplotlib.__file__)
    matplotlib_init_sha = digest(matplotlib_init.read_bytes())
    speed_rows, xy_rows = plot_selection(rows)
    times = [i * .1 for i in range(65)]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for d, dtype in enumerate(DTYPES):
        for s, requested in enumerate((.4, 4.)):
            ax = axes[d, s]
            for row in speed_rows:
                if row["dtype"] != dtype or row["requested_current_speed_mps"] != requested:
                    continue
                candidate = row["candidates"][0]
                ax.plot(times, [row["effective_current_speed_mps"], *candidate["speed_mps"]],
                    label=f"z={row['duration_logit']:g}; T={candidate['stop_duration_s']:.3f}s; {candidate['runtime_gate']['status']}")
            ax.set_title(f"{dtype}: requested v0={requested:g} m/s, candidate 0 / straight")
            ax.set_xlabel("Synthetic future time (s)"); ax.set_ylabel("Endpoint speed (m/s)")
            ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.suptitle("SYNTHETIC STOP — all 5 duration inputs at fixed initial speeds", fontsize=15)
    fig.text(.5, .012, "Stored CPU primitive outputs. STOP assumed; not a learned decision, road following, collision or control proof.",
        ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .95))
    with (output / "synthetic_stop_speed.png").open("xb") as stream:
        fig.savefig(stream, format="png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(13, 7))
    extent = max(abs(value) for row in xy_rows for c in row["candidates"] for point in c["xy_base_m"] for value in point)
    extent = max(.5, extent * 1.1)
    for ax, row in zip(axes, xy_rows):
        for candidate in row["candidates"]:
            xy = [[0., 0.], *candidate["xy_base_m"]]
            ax.plot([p[0] for p in xy], [p[1] for p in xy],
                label=f"c{candidate['candidate_index']} {candidate['profile']} ({candidate['runtime_gate']['status']})")
        ax.scatter([0.], [0.], marker="s", c="black", s=65, zorder=10, label="Ego anchor (0, 0)")
        ax.set(xlim=(-extent, extent), ylim=(-extent, extent), xlabel="Forward X (m)", ylabel="Left Y (m)",
            title=f"{row['dtype']}: requested v0=4 m/s; duration input z=0")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.25); ax.legend(fontsize=7, loc="lower left")
    fig.suptitle("SYNTHETIC STOP — all 6 candidate XY traces; ego-centered axes", fontsize=15)
    fig.text(.5, .015, "Discrete endpoint-speed integration, not continuous v0*T/2. No scene, learned model, road or actuation evaluation.",
        ha="center", fontsize=9)
    # HH_260906 - Reserve footer space after equal-aspect axes layout so the X labels remain readable.
    fig.tight_layout(rect=(0, .10, 1, .95))
    with (output / "synthetic_stop_xy.png").open("xb") as stream:
        fig.savefig(stream, format="png", dpi=150)
    plt.close(fig)
    require(digest(matplotlib_init.read_bytes()) == matplotlib_init_sha, "matplotlib initializer changed during rendering")
    return {"matplotlib_version": matplotlib.__version__,
        "matplotlib_init_sha256": matplotlib_init_sha,
        "matplotlib_hash_scope": "Installed package initializer only; not a complete rendering dependency archive.",
        "speed_plot_case_indices": [row["case_index"] for row in speed_rows],
        "speed_plot_candidate_index": 0, "xy_plot_case_indices": [row["case_index"] for row in xy_rows],
        "xy_plot_candidate_indices": list(range(6)), "selection_rule": "Fixed requested inputs; no outcome-based selection."}


def write_new(path, raw):
    with Path(path).open("xb") as stream:
        stream.write(raw)


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def publish(input_root, output, expected_summary_sha256, expected_checksums_sha256):
    output = checked_output(output, input_root)
    source_path = Path(__file__)
    source_sha = digest(source_path.read_bytes())
    bundle = authenticate(input_root, expected_summary_sha256, expected_checksums_sha256)
    plot_selection(bundle["rows"])
    output.mkdir(parents=True, exist_ok=False)
    rendered = render(bundle["rows"], output)
    # HH_260906 - Re-read every input after plotting; no publication success manifest is written for a changed source.
    require(read_inventory(input_root, expected_summary_sha256, expected_checksums_sha256) == bundle["raw"], "inputs changed during publication")
    require(digest(source_path.read_bytes()) == source_sha, "publisher source changed during publication")
    for name in ("summary.json", "rows.jsonl"):
        write_new(output / name, bundle["raw"][name])
    provenance = {"schema": PUBLIC_SCHEMA, "status": "PUBLISHED_SYNTHETIC_NOT_ADMITTED",
        "expected_summary_sha256": expected_summary_sha256, "expected_checksums_sha256": expected_checksums_sha256,
        "source_inputs": {name: {"sha256": digest(raw), "size_bytes": len(raw)} for name, raw in sorted(bundle["raw"].items())},
        "source_pins": bundle["summary"]["source_pins"], "publisher_source_sha256": source_sha,
        "full_ledger_recount": bundle["recount"], "numeric_notes": numerical_notes(bundle["rows"]), "plots": rendered,
        "scope": SCOPE, "original_inputs_modified": False, "input_and_publisher_postcheck_pass": True,
        "limits": ["The publisher authenticates and recounts archived results; it does not rerun the primitive or runtime gate.",
            "Runtime PASS and numeric MATCH are archived executed judgments, not model/data/control/road approval.",
            "All 600 candidate records remain in rows.jsonl, including any failures; plotted cases follow fixed requested inputs.",
            "Float32 arithmetic may fall slightly below mathematical -2.9 m/s^2 within declared diagnostic epsilon; runtime tolerances were not changed.",
            "Discrete endpoint-speed integration is not the continuous v0*T/2 braking-distance reference.",
            "STOP is supplied by the caller. No learned STOP decision, inference latency, road following or collision avoidance was tested."]}
    raw_provenance = json_bytes(provenance)
    public_safe(raw_provenance)
    write_new(output / "provenance.json", raw_provenance)
    require(read_inventory(input_root, expected_summary_sha256, expected_checksums_sha256) == bundle["raw"]
        and digest(source_path.read_bytes()) == source_sha, "final input/source check failed")
    names = ("summary.json", "rows.jsonl", "synthetic_stop_speed.png", "synthetic_stop_xy.png", "provenance.json")
    checksum = "".join(f"{digest((output / name).read_bytes())}  {name}\n" for name in sorted(names))
    write_new(output / "SHA256SUMS", checksum.encode("ascii"))
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-summary-sha256", required=True)
    parser.add_argument("--expected-checksums-sha256", required=True)
    args = parser.parse_args(argv)
    try:
        result = publish(args.input_root, args.output_dir, args.expected_summary_sha256, args.expected_checksums_sha256)
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(1, f"STOP publication rejected ({type(error).__name__}); no completion checksum is asserted.\n")
    print(json.dumps({"status": result["status"], "case_count": result["full_ledger_recount"]["reported_case_count"],
        "candidate_count": result["full_ledger_recount"]["reported_candidate_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
