#!/usr/bin/env python3
"""HH_260906 - Measure fixed raw micro-motion without repairing labels, relaxing gates or admitting data."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
PINS = {
    "summary.json": "7f1ed0bd40c7b1ec531e3402f3b90ad21eccc4e45ac513fb6d69b5fd953bce33",
    "future_geometry_audit.jsonl": "bbf894187bd129146da4515e513341b97795ed470fccce49ab25fbe5819353f9",
    "native_snapshot_audit.jsonl": "0f4ff705919ed25db8bc2bba615506a497ab05c66a77a3928307ddfe7112124e",
    "jpeg_audit.jsonl": "417d7f4c205bf0de8d0870961d178ca86b2425d2ae52128c4d0626daf9d4ba18",
    "SHA256SUMS": "1df3034160cd3a9d3b4d686cb57415aba130dbd37f9d63649d388e5168fcc738",
}
ORDER = (15, 13, 14, 12, 12, 14, 13, 15)
NATIVE_COUNTS = (1939, 1939, 1935, 1920, 1920, 1935, 1939, 1939)
CAMERA_COUNTS = (970, 970, 968, 960, 960, 968, 970, 970)
FULL_COUNTS = (905, 905, 903, 895, 895, 903, 905, 905)
CURVATURE_ANCHORS = (215, 161, 205, 199, 199, 205, 161, 215)
PHASES = ("stationary_warmup", "driving", "stationary_tail")
HEADING_MINIMUM_STEP_M = 1.0e-4
ORIGINAL_CURVATURE_LIMIT = 0.2
ORIGINAL_ROUNDING_TOLERANCE = 1.0e-6
SCHEMA = "carla_expert.micro_motion_sidecar.v1"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite non-Boolean number required")
    return float(value)


def integer(value):
    require(type(value) is int and value >= 0, "nonnegative integer required")
    return value


def vector(value, size=3):
    require(isinstance(value, list) and len(value) == size, "vector length differs")
    return [number(x) for x in value]


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
        parse_constant=lambda _: require(False, "nonfinite JSON constant"))


def rows(path):
    with path.open() as stream:
        for line in stream:
            require(line.strip(), "empty JSONL row")
            yield load_json(line)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def emit(stream, value):
    stream.write(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n")


def expected_cases():
    repeats = Counter()
    result = []
    for index, pedal in enumerate(ORDER, 1):
        repeats[pedal] += 1
        result.append(f"{index:02d}_turn_launch_0{pedal}_v1_r{repeats[pedal]}")
    return result


def input_pins(root):
    require(root.is_dir() and not root.is_symlink(), "input directory missing or symlinked")
    require({p.name for p in root.iterdir()} == set(PINS), "input inventory differs")
    result = {}
    for name, expected in PINS.items():
        path = root / name
        require(path.is_file() and not path.is_symlink(), "input file missing or symlinked")
        actual = sha(path)
        require(actual == expected, "reviewed original input SHA differs: " + name)
        result[name] = {"sha256": actual, "size_bytes": path.stat().st_size}
    return result


def validate_summary(summary):
    require(summary["schema"] == "carla_expert.turn_launch_raw_geometry_diagnostic.v1"
        and summary["status"] == "DIAGNOSED_NOT_ADMITTED", "unreviewed original diagnostic")
    require(type(summary["planned_case_count"]) is int and summary["planned_case_count"] == 8
        and type(summary["finalized_case_count"]) is int and summary["finalized_case_count"] == 8
        and summary["all_planned_cases_retained"] is True, "eight finalized cases required")
    for key in ("training_data_approved", "dataset_admission", "common10_dataset_written", "labels_modified",
                "training", "model_loaded", "model_inference", "test_payload_read", "automatic_winner_selection"):
        require(summary["scope"][key] is False, "original denial/scope changed")
    cases = summary["cases"]
    require([c["case_id"] for c in cases] == expected_cases(), "original case order differs")
    for i, case in enumerate(cases):
        require(case["original_training_data_approved"] is False and case["original_development_only"] is True
            and case["training_data_approved"] is False, "case denial changed")
        future = case["measured_future"]
        require(integer(case["raw_native_state_count"]) == NATIVE_COUNTS[i]
            and integer(future["camera_anchor_count"]) == CAMERA_COUNTS[i]
            and integer(future["available_64_point_anchor_count"]) == FULL_COUNTS[i]
            and integer(future["bound_assessed_64_point_anchor_count"]) == FULL_COUNTS[i]
            and integer(future["bound_unassessed_anchor_count"]) == 0, "frozen case denominator differs")
        require(integer(future["all_available_prefixes"]["64"]["metrics"]["xy_curvature"]
            ["horizon_eligible_violating_anchor_count"]) == CURVATURE_ANCHORS[i], "original curvature verdict count differs")
    return {case["case_id"]: case for case in cases}


def native_sample(row):
    require(row["capture_phase"] in PHASES, "unknown native phase")
    stamp, frame = integer(row["timestamp_ns"]), integer(row["frame"])
    api, velocity = vector(row["actor_api_reference_ros_xyz_m"]), vector(row["endpoint_actor_api_velocity_ros_mps"])
    rear, raw_yaw = vector(row["recorded_rear_ros_xyz_m"]), number(row["body_yaw_ros_rad"])
    original = row["original_recorded_planar_state"]
    yaw = number(original["yaw"])
    require(rear == [number(original[k]) for k in ("x", "y", "z")], "rear snapshot association differs")
    transform, world = row["source_actor_snapshot_transform_carla"], vector(row["source_world_velocity_carla"])
    # HH_260906 - Original yaw is wrapped; retain that exact recorded angle for the legacy velocity rotation.
    require(raw_yaw == -math.radians(number(transform["yaw"])) and yaw == wrapped(raw_yaw), "original yaw wrapping differs")
    require(api == [number(transform["x"]), -number(transform["y"]), number(transform["z"])]
        and velocity == [world[0], -world[1], world[2]], "API reference conversion differs")
    vx, vy = number(original["vx"]), number(original["vy"])
    legacy_world = [math.cos(yaw) * vx - math.sin(yaw) * vy, math.sin(yaw) * vx + math.cos(yaw) * vy]
    return {"frame": frame, "timestamp_ns": stamp, "phase": row["capture_phase"], "api": api,
        "api_velocity": velocity, "rear": rear, "legacy_world_velocity": legacy_world}


def subtract(a, b):
    return [x - y for x, y in zip(a, b)]


def speed_bin(speed):
    return "lt0p1" if speed < .1 else "0p1_to_0p5" if speed < .5 else "0p5_to_1" if speed < 1. else "ge1"


def integrate(samples, field, size):
    # HH_260906 - Both 100 ms offsets integrate every intermediate native sample, not only interval endpoints.
    left, right = [0.] * size, [0.] * size
    for a, b in zip(samples, samples[1:]):
        dt = (b["timestamp_ns"] - a["timestamp_ns"]) / 1e9
        require(b["frame"] == a["frame"] + 1 and abs(dt - .05) <= 5e-7, "native interval gap or cadence differs")
        for i in range(size):
            left[i] += a[field][i] * dt
            right[i] += b[field][i] * dt
    return {"left": left, "right": right, "trapezoid": [(a + b) / 2 for a, b in zip(left, right)]}


def interval_record(samples, series):
    require(len(samples) in (2, 3), "interval requires two or three native samples")
    first, last = samples[0], samples[-1]
    dt = (last["timestamp_ns"] - first["timestamp_ns"]) / 1e9
    api_delta, rear_delta = subtract(last["api"], first["api"]), subtract(last["rear"], first["rear"])
    api_integrals = integrate(samples, "api_velocity", 3)
    rear_integrals = integrate(samples, "legacy_world_velocity", 2)
    references = {}
    for name, delta, integrals in (("actor_api_reference", api_delta[:2], api_integrals),
                                  ("recorded_rear_legacy_planar", rear_delta[:2], rear_integrals)):
        residuals = {key: subtract(delta, value[:2]) for key, value in integrals.items()}
        references[name] = {"delta_xy_m": delta, "xy_speed_bin": speed_bin(math.hypot(*delta) / dt),
            "integrals_xy_m": {key: value[:2] for key, value in integrals.items()}, "residual_xy_m": residuals,
            "residual_norm_m": {key: math.hypot(*value) for key, value in residuals.items()},
            "residual_norm_over_dt_mps": {key: math.hypot(*value) / dt for key, value in residuals.items()}}
    offset_delta = subtract(subtract(last["rear"], last["api"]), subtract(first["rear"], first["api"]))
    difference = subtract(rear_delta, api_delta)
    return {"series": series, "first_frame": first["frame"], "last_frame": last["frame"],
        "first_timestamp_ns": first["timestamp_ns"], "target_timestamp_ns": last["timestamp_ns"], "dt_s": dt,
        "native_phases": [x["phase"] for x in samples], "crosses_phase": first["phase"] != last["phase"],
        "references": references, "api_z_separate": {"delta_m": api_delta[2],
            "integrals_m": {key: value[2] for key, value in api_integrals.items()},
            "residual_m": {key: api_delta[2] - value[2] for key, value in api_integrals.items()}},
        "reference_offset_identity": {"rear_delta_minus_api_delta_xyz_m": difference,
            "delta_rear_minus_api_offset_xyz_m": offset_delta,
            "floating_residual_xyz_m": subtract(difference, offset_delta)}}


def describe(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "rms": None, "p50": None, "p95": None, "maximum": None}
    def percentile(p):
        x = p * (len(values) - 1); lo, hi = math.floor(x), math.ceil(x)
        return values[lo] * (hi - x) + values[hi] * (x - lo) if hi != lo else values[lo]
    return {"count": len(values), "rms": math.sqrt(math.fsum(x*x for x in values) / len(values)),
        "p50": percentile(.5), "p95": percentile(.95), "maximum": values[-1]}


def wrapped(value):
    return math.atan2(math.sin(value), math.cos(value))


def future_direction_records(row):
    # HH_260906 - The old 100 ms heading state starts at zero and is retained, not reset, through tiny displacements.
    require(row["training_data_approved"] is False and integer(row["valid_future_points"]) == 64, "full denied future required")
    fields = ("diagnostic_future_xy_m", "diagnostic_body_yaw_delta_rad", "diagnostic_target_timestamp_ns", "valid_mask", "invalid_reasons")
    require(all(isinstance(row[k], list) and len(row[k]) == 64 for k in fields), "future arrays must contain all 64 indices")
    require(all(v is True for v in row["valid_mask"]) and all(v is None for v in row["invalid_reasons"]), "full future mask/reason differs")
    anchor = integer(row["anchor_timestamp_ns"])
    points = [[0., 0.]] + [vector(p, 2) for p in row["diagnostic_future_xy_m"]]
    yaw = [number(v) for v in row["diagnostic_body_yaw_delta_rad"]]
    steps = row["discrete_bound_diagnostic"]["steps"]
    require(len(steps) == 64 and integer(row["discrete_bound_diagnostic"]["valid_points"]) == 64
        and integer(row["discrete_bound_diagnostic"]["invalid_points"]) == 0, "original per-index verdict denominator differs")
    previous_heading = 0.
    for index in range(64):
        stamp, old = integer(row["diagnostic_target_timestamp_ns"][index]), steps[index]
        require(stamp == anchor + (index + 1) * 100_000_000 and integer(old["index"]) == index
            and integer(old["target_timestamp_ns"]) == stamp and number(old["dt_s"]) == .1, "future grid/index differs")
        require(old["phase"] in ("pre_tail", "stationary_tail"), "unknown original target phase")
        windows = {}
        for count in (1, 2, 5):
            if index + 1 < count:
                windows[str(count)] = {"status": "INSUFFICIENT_WITHIN_ANCHOR_PREFIX", "heading_rad": None}
                continue
            delta = subtract(points[index + 1], points[index + 1 - count]); distance = math.hypot(*delta)
            windows[str(count)] = {"status": "ASSESSED" if distance > HEADING_MINIMUM_STEP_M else "TINY_DISPLACEMENT_UNASSESSED",
                "delta_xy_m": delta, "distance_m": distance, "duration_s": count / 10,
                "heading_rad": math.atan2(delta[1], delta[0]) if distance > HEADING_MINIMUM_STEP_M else None}
        base = windows["1"]; heading = base["heading_rad"]; before = previous_heading
        curvature = abs(wrapped(heading - before)) / base["distance_m"] if heading is not None else None
        failed = curvature is not None and curvature > ORIGINAL_CURVATURE_LIMIT + ORIGINAL_ROUNDING_TOLERANCE
        value, flag = old["metrics"]["xy_curvature"]
        require(type(flag) is bool and flag is failed, "original curvature Boolean verdict differs")
        require(value is None if curvature is None else value is not None and math.isclose(number(value), curvature, rel_tol=1e-12, abs_tol=1e-12), "original curvature value/availability differs")
        if heading is not None:
            previous_heading = heading
        comparisons = {}
        for count in (2, 5):
            other = windows[str(count)]["heading_rad"]
            comparisons[f"direction_{count}00ms_vs_100ms_abs_rad"] = None if heading is None or other is None else abs(wrapped(other - heading))
        for count in (1, 2, 5):
            other = windows[str(count)]["heading_rad"]
            comparisons[f"direction_{count}00ms_vs_endpoint_body_yaw_proxy_abs_rad"] = None if other is None else abs(wrapped(other - yaw[index]))
        yield {"index": index, "target_timestamp_ns": stamp, "original_episode_relative_100ms_tick": integer(old["episode_relative_100ms_tick"]),
            "original_target_tail_classification": old["phase"], "xy_speed_bin_100ms": speed_bin(base["distance_m"] / .1),
            "windows": windows, "endpoint_body_yaw_proxy_rad": yaw[index], "comparisons": comparisons,
            "original_heading_state_before_rad": before, "original_heading_state_after_rad": previous_heading,
            "original_100ms_curvature_rad_per_m": curvature, "original_curvature_failed": failed,
            "coarse_interval_pass_claimed": False}


def target_bracket(samples, timestamps, stamp):
    # HH_260906 - A future timestamp can lie between two native phases; report both without inventing a single target phase.
    index = bisect_left(timestamps, stamp)
    require(index < len(samples), "target exceeds native observations")
    if timestamps[index] == stamp:
        selected, kind = [samples[index]], "exact_native_timestamp"
    else:
        require(index > 0, "target precedes native observations")
        selected, kind = samples[index-1:index+1], "native_timestamp_bracket"
    return {"association": kind, "frames": [x["frame"] for x in selected],
        "timestamps_ns": [x["timestamp_ns"] for x in selected], "phases": [x["phase"] for x in selected]}


def analyze(root, output):
    root, output = Path(root), Path(output)
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(root.resolve())
        and not output.resolve().is_relative_to((ROOT / "datasets").resolve()), "output must be fresh and outside original data")
    pins = input_pins(root); summary = load_json((root / "summary.json").read_bytes()); cases = validate_summary(summary)
    source = Path(__file__).read_bytes(); source_sha = hashlib.sha256(source).hexdigest()
    output.mkdir(parents=True, exist_ok=False); (output / "executed_source.py").write_bytes(source)
    status = {"schema": SCHEMA, "status": "RUNNING", "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "input_pins": pins, "source_sha256": source_sha, "training_data_approved": False}
    write_json(output / "execution_status.json", status)
    try:
        native = {key: [] for key in cases}; sequence = []
        for row in rows(root / "native_snapshot_audit.jsonl"):
            key = row["case_id"]; require(key in cases, "unknown native case")
            if not sequence or sequence[-1] != key: sequence.append(key)
            native[key].append(native_sample(row))
        require(sequence == list(cases), "native case order differs")
        stats = defaultdict(lambda: defaultdict(list)); interval_counts = Counter(); max_identity = 0.
        with (output / "native_intervals.jsonl").open("w") as stream:
            for key, samples in native.items():
                require(len(samples) == cases[key]["raw_native_state_count"] and dict(Counter(s["phase"] for s in samples))
                    == cases[key]["raw_native_state_counts_by_phase"], "native count/phase denominator differs")
                require([s["phase"] for i, s in enumerate(samples) if i == 0 or s["phase"] != samples[i-1]["phase"]]
                    == list(PHASES), "native phase progression differs")
                for series, start, stride in (("native_20hz", 0, 1), ("derived_10hz_offset0", 0, 2), ("derived_10hz_offset1", 1, 2)):
                    for index in range(start, len(samples) - stride, stride):
                        item = interval_record(samples[index:index+stride+1], series); item["case_id"] = key
                        emit(stream, item); interval_counts[key + "/" + series] += 1
                        max_identity = max(max_identity, *map(abs, item["reference_offset_identity"]["floating_residual_xyz_m"]))
                        phase = ">".join(dict.fromkeys(item["native_phases"]))
                        for ref, data in item["references"].items():
                            group = "/".join((key, series, phase, ref, data["xy_speed_bin"]))
                            for metric in ("residual_norm_m", "residual_norm_over_dt_mps"):
                                for rule, value in data[metric].items(): stats[group][metric + "/" + rule].append(value)
        native_report = {key: {m: describe(values) for m, values in group.items()} for key, group in stats.items()}
        del stats
        native_frames = {key: {x["frame"]: x for x in value} for key, value in native.items()}
        native_times = {key: [x["timestamp_ns"] for x in value] for key, value in native.items()}
        counts = Counter(); phases = defaultdict(Counter); full = Counter(); violations = Counter(); previous = {}; seen_order = []
        groups = {}; exact_ticks = set(); rounded_ticks = set(); failed_exact = set(); failed_rounded = set()
        target_phase_counts = Counter()
        with (output / "future_directions.jsonl").open("w") as stream:
            for row in rows(root / "future_geometry_audit.jsonl"):
                key = row["case_id"]; require(key in cases and row["training_data_approved"] is False, "unknown/approved future row")
                if not seen_order or seen_order[-1] != key: seen_order.append(key)
                frame, stamp = integer(row["frame"]), integer(row["anchor_timestamp_ns"])
                require(key not in previous or frame == previous[key] + 2, "future anchor cadence differs")
                previous[key] = frame; source_row = native_frames[key].get(frame)
                require(source_row is not None and source_row["timestamp_ns"] == stamp and source_row["phase"] == row["capture_phase"], "future/native anchor association differs")
                counts[key] += 1; phases[key][row["capture_phase"]] += 1
                if row["capture_phase"] == "stationary_tail":
                    require(integer(row["valid_future_points"]) == 0 and row["disposition"] == "tail_label_context_only", "tail context was made a target")
                    continue
                require(row["disposition"] == "full_64_point_anchor", "non-tail future unexpectedly unavailable")
                full[key] += 1; any_failed = False
                for item in future_direction_records(row):
                    require(item["target_timestamp_ns"] <= native[key][-1]["timestamp_ns"], "future exceeds native record")
                    tick = round((item["target_timestamp_ns"] - native[key][0]["timestamp_ns"]) / 1e8)
                    require(tick == item["original_episode_relative_100ms_tick"], "original rounded target tick differs")
                    item.update(case_id=key, anchor_frame=frame, anchor_timestamp_ns=stamp, anchor_phase=row["capture_phase"])
                    bracket = target_bracket(native[key], native_times[key], item["target_timestamp_ns"])
                    item["target_native_bracket"] = bracket
                    signature = ">".join(dict.fromkeys(bracket["phases"]))
                    target_phase_counts["/".join((key, signature, item["xy_speed_bin_100ms"]))] += 1
                    emit(stream, item)
                    exact = (key, item["target_timestamp_ns"]); rounded = (key, tick)
                    exact_ticks.add(exact); rounded_ticks.add(rounded)
                    if item["original_curvature_failed"]:
                        any_failed = True; failed_exact.add(exact); failed_rounded.add(rounded)
                    group_key = "/".join((key, row["capture_phase"], item["xy_speed_bin_100ms"], str(item["index"])))
                    group = groups.setdefault(group_key, {"count": 0, "curvature_failed": 0, "window_status_counts": Counter(),
                        "comparisons": defaultdict(list), "exact": set(), "rounded": set()})
                    group["count"] += 1; group["curvature_failed"] += item["original_curvature_failed"]
                    group["exact"].add(item["target_timestamp_ns"]); group["rounded"].add(tick)
                    for window, data in item["windows"].items(): group["window_status_counts"][window + "/" + data["status"]] += 1
                    for metric, value in item["comparisons"].items():
                        if value is not None: group["comparisons"][metric].append(value)
                stream.flush(); violations[key] += any_failed
        require(seen_order == list(cases), "future case order differs")
        for index, key in enumerate(cases):
            require(counts[key] == CAMERA_COUNTS[index] and full[key] == FULL_COUNTS[index]
                and violations[key] == CURVATURE_ANCHORS[index] and dict(phases[key]) == cases[key]["measured_future"]["anchor_count_by_phase"], "future or original failure denominator differs")
        direction_report = {}
        for key, group in groups.items():
            direction_report[key] = {"count": group["count"], "original_curvature_failed": group["curvature_failed"],
                "window_status_counts": dict(group["window_status_counts"]),
                "comparison_stats": {m: describe(v) for m, v in group["comparisons"].items()},
                "unique_exact_target_timestamps": len(group["exact"]), "unique_original_rounded_100ms_ticks": len(group["rounded"])}
        require(input_pins(root) == pins and Path(__file__).read_bytes() == source, "input/source changed during study")
        result = {"schema": SCHEMA, "status": "MEASURED_NOT_ADMITTED", "input_pins": pins, "source_sha256": source_sha,
            "python_version": sys.version, "native_state_count": sum(map(len, native.values())), "native_interval_counts": dict(interval_counts),
            "native_residual_groups": native_report, "offset_identity_maximum_floating_residual_m": max_identity,
            "camera_anchor_count": sum(counts.values()), "full_future_anchor_count": sum(full.values()),
            "future_index_row_count": sum(full.values()) * 64, "original_curvature_violating_anchors_by_case": dict(violations),
            "future_target_native_phase_bin_counts": dict(target_phase_counts),
            "future_direction_groups": direction_report, "unique_exact_target_timestamps": len(exact_ticks),
            "unique_original_rounded_100ms_ticks": len(rounded_ticks), "original_failed_unique_exact_timestamps": len(failed_exact),
            "original_failed_unique_rounded_100ms_ticks": len(failed_rounded), "input_and_source_postcheck_pass": True,
            "scope": {"training_data_approved": False, "dataset_admission": False, "labels_modified": False,
                "model_loaded": False, "training": False, "live_simulator_access": False, "thresholds_changed": False},
            "limitations": ["Actor API reference is not proven COM; legacy rear planar velocity is not full 3D rigid-body ground truth.",
                "XY integration residuals and API Z settling are separate; none proves a physics or sensor cause.",
                "100 ms native parity is relative to each first recorded row; all phase boundaries and failures remain.",
                "Trailing 200/500 ms directions cannot yield coarse-interval PASS or repair original 100 ms curvature flags.",
                "Exact target timestamps can differ by nanoseconds across overlapping anchor grids; rounded ticks are also reported. Neither count denotes independent events.",
                "Endpoint body yaw is a direction proxy, not measured trajectory heading. Original tiny-step heading state is retained."]}
        write_json(output / "summary.json", result)
        status.update(status="COMPLETE", ended_at_utc=datetime.now(timezone.utc).isoformat(), input_and_source_postcheck_pass=True)
        write_json(output / "execution_status.json", status)
        (output / "SHA256SUMS").write_text("".join(f"{sha(p)}  {p.name}\n" for p in sorted(output.iterdir()) if p.is_file()))
        return result
    except BaseException as error:
        status.update(status="FAILED", error_type=type(error).__name__, error=str(error), ended_at_utc=datetime.now(timezone.utc).isoformat())
        # HH_260906 - Failure preserves partial streams and records whether original input/source bytes still match.
        status["source_postcheck_pass"] = Path(__file__).read_bytes() == source
        try:
            status["input_postcheck_pass"] = input_pins(root) == pins
        except Exception as postcheck_error:
            status["input_postcheck_pass"] = False
            status["input_postcheck_error"] = str(postcheck_error)
        write_json(output / "execution_status.json", status)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    result = analyze(args.input_root, args.output_dir)
    print(json.dumps({k: result[k] for k in ("status", "native_state_count", "camera_anchor_count", "full_future_anchor_count", "future_index_row_count")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
