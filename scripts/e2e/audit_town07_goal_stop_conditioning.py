#!/usr/bin/env python3
"""HH_260906 - Join all historical Town07 curvature failures to raw native motion without changing labels or admission."""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[2]
RAW = "artifacts/training/2026-09-08/brake_free_goal_stop_v4/town07_straight_calibration"
DIAG = "artifacts/training/2026-09-08/raw_pre_admission_v4_diagnostic_v1"
SUMMARY_SHA = "c85a95bafe277b15ba1c2b56d258832bef6d9dd31660c8a63b4225167a8b8a68"
FUTURES_SHA = "1d856532424155f04ed479931e068d2cadfacd737d703a126b17847d60dd0dc6"
TRIAL_PINS = {
    "run_001": {"states.jsonl": "8df19ac93f4399c86bcd6d46ffb18c78630d1618177b7457db73b70e5edb56c1",
        "manifest.json": "4a6bb9a25fe0277ab501daa8c0fc374f7ef49c86f1bba828f41404b97d3fed9b"},
    "run_002": {"states.jsonl": "0021f6d7dad86f0bf56160a8f14d9b4432ac165a37c31a1c60ba748be25d05ca",
        "manifest.json": "c8900fa623769f02b5b4788cbf6dd97885494ad965fd1e7c3f7aa55831a1ee8e"},
}
ARCHIVED_SOURCE_PINS = {
    "portable_e2e/model.py": "b72c0fcbaf558254a3e7b02aa90406ed157724a07f63d0ef9f46d0d444f92fc4",
    "portable_e2e/runtime_contract.py": "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3",
    "scripts/e2e/collect_carla_vad_expert.py": "53d213bdef56895a27969cc6dd4eac638ba4fa6697badeaaa3367bd5741f01d0",
}
CURVATURE_LIMIT, MINIMUM_HEADING_DISTANCE, ROUNDING_TOLERANCE = .2, .0001, .000001
EXPECTED = {"run_001": (671, 145, 576, 60), "run_002": (666, 154, 695, 59)}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "expected a regular nonsymlink input")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def pins():
    result = {f"{DIAG}/summary.json": SUMMARY_SHA, f"{DIAG}/future_anchor_audit.jsonl": FUTURES_SHA}
    for trial, values in TRIAL_PINS.items():
        result.update({f"{RAW}/{trial}/episode/{name}": value for name, value in values.items()})
        result.update({f"{RAW}/{trial}/provenance/{name}": value for name, value in ARCHIVED_SOURCE_PINS.items()})
    return result


def verify_inputs():
    expected = pins()
    for name, value in expected.items():
        require(sha(ROOT / name) == value, "historical input differs from the reviewed SHA256")
    return expected


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def f32_spacing(value):
    # HH_260906 - This is representational spacing, not a measurement of CARLA error or proof of the server binary type.
    require(type(value) in (int, float) and math.isfinite(value), "float32 diagnostic input must be finite")
    packed = struct.pack("!f", abs(value))
    rounded = struct.unpack("!f", packed)[0]
    bits = struct.unpack("!I", packed)[0]
    require(bits < 0x7f7fffff, "float32 diagnostic input is outside the finite spacing range")
    above = struct.unpack("!f", struct.pack("!I", bits + 1))[0]
    below = struct.unpack("!f", struct.pack("!I", bits - 1))[0] if bits else 0.
    return {"observed_value_exactly_float32_representable": rounded == abs(value),
        "float32_spacing_upper_bound": max(above - rounded, rounded - below)}


def pose_rounding_bound(state):
    # HH_260906 - Conditional worst-case rounding only: assume each source pose component is nearest-rounded binary32 once.
    pose = state["actor_snapshot_transform_carla"]
    values = {key: f32_spacing(pose[key]) for key in ("x", "y", "z", "yaw", "pitch", "roll")}
    require(all(item["observed_value_exactly_float32_representable"] for item in values.values()),
            "observed actor pose does not meet the stated binary32 representability condition")
    radius = 2.85 / 2.
    xy_error = math.hypot(values["x"]["float32_spacing_upper_bound"] / 2,
        values["y"]["float32_spacing_upper_bound"] / 2)
    angle_error = math.radians((values["yaw"]["float32_spacing_upper_bound"] +
        values["pitch"]["float32_spacing_upper_bound"]) / 2)
    return {"components": values, "rear_planar_position_error_bound_m": xy_error + radius * angle_error,
            "body_yaw_error_bound_rad": math.radians(values["yaw"]["float32_spacing_upper_bound"] / 2)}


def timeline(rows):
    require(rows, "native timeline must not be empty")
    times = [round(row["timestamp"] * 1e9) for row in rows]
    for index, row in enumerate(rows):
        require(type(row["frame"]) is int and all(type(row[k]) in (int, float) and math.isfinite(row[k])
                for k in ("timestamp", "x", "y", "z", "yaw", "vx", "vy", "yaw_rate")), "native motion is malformed")
        if index:
            require(row["frame"] == rows[index - 1]["frame"] + 1 and abs(times[index] - times[index - 1] - 50000000) <= 500,
                    "native timeline has a gap; never bridge missing samples")
    return times


def interpolate(rows, times, target):
    # HH_260906 - Independently reproduce the published linear bracket and wrapped-yaw diagnostic, without calling the converter.
    upper = bisect_right(times, target)
    require(upper > 0 and (upper < len(times) or times[-1] == target), "future timestamp outside native evidence")
    if times[upper - 1] == target:
        row = rows[upper - 1]
        return {key: float(row[key]) for key in ("x", "y", "z", "yaw", "vx", "vy")}
    lower = upper - 1
    ratio = (target - times[lower]) / (times[upper] - times[lower])
    value = {key: rows[lower][key] + ratio * (rows[upper][key] - rows[lower][key]) for key in ("x", "y", "z", "vx", "vy")}
    value["yaw"] = wrap(rows[lower]["yaw"] + ratio * wrap(rows[upper]["yaw"] - rows[lower]["yaw"]))
    return value


def nearest_index(times, target):
    index = bisect_left(times, target)
    index = min((x for x in (index - 1, index) if 0 <= x < len(times)), key=lambda x: abs(times[x] - target))
    require(abs(times[index] - target) <= 74, "historical target/native nearest association exceeds the reviewed 74 ns")
    return index


def point_error_bound(rows, times, target):
    upper = bisect_right(times, target)
    if upper and times[upper - 1] == target:
        return pose_rounding_bound(rows[upper - 1])["rear_planar_position_error_bound_m"]
    require(0 < upper < len(times), "rounding-bound interpolation is out of range")
    fraction = (target - times[upper - 1]) / (times[upper] - times[upper - 1])
    return sum(weight * pose_rounding_bound(rows[index])["rear_planar_position_error_bound_m"]
        for index, weight in ((upper - 1, 1 - fraction), (upper, fraction)))


def direction_bound(error, distance):
    # HH_260906 - A segment shorter than its conditional error radius has no informative angular bound.
    return math.asin(error / distance) if 0 <= error < distance else math.pi


def check_future(anchor, rows, times):
    frame_lookup = {row["frame"]: index for index, row in enumerate(rows)}
    require(anchor["frame"] in frame_lookup, "future anchor has no recorded native frame")
    anchor_index = frame_lookup[anchor["frame"]]
    current = rows[anchor_index]
    require(times[anchor_index] == anchor["anchor_timestamp_ns"], "future anchor timestamp differs from its native frame")
    if anchor["disposition"] != "full_64_point_anchor":
        require(anchor["disposition"] == "tail_label_context_only", "unexpected historical anchor disposition")
        return []
    steps = anchor["diagnostic"]["steps"]
    require(len(steps) == 64 and anchor["valid_points"] == 64 and anchor["valid_mask"] == [True] * 64,
            "historical full-window denominator changed")
    previous_xy, previous_heading = (0., 0.), 0.
    previous_time = anchor["anchor_timestamp_ns"]
    previous_heading_bound = pose_rounding_bound(current)["body_yaw_error_bound_rad"]
    previous_position_bound = point_error_bound(rows, times, previous_time)
    findings = []
    cosine, sine = math.cos(current["yaw"]), math.sin(current["yaw"])
    for index, step in enumerate(steps):
        target = anchor["anchor_timestamp_ns"] + (index + 1) * 100000000
        require(step["index"] == index and step["target_timestamp_ns"] == target, "stored future clock changed")
        state = interpolate(rows, times, target)
        dx, dy = state["x"] - current["x"], state["y"] - current["y"]
        xy = (cosine * dx + sine * dy, -sine * dx + cosine * dy)
        vector = (xy[0] - previous_xy[0], xy[1] - previous_xy[1])
        distance = math.hypot(*vector)
        require(math.isclose(distance / .1, step["metrics"]["xy_speed_limit"][0], rel_tol=1e-8, abs_tol=1e-8),
                "independently reconstructed future XY speed differs")
        curvature = None
        position_bound = point_error_bound(rows, times, target)
        if distance > MINIMUM_HEADING_DISTANCE:
            heading = math.atan2(vector[1], vector[0])
            change = abs(wrap(heading - previous_heading))
            curvature = change / distance
            current_heading_bound = direction_bound(previous_position_bound + position_bound, distance)
            published, flag = step["metrics"]["xy_curvature"]
            require(published is not None and math.isclose(curvature, published, rel_tol=1e-7, abs_tol=1e-8)
                and flag is (curvature > CURVATURE_LIMIT + ROUNDING_TOLERANCE), "published curvature or original flag differs from native reconstruction")
            if flag:
                findings.append({"tick": step["episode_relative_100ms_tick"], "target_timestamp_ns": target,
                    "anchor_frame": anchor["frame"], "anchor_capture_phase": anchor["capture_phase"], "future_index": index,
                    "xy_interval_speed_mps": distance / .1, "xy_displacement_m": distance,
                    "target_endpoint_planar_speed_mps": math.hypot(state["vx"], state["vy"]),
                    "xy_heading_change_rad": change, "xy_curvature_rad_per_m": curvature,
                    "xy_curvature_limit_rad_per_m": CURVATURE_LIMIT,
                    "allowed_heading_change_at_observed_distance_rad": CURVATURE_LIMIT * distance,
                    "conditional_rounding_heading_change_bound_rad": previous_heading_bound + current_heading_bound,
                    "conditional_bound_over_allowed_heading_change": (previous_heading_bound + current_heading_bound) / (CURVATURE_LIMIT * distance),
                    "recomputed_original_violation": True})
            previous_heading, previous_heading_bound = heading, current_heading_bound
        else:
            require(step["metrics"]["xy_curvature"] == [None, False], "unassessed heading was silently marked assessed")
        previous_xy, previous_time, previous_position_bound = xy, target, position_bound
    return findings


def native_witness(finding, rows, times):
    end_index = nearest_index(times, finding["target_timestamp_ns"])
    require(end_index >= 2, "missing native interval start")
    start, middle, end = rows[end_index - 2:end_index + 1]
    dt = (times[end_index] - times[end_index - 2]) / 1e9
    vector = (end["x"] - start["x"], end["y"] - start["y"])
    distance, planar = math.hypot(*vector), math.hypot(end["vx"], end["vy"])
    heading = math.atan2(vector[1], vector[0]) if distance else None
    velocity_heading = wrap(end["yaw"] + math.atan2(end["vy"], end["vx"])) if planar > 0 else None
    controls = [row["current_control"] for row in (start, middle, end)]
    goal = end["goal_stop"]
    source_start, source_end = start["actor_snapshot_transform_carla"], end["actor_snapshot_transform_carla"]
    source_vector = (source_end["x"] - source_start["x"], -(source_end["y"] - source_start["y"]))
    return {**finding, "native_from_frame": start["frame"], "native_to_frame": end["frame"],
        "nearest_native_minus_target_ns": times[end_index] - finding["target_timestamp_ns"],
        "native_target_phase": end["capture_phase"], "native_target_pilot_state": goal["pilot_state"],
        "native_interval_seconds": dt, "native_xy_displacement_m": distance,
        "native_interval_average_xy_speed_mps": distance / dt,
        "actor_reference_displacement_ros_xy_m": list(source_vector),
        "recorded_rear_displacement_ros_xy_m": list(vector),
        "recorded_reference_shift_displacement_contribution_xy_m": [vector[i] - source_vector[i] for i in range(2)],
        "raw_body_planar_endpoint_speeds_mps": [math.hypot(row["vx"], row["vy"]) for row in (start, middle, end)],
        "raw_body_velocity_endpoint_mps": [end["vx"], end["vy"]],
        "world_displacement_heading_rad": heading, "raw_endpoint_velocity_world_heading_rad": velocity_heading,
        "endpoint_velocity_vs_interval_direction_delta_rad": wrap(velocity_heading - heading) if heading is not None and velocity_heading is not None else None,
        "body_yaw_change_over_interval_rad": wrap(end["yaw"] - start["yaw"]),
        "body_yaw_interval_average_rate_radps": wrap(end["yaw"] - start["yaw"]) / dt,
        "body_yaw_rate_endpoint_radps": end["yaw_rate"],
        "api_controls_start_middle_end": controls,
        "api_steering_constant_across_interval": len({c["steer"] for c in controls}) == 1,
        "api_steering_zero_across_interval": all(c["steer"] == 0 for c in controls),
        "wheel_steering_tire_angle_endpoint_rad": end["steering_tire_angle_rad"],
        "remaining_route_m": goal["remaining_route_arc_m"],
        "actor_transform_start_middle_end": [row["actor_snapshot_transform_carla"] for row in (start, middle, end)],
        "world_velocity_start_middle_end_carla": [row["world_velocity_carla"] for row in (start, middle, end)],
        "world_acceleration_start_middle_end_carla": [row["world_acceleration_carla"] for row in (start, middle, end)],
        "world_angular_velocity_start_middle_end_carla_deg_s": [row["world_angular_velocity_carla_deg_s"] for row in (start, middle, end)],
        "pose_resolution": pose_rounding_bound(end)}


def bounds(values):
    return {"minimum": min(values), "maximum": max(values)} if values else None


def summarize_native(witnesses):
    groups = defaultdict(list)
    for row in witnesses:
        groups[f"{row['native_target_phase']}/{row['native_target_pilot_state']}"].append(row)
    return {name: {"unique_100ms_tick_count": len(rows),
        "endpoint_planar_speed_mps": bounds([row["raw_body_planar_endpoint_speeds_mps"][-1] for row in rows]),
        "xy_displacement_m": bounds([row["native_xy_displacement_m"] for row in rows]),
        "xy_heading_change_rad": bounds([row["xy_heading_change_rad"] for row in rows]),
        "body_yaw_change_rad": bounds([row["body_yaw_change_over_interval_rad"] for row in rows]),
        "body_yaw_rate_radps": bounds([row["body_yaw_rate_endpoint_radps"] for row in rows]),
        "api_steer_endpoint": bounds([row["api_controls_start_middle_end"][-1]["steer"] for row in rows]),
        "constant_api_steering_interval_count": sum(row["api_steering_constant_across_interval"] for row in rows),
        "zero_api_steering_interval_count": sum(row["api_steering_zero_across_interval"] for row in rows),
        "conditional_rounding_bound_over_allowed_heading_change": bounds([row["conditional_bound_over_allowed_heading_change"] for row in rows]),
        "conditional_rounding_bound_exceeds_allowed_heading_step_count": sum(row["conditional_bound_over_allowed_heading_change"] > 1 for row in rows),
        "observed_heading_exceeds_conditional_bound_plus_fixed_distance_allowance_count": sum(
            row["xy_heading_change_rad"] > row["conditional_rounding_heading_change_bound_rad"] +
            row["allowed_heading_change_at_observed_distance_rad"] for row in rows),
        "maximum_curvature_witness": max(rows, key=lambda row: row["xy_curvature_rad_per_m"])}
        for name, rows in sorted(groups.items())}


def analyze():
    inputs = verify_inputs()
    original = json.loads((ROOT / DIAG / "summary.json").read_text())
    require(original["status"] == "DIAGNOSED_NOT_ADMITTED" and original["trial_count"] == 2,
            "original raw diagnosis is not the reviewed two-trial report")
    records, times, futures = {}, {}, defaultdict(list)
    counts = defaultdict(Counter)
    for trial in TRIAL_PINS:
        records[trial] = [json.loads(line) for line in (ROOT / RAW / trial / "episode/states.jsonl").read_text().splitlines()]
        times[trial] = timeline(records[trial])
    with (ROOT / DIAG / "future_anchor_audit.jsonl").open() as stream:
        for line in stream:
            anchor = json.loads(line); trial = anchor["trial"]
            require(trial in TRIAL_PINS, "unknown trial in frozen future report")
            counts[trial][anchor["disposition"]] += 1
            findings = check_future(anchor, records[trial], times[trial])
            if findings: counts[trial]["violating_anchor_count"] += 1
            futures[trial].extend(findings)
    result, joined = [], []
    for trial in TRIAL_PINS:
        selected = {}
        multiplicity = Counter(row["tick"] for row in futures[trial])
        for row in futures[trial]:
            if row["tick"] not in selected or row["xy_curvature_rad_per_m"] > selected[row["tick"]]["xy_curvature_rad_per_m"]:
                selected[row["tick"]] = row
        expected = EXPECTED[trial]
        require((counts[trial]["full_64_point_anchor"], counts[trial]["violating_anchor_count"], len(futures[trial]), len(selected)) == expected,
                "recomputed full-window, failure or unique-tick counts differ")
        witnesses = [dict(native_witness(row, records[trial], times[trial]), trial=trial,
            repeated_violating_window_point_count=multiplicity[tick]) for tick, row in sorted(selected.items())]
        joined.extend(witnesses)
        result.append({"trial": trial, "all_native_state_count": len(records[trial]), "anchor_disposition_counts": dict(counts[trial]),
            "original_scalar_quality_pass": original["trials"][list(TRIAL_PINS).index(trial)]["raw_scalar_quality"]["raw_scalar_quality_clear"],
            "violating_window_point_count": len(futures[trial]), "unique_100ms_tick_count": len(witnesses),
            "first_future_point_violation_count": sum(row["future_index"] == 0 for row in futures[trial]),
            "later_future_point_violation_count": sum(row["future_index"] > 0 for row in futures[trial]),
            "violating_window_point_xy_speed_mps": bounds([row["xy_interval_speed_mps"] for row in futures[trial]]),
            "violating_future_endpoint_planar_speed_mps": bounds([row["target_endpoint_planar_speed_mps"] for row in futures[trial]]),
            "maximum_nearest_native_timestamp_offset_ns": max(abs(row["nearest_native_minus_target_ns"]) for row in witnesses),
            "unique_tick_groups": summarize_native(witnesses)})
    require(verify_inputs() == inputs, "historical inputs changed during diagnosis")
    return {"schema": "carla_expert.town07_goal_stop_conditioning.v1", "status": "DIAGNOSED_NOT_ADMITTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "input_sha256": inputs,
        "original_diagnostic_source_identity": original["source_identity"], "trials": result,
        "scope": {"model_loaded": False, "live_simulator_access": False, "test_payload_read": False,
            "source_data_or_labels_modified": False, "training_data_approved": False, "gate_modified": False,
            "all_full_64point_windows_reconstructed": True, "all_original_curvature_flags_preserved": True},
        "limitations": ["Unique ticks are not independent stopping events; the two trials share the same route and conditions.",
            "All 64 future points are independently reconstructed. The joined unique-tick witness is the largest-curvature occurrence, not a selected passing example.",
            "Nearest native state association is at most 74 ns; original future timestamps/interpolation and state rows are not retimed.",
            "Body endpoint velocity, 100 ms displacement direction and body yaw are distinct observables; discrepancies alone do not establish control delay or physical sensor error.",
            "Binary32 spacing is inferred from exactly representable stored actor-pose numbers, not independently verified server arithmetic or physical precision.",
            "The conditional rounding bound assumes one nearest-rounded binary32 error per actor pose component and the recorded 1.425 m shift; it is not a measured error, noise estimate or cause verdict.",
            "Comparison with the observed-distance heading allowance is descriptive, not a new gate or a complete interval-arithmetic feasibility proof. Some observed heading changes exceed even that allowance plus the conditional rounding bound.",
            "A bound larger than the allowed heading step shows poor numerical conditioning only; it does not excuse any curvature violation or permit label smoothing, sample deletion or admission.",
            "API-reported steering and wheel angle do not prove realized tire force or actuator timing. Snapshot reference point is not independently proven physical COM/rear axle."]}, joined


def publish(output):
    output = Path(output).absolute()
    require(not output.exists() and not any(p.is_symlink() for p in (output, *output.parents)), "output must be a fresh nonsymlink directory")
    forbidden = (ROOT / RAW, ROOT / DIAG, ROOT / "datasets", (ROOT / "datasets").resolve())
    require(not any(output.resolve() == p or p in output.resolve().parents for p in forbidden), "output cannot be inside historical inputs or datasets")
    analyzer_sha = sha(Path(__file__))
    report, rows = analyze()
    require(sha(Path(__file__)) == analyzer_sha, "analyzer changed during diagnosis")
    report["analyzer"] = {"path": "scripts/e2e/audit_town07_goal_stop_conditioning.py", "sha256": analyzer_sha,
        "same_bytes_before_after": True, "commit_identity_claimed": False}
    output.mkdir(parents=True, exist_ok=False)
    with (output / "unique_ticks.jsonl").open("x") as stream:
        stream.write("".join(json.dumps(row, sort_keys=True, allow_nan=False) + "\n" for row in rows))
    with (output / "summary.json").open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")
    require(verify_inputs() == report["input_sha256"] and sha(Path(__file__)) == analyzer_sha,
            "inputs or analyzer changed before final checksum publication")
    with (output / "SHA256SUMS").open("x") as stream:
        stream.write("".join(f"{sha(output / name)}  {name}\n" for name in ("summary.json", "unique_ticks.jsonl")))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = publish(args.output_dir)
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(json.dumps({"status": "INCOMPLETE", "error_type": type(error).__name__}))
        return 1
    print(json.dumps({"status": result["status"], "unique_tick_count": sum(t["unique_100ms_tick_count"] for t in result["trials"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
