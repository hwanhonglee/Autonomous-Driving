#!/usr/bin/env python3
"""HH_260906 - Explain sealed v5 curvature failures without changing targets, gates or admission."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import stat
import struct

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "carla.curvature_conditioning_diagnostic.v1"
DT_NS = 100_000_000
MINIMUM_STEP_M = 1e-4
MAXIMUM_CURVATURE = .2
ROUNDING_TOLERANCE = 1e-6
PHASES = ("stationary_warmup", "driving", "stationary_tail")
MAX_BYTES = 256 * 1024 * 1024
# HH_260906 - Read these source bytes, never import model/torch/CARLA or execute the old auditing entry points.
MATH_SOURCE_PINS = {
    "portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "scripts/e2e/audit_portable_target_feasibility.py": "8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894",
    "scripts/e2e/prepare_carla_common10_dataset.py": "c63c7e5b7e0c06336c885f2d6120e3e8a8a74daf372ddf5df167eac2b4b4b666",
    "scripts/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "scripts/e2e/audit_carla_raw_pre_admission.py": "0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00",
}
SOURCE_PATHS = set(MATH_SOURCE_PINS) | {
    "portable_e2e/config/common_10hz_v1.contract.json", "portable_e2e/contract.py",
    "portable_e2e/runtime_contract.py", "scripts/e2e/audit_carla_brake_free_goal_stop.py",
    "scripts/e2e/audit_carla_comfortable_v3_trial.py", "scripts/e2e/summarize_carla_goal_stop_trials.py",
    "artifacts/training/2026-09-14/brake_hold_goal_stop_v5/summarize_pilot.py",
    "artifacts/training/2026-09-14/brake_hold_goal_stop_v5/README.public.md",
}
RAW_PATHS = {"owner_plan.json", "owner_result.json", "owner_started.json", "lifecycle/stopped.json",
    "episode/manifest.json", "episode/route.json", "episode/states.jsonl",
    "episode/camera_frames.jsonl", "episode/control_receipts.jsonl"}


class EvidenceError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise EvidenceError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "nonfinite or nonnumeric observation")
    return float(value)


def integer(value):
    require(type(value) is int and value >= 0, "invalid nonnegative integer")
    return value


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def safe_path(path):
    require(".." not in Path(path).parts, "parent traversal is not an evidence path")
    path = Path(path).absolute()
    require(not any(p.is_symlink() for p in (path, *path.parents)), "symlink path is not evidence")
    return path


def read(path, expected, ledger):
    require(isinstance(expected, str) and re.fullmatch(r"[0-9a-f]{64}", expected), "invalid SHA-256 pin")
    path = safe_path(path)
    info = path.stat()
    require(stat.S_ISREG(info.st_mode) and info.st_size <= MAX_BYTES, "bounded regular file required")
    with path.open("rb") as stream:
        payload = stream.read(MAX_BYTES + 1)
    require(len(payload) <= MAX_BYTES and sha(payload) == expected, f"SHA-256 mismatch: {path}")
    ledger[str(path)] = expected
    return payload


def loads(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def bad_constant(value):
        raise EvidenceError(f"nonfinite JSON constant: {value}")
    return json.loads(payload, object_pairs_hook=pairs, parse_constant=bad_constant,
        parse_float=lambda token: number(float(token)))


def json_lines(payload):
    rows = [loads(line) for line in payload.splitlines() if line.strip()]
    require(rows and all(isinstance(row, dict) for row in rows), "nonempty object JSONL required")
    return rows


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def stamp(row):
    result = round(number(row["timestamp"]) * 1e9)
    return integer(result)


def actor_xy(row):
    pose = row["actor_snapshot_transform_carla"]
    return [number(pose["x"]), -number(pose["y"])]


def validate_states(states, wheelbase):
    previous_phase, maximum = 0, 0.
    for index, row in enumerate(states):
        integer(row["frame"])
        require(row["capture_phase"] in PHASES, "unknown capture phase")
        phase = PHASES.index(row["capture_phase"])
        require(phase >= previous_phase, "reversed capture phase")
        previous_phase = phase
        for key in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate"):
            number(row[key])
        pose = row["actor_snapshot_transform_carla"]
        require(set(pose) == {"x", "y", "z", "roll", "pitch", "yaw"}, "incomplete actor pose")
        for value in pose.values():
            number(value)
        for name in ("world_velocity_carla", "world_acceleration_carla", "world_angular_velocity_carla_deg_s"):
            require(isinstance(row[name], list) and len(row[name]) == 3, "invalid raw vector")
            for value in row[name]:
                number(value)
        yaw, pitch, half = math.radians(pose["yaw"]), math.radians(pose["pitch"]), wheelbase / 2
        c, s = math.cos(yaw), math.sin(yaw)
        expected = [pose["x"] - half * math.cos(pitch) * c,
            -(pose["y"] - half * math.cos(pitch) * s), wrap(-yaw)]
        residual = max(abs(row["x"] - expected[0]), abs(row["y"] - expected[1]), abs(wrap(row["yaw"] - expected[2])))
        maximum = max(maximum, residual)
        require(residual <= 1e-12, "raw virtual-base pose does not match the pinned conversion")
        if index:
            require(row["frame"] == states[index - 1]["frame"] + 1 and
                abs(stamp(row) - stamp(states[index - 1]) - 50_000_000) <= 500, "native frame/time gap")
    require(states, "no native states")
    return maximum


def interpolate(states, stamps, timestamp):
    # HH_260906 - Same operation order as the pinned adapter; actor XY is a separate diagnostic, not a replacement target.
    upper = bisect_right(stamps, timestamp)
    require(upper, "future precedes native timeline")
    if stamps[upper - 1] == timestamp:
        row = states[upper - 1]
        result = {key: float(row[key]) for key in ("x", "y", "z", "yaw", "vx", "vy")}
        result["actor_xy"] = actor_xy(row)
        return result, {"mode": "exact_native_state", "frames": [row["frame"]], "timestamps_ns": [timestamp]}
    require(upper < len(stamps), "future exceeds native timeline")
    first, second = states[upper - 1], states[upper]
    require(second["frame"] == first["frame"] + 1 and
        abs(stamps[upper] - stamps[upper - 1] - 50_000_000) <= 500, "interpolation cannot bridge a native gap")
    ratio = (timestamp - stamps[upper - 1]) / (stamps[upper] - stamps[upper - 1])
    result = {key: float(first[key]) + ratio * (float(second[key]) - float(first[key]))
        for key in ("x", "y", "z", "vx", "vy")}
    result["yaw"] = wrap(float(first["yaw"]) + ratio * wrap(float(second["yaw"]) - float(first["yaw"])))
    result["actor_xy"] = [a + ratio * (b - a) for a, b in zip(actor_xy(first), actor_xy(second))]
    return result, {"mode": "linear_native_bracket", "frames": [first["frame"], second["frame"]],
        "timestamps_ns": [stamps[upper - 1], stamps[upper]], "ratio": ratio}


def relative(point, origin, yaw):
    dx, dy = point[0] - origin[0], point[1] - origin[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * dx + s * dy, -s * dx + c * dy]


def geometry(xy, previous_xy, previous_heading, heading_index, point_index):
    displacement = [xy[0] - previous_xy[0], xy[1] - previous_xy[1]]
    distance = math.hypot(*displacement)
    reference = ("anchor_body_heading_first_point" if point_index == 0 else
        "anchor_body_heading_after_unassessed_steps" if heading_index is None else "last_assessed_segment_heading")
    result = {"displacement_in_anchor_axes_m": displacement, "distance_m": distance,
        "heading_reference": reference, "reference_point_index": heading_index,
        "heading_change_rad": None, "curvature_rad_per_m": None}
    if distance > MINIMUM_STEP_M:
        heading = math.atan2(displacement[1], displacement[0])
        delta = wrap(heading - previous_heading)
        result.update(heading_change_rad=delta, curvature_rad_per_m=abs(delta) / distance)
        return result, heading, point_index
    return result, previous_heading, heading_index


def float32_resolution(value):
    # HH_260906 - A representational spacing proxy, NOT a calibrated sensor error bound or a reason to erase a failure.
    value = number(value)
    try:
        packed = struct.pack("!f", abs(value))
    except OverflowError as error:
        raise EvidenceError("coordinate outside float32 representable range") from error
    rounded = struct.unpack("!f", packed)[0]
    require(math.isfinite(rounded), "coordinate outside float32 finite range")
    bits = struct.unpack("!I", packed)[0]
    adjacent = struct.unpack("!f", struct.pack("!I", bits + 1))[0]
    require(math.isfinite(adjacent), "float32 spacing has no finite upper neighbor")
    return {"absolute_float32_spacing_m": adjacent - rounded, "exactly_float32_representable": rounded == abs(value)}


def native_segment(first, second, earlier=None):
    yaw = first["yaw"]
    result = {"start_frame": first["frame"], "end_frame": second["frame"],
        "start_timestamp_ns": stamp(first), "end_timestamp_ns": stamp(second),
        "duration_s": (stamp(second) - stamp(first)) / 1e9,
        "start_planar_speed_mps": math.hypot(first["vx"], first["vy"]),
        "end_planar_speed_mps": math.hypot(second["vx"], second["vy"]),
        "body_yaw_change_rad": wrap(second["yaw"] - yaw),
        "actor_pitch_change_deg": second["actor_snapshot_transform_carla"]["pitch"] - first["actor_snapshot_transform_carla"]["pitch"],
        "start_current_control": first["current_control"], "start_next_control": first["next_control"],
        "end_current_control": second["current_control"],
        "start_actor_snapshot_transform_carla": first["actor_snapshot_transform_carla"],
        "end_actor_snapshot_transform_carla": second["actor_snapshot_transform_carla"],
        "start_body_velocity_mps": [first["vx"], first["vy"]],
        "end_body_velocity_mps": [second["vx"], second["vy"]],
        "start_yaw_rate_radps": first["yaw_rate"], "end_yaw_rate_radps": second["yaw_rate"]}
    for name, get in (("virtual_base", lambda r: [r["x"], r["y"]]), ("actor_reference", actor_xy)):
        a, b = get(first), get(second)
        delta = [b[0] - a[0], b[1] - a[1]]
        distance, body = math.hypot(*delta), relative(b, a, yaw)
        heading = math.atan2(delta[1], delta[0])
        prior = None if earlier is None else [a[0] - get(earlier)[0], a[1] - get(earlier)[1]]
        assessed = distance > MINIMUM_STEP_M
        turn = None if not assessed or prior is None or math.hypot(*prior) <= MINIMUM_STEP_M else wrap(heading - math.atan2(prior[1], prior[0]))
        result[name] = {"world_displacement_ros_xy_m": delta, "body_displacement_forward_left_m": body,
            "distance_m": distance, "heading_change_from_body_rad": wrap(heading - yaw) if assessed else None,
            "body_heading_curvature_rad_per_m": abs(wrap(heading - yaw)) / distance if assessed else None,
            "turn_from_immediately_previous_native_segment_rad": turn,
            "adjacent_segment_curvature_rad_per_m": abs(turn) / distance if turn is not None else None}
    actor = result["actor_reference"]
    resolution = [float32_resolution(value) for value in actor_xy(second)]
    actor["endpoint_float32_resolution"] = resolution
    actor["coordinate_step_in_endpoint_float32_spacings"] = [d / r["absolute_float32_spacing_m"]
        for d, r in zip(actor["world_displacement_ros_xy_m"], resolution)]
    offset = [b - a for b, a in zip(result["virtual_base"]["world_displacement_ros_xy_m"], actor["world_displacement_ros_xy_m"])]
    result["virtual_base_minus_actor_displacement_ros_xy_m"] = offset
    return result


def analyze(states, cameras, futures, published, wheelbase):
    pose_residual = validate_states(states, wheelbase)
    stamps = [stamp(row) for row in states]
    by_frame = {row["frame"]: index for index, row in enumerate(states)}
    require(len(cameras) == len(futures) == len(states[::2]), "camera/future/native cadence count mismatch")
    witnesses, anchors = defaultdict(list), set()
    full, assessed, total_steps, first_points = 0, 0, 0, 0
    dispositions, heading_references, other_counts = Counter(), Counter(), Counter()
    for anchor_index, (camera, record) in enumerate(zip(cameras, futures)):
        native_index = anchor_index * 2
        anchor = states[native_index]
        timestamp = stamps[native_index]
        require(camera["frame"] == record["frame"] == anchor["frame"] and
            stamp(camera) == record["anchor_timestamp_ns"] == timestamp and
            camera["capture_phase"] == record["capture_phase"] == anchor["capture_phase"], "camera/native/future frame-time-phase mismatch")
        dispositions[record["disposition"]] += 1
        if anchor["capture_phase"] == "stationary_tail":
            require(record["disposition"] == "tail_label_context_only" and record["valid_points"] == 0 and
                "diagnostic" not in record, "tail context was changed into targets")
            continue
        require(record["disposition"] == "full_64_point_anchor" and record["valid_points"] == 64 and
            record["valid_mask"] == [True] * 64 and all(type(v) is bool for v in record["valid_mask"]) and
            record["invalid_reasons"] == [None] * 64 and record["unavailable_reason"] is None, "only sealed complete non-tail windows are supported; no failed prefix is dropped")
        diagnostic = record["diagnostic"]
        require(diagnostic["sample_id"] == f"raw-frame:{anchor['frame']}" and diagnostic["valid_points"] == 64 and
            diagnostic["invalid_points"] == 0 and len(diagnostic["steps"]) == 64, "diagnostic anchor identity/count mismatch")
        full += 1
        base_previous = actor_previous = [0., 0.]
        base_heading = actor_heading = 0.
        base_heading_index = actor_heading_index = None
        previous_future = {**anchor, "actor_xy": actor_xy(anchor)}
        previous_binding = {"mode": "exact_native_state", "frames": [anchor["frame"]], "timestamps_ns": [timestamp]}
        for index, step in enumerate(diagnostic["steps"]):
            target = timestamp + (index + 1) * DT_NS
            tick = round((target - stamps[0]) / DT_NS)
            require(step["index"] == index and step["target_timestamp_ns"] == target and
                step["episode_relative_100ms_tick"] == tick and step["dt_s"] == .1, "future index/timestamp/grid mismatch")
            future, binding = interpolate(states, stamps, target)
            base_xy = relative([future["x"], future["y"]], [anchor["x"], anchor["y"]], anchor["yaw"])
            actor_point = relative(future["actor_xy"], actor_xy(anchor), anchor["yaw"])
            base, base_heading, base_heading_index = geometry(base_xy, base_previous, base_heading, base_heading_index, index)
            actor, actor_heading, actor_heading_index = geometry(actor_point, actor_previous, actor_heading, actor_heading_index, index)
            metric = step["metrics"]["xy_curvature"]
            require(isinstance(metric, list) and len(metric) == 2 and type(metric[1]) is bool, "invalid saved curvature metric")
            value = base["curvature_rad_per_m"]
            failed = value is not None and value > MAXIMUM_CURVATURE + ROUNDING_TOLERANCE
            # HH_260906 - Exact scalar/flag equality; no tolerance is added to the unchanged .2 + 1e-6 decision.
            require(metric[0] == value and metric[1] is failed, "saved curvature differs from independent raw replay")
            require(step["metrics"]["speed_limit"][0] == math.hypot(future["vx"], future["vy"]), "saved endpoint speed differs from raw replay")
            assessed += value is not None
            total_steps += 1
            for name, entry in step["metrics"].items():
                require(isinstance(entry, list) and len(entry) == 2 and type(entry[1]) is bool, "invalid stored metric flag")
                if entry[0] is not None:
                    number(entry[0])
                other_counts[name] += entry[1]
            if failed:
                anchors.add(anchor["frame"])
                first_points += index == 0
                heading_references[base["heading_reference"]] += 1
                nearest = min(range(max(0, bisect_right(stamps, target) - 1), min(len(stamps), bisect_right(stamps, target) + 1)), key=lambda i: abs(stamps[i] - target))
                require(abs(stamps[nearest] - target) <= 500 and nearest == tick * 2, "rounded failure tick is not bound to its nearest native endpoint")
                witnesses[tick].append({"anchor_frame": anchor["frame"], "sample_id": diagnostic["sample_id"],
                    "point_index": index, "target_timestamp_ns": target, "native_endpoint_frame": states[nearest]["frame"],
                    "target_minus_native_timestamp_ns": target - stamps[nearest], "original_failure_retained": True,
                    "original_curvature_rad_per_m": metric[0], "virtual_base": base, "actor_reference": actor,
                    "start_binding": previous_binding, "end_binding": binding,
                    "endpoint_planar_speed_mps": math.hypot(future["vx"], future["vy"]),
                    "body_yaw_change_rad": wrap(future["yaw"] - previous_future["yaw"]),
                    "original_xy_lateral_acceleration_mps2": step["metrics"]["xy_lateral_acceleration"][0]})
            base_previous, actor_previous = base_xy, actor_point
            previous_future, previous_binding = future, binding
    expected = published["measured_future"]["full_64_point_anchors"]
    counts = {"native_state_count": len(states), "camera_anchor_count": len(cameras), "full_64_point_anchor_count": full,
        "original_curvature_assessed_step_count": assessed, "original_violating_anchor_count": len(anchors),
        "original_violating_window_step_count": sum(map(len, witnesses.values())),
        "unique_episode_relative_100ms_violation_tick_count": len(witnesses),
        "first_point_failure_occurrence_count": first_points,
        "interior_point_failure_occurrence_count": sum(map(len, witnesses.values())) - first_points,
        "unique_ticks_failing_only_at_first_point": sum(all(row["point_index"] == 0 for row in rows) for rows in witnesses.values())}
    require(full == expected["sample_count"] == expected["horizon_eligible_anchor_count"] and
        total_steps == expected["valid_step_count"] and expected["invalid_step_count"] == 0, "sealed horizon denominator mismatch")
    metric = expected["metrics"]["xy_curvature"]
    require((assessed, counts["original_violating_window_step_count"], len(anchors), len(witnesses)) ==
        (metric["assessed_step_count"], metric["violating_step_count"], metric["violating_anchor_count"],
         metric["unique_episode_relative_100ms_violation_tick_count"]), "sealed curvature counts mismatch")
    require(set(other_counts) == set(expected["metrics"]) and all(count == expected["metrics"][name]["violating_step_count"]
        for name, count in other_counts.items()), "sealed other-metric failure counts mismatch")
    require(dict(dispositions) == published["measured_future"]["disposition_counts"] and
        len(cameras) == published["measured_future"]["camera_anchor_count"], "sealed anchor dispositions mismatch")
    records, phase_counts, pilot_counts = [], Counter(), Counter()
    for tick, occurrences in sorted(witnesses.items()):
        index = by_frame[occurrences[0]["native_endpoint_frame"]]
        require(index >= 2 and all(r["native_endpoint_frame"] == states[index]["frame"] for r in occurrences), "ambiguous native endpoint group")
        row = states[index]
        phase_counts[row["capture_phase"]] += 1
        pilot_counts[row["goal_stop"]["pilot_state"]] += 1
        records.append({"episode_relative_100ms_tick": tick, "native_endpoint_frame": row["frame"],
            "native_endpoint_timestamp_ns": stamps[index], "capture_phase": row["capture_phase"],
            "pilot_state": row["goal_stop"]["pilot_state"], "original_failure_retained": True,
            "duplicate_window_failure_count": len(occurrences),
            "first_point_occurrence_count": sum(r["point_index"] == 0 for r in occurrences),
            "interior_point_occurrence_count": sum(r["point_index"] > 0 for r in occurrences),
            "only_first_point_failure": all(r["point_index"] == 0 for r in occurrences),
            "native_100ms_segment": native_segment(states[index - 2], row, states[index - 4] if index >= 4 else None),
            "native_50ms_subintervals": [native_segment(states[j - 1], states[j], states[j - 2] if j >= 2 else None)
                for j in (index - 1, index)], "window_occurrences": occurrences})
    return {"counts": counts, "original_metric_failure_counts": dict(other_counts),
        "unique_failure_ticks_by_capture_phase": dict(phase_counts), "unique_failure_ticks_by_pilot_state": dict(pilot_counts),
        "window_failure_heading_reference_counts": dict(heading_references),
        "raw_virtual_base_pose_maximum_replay_residual": pose_residual,
        "curvature_replay_exact_value_and_flag_match": True}, records


def run(trial_root, evidence_dir, seal_sha256, script_sha256, *, output_dir=None):
    ledger = {}
    read(Path(__file__), script_sha256, ledger)
    trial, evidence = safe_path(trial_root), safe_path(evidence_dir)
    require(trial.is_dir() and evidence.is_dir(), "existing raw and sealed evidence directories required")
    output = None if output_dir is None else safe_path(output_dir)
    if output is not None:
        require(not output.exists() and output.parent.is_dir(), "fresh output with existing parent required")
        require(all(output != root and root not in output.parents and output not in root.parents for root in (trial, evidence)),
            "output must be disjoint from raw and sealed evidence")
    seal = read(evidence / "SHA256SUMS", seal_sha256, ledger)
    pins = {}
    for line in seal.decode().splitlines():
        require(re.fullmatch(r"[0-9a-f]{64}  [^\r\n]+", line), "invalid evidence seal line")
        digest, name = line.split("  ", 1)
        require(name not in pins and not Path(name).is_absolute() and ".." not in Path(name).parts, "unsafe or duplicate evidence seal path")
        pins[name] = digest
    require({"summary.json", "future_windows.jsonl"} <= pins.keys(), "seal lacks summary/future windows")
    published = loads(read(evidence / "summary.json", pins["summary.json"], ledger))
    require(published["schema"] == "carla.expert_brake_hold_pilot_summary.v1" and
        published["status"] == "DIAGNOSED_NOT_ADMITTED" and published["training_data_approved"] is False and
        published["dataset_export"] is False and published["original_input_bytes_unchanged"] is True,
        "sealed v5 diagnostic scope/denial changed")
    require(set(published["input_metadata_sha256"]) == RAW_PATHS, "unexpected raw input closure")
    raw = {name: read(trial / name, pin, ledger) for name, pin in published["input_metadata_sha256"].items()}
    source_pins = published["source_sha256"]["files"]
    require(set(source_pins) == SOURCE_PATHS and all(source_pins[name] == pin for name, pin in MATH_SOURCE_PINS.items()),
        "unsupported source math/closure")
    for name, pin in source_pins.items():
        read(ROOT / name, pin, ledger)
    manifest = loads(raw["episode/manifest.json"])
    require(manifest["capture_contract"]["goal_stop_profile"]["profile_id"] == "comfortable_v5_brake_hold" and
        manifest["result"]["training_data_approved"] is False, "raw v5 denial/profile changed")
    wheelbase = number(manifest["coordinate_contract"]["wheelbase_m"])
    require(wheelbase == 2.85, "unsupported virtual-base reference offset")
    states, cameras = json_lines(raw["episode/states.jsonl"]), json_lines(raw["episode/camera_frames.jsonl"])
    futures = json_lines(read(evidence / "future_windows.jsonl", pins["future_windows.jsonl"], ledger))
    summary, witnesses = analyze(states, cameras, futures, published, wheelbase)
    summary.update(schema=SCHEMA, status="DIAGNOSTIC_FAILURES_RETAINED_NOT_ADMITTED",
        generated_at_utc=datetime.now(timezone.utc).isoformat(),
        scope={"training_data_approved": False, "new_data_admission": False, "targets_changed": False,
            "thresholds_changed": False, "failures_removed": False, "images_opened": False,
            "model_executed": False, "simulator_contacted": False,
            "other_metrics_recomputed": False, "prior_capture_or_pixel_QA_rerun": False},
        unchanged_curvature_contract={"time_step_ns": DT_NS, "minimum_heading_step_m": MINIMUM_STEP_M,
            "maximum_curvature_rad_per_m": MAXIMUM_CURVATURE, "rounding_tolerance": ROUNDING_TOLERANCE,
            "initial_heading": "anchor body-forward; held until a displacement exceeds the minimum step",
            "curvature": "abs(wrapped displacement-heading change) / current displacement length"},
        sources={"script_sha256": script_sha256, "original_auditor_source_sha256": source_pins},
        input_sha256={"sealed06_sha256sums": seal_sha256, "sealed06_summary": pins["summary.json"],
            "sealed06_future_windows": pins["future_windows.jsonl"], "raw_metadata": published["input_metadata_sha256"]},
        limitations=["All original curvature failures remain failures; overlapping windows are not independent physical events.",
            "Actor-reference curvature is a coordinate diagnostic, not replacement training labels or physical wheel-geometry approval.",
            "Float32 spacing is representational sensitivity, not calibrated uncertainty or proof that a failure is an artifact.",
            "Native 100 ms witnesses use exact recorded endpoints; individual window targets retain their original nanosecond interpolation brackets.",
            "Small displacement makes direction-based curvature sensitive; body yaw, path heading and lateral acceleration are distinct observables.",
            "Control acknowledgements do not prove substep torque timing; simultaneous control changes and physics cause are not isolated.",
            "Other metric counts are preserved from the sealed scalar report, not independently recomputed here."])
    # HH_260906 - Recheck every opened byte before creating any report; no original source/evidence is modified.
    for name, pin in tuple(ledger.items()):
        read(name, pin, {})
    if output is not None:
        output.mkdir()
        payloads = {"summary.json": (json.dumps(summary, indent=2, allow_nan=False) + "\n").encode(),
            "witnesses.jsonl": b"".join((json.dumps(row, sort_keys=True, allow_nan=False) + "\n").encode() for row in witnesses)}
        for name, payload in payloads.items():
            with (output / name).open("xb") as stream:
                stream.write(payload)
        with (output / "SHA256SUMS").open("x") as stream:
            stream.write("".join(f"{sha(payload)}  {name}\n" for name, payload in sorted(payloads.items())))
    return summary, witnesses


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--expected-evidence-seal-sha256", required=True)
    parser.add_argument("--expected-script-sha256", required=True)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--output-dir", type=Path)
    choice.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary, _ = run(args.trial_root, args.evidence_dir, args.expected_evidence_seal_sha256,
            args.expected_script_sha256, output_dir=args.output_dir)
    except (EvidenceError, OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        parser.exit(1, f"curvature conditioning diagnostic failed: {error}\n")
    print(json.dumps({"status": summary["status"], "counts": summary["counts"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
