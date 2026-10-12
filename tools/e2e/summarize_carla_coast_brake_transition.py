#!/usr/bin/env python3
"""HH_260906 - Offline coast-to-brake ABBA observations; no model, gate evaluation, or data admission."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

if __package__:
    from . import summarize_carla_low_speed_coast as coast
else:
    import summarize_carla_low_speed_coast as coast

stationary = coast.stationary
require, number, integer = stationary.require, stationary.number, stationary.integer
read_bytes, read_rows, strict_json = stationary.read_bytes, stationary.read_rows, stationary.strict_json
CAMERAS = stationary.CAMERAS
PHASES = ("settle", "launch", "coast", "tail")
WORKER = "scripts/e2e/probe_carla_coast_brake_transition.py"
WORKER_SOURCES = coast.WORKER_SOURCES | {WORKER}
CASES = [{"id": key, "arm": arm} for key, arm in (("01_coast_A1", "coast"), ("02_service_B1", "service"),
         ("03_service_B2", "service"), ("04_coast_A2", "coast"))]
ARTIFACTS = {"native_states.jsonl", "native_observations.jsonl", "control_receipts.jsonl", "camera_frames.jsonl",
             "route.json", "mapping.yaml", "calibration.yaml"}
DENIALS = {**stationary.DENIALS, "vehicle_control_approved": False, "physical_actuation_proven": False,
           "path_heading_assessed": False, "curvature_assessed": False}
FIXED = {"settle_ticks": 70, "max_launch_ticks": 160, "coast_ticks": 600, "tail_ticks": 300, "tail_entry_max_speed_mps": .1, "launch_throttle": .13,
         "target_speed_mps": .5, "max_speed_mps": .75, "max_net_motion_m": 10., "max_route_cte_m": 1.,
         "minimum_body_vx_mps": -.1, "maximum_native_ticks": 1130, "physics_hz": 20, "capture_hz": 10,
         "wall_timeout_sec": 300}


def sample(row):
    require(row.get("phase") in PHASES, "unknown coast phase")
    # HH_260906 - Reuse only the frozen raw/reference transform; preserve the original coast phase explicitly.
    result = stationary.motion_sample(dict(row, phase="settle"))
    result.update(phase=row["phase"], tick=integer(row["tick"]), phase_tick=integer(row["phase_tick"]))
    result.update(body_vx=number(row["vx"]), reported_gear=row.get("reported_control", {}).get("gear"),
                  comparison_fields={key: row.get(key) for key in ("actor_snapshot_transform_carla", "world_velocity_carla",
                      "world_acceleration_carla", "world_angular_velocity_carla_deg_s", "x", "y", "z", "yaw", "vx", "vy",
                      "ax", "ay", "yaw_rate", "steering_tire_angle_rad", "requested_control", "reported_control")})
    return result


interval_stats = coast.interval_stats

def summarize_motion(samples):
    output, series_rows = {}, {}
    for name, offset, stride in (("native_50ms", 0, 1), ("derived_100ms_offset0", 0, 2), ("derived_100ms_offset1", 1, 2)):
        records = []
        for start in range(offset, len(samples) - stride, stride):
            points = samples[start:start + stride + 1]
            record = stationary.interval_record(points)
            record.update(first_tick=points[0]["tick"], last_tick=points[-1]["tick"],
                          maximum_native_actor_speed_mps=max(math.hypot(*point["actor_velocity"]) for point in points))
            records.append(record)
        phases = {}
        for phase in ("all", *PHASES):
            chosen = records if phase == "all" else [row for row in records if row["to_phase"] == phase]
            points = samples if phase == "all" else [row for row in samples if row["phase"] == phase]
            stats = interval_stats(chosen)
            stats["native_sample_count"] = len(points)
            stats["native_sample_speed_mps"] = {ref: stationary.describe([math.hypot(*row[ref + "_velocity"]) for row in points]) for ref in ("actor", "base")}
            stats["sample_net_drift_m"] = {ref: math.dist(points[0][ref], points[-1][ref]) if points else None for ref in ("actor", "base")}
            phases[phase] = stats
        output[name] = {"phases": phases}
        series_rows[name] = records
    return output, series_rows


def tail_statistics(samples):
    """HH_260906 - Signed velocity integration and scalar speed integration are deliberately separate observations."""
    tail = [point for point in samples if point["phase"] == "tail"]
    if not tail:
        return {"status": "TAIL_NOT_OBSERVED", "native_sample_count": 0}
    points = [point for point in tail if point["timestamp_ns"] >= tail[-1]["timestamp_ns"] - 10_000_000_000]
    records = [stationary.interval_record([a, b]) for a, b in zip(points, points[1:])]
    span = (points[-1]["timestamp_ns"] - points[0]["timestamp_ns"]) / 1e9
    result = {"status": "MEASURED", "native_sample_count": len(points), "interval_count": len(records),
              "first_tick": points[0]["tick"], "last_tick": points[-1]["tick"], "sample_span_s": span,
              "full_10s_window_available": tail[-1]["timestamp_ns"] - tail[0]["timestamp_ns"] >= 10_000_000_000,
              "references": {}}
    for ref in ("actor", "base"):
        values = [record["references"][ref] for record in records]
        delta = [points[-1][ref][j] - points[0][ref][j] for j in range(2)]
        integral = [sum(value["trapezoidal_velocity_integral_xy_m"][j] for value in values) for j in range(2)]
        residual = [delta[j] - integral[j] for j in range(2)]
        speeds = [math.hypot(*point[ref + "_velocity"]) for point in points]
        scalar = sum((a + b) * .5 * (q["timestamp_ns"] - p["timestamp_ns"]) / 1e9
                     for a, b, p, q in zip(speeds, speeds[1:], points, points[1:]))
        result["references"][ref] = {"position_displacement_xy_ros_m": delta, "position_net_displacement_norm_m": math.hypot(*delta),
            "position_travel_m": sum(value["distance_m"] for value in values), "vector_velocity_integral_xy_ros_m": integral,
            "vector_velocity_integral_norm_m": math.hypot(*integral), "scalar_speed_integral_m": scalar,
            "aggregate_residual_xy_m": residual, "aggregate_residual_norm_m": math.hypot(*residual),
            "native_residual_norm_m": stationary.describe([value["residual_norm_m"] for value in values]),
            "reported_speed_mps": {**stationary.describe(speeds), "mean": sum(speeds) / len(speeds)},
            "final_reported_speed_mps": speeds[-1]}
    signs = [(point["body_vx"] > 0) - (point["body_vx"] < 0) for point in points]
    result["signed_body_vx"] = {"minimum_mps": min(point["body_vx"] for point in points),
        "maximum_mps": max(point["body_vx"] for point in points), "negative_count": signs.count(-1),
        "zero_count": signs.count(0), "positive_count": signs.count(1),
        "sign_change_count": sum(a != b for a, b in zip(signs, signs[1:])), "by_global_tick_parity": {}}
    for parity in (0, 1):
        values = [point["body_vx"] for point in points if point["tick"] % 2 == parity]
        result["signed_body_vx"]["by_global_tick_parity"][str(parity)] = {"count": len(values),
            "mean_mps": sum(values) / len(values) if values else None,
            "minimum_mps": min(values) if values else None, "maximum_mps": max(values) if values else None}
    forward = [math.cos(a["yaw"]) * (b["actor"][0] - a["actor"][0]) + math.sin(a["yaw"]) * (b["actor"][1] - a["actor"][1])
               for a, b in zip(points, points[1:])]
    result["actor_forward_position_increments"] = {"count": len(forward), "negative_count": sum(value < 0 for value in forward),
        "zero_count": sum(value == 0 for value in forward), "minimum_m": min(forward) if forward else None,
        "maximum_m": max(forward) if forward else None}
    result["reported_gear_counts"] = dict(Counter(str(point["reported_gear"]) for point in points))
    return result


def pre_tail_comparison(trials):
    """HH_260906 - Observed prefix agreement qualifies interpretation; it is not a new execution/admission gate."""
    if not all(trial[0]["status"] == "COMPLETE" for trial in trials):
        return {"status": "NOT_ASSESSED_INCOMPLETE_CASES", "is_execution_or_admission_gate": False}
    prefixes = [[point["comparison_fields"] for point in trial[1] if point["phase"] != "tail"] for trial in trials]
    counts = [len(prefix) for prefix in prefixes]
    differences = [sum(a != b for a, b in zip(prefixes[0], prefix)) + abs(len(prefix) - counts[0]) for prefix in prefixes]
    return {"status": "OBSERVED_MATCH" if not any(differences) else "OBSERVED_DIFFERENCE",
            "reference_case": trials[0][0]["case_id"], "is_execution_or_admission_gate": False,
            "native_prefix_counts": counts, "differing_tick_count_from_first_case": differences,
            "comparison": "Raw actor pose/vectors, virtual-base state, measured steering and requested/reported controls; absolute frames/timestamps excluded."}

def validate_plan(plan):
    require(plan.get("schema") == "carla.coast_brake_transition_campaign.v1" and plan.get("cases") == CASES,
            "fixed four-case coast/brake ABBA plan required")
    for key, value in FIXED.items():
        require(number(plan.get(key)) == value, "fixed campaign setting differs: " + key)
    require(plan.get("port") == 2100 and plan.get("quality") == "Epic" and plan.get("map") == "C_track_1_0_7",
            "fixed campaign server differs")
    require(plan.get("training_data_approved") is False and plan.get("vehicle_control_approved") is False
            and plan.get("no_automatic_retry") is True, "campaign approval/retry boundary differs")
    for name in ("source_sha256", "input_sha256"):
        require(isinstance(plan.get(name), dict) and all(type(value) is str and len(value) == 64
                and set(value) <= set("0123456789abcdef") for value in plan[name].values()), "campaign hash pins differ")
    require(len(plan["source_sha256"]) == 17 and WORKER_SOURCES <= set(plan["source_sha256"]), "full execution-source closure required")
    require(set(plan["input_sha256"]) == {"route_file", "mapping", "calibration"}
            and plan["route_sha256"] == plan["input_sha256"]["route_file"], "fixed input closure differs")


def check_control(value, phase, arm):
    require(isinstance(value, dict), "control object required")
    for field, expected in (("throttle", .13 if phase == "launch" else 0.), ("brake", 1. if phase == "settle" or phase == "tail" and arm == "service" else 0.), ("steer", 0.)):
        require(abs(number(value.get(field)) - expected) <= 1e-6, "phase control differs: " + field)
    require(all(value.get(name) is False for name in ("hand_brake", "reverse", "manual_gear_shift"))
            and type(value.get("gear")) is int, "control flag/automatic gear differs")


def verify_owner(root, case, plan, pins):
    prefix = case["id"] + "/"
    owner = strict_json(read_bytes(root, prefix + "owner_plan.json", pins))
    route = str((Path(__file__).resolve().parents[2] / plan["route_path"]).resolve())
    require(owner.get("schema") == "portable_e2e.owned_expert_trial.v1" and owner.get("capture_mode") == "coast-brake-transition"
            and owner.get("worker_path") == WORKER and owner.get("host") == "127.0.0.1", "owned coast worker identity differs")
    require(owner.get("source_sha256") == plan["source_sha256"] and owner.get("source_bytes_archived") is True,
            "owner execution sources differ")
    require(owner.get("route_path") == route and owner.get("route_sha256") == plan["route_sha256"]
            and owner.get("map") == plan["map"] and owner.get("quality") == plan["quality"]
            and owner.get("port") == plan["port"] and owner.get("collector_wall_timeout_sec") == plan["wall_timeout_sec"],
            "owner inputs/settings differ")
    require(owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False, "owner approval differs")
    declared = datetime.fromisoformat(plan["declared_at_utc"].replace("Z", "+00:00"))
    planned = datetime.fromisoformat(owner["planned_at_utc"].replace("Z", "+00:00"))
    require(declared.utcoffset() is not None and planned.utcoffset() is not None and planned >= declared,
            "owned trial predates campaign")
    require(owner.get("collector_argv") == [str((root / case["id"] / "brake_probe").resolve()), route,
            "--host", "127.0.0.1", "--port", str(plan["port"]), "--arm", case["arm"]], "owned worker arguments differ")
    for name, expected in plan["source_sha256"].items():
        relative = prefix + "provenance/" + name
        read_bytes(root, relative, pins)
        require(pins[relative]["sha256"] == expected, "archived source SHA differs: " + name)
    result = None
    if (root / prefix / "owner_result.json").exists():
        result = strict_json(read_bytes(root, prefix + "owner_result.json", pins))
        require(result.get("capture_mode") == "coast-brake-transition" and type(result.get("exit_code")) is int
                and result.get("learned_model_control") is False and result.get("vehicle_control_approved") is False,
                "owner result contract differs")
        require(result.get("source_bytes_unchanged_and_archived") is True
                and result.get("source_checks") == {key: True for key in plan["source_sha256"]}, "owned source preservation differs")
    return owner, result


def check_receipt(receipt, row, arm):
    require(all(type(receipt.get(key)) is int for key in ("sequence", "actor_id", "response_actor_id",
            "before_frame", "expected_before_frame", "after_ack_frame")), "integer ACK identities required")
    require(receipt.get("sequence") == row["tick"] and receipt.get("reason") == row["phase"]
            and receipt.get("mode") == "acknowledged_batch" and receipt.get("do_tick") is False
            and receipt.get("physical_actuation_proven") is False, "ACK journal identity differs")
    require(receipt.get("status") == "ACKNOWLEDGED" and receipt.get("server_accepted") is True
            and integer(receipt.get("actor_id")) > 0 and receipt.get("response_actor_id") == receipt["actor_id"], "ACK acceptance differs")
    require(receipt.get("before_frame") == receipt.get("expected_before_frame") == receipt.get("after_ack_frame")
            == row["frame"] - 1, "ACK frame bracket differs")
    check_control(receipt["requested_control"], row["phase"], arm)
    require(receipt["requested_control"] == row["requested_control"], "ACK/native request differs")


def check_native_protocol(row, receipt, arm):
    check_receipt(receipt, row, arm)
    check_control(row["reported_control"], row["phase"], arm)
    alignment = row["control_alignment"]
    require(alignment.get("status") == "PASS" and alignment.get("frame_binding_pass") is True
            and alignment.get("expected_receipt_sequence") == row["tick"]
            and alignment.get("expected_command_before_frame") == row["frame"] - 1
            and alignment.get("observed_frame") == alignment.get("control_read_frame_before")
            == alignment.get("control_read_frame_after") == row["observed_aux_frame_before"]
            == row["observed_aux_frame_after"] == row["frame"]
            and alignment.get("mismatched_fields") == [] and alignment.get("physical_actuation_proven") is False,
            "control readback alignment differs")
    require(row.get("collision") == [] and row.get("measurement_complete") is True, "collision or incomplete measurement")


def check_phase_sequence(native, complete):
    require([integer(row["tick"]) for row in native] == list(range(1, len(native) + 1)) and len(native) <= 1130,
            "native denominator/tick sequence differs")
    counts = Counter()
    previous = "settle"
    first_reached = None
    for row in native:
        phase = row["phase"]
        require(phase in PHASES and PHASES.index(phase) in (PHASES.index(previous), PHASES.index(previous) + 1),
                "noncontiguous phase sequence")
        counts[phase] += 1
        require(integer(row["phase_tick"]) == counts[phase], "phase-local tick differs")
        if phase == "settle": require(counts[phase] <= 70, "too many settle ticks")
        if phase == "launch":
            require(counts["settle"] == 70 and counts[phase] <= 160 and first_reached is None, "launch bounds or first-threshold handoff differs")
            if "world_velocity_carla" in row and math.hypot(*stationary.vector(row["world_velocity_carla"])[:2]) >= .5:
                first_reached = row["tick"]
        if phase == "coast":
            require(first_reached == 70 + counts["launch"] and counts[phase] <= 600, "coast lacks first observed launch threshold")
        if phase == "tail":
            require(counts["coast"] == 600 and counts[phase] <= 300, "tail lacks full600 coast or exceeds300 ticks")
        previous = phase
    if complete:
        require(counts["settle"] == 70 and 1 <= counts["launch"] <= 160 and counts["coast"] == 600 and counts["tail"] == 300
                and first_reached == 70 + counts["launch"], "complete trial lacks full600 coast and300 tail")
    return dict(counts), first_reached


validate_cameras = coast.validate_cameras
check_environment = coast.check_environment

def validate_trial(root, case, plan, pins):
    empty = {"case_id": case["id"], "arm": case["arm"], "native_state_count": 0, "camera_anchor_count": 0,
             "maximum_native_ticks": 1130, **DENIALS}
    if not (root / case["id"]).exists():
        return dict(empty, status="NOT_RUN"), [], [], None
    owner, owned_result = verify_owner(root, case, plan, pins)
    folders = [case["id"] + "/" + name for name in ("brake_probe", "brake_probe.partial") if (root / case["id"] / name).exists()]
    require(len(folders) <= 1, "ambiguous coast output")
    if not folders or not (root / folders[0] / "manifest.json").exists():
        return dict(empty, status="NO_FINAL_WORKER_MANIFEST", count_scope="No authenticated finalized native/camera rows."), [], [], None
    worker = folders[0] + "/"
    manifest = strict_json(read_bytes(root, worker + "manifest.json", pins))
    require(manifest.get("schema") == "carla.coast_brake_transition_probe.v1" and manifest.get("status") in ("COMPLETE", "FAIL"),
            "coast manifest identity/status differs")
    require(manifest.get("arm") == case["arm"], "worker arm differs")
    complete = manifest["status"] == "COMPLETE"
    require(all(manifest.get(key) is False for key in ("training_data_approved", "vehicle_approved", "vehicle_control_approved",
            "learned_model_control", "data_admission_performed")), "worker approval boundary differs")
    for key in ("settle_ticks", "max_launch_ticks", "coast_ticks", "tail_ticks", "tail_entry_max_speed_mps", "launch_throttle", "target_speed_mps"):
        require(number(manifest.get(key)) == FIXED[key], "worker schedule differs: " + key)
    require(manifest.get("maximum_native_ticks") == 1130 and number(manifest.get("physics_hz")) == 20.
            and number(manifest.get("camera_hz")) == 10. and manifest.get("camera_bundle_observed_every_native_tick") is True,
            "worker native/camera schedule differs")
    expected_limits = {"max_speed_mps": .75, "minimum_body_vx_mps": -.1, "max_net_motion_m": 10.,
                       "max_route_cte_m": 1., "settled_max_speed_mps": .1, "settled_max_net_motion_m": .5}
    require(manifest.get("operational_limits") == expected_limits, "worker operational limits differ")
    require(manifest.get("town") == plan["map"] and manifest.get("route_sha256") == plan["route_sha256"]
            and number(manifest.get("wheelbase_m")) == 2.85, "worker map/reference differs")
    for field, expected in (("source_sha256", {key: plan["source_sha256"][key] for key in WORKER_SOURCES}),
                            ("input_sha256", plan["input_sha256"])):
        require(manifest.get(field + "_before") == manifest.get(field + "_after") == expected, "worker before/after pins differ")
    require(set(manifest["artifact_sha256"]) == ARTIFACTS, "worker artifact closure differs")
    for name, expected in manifest["artifact_sha256"].items():
        read_bytes(root, worker + name, pins)
        require(pins[worker + name]["sha256"] == expected, "worker artifact SHA differs: " + name)
    for name, logical in (("route.json", "route_file"), ("mapping.yaml", "mapping"), ("calibration.yaml", "calibration")):
        require(manifest["artifact_sha256"][name] == plan["input_sha256"][logical], "copied fixed input differs")
    native, observations, receipts, cameras = [read_rows(root, worker + name, pins) for name in
        ("native_states.jsonl", "native_observations.jsonl", "control_receipts.jsonl", "camera_frames.jsonl")]
    require(integer(manifest["state_count"]) == len(native), "worker native denominator differs")
    phase_counts, threshold_tick = check_phase_sequence(native, complete)
    require(manifest["phase_counts"] == {phase: phase_counts.get(phase, 0) for phase in PHASES}, "worker phase denominator differs")
    require(len(native) <= len(receipts) <= min(1130, len(native) + 1)
            and [row["sequence"] for row in receipts] == list(range(1, len(receipts) + 1)), "ACK denominator differs")
    require(len({row["actor_id"] for row in receipts}) <= 1, "multiple ACK actors")
    require(len(observations) <= len(native) and [row["tick"] for row in observations] == list(range(1, len(observations) + 1)),
            "pre-camera observation denominator differs")
    for observation, row in zip(observations, native):
        require(observation.get("stage") == "OBSERVED_BEFORE_CAMERA" and observation.get("measurement_complete") is False
                and all(row.get(key) == value for key, value in observation.items()
                        if key not in ("stage", "measurement_complete", "collision")), "pre-camera/native evidence differs")
    issues, unavailable, samples = [], [], []
    try:
        stationary.check_settings(manifest.get("world_settings"))
        for phase in PHASES:
            check_control(manifest["controls_by_phase"][phase], phase, case["arm"])
    except (ValueError, KeyError, TypeError) as error:
        issues.append({"tick": None, "reason": str(error)})
    for row in native:
        try:
            point = sample(row)
            require(not unavailable, "motion unavailable after interrupted prefix; no gap bridging")
            if samples: stationary.interval_record([samples[-1], point])
            samples.append(point)
        except (ValueError, KeyError, TypeError) as error:
            unavailable.append({"tick": row["tick"], "reason": str(error)})
        try:
            check_native_protocol(row, receipts[row["tick"] - 1], case["arm"])
            speed = math.hypot(*stationary.vector(row["world_velocity_carla"])[:2])
            require(math.isclose(number(row["planar_speed_mps"]), speed, rel_tol=0, abs_tol=1e-12), "recorded actor speed differs")
            origin, pose = manifest["requested_spawn_transform_carla"], row["actor_snapshot_transform_carla"]
            motion = math.hypot(number(pose["x"]) - number(origin["x"]), number(pose["y"]) - number(origin["y"]))
            require(manifest.get("motion_origin_source") == "REQUESTED_SPAWN_TRANSFORM_NOT_PRETICK_ACTOR_API"
                    and manifest["motion_origin_transform_carla"] == origin
                    and math.isclose(number(row["net_planar_motion_m"]), motion, rel_tol=0, abs_tol=1e-12), "requested-spawn motion baseline differs")
            require(speed <= .75 and number(row["vx"]) >= -.1 and number(row["net_planar_motion_m"]) <= 10.
                    and 0 <= number(row["route_cte_m"]) <= 1., "operational motion abort observed")
            # HH_260906 - Settling has stricter declared bounds than the later moving phases.
            if row["phase"] == "settle" and row["phase_tick"] == 70:
                require(speed <= .1 and number(row["net_planar_motion_m"]) <= .5, "settle boundary abort observed")
            if row["phase"] == "coast" and row["phase_tick"] == 600:
                require(speed <= .1, "tail entry above0.1m/s for this arm")
        except (ValueError, KeyError, TypeError) as error:
            issues.append({"tick": row["tick"], "reason": str(error)})
    if len(receipts) > len(native):
        issues.append({"tick": receipts[-1]["sequence"], "reason": "ACK journal contains a final command without an observed native state"})
    saved, partial, decoded = validate_cameras(root, worker, cameras, native, manifest, pins, complete)
    transitions = manifest.get("phase_transitions", [])
    if complete:
        # HH_260906 - A success marker cannot override a retained worker exception or safety failure.
        require(manifest.get("error") is None and manifest.get("safety_failure") is None,
                "complete worker retains an error or safety failure")
        require(not issues and not unavailable and len(observations) == len(native), "complete coast contains failed/incomplete evidence")
        require(len(transitions) == 3 and manifest.get("launch_handoff") == transitions[1]
                and manifest.get("tail_handoff") == transitions[2], "phase transition/handoff ledger differs")
        entry, source = manifest.get("tail_entry_check", {}), native[threshold_tick + 600 - 1]
        require(entry.get("status") == "PASS" and entry.get("applies_to_both_arms") is True
                and number(entry.get("maximum_speed_mps")) == .1 and entry.get("source_tick") == source["tick"]
                and entry.get("source_frame") == source["frame"] and entry.get("source_timestamp") == source["timestamp"]
                and entry.get("source_planar_speed_mps") == source["planar_speed_mps"], "tail entry receipt differs")
        for transition, before, after, source_tick in zip(transitions, PHASES[:3], PHASES[1:], (70, threshold_tick, threshold_tick + 600)):
            source = native[source_tick - 1]
            require(transition.get("from_phase") == before and transition.get("to_phase") == after
                    and transition.get("source_tick") == source_tick and transition.get("source_phase_tick") == source["phase_tick"]
                    and transition.get("source_frame") == source["frame"] and transition.get("source_timestamp") == source["timestamp"]
                    and transition.get("source_planar_speed_mps") == source["planar_speed_mps"]
                    and transition.get("effective_tick") == source_tick + 1 and transition.get("physical_transition_time_proven") is False,
                    "phase handoff is not bound to its actual source observation")
            require(transition.get("next_requested_control") == native[source_tick]["requested_control"], "handoff/next ACK control differs")
        require(owned_result is not None and owned_result["exit_code"] == 0 and manifest.get("cleanup_errors") == []
                and manifest.get("collision_events") == [] and manifest.get("completion_reason") == "full_600_tick_coast_and_300_tick_tail_observed",
                "complete coast exit/cleanup differs")
        owned = manifest.get("owned_actor_ids")
        require(isinstance(owned, list) and len(owned) == len(set(owned)) == 8
                and all(integer(value) > 0 for value in owned) and set(manifest.get("destroy_rpc_confirmed_actor_ids", [])) == set(owned),
                "owned actor cleanup differs")
        require(all(receipt["actor_id"] == owned[0] for receipt in receipts), "ACK actor is not owned ego")
        stationary.check_lifecycle(root, case, plan, pins, native)
        check_environment(manifest, native[0]["frame"])
    motion, _ = summarize_motion(samples)
    expected_final = None if threshold_tick is None else threshold_tick + 900
    anchors = {"tail_entry": None if threshold_tick is None else threshold_tick + 601 + ((threshold_tick + 601) % 2),
               "scheduled_final": None if expected_final is None else expected_final - expected_final % 2}
    branch = next((point for point in samples if point["phase"] == "coast" and point["phase_tick"] == 600), None)
    tail_end = next((point for point in reversed(samples) if point["phase"] == "tail"), None)
    branch_motion = None if branch is None or tail_end is None else {
        "source_tick": branch["tick"], "last_observed_tail_tick": tail_end["tick"],
        "observed_elapsed_s": (tail_end["timestamp_ns"] - branch["timestamp_ns"]) / 1e9,
        "observed_interval_count": motion["native_50ms"]["phases"]["tail"]["interval_count"],
        "references": motion["native_50ms"]["phases"]["tail"]["references"]}
    return {**empty, "status": manifest["status"], "worker_folder": worker.rstrip("/"),
            "owner_exit_code": None if owned_result is None else owned_result["exit_code"],
            "worker_error": manifest.get("error"), "worker_safety_failure": manifest.get("safety_failure"),
            "worker_cleanup_errors": manifest.get("cleanup_errors"), "native_state_count": len(native),
            "preflight_status": manifest.get("preflight", {}).get("status", "NOT_RECORDED"),
            "measurement_complete_count": sum(row.get("measurement_complete") is True for row in native),
            "motion_sample_count": len(samples), "motion_unavailable": unavailable, "protocol_issues": issues,
            "phase_counts": {phase: phase_counts.get(phase, 0) for phase in PHASES},
            "control_receipt_count": len(receipts), "pre_camera_observation_count": len(observations),
            "camera_anchor_count": saved, "partial_camera_anchor_count": partial, "decoded_image_count": decoded,
            "expected_final_native_tick": expected_final, "camera_anchor_ticks": anchors,
            "final_camera_native_offset_ticks": None if expected_final is None else expected_final % 2,
            "tail_entry_check": manifest.get("tail_entry_check"), "tail_from_command_boundary": branch_motion,
            "motion": motion, "tail_last10s": tail_statistics(samples)}, samples, cameras, worker


def render_trial(root, output, result, samples, cameras, worker, pins):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    key = result["case_id"]
    times = [(row["timestamp_ns"] - samples[0]["timestamp_ns"]) / 1e9 for row in samples] if samples else []
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for ref, label in (("actor", "Actor API"), ("base", "Virtual base")):
        if samples:
            axes[0, 0].plot(times, [math.hypot(*row[ref + "_velocity"]) for row in samples], label=label)
            tail = [row for row in samples if row["phase"] == "tail"
                    and row["timestamp_ns"] >= samples[-1]["timestamp_ns"] - 10_000_000_000]
            if tail:
                tail_times = [(row["timestamp_ns"] - samples[0]["timestamp_ns"]) / 1e9 for row in tail]
                axes[1, 0].plot(tail_times, [1000 * math.dist(row[ref], tail[0][ref]) for row in tail], label=label)
                residuals = [stationary.interval_record([a, b])["references"][ref]["residual_norm_m"] * 1000
                             for a, b in zip(tail, tail[1:])]
                axes[1, 1].plot(tail_times[1:], residuals, label=label)
    if samples:
        axes[0, 1].plot(times, [row["body_vx"] for row in samples], label="Signed measured body vx")
        axes[0, 1].axhline(0, color="black", linewidth=.7)
        for i, point in enumerate(samples[1:], 1):
            if point["phase"] != samples[i - 1]["phase"]:
                for axis in axes[0]:
                    axis.axvline(times[i - 1], color="gray", linestyle="--", label=point["phase"] + " command boundary")
    for axis, title, unit in zip(axes.flat, ("Full-history reported planar speed", "Full-history signed body velocity",
            "Last10s tail: displacement from window's first sample", "Last10s tail: 50ms displacement minus velocity integral"), ("m/s", "m/s", "mm", "mm")):
        axis.set(title=title, xlabel="Seconds from first native observation", ylabel=unit)
        axis.grid(alpha=.25)
        if axis.lines: axis.legend(fontsize=7)
    figure.suptitle(f"{key}: {result['status']} | measured controls and motion; no model or admission")
    figure.savefig(output / f"{key}_measured_traces.png", dpi=135)
    plt.close(figure)
    for anchor_name in ("tail_entry", "scheduled_final"):
        tick = result.get("camera_anchor_ticks", {}).get(anchor_name)
        bundle = next((row for row in cameras if row["tick"] == tick), None)
        points = [row for row in samples if tick is not None and row["tick"] <= tick]
        anchor = next((row for row in points if row["tick"] == tick), None)
        figure = plt.figure(figsize=(15, 10), constrained_layout=True)
        grid = figure.add_gridspec(3, 3, height_ratios=(1, 1, 1.15))
        for index, camera in enumerate(CAMERAS):
            axis = figure.add_subplot(grid[index // 3, index % 3])
            metadata = None if bundle is None else bundle["images"].get(camera)
            if metadata is not None:
                axis.imshow(stationary.decode_image(read_bytes(root, worker + metadata["path"], pins)))
            else:
                axis.text(.5, .5, "DECLARED ANCHOR NOT RECORDED", ha="center", va="center", transform=axis.transAxes)
            axis.set_title(camera, fontsize=10)
            axis.axis("off")
        axis = figure.add_subplot(grid[2, :])
        extent = 2.
        if anchor:
            c, s = math.cos(anchor["yaw"]), math.sin(anchor["yaw"])
            for ref, label in (("actor", "Actor API path"), ("base", "Virtual-base path")):
                xy = [(row[ref][0] - anchor["base"][0], row[ref][1] - anchor["base"][1]) for row in points]
                right, forward = [s*x-c*y for x, y in xy], [c*x+s*y for x, y in xy]
                axis.plot(right, forward, label=label, marker=".", markevery=[-1], linewidth=1.5)
                extent = max(extent, max(map(abs, right + forward)) * 1.15)
            axis.scatter([0], [0], marker="^", color="black", label="Selected camera-frame virtual base")
            axis.legend(fontsize=8)
        axis.set(xlim=(-extent, extent), ylim=(-extent, extent), xlabel="Right (m)", ylabel="Forward (m)",
                 title="Actual trace through selected camera tick; ego centered; no predicted trajectory")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=.25)
        label = "NOT RECORDED" if bundle is None else f"tick{tick}, frame{bundle['frame']}, {len(bundle['images'])}/6 full-FOV views"
        final_note = ""
        if anchor_name == "scheduled_final" and result.get("expected_final_native_tick") is not None:
            final_note = f" | final native tick{result['expected_final_native_tick']}; camera offset{result['final_camera_native_offset_ticks']} tick"
        figure.suptitle(f"{key}: {result['status']} | {anchor_name}: {label}{final_note}")
        figure.savefig(output / f"{key}_{anchor_name}_six_camera_trace.png", dpi=135)
        plt.close(figure)


def analyze(campaign_root, output_dir, *, render=True):
    root, output = Path(campaign_root), Path(output_dir)
    require(root.is_dir() and not root.is_symlink(), "campaign root missing or symlinked")
    require(not output.exists() and not output.is_symlink() and all(not parent.is_symlink() for parent in output.parents),
            "fresh nonsymlink output required")
    require(not root.resolve().is_relative_to(output.resolve()) and not output.resolve().is_relative_to(root.resolve()),
            "input/output trees must be disjoint")
    analysis_sources = [Path(__file__).resolve(), Path(coast.__file__).resolve(), Path(stationary.__file__).resolve()]
    source_before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in analysis_sources}
    pins = {}
    plan = strict_json(read_bytes(root, "plan.json", pins))
    validate_plan(plan)
    trials = [validate_trial(root, case, plan, pins) for case in plan["cases"]]
    # HH_260906 - A failed first repeat cannot be followed by a new planned launch in this fixed campaign.
    for index, trial in enumerate(trials):
        if trial[0]["status"] != "COMPLETE":
            require(all(later[0]["status"] == "NOT_RUN" for later in trials[index + 1:]), "later arm executed after incomplete/failed arm")
    output.mkdir(parents=True, exist_ok=False)
    if render:
        for result, samples, cameras, worker in trials:
            render_trial(root, output, result, samples, cameras, worker, pins)
    for relative in tuple(pins):
        read_bytes(root, relative, pins, maximum=max(8 * 1024 * 1024, pins[relative]["size_bytes"]))
    require(source_before == {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in analysis_sources}, "analysis source changed")
    cases = [trial[0] for trial in trials]
    result = {"schema": "carla.coast_brake_transition_summary.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_id": plan["campaign_id"], "status": "MEASURED_NOT_ADMITTED" if all(row["status"] == "COMPLETE" for row in cases) else "PARTIAL_OR_FAILED_NOT_ADMITTED",
        "planned_case_count": 4, "complete_case_count": sum(row["status"] == "COMPLETE" for row in cases),
        "native_state_count": sum(row["native_state_count"] for row in cases), "camera_anchor_count": sum(row["camera_anchor_count"] for row in cases),
        "cases": cases, "pre_tail_comparison": pre_tail_comparison(trials),
        "source_sha256_before_after": source_before, "input_sha256_before_after": pins,
        "input_files_unchanged": True, "rendered": render, "scope": {**DENIALS,
            "intervals": "Phase assigned by endpoint; all crossing intervals retained.100ms global offsets integrate both native50ms subintervals.",
            "tail_window": "Tail-phase observations within the final10 observed seconds; no earlier-phase substitution. Signed vector and scalar-speed integrals are different quantities.",
            "native_sample_speed": "Phase speed statistics use all native20Hz samples, not decimated100ms endpoints.",
            "route_cte": "Worker-recorded route projection checked against fixed operational bound; not independently reprojected by this offline summary.",
            "camera_anchors": "First even native tick at/after first tail observation; final even tick at/before expected final native tick. Never substitute an earlier available bundle. Trace ends at the same camera tick.",
            "interpretation": "Descriptive coast/brake comparison only; sampled velocity signs do not prove substep backtracking or a physical/numerical cause. No actuator-timing or noise/gate conclusions."}}
    with (output / "summary.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.campaign_root, args.output_dir)
    print(json.dumps({key: result[key] for key in ("status", "complete_case_count", "native_state_count", "camera_anchor_count")}))
    return 0


if __name__ == "__main__":
    main()
