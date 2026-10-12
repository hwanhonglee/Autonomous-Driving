#!/usr/bin/env python3
"""HH_260906 - Offline measured coast diagnostics; no model, gate evaluation, or data admission."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

if __package__:
    from . import summarize_carla_stationary_motion as stationary
else:
    import summarize_carla_stationary_motion as stationary

require, number, integer = stationary.require, stationary.number, stationary.integer
read_bytes, read_rows, strict_json = stationary.read_bytes, stationary.read_rows, stationary.strict_json
CAMERAS = stationary.CAMERAS
PHASES = ("settle", "launch", "coast")
WORKER = "scripts/e2e/probe_carla_low_speed_coast.py"
WORKER_SOURCES = stationary.WORKER_SOURCES | {WORKER}
CASES = [{"id": "01_coast_repeat_1"}, {"id": "02_coast_repeat_2"}]
ARTIFACTS = {"native_states.jsonl", "native_observations.jsonl", "control_receipts.jsonl", "camera_frames.jsonl",
             "route.json", "mapping.yaml", "calibration.yaml"}
DENIALS = {**stationary.DENIALS, "vehicle_control_approved": False, "physical_actuation_proven": False,
           "path_heading_assessed": False, "curvature_assessed": False}
FIXED = {"settle_ticks": 70, "max_launch_ticks": 160, "coast_ticks": 900, "launch_throttle": .13,
         "target_speed_mps": .5, "max_speed_mps": .75, "max_net_motion_m": 10., "max_route_cte_m": 1.,
         "minimum_body_vx_mps": -.1, "maximum_native_ticks": 1130, "physics_hz": 20, "capture_hz": 10,
         "wall_timeout_sec": 300}


def sample(row):
    require(row.get("phase") in PHASES, "unknown coast phase")
    # HH_260906 - Reuse only the frozen raw/reference transform; preserve the original coast phase explicitly.
    result = stationary.motion_sample(dict(row, phase="settle"))
    result.update(phase=row["phase"], tick=integer(row["tick"]), phase_tick=integer(row["phase_tick"]))
    return result


def interval_stats(rows):
    result = {"interval_count": len(rows), "phase_crossing_interval_count": sum(row["crosses_phase"] for row in rows),
              "yaw_total_variation_rad": sum(row["yaw_variation_rad"] for row in rows), "references": {}}
    for ref in ("actor", "base"):
        records = [row["references"][ref] for row in rows]
        delta = [sum(row["delta_xy_m"][axis] for row in records) for axis in range(2)]
        result["references"][ref] = {"travel_m": sum(row["distance_m"] for row in records),
            "net_displacement_xy_m": delta, "net_displacement_m": math.hypot(*delta),
            "displacement_minus_velocity_integral_m": stationary.describe([row["residual_norm_m"] for row in records])}
    return result


def speed_bin(speed):
    return "0_to_0.1" if speed <= .1 else "over0.1_to0.25" if speed <= .25 else "over0.25_to0.5" if speed <= .5 else "over0.5"


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
        output[name] = {"phases": phases, "coast_max_native_speed_bins": {
            key: interval_stats([row for row in records if row["to_phase"] == "coast"
                                 and speed_bin(row["maximum_native_actor_speed_mps"]) == key])
            for key in ("0_to_0.1", "over0.1_to0.25", "over0.25_to0.5", "over0.5")}}
        series_rows[name] = records
    return output, series_rows


def coast_observation(samples):
    """HH_260906 - Require two seconds between low-speed observations, not forty samples mistaken for two seconds."""
    points = [row for row in samples if row["phase"] == "coast"]
    if not points:
        return {"status": "COAST_NOT_OBSERVED", "native_sample_count": 0}
    previous = next((row for row in samples if row["tick"] == points[0]["tick"] - 1), None)
    origin_ns = previous["timestamp_ns"] if previous else points[0]["timestamp_ns"]
    low_start = confirmed = first_start = first_low = None
    reaccelerated = False
    for row in points:
        low = math.hypot(*row["actor_velocity"]) <= .1
        if not low:
            low_start = None
            reaccelerated |= confirmed is not None
        else:
            if first_low is None:
                first_low = row
            if low_start is None:
                low_start = row
            if confirmed is None and row["timestamp_ns"] - low_start["timestamp_ns"] >= 2_000_000_000:
                confirmed, first_start = row, low_start
    relative = lambda row: None if row is None else (row["timestamp_ns"] - origin_ns) / 1e9
    final = points[-1]
    tail = [row for row in points if row["timestamp_ns"] >= final["timestamp_ns"] - 10_000_000_000]
    return {"status": "LOW_SPEED_DWELL_OBSERVED" if confirmed else "LOW_SPEED_DWELL_NOT_OBSERVED",
            "criterion": "Observed actor planar speed <=0.1 m/s continuously for at least2.0 s; not physical standstill or a gate pass.",
            "native_sample_count": len(points), "coast_control_time_origin_tick": None if previous is None else previous["tick"],
            "observed_elapsed_from_control_boundary_s": relative(final),
            "first_observed_speed_at_or_below_0.1_tick": None if first_low is None else first_low["tick"],
            "first_observed_speed_at_or_below_0.1_time_s": relative(first_low),
            "first_qualifying_low_speed_tick": None if first_start is None else first_start["tick"],
            "first_qualifying_low_speed_time_s": relative(first_start),
            "dwell_confirmed_tick": None if confirmed is None else confirmed["tick"], "dwell_confirmed_time_s": relative(confirmed),
            "final_continuous_low_speed_dwell_s": 0. if low_start is None else (final["timestamp_ns"] - low_start["timestamp_ns"]) / 1e9,
            "speed_rose_above_0.1_after_first_confirmation": reaccelerated,
            "final_actor_planar_speed_mps": math.hypot(*final["actor_velocity"]),
            "final_body_forward_speed_mps": math.cos(final["yaw"]) * final["actor_velocity"][0] + math.sin(final["yaw"]) * final["actor_velocity"][1],
            "coast_sample_net_drift_m": {ref: math.dist(points[0][ref], final[ref]) for ref in ("actor", "base")},
            "tail": {"definition": "Coast observations within the last10 observed seconds; truncated if coast is shorter.",
                "native_sample_count": len(tail), "sample_span_s": (tail[-1]["timestamp_ns"] - tail[0]["timestamp_ns"]) / 1e9,
                "actor_speed_mps": stationary.describe([math.hypot(*row["actor_velocity"]) for row in tail]),
                "actor_travel_m": sum(math.dist(a["actor"], b["actor"]) for a, b in zip(tail, tail[1:])),
                "actor_net_drift_m": math.dist(tail[0]["actor"], tail[-1]["actor"])} }


def validate_plan(plan):
    require(plan.get("schema") == "carla.low_speed_coast_campaign.v1" and plan.get("cases") == CASES,
            "fixed two-repeat coast plan required")
    for key, value in FIXED.items():
        require(number(plan.get(key)) == value, "fixed campaign setting differs: " + key)
    require(plan.get("port") == 2100 and plan.get("quality") == "Epic" and plan.get("map") == "C_track_1_0_7",
            "fixed campaign server differs")
    require(plan.get("training_data_approved") is False and plan.get("vehicle_control_approved") is False
            and plan.get("no_automatic_retry") is True, "campaign approval/retry boundary differs")
    for name in ("source_sha256", "input_sha256"):
        require(isinstance(plan.get(name), dict) and all(type(value) is str and len(value) == 64
                and set(value) <= set("0123456789abcdef") for value in plan[name].values()), "campaign hash pins differ")
    require(len(plan["source_sha256"]) == 16 and WORKER_SOURCES <= set(plan["source_sha256"]), "full execution-source closure required")
    require(set(plan["input_sha256"]) == {"route_file", "mapping", "calibration"}
            and plan["route_sha256"] == plan["input_sha256"]["route_file"], "fixed input closure differs")


def check_control(value, phase):
    require(isinstance(value, dict), "control object required")
    for field, expected in (("throttle", .13 if phase == "launch" else 0.), ("brake", 1. if phase == "settle" else 0.), ("steer", 0.)):
        require(abs(number(value.get(field)) - expected) <= 1e-6, "phase control differs: " + field)
    require(all(value.get(name) is False for name in ("hand_brake", "reverse", "manual_gear_shift"))
            and type(value.get("gear")) is int, "control flag/automatic gear differs")


def verify_owner(root, case, plan, pins):
    prefix = case["id"] + "/"
    owner = strict_json(read_bytes(root, prefix + "owner_plan.json", pins))
    route = str((Path(__file__).resolve().parents[2] / plan["route_path"]).resolve())
    require(owner.get("schema") == "portable_e2e.owned_expert_trial.v1" and owner.get("capture_mode") == "low-speed-coast"
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
    require(owner.get("collector_argv") == [str((root / case["id"] / "coast_probe").resolve()), route,
            "--host", "127.0.0.1", "--port", str(plan["port"])], "owned worker arguments differ")
    for name, expected in plan["source_sha256"].items():
        relative = prefix + "provenance/" + name
        read_bytes(root, relative, pins)
        require(pins[relative]["sha256"] == expected, "archived source SHA differs: " + name)
    result = None
    if (root / prefix / "owner_result.json").exists():
        result = strict_json(read_bytes(root, prefix + "owner_result.json", pins))
        require(result.get("capture_mode") == "low-speed-coast" and type(result.get("exit_code")) is int
                and result.get("learned_model_control") is False and result.get("vehicle_control_approved") is False,
                "owner result contract differs")
        require(result.get("source_bytes_unchanged_and_archived") is True
                and result.get("source_checks") == {key: True for key in plan["source_sha256"]}, "owned source preservation differs")
    return owner, result


def check_receipt(receipt, row):
    require(all(type(receipt.get(key)) is int for key in ("sequence", "actor_id", "response_actor_id",
            "before_frame", "expected_before_frame", "after_ack_frame")), "integer ACK identities required")
    require(receipt.get("sequence") == row["tick"] and receipt.get("reason") == row["phase"]
            and receipt.get("mode") == "acknowledged_batch" and receipt.get("do_tick") is False
            and receipt.get("physical_actuation_proven") is False, "ACK journal identity differs")
    require(receipt.get("status") == "ACKNOWLEDGED" and receipt.get("server_accepted") is True
            and integer(receipt.get("actor_id")) > 0 and receipt.get("response_actor_id") == receipt["actor_id"], "ACK acceptance differs")
    require(receipt.get("before_frame") == receipt.get("expected_before_frame") == receipt.get("after_ack_frame")
            == row["frame"] - 1, "ACK frame bracket differs")
    check_control(receipt["requested_control"], row["phase"])
    require(receipt["requested_control"] == row["requested_control"], "ACK/native request differs")


def check_native_protocol(row, receipt):
    check_receipt(receipt, row)
    check_control(row["reported_control"], row["phase"])
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
            require(first_reached == 70 + counts["launch"] and counts[phase] <= 900, "coast lacks first observed launch threshold")
        previous = phase
    if complete:
        require(counts["settle"] == 70 and 1 <= counts["launch"] <= 160 and counts["coast"] == 900
                and first_reached == 70 + counts["launch"], "complete trial lacks full fixed coast")
    return dict(counts), first_reached


def validate_cameras(root, worker, rows, native, manifest, pins, complete):
    by_tick = {row["tick"]: row for row in native}
    require([integer(row["tick"]) for row in rows] == sorted({row["tick"] for row in rows}), "camera tick order/reuse differs")
    saved = partial = 0
    files = set()
    for row in rows:
        tick = row["tick"]
        require(tick > 0 and tick % 2 == 0 and tick in by_tick and row["frame"] == by_tick[tick]["frame"], "camera/native frame differs")
        require(abs(number(row["timestamp"]) - number(by_tick[tick]["timestamp"])) <= 5e-7
                and integer(row["timestamp_ns"]) == by_tick[tick]["timestamp_ns"], "camera/native time differs")
        require(type(row.get("complete")) is bool and isinstance(row.get("images"), dict)
                and set(row["images"]) <= set(CAMERAS), "camera availability mask differs")
        require(not row["complete"] or len(row["images"]) == 6, "complete camera bundle missing views")
        saved += row["complete"]
        partial += not row["complete"]
        for metadata in row["images"].values():
            path = worker + metadata["path"]
            require(path not in files, "camera image reused")
            files.add(path)
            payload = read_bytes(root, path, pins)
            require(pins[path]["sha256"] == metadata["sha256"]
                    and abs(number(metadata["timestamp"]) - number(row["timestamp"])) <= 5e-7, "camera content/time pin differs")
            stationary.decode_image(payload)
    require(integer(manifest["saved_camera_anchor_count"]) == saved
            and integer(manifest["partial_camera_anchor_count"]) == partial, "camera denominator differs")
    if complete:
        require([row["tick"] for row in rows] == list(range(2, len(native) + 1, 2)) and partial == 0, "complete camera schedule differs")
    return saved, partial, len(files)


def check_environment(manifest, first_native_frame):
    boolean = {"synchronous_mode", "no_rendering_mode", "substepping", "deterministic_ragdolls", "spectator_as_ego"}
    scalar = {"fixed_delta_seconds", "max_substep_delta_time", "max_culling_distance", "tile_stream_distance", "actor_active_distance"}
    for key in ("world_settings_before", "world_settings_requested", "world_settings", "world_settings_restored"):
        settings = manifest[key]
        require(set(settings) == boolean | scalar | {"max_substeps"}
                and all(type(settings[name]) is bool for name in boolean), "full eleven-field settings contract differs")
        for name in scalar:
            require(name == "fixed_delta_seconds" and settings[name] is None or number(settings[name]) >= 0., "invalid world scalar")
        require(1 <= integer(settings["max_substeps"]) <= 16, "invalid substep count")
    require(manifest["world_settings_requested"] == manifest["world_settings"]
            and manifest["world_settings_before"] == manifest["world_settings_restored"], "full settings readback/restore differs")
    expected = dict(manifest["world_settings_before"], synchronous_mode=True, no_rendering_mode=False,
                    fixed_delta_seconds=.05, substepping=True, max_substep_delta_time=.01, max_substeps=10)
    require(manifest["world_settings_requested"] == expected, "undeclared world-setting change")
    weather_fields = {"cloudiness", "dust_storm", "fog_density", "fog_distance", "fog_falloff", "mie_scattering_scale",
        "precipitation", "precipitation_deposits", "rayleigh_scattering_scale", "scattering_intensity", "sun_altitude_angle",
        "sun_azimuth_angle", "wetness", "wind_intensity"}
    for key in ("weather_before", "weather_requested", "weather_actual", "weather_restored"):
        require(set(manifest[key]) == weather_fields, "weather scalar closure differs")
        for value in manifest[key].values(): number(value)
    require(manifest["weather_requested"] == manifest["weather_actual"]
            and manifest["weather_before"] == manifest["weather_restored"] and manifest["weather"] == "ClearNoon", "weather readback/restore differs")
    preflight = manifest["preflight"]
    require(preflight.get("status") == "PASS" and preflight.get("foreign_dynamic_actors_absent") is True
            and integer(preflight["frame_before"]) == preflight["frame_after"] < first_native_frame, "preflight frame/ownership differs")
    lanes, rays = preflight["corridor_samples"], preflight["obstacle_rays"]
    require(lanes and all(row.get("status") == "PASS" for row in lanes), "corridor samples failed")
    require(len(rays) == 6 and {(row["lateral_offset_m"], row["height_above_road_m"]) for row in rays}
            == {(lateral, height) for lateral in (0., -.75, .75) for height in (.5, 1.25)}
            and all(row.get("status") == "CLEAR_SAMPLED_RAY" and row.get("hits") == [] for row in rays), "preflight sampled rays differ")


def validate_trial(root, case, plan, pins):
    empty = {"case_id": case["id"], "native_state_count": 0, "camera_anchor_count": 0,
             "maximum_native_ticks": 1130, **DENIALS}
    if not (root / case["id"]).exists():
        return dict(empty, status="NOT_RUN"), [], [], None
    owner, owned_result = verify_owner(root, case, plan, pins)
    folders = [case["id"] + "/" + name for name in ("coast_probe", "coast_probe.partial") if (root / case["id"] / name).exists()]
    require(len(folders) <= 1, "ambiguous coast output")
    if not folders or not (root / folders[0] / "manifest.json").exists():
        return dict(empty, status="NO_FINAL_WORKER_MANIFEST", count_scope="No authenticated finalized native/camera rows."), [], [], None
    worker = folders[0] + "/"
    manifest = strict_json(read_bytes(root, worker + "manifest.json", pins))
    require(manifest.get("schema") == "carla.low_speed_coast_probe.v1" and manifest.get("status") in ("COMPLETE", "FAIL"),
            "coast manifest identity/status differs")
    complete = manifest["status"] == "COMPLETE"
    require(all(manifest.get(key) is False for key in ("training_data_approved", "vehicle_approved", "vehicle_control_approved",
            "learned_model_control", "data_admission_performed")), "worker approval boundary differs")
    for key in ("settle_ticks", "max_launch_ticks", "coast_ticks", "launch_throttle", "target_speed_mps"):
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
            check_control(manifest["controls_by_phase"][phase], phase)
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
            check_native_protocol(row, receipts[row["tick"] - 1])
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
        require(len(transitions) == 2 and manifest.get("launch_handoff") == transitions[1], "phase transition/handoff ledger differs")
        for transition, before, after, source_tick in zip(transitions, PHASES[:2], PHASES[1:], (70, threshold_tick)):
            source = native[source_tick - 1]
            require(transition.get("from_phase") == before and transition.get("to_phase") == after
                    and transition.get("source_tick") == source_tick and transition.get("source_phase_tick") == source["phase_tick"]
                    and transition.get("source_frame") == source["frame"] and transition.get("source_timestamp") == source["timestamp"]
                    and transition.get("source_planar_speed_mps") == source["planar_speed_mps"]
                    and transition.get("effective_tick") == source_tick + 1 and transition.get("physical_transition_time_proven") is False,
                    "phase handoff is not bound to its actual source observation")
            require(transition.get("next_requested_control") == native[source_tick]["requested_control"], "handoff/next ACK control differs")
        require(owned_result is not None and owned_result["exit_code"] == 0 and manifest.get("cleanup_errors") == []
                and manifest.get("collision_events") == [] and manifest.get("completion_reason") == "full_900_tick_zero_pedal_coast_observed",
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
    anchors = {"coast_entry": None if threshold_tick is None else threshold_tick + 1 + ((threshold_tick + 1) % 2),
               "scheduled_final": None if expected_final is None else expected_final - expected_final % 2}
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
            "motion": motion, "coast_observation": coast_observation(samples)}, samples, cameras, worker


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
            axes[0, 1].plot(times, [math.dist(row[ref], samples[0][ref]) for row in samples], label=label)
            residuals = [stationary.interval_record([a, b])["references"][ref]["residual_norm_m"] * 1000
                         for a, b in zip(samples, samples[1:])]
            axes[1, 0].plot(times[1:], residuals, label=label)
    if samples:
        axes[1, 1].plot(times, [math.degrees(stationary.wrapped(row["yaw"] - samples[0]["yaw"])) for row in samples], label="Measured body yaw")
        for i, point in enumerate(samples[1:], 1):
            if point["phase"] != samples[i - 1]["phase"]:
                for axis in axes.flat:
                    axis.axvline(times[i - 1], color="gray", linestyle="--", label=point["phase"] + " command boundary")
    for axis, title, unit in zip(axes.flat, ("Reported planar speed", "Distance from first native observation",
            "50ms position minus native trapezoidal velocity integral", "Body yaw (not path heading)"), ("m/s", "m", "mm", "degrees")):
        axis.set(title=title, xlabel="Seconds from first native observation", ylabel=unit)
        axis.grid(alpha=.25)
        if axis.lines: axis.legend(fontsize=7)
    figure.suptitle(f"{key}: {result['status']} | measured controls and motion; no model or admission")
    figure.savefig(output / f"{key}_measured_traces.png", dpi=135)
    plt.close(figure)
    for anchor_name in ("coast_entry", "scheduled_final"):
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
    analysis_sources = [Path(__file__).resolve(), Path(stationary.__file__).resolve()]
    source_before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in analysis_sources}
    pins = {}
    plan = strict_json(read_bytes(root, "plan.json", pins))
    validate_plan(plan)
    trials = [validate_trial(root, case, plan, pins) for case in plan["cases"]]
    # HH_260906 - A failed first repeat cannot be followed by a new planned launch in this fixed campaign.
    require(trials[0][0]["status"] == "COMPLETE" or trials[1][0]["status"] == "NOT_RUN", "second repeat executed after first failed")
    output.mkdir(parents=True, exist_ok=False)
    if render:
        for result, samples, cameras, worker in trials:
            render_trial(root, output, result, samples, cameras, worker, pins)
    for relative in tuple(pins):
        read_bytes(root, relative, pins, maximum=max(8 * 1024 * 1024, pins[relative]["size_bytes"]))
    require(source_before == {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in analysis_sources}, "analysis source changed")
    cases = [trial[0] for trial in trials]
    result = {"schema": "carla.low_speed_coast_summary.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_id": plan["campaign_id"], "status": "MEASURED_NOT_ADMITTED" if all(row["status"] == "COMPLETE" for row in cases) else "PARTIAL_OR_FAILED_NOT_ADMITTED",
        "planned_case_count": 2, "complete_case_count": sum(row["status"] == "COMPLETE" for row in cases),
        "native_state_count": sum(row["native_state_count"] for row in cases), "camera_anchor_count": sum(row["camera_anchor_count"] for row in cases),
        "cases": cases, "source_sha256_before_after": source_before, "input_sha256_before_after": pins,
        "input_files_unchanged": True, "rendered": render, "scope": {**DENIALS,
            "intervals": "Phase assigned by endpoint; all crossing intervals retained.100ms global offsets integrate both native50ms subintervals.",
            "speed_bins": "Coast-endpoint intervals classified by maximum observed actor speed among all native points in the interval; fixed descriptive bins only.",
            "native_sample_speed": "Phase speed statistics use all native20Hz samples, not decimated100ms endpoints.",
            "route_cte": "Worker-recorded route projection checked against fixed operational bound; not independently reprojected by this offline summary.",
            "camera_anchors": "First even native tick at/after first coast observation; final even tick at/before expected final native tick. Never substitute an earlier available bundle. Trace ends at the same camera tick.",
            "interpretation": "Observed low-speed dwell is not exact standstill, physical actuator timing, a learned result, or permission to change labels/gates/noise tolerances."}}
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
