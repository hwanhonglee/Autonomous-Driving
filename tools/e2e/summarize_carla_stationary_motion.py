#!/usr/bin/env python3
"""HH_260906 - Measure stationary observations without replacing labels or approving data."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import stat


SCHEMA = "carla.stationary_motion_summary.v1"
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
PHASES = ("settle", "hold")
WORKER_SOURCES = {"scripts/e2e/" + name + ".py" for name in (
    "probe_carla_stationary_motion", "collect_carla_vad_expert", "calibrate_carla_low_speed_response",
    "probe_carla_stationary_camera_quality", "carla_goal_stop_profile", "carla_low_speed_response_matrix",
    "carla_physics_substeps")}
DENIALS = {name: False for name in (
    "training_data_approved", "dataset_admission", "labels_modified", "model_loaded",
    "model_inference", "training", "runtime_gate_evaluated", "decoder_bounds_evaluated",
    "noise_tolerance_established", "automatic_winner_selection")}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite non-Boolean number required")
    return float(value)


def integer(value):
    require(type(value) is int and value >= 0, "nonnegative integer required")
    return value


def vector(value):
    require(isinstance(value, (list, tuple)) and len(value) == 3, "three-vector required")
    return tuple(number(item) for item in value)


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def strict_json(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
                      parse_constant=lambda _: require(False, "nonfinite JSON constant"))


def checked_path(root, relative):
    require(type(relative) is str and relative and not Path(relative).is_absolute(), "relative input path required")
    require(all(part not in ("", ".", "..") for part in relative.split("/")), "unsafe input path")
    root = Path(root)
    path = root
    for part in Path(relative).parts:
        path = path / part
        require(not path.is_symlink(), "symlinked input path")
    require(path.is_file() and path.resolve().is_relative_to(root.resolve()), "missing or escaping input file")
    return path


def read_bytes(root, relative, pins, maximum=8 * 1024 * 1024):
    path = checked_path(root, relative)
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
        before = os.fstat(stream.fileno())
        require(stat.S_ISREG(before.st_mode) and before.st_size <= maximum, "input type or size differs")
        payload = stream.read(maximum + 1)
        after = os.fstat(stream.fileno())
    identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    require(identity(before) == identity(after) == identity(path.stat()) and len(payload) == before.st_size,
            "input changed during read")
    pin = {"sha256": hashlib.sha256(payload).hexdigest(), "size_bytes": len(payload)}
    require(relative not in pins or pins[relative] == pin, "input changed between reads")
    pins[relative] = pin
    return payload


def describe(values):
    values = [number(value) for value in values]
    if not values:
        return {"count": 0, "maximum": None, "rms": None}
    return {"count": len(values), "maximum": max(values),
            "rms": math.sqrt(sum(value * value for value in values) / len(values))}


def motion_sample(row, wheelbase_m=2.85):
    """HH_260906 - Keep the observed actor and legacy planar virtual-base references distinct."""
    require(row["phase"] in PHASES, "unknown native phase")
    transform = row["actor_snapshot_transform_carla"]
    transform = {key: number(transform[key]) for key in ("x", "y", "z", "yaw", "pitch", "roll")}
    velocity = vector(row["world_velocity_carla"])
    acceleration = vector(row["world_acceleration_carla"])
    angular = vector(row["world_angular_velocity_carla_deg_s"])
    base = {key: number(row[key]) for key in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate")}
    yaw, pitch = math.radians(transform["yaw"]), math.radians(transform["pitch"])
    offset = -number(wheelbase_m) / 2.0
    expected = {
        "x": transform["x"] + offset * math.cos(pitch) * math.cos(yaw),
        "y": -(transform["y"] + offset * math.cos(pitch) * math.sin(yaw)),
        "z": transform["z"] + offset * math.sin(pitch),
        "yaw": wrapped(-yaw),
        "vx": math.cos(yaw) * velocity[0] + math.sin(yaw) * velocity[1],
        "ax": math.cos(yaw) * acceleration[0] + math.sin(yaw) * acceleration[1],
        "ay": math.sin(yaw) * acceleration[0] - math.cos(yaw) * acceleration[1],
        "yaw_rate": -math.radians(angular[2]),
    }
    expected["vy"] = math.sin(yaw) * velocity[0] - math.cos(yaw) * velocity[1] + offset * expected["yaw_rate"]
    require(all(math.isclose(base[key], value, rel_tol=0.0, abs_tol=1e-10) for key, value in expected.items()),
            "recorded virtual-base transform differs")
    c, s = math.cos(base["yaw"]), math.sin(base["yaw"])
    stamp = integer(row["timestamp_ns"]) if "timestamp_ns" in row else round(number(row["timestamp"]) * 1e9)
    require(stamp >= 0 and stamp == round(number(row["timestamp"]) * 1e9), "native timestamp representations differ")
    return {"frame": integer(row["frame"]), "timestamp_ns": stamp, "phase": row["phase"],
            "yaw": base["yaw"], "actor": (transform["x"], -transform["y"]),
            "actor_velocity": (velocity[0], -velocity[1]), "base": (base["x"], base["y"]),
            "base_velocity": (c * base["vx"] - s * base["vy"], s * base["vx"] + c * base["vy"]),
            "actor_z": transform["z"], "base_z": base["z"]}


def interval_record(samples):
    require(len(samples) in (2, 3), "two or three native samples required")
    for first, last in zip(samples, samples[1:]):
        dt = (last["timestamp_ns"] - first["timestamp_ns"]) / 1e9
        require(last["frame"] == first["frame"] + 1 and abs(dt - .05) <= 5e-7, "native frame/cadence gap")
    first, last = samples[0], samples[-1]
    result = {"first_frame": first["frame"], "last_frame": last["frame"],
              "from_phase": first["phase"], "to_phase": last["phase"],
              "crosses_phase": len({sample["phase"] for sample in samples}) > 1,
              "yaw_variation_rad": sum(abs(wrapped(b["yaw"] - a["yaw"])) for a, b in zip(samples, samples[1:])),
              "references": {}}
    for ref in ("actor", "base"):
        delta = [last[ref][j] - first[ref][j] for j in range(2)]
        integral = [sum((a[ref + "_velocity"][j] + b[ref + "_velocity"][j]) * .5
                        * ((b["timestamp_ns"] - a["timestamp_ns"]) / 1e9)
                        for a, b in zip(samples, samples[1:])) for j in range(2)]
        residual = [delta[j] - integral[j] for j in range(2)]
        result["references"][ref] = {"delta_xy_m": delta, "distance_m": math.hypot(*delta),
            "trapezoidal_velocity_integral_xy_m": integral, "residual_xy_m": residual,
            "residual_norm_m": math.hypot(*residual)}
    return result


def summarize_motion(samples):
    """HH_260906 - Partition every interval by its endpoint phase; explicitly count phase crossings."""
    intervals = {"native_50ms": [interval_record(samples[i:i + 2]) for i in range(len(samples) - 1)]}
    for offset in (0, 1):
        intervals[f"derived_100ms_offset{offset}"] = [interval_record(samples[i:i + 3]) for i in range(offset, len(samples) - 2, 2)]
    output = {}
    for series, rows in intervals.items():
        groups = {}
        for phase in ("all", *PHASES):
            selected = rows if phase == "all" else [row for row in rows if row["to_phase"] == phase]
            points = samples if phase == "all" else [sample for sample in samples if sample["phase"] == phase]
            item = {"sample_count": len(points), "interval_count": len(selected),
                    "phase_crossing_interval_count": sum(row["crosses_phase"] for row in selected),
                    "yaw_total_variation_rad": sum(row["yaw_variation_rad"] for row in selected), "references": {}}
            for ref in ("actor", "base"):
                values = [row["references"][ref] for row in selected]
                delta = [sum(row["delta_xy_m"][j] for row in values) for j in range(2)]
                item["references"][ref] = {
                    "sample_speed_mps": describe([math.hypot(*point[ref + "_velocity"]) for point in points]),
                    "sample_net_drift_m": math.dist(points[0][ref], points[-1][ref]) if points else None,
                    "sample_yaw_net_change_rad": wrapped(points[-1]["yaw"] - points[0]["yaw"]) if points else None,
                    "interval_travel_m": sum(row["distance_m"] for row in values),
                    "interval_net_displacement_xy_m": delta,
                    "interval_net_displacement_m": math.hypot(*delta),
                    "displacement_minus_velocity_integral_m": describe([row["residual_norm_m"] for row in values]),
                }
            groups[phase] = item
        output[series] = groups
    return output, intervals


def validate_plan(plan):
    require(plan.get("schema") == "carla.stationary_motion_campaign.v1", "unknown campaign plan")
    expected_cases = [{"id": key, "arm": arm} for key, arm in (
        ("01_service_A1", "service"), ("02_parking_B1", "parking"),
        ("03_parking_B2", "parking"), ("04_service_A2", "service"))]
    require(plan.get("cases") == expected_cases, "fixed four-case ABBA plan required")
    for field, expected in (("physics_hz", 20), ("capture_hz", 10), ("settle_ticks", 70),
                            ("hold_ticks", 400), ("total_ticks", 470)):
        require(type(plan.get(field)) in (int, float) and plan[field] == expected, "campaign setting differs: " + field)
    for field in ("training_data_approved", "vehicle_control_approved"):
        require(plan.get(field) is False, "campaign approval boundary changed")
    require(plan.get("no_automatic_retry") is True, "no-retry plan required")
    for field in ("source_sha256", "input_sha256"):
        require(isinstance(plan.get(field), dict) and plan[field], "campaign pins missing: " + field)
        require(all(type(value) is str and len(value) == 64 and set(value) <= set("0123456789abcdef")
                    for value in plan[field].values()), "invalid SHA256")
    require("scripts/e2e/probe_carla_stationary_motion.py" in plan["source_sha256"], "worker source pin missing")
    require(len(plan["source_sha256"]) == 15, "fifteen execution-source pins required")
    require(WORKER_SOURCES <= set(plan["source_sha256"]), "worker import closure missing")
    require(set(plan["input_sha256"]) == {"route_file", "mapping", "calibration"}
            and plan.get("route_sha256") == plan["input_sha256"]["route_file"], "fixed input pins differ")
    require(plan.get("port") == 2100 and plan.get("quality") == "Epic"
            and plan.get("wall_timeout_sec") == 300, "fixed campaign ownership bounds differ")


def check_control(control, arm):
    require(isinstance(control, dict), "control object required")
    expected = {"throttle": 0.0, "brake": 1.0, "steer": 0.0,
                "hand_brake": arm == "parking", "reverse": False, "manual_gear_shift": False}
    for field, value in expected.items():
        if type(value) is bool:
            require(control.get(field) is value, "fixed control differs: " + field)
        else:
            require(abs(number(control.get(field)) - value) <= 1e-6, "fixed control differs: " + field)
    require(type(control.get("gear")) is int, "integer automatic gear required")


def check_settings(settings):
    require(isinstance(settings, dict) and settings.get("synchronous_mode") is True
            and settings.get("no_rendering_mode") is False and settings.get("substepping") is True,
            "rendered synchronous substep settings required")
    require(abs(number(settings.get("fixed_delta_seconds")) - .05) < 1e-9
            and abs(number(settings.get("max_substep_delta_time")) - .01) < 1e-9,
            "fixed20Hz/10ms settings differ")
    require(integer(settings.get("max_substeps")) == 10, "exact reference substep count required")


def decode_image(payload):
    from PIL import Image
    with Image.open(io.BytesIO(payload)) as source:
        require(source.format == "JPEG" and source.size == (640, 360), "camera JPEG format/size differs")
        source.load()
        return source.convert("RGB")


def read_rows(root, relative, pins):
    payload = read_bytes(root, relative, pins)
    require(not payload or payload.endswith(b"\n"), "truncated JSONL final record")
    rows = []
    for line in payload.splitlines():
        require(bool(line.strip()), "empty JSONL record")
        row = strict_json(line)
        require(isinstance(row, dict), "JSONL object required")
        rows.append(row)
    return rows


def verify_owner(root, case, plan, pins):
    prefix = case["id"] + "/"
    owner = strict_json(read_bytes(root, prefix + "owner_plan.json", pins))
    require(owner.get("schema") == "portable_e2e.owned_expert_trial.v1"
            and owner.get("capture_mode") == "stationary-motion", "owned stationary worker required")
    require(owner.get("source_sha256") == plan["source_sha256"] and owner.get("source_bytes_archived") is True,
            "owner execution sources differ from campaign")
    require(owner.get("route_sha256") == plan.get("route_sha256"), "owner route pin differs")
    planned_route = (Path(__file__).resolve().parents[2] / plan["route_path"]).resolve()
    require(Path(owner.get("route_path", "")).resolve() == planned_route and owner.get("host") == "127.0.0.1"
            and owner.get("worker_path") == "scripts/e2e/probe_carla_stationary_motion.py", "owner route/host/worker differs")
    declared = datetime.fromisoformat(plan["declared_at_utc"].replace("Z", "+00:00"))
    planned = datetime.fromisoformat(owner["planned_at_utc"].replace("Z", "+00:00"))
    require(declared.utcoffset() is not None and planned.utcoffset() is not None and declared <= planned,
            "owner predates declared campaign")
    argv = owner.get("collector_argv")
    require(isinstance(argv, list) and len(argv) == 8 and argv[1:] == [str(planned_route), "--host", "127.0.0.1",
            "--port", str(plan["port"]), "--arm", case["arm"]]
            and Path(argv[0]).resolve() == (root / case["id"] / "motion_probe").resolve(), "owner worker arguments differ")
    require(owner.get("quality") == plan.get("quality") and owner.get("port") == plan.get("port")
            and owner.get("collector_wall_timeout_sec") == plan.get("wall_timeout_sec"), "owner execution settings differ")
    require(owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False,
            "owner approval boundary changed")
    for name, expected in plan["source_sha256"].items():
        relative = prefix + "provenance/" + name
        read_bytes(root, relative, pins)
        require(pins[relative]["sha256"] == expected, "archived execution source differs: " + name)
    result_path = prefix + "owner_result.json"
    result = strict_json(read_bytes(root, result_path, pins)) if (root / result_path).exists() else None
    if result is not None:
        require(result.get("capture_mode") == "stationary-motion"
                and result.get("learned_model_control") is False and result.get("vehicle_control_approved") is False,
                "owner result identity/approval differs")
        require(type(result.get("exit_code")) is int, "owner exit status missing")
        require(result.get("source_bytes_unchanged_and_archived") is True
                and result.get("source_checks") == {key: True for key in plan["source_sha256"]},
                "owner source preservation failed")
    return owner, result


def check_receipt(receipt, arm, sequence, row=None):
    require(integer(receipt.get("sequence")) == sequence and receipt.get("mode") == "acknowledged_batch"
            and receipt.get("do_tick") is False and receipt.get("physical_actuation_proven") is False,
            "control receipt contract differs")
    require(receipt.get("reason") == ("settle" if sequence <= 70 else "hold"), "receipt phase differs")
    check_control(receipt.get("requested_control"), arm)
    require(receipt.get("status") == "ACKNOWLEDGED" and receipt.get("server_accepted") is True,
            "control receipt was not acknowledged")
    require(integer(receipt.get("actor_id")) > 0 and receipt["actor_id"] == receipt.get("response_actor_id"),
            "ACK actor differs")
    before = integer(receipt.get("before_frame"))
    require(before == receipt.get("expected_before_frame") == receipt.get("after_ack_frame"), "ACK frame bracket differs")
    if row is not None:
        require(before == row["frame"] - 1 and receipt["requested_control"] == row["requested_control"],
                "ACK/native binding differs")


def check_native_protocol(row, receipt, arm):
    check_receipt(receipt, arm, row["tick"], row)
    check_control(row["reported_control"], arm)
    alignment = row["control_alignment"]
    require(alignment.get("status") == "PASS" and alignment.get("frame_binding_pass") is True
            and alignment.get("expected_receipt_sequence") == row["tick"]
            and alignment.get("expected_command_before_frame") == row["frame"] - 1
            and alignment.get("observed_frame") == alignment.get("control_read_frame_before")
            == alignment.get("control_read_frame_after") == row["frame"]
            and alignment.get("mismatched_fields") == [] and alignment.get("physical_actuation_proven") is False,
            "control alignment evidence differs")
    require(row["observed_aux_frame_before"] == row["observed_aux_frame_after"] == row["frame"],
            "auxiliary frame bracket differs")
    require(row.get("collision") == [], "collision observed")
    require(row.get("measurement_complete") is True, "native measurement unfinished")


def check_lifecycle(root, case, plan, pins, native):
    prefix = case["id"] + "/"
    started = strict_json(read_bytes(root, prefix + "owner_started.json", pins))
    pid = integer(started["server_pid"])
    require(pid > 1 and started.get("server_pgid") == pid and started.get("route_sha256") == plan["route_sha256"]
            and started.get("map") == "C_track_1_0_7" and started.get("port") == 2100
            and started.get("quality") == "Epic" and started.get("learned_model_control") is False,
            "started owner identity differs")
    log = read_bytes(root, prefix + "server.log", pins, maximum=64 * 1024 * 1024)
    for stage in ("ready", "after_capture", "stopped"):
        row = strict_json(read_bytes(root, prefix + "lifecycle/" + stage + ".json", pins))
        require(row.get("schema_version") == 1 and row.get("status") == "PASS" and row.get("error") is None
                and row.get("stage") == stage and row.get("generation_id") == f"expert_{pid}"
                and row.get("owner_pid") == row.get("owner_pgid") == pid and row.get("host") == "127.0.0.1"
                and row.get("port") == 2100 and row.get("expected_map") == "C_track_1_0_7"
                and row.get("read_only") is True, "lifecycle ownership/status differs")
        if stage == "stopped":
            require(row.get("mode") == "stopped" and row.get("owner_process_state") is None
                    and row.get("port_released") is True and row.get("rpc_sequence") == [], "owned stop evidence differs")
        else:
            require(row.get("mode") == "running" and row.get("active_map_basename") == "C_track_1_0_7"
                    and row.get("owner_process_state") not in (None, "Z")
                    and row.get("rpc_sequence") == ["get_world", "world.get_map", "world.get_snapshot"], "running evidence differs")
            frame, timestamp = integer(row["snapshot_frame"]), number(row["snapshot_elapsed_seconds"])
            require(timestamp >= 0, "negative lifecycle time")
            if stage == "ready":
                require(frame <= native[0]["frame"] and timestamp <= native[0]["timestamp"], "ready lifecycle follows native capture")
            else:
                require(frame >= native[-1]["frame"] and timestamp >= native[-1]["timestamp"], "after-capture lifecycle predates native capture")
        recorded = row["server_log"]
        size = integer(recorded["size_bytes"])
        require(size <= len(log) and hashlib.sha256(log[:size]).hexdigest() == recorded["sha256"]
                and (stage != "stopped" or size == len(log)), "lifecycle server-log prefix differs")


def validate_trial(root, case, plan, pins):
    prefix = case["id"] + "/"
    if not (root / case["id"]).exists():
        return {"case_id": case["id"], "arm": case["arm"], "status": "NOT_RUN", "native_state_count": 0,
                "camera_anchor_count": 0, "planned_native_state_count": 470, **DENIALS}, [], [], None
    owner, owner_result = verify_owner(root, case, plan, pins)
    folders = [prefix + name for name in ("motion_probe", "motion_probe.partial") if (root / prefix / name).exists()]
    require(len(folders) <= 1, "ambiguous complete/partial worker output")
    if not folders or not (root / folders[0] / "manifest.json").exists():
        return {"case_id": case["id"], "arm": case["arm"], "status": "NO_FINAL_WORKER_MANIFEST",
                "owner_exit_code": None if owner_result is None else owner_result["exit_code"],
                "native_state_count": 0, "camera_anchor_count": 0, "planned_native_state_count": 470, **DENIALS}, [], [], None
    worker = folders[0] + "/"
    manifest = strict_json(read_bytes(root, worker + "manifest.json", pins))
    require(manifest.get("schema") == "carla.stationary_motion_probe.v1" and manifest.get("arm") == case["arm"],
            "worker identity differs")
    require(manifest.get("status") in ("COMPLETE", "FAIL") and manifest.get("training_data_approved") is False
            and manifest.get("vehicle_approved") is False, "worker status or approval differs")
    complete = manifest["status"] == "COMPLETE"
    for field, expected in (("scheduled_ticks", 470), ("settle_ticks", 70), ("hold_ticks", 400),
                            ("physics_hz", 20), ("camera_hz", 10)):
        require(type(manifest.get(field)) in (int, float) and manifest[field] == expected, "worker schedule differs")
    require(manifest.get("camera_bundle_observed_every_native_tick") is True, "per-native-frame camera observation required")
    for field, campaign_pins in (("source_sha256", plan["source_sha256"]), ("input_sha256", plan["input_sha256"])):
        before, after = manifest.get(field + "_before"), manifest.get(field + "_after")
        require(isinstance(before, dict) and before and before == after, "worker input/source preservation differs")
        require(all(campaign_pins.get(name) == digest for name, digest in before.items()), "worker pin not bound by campaign")
    require(set(manifest["source_sha256_before"]) == WORKER_SOURCES, "worker import closure differs")
    require(manifest["input_sha256_before"] == plan["input_sha256"], "worker fixed input set differs")
    protocol_issues = []
    try:
        check_settings(manifest.get("world_settings"))
        check_control(manifest["requested_control"], case["arm"])
    except (ValueError, KeyError) as error:
        protocol_issues.append({"tick": None, "reason": str(error)})
    native = read_rows(root, worker + "native_states.jsonl", pins)
    cameras = read_rows(root, worker + "camera_frames.jsonl", pins)
    receipts = read_rows(root, worker + "control_receipts.jsonl", pins)
    observations = read_rows(root, worker + "native_observations.jsonl", pins)
    artifacts = {"native_states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl", "native_observations.jsonl",
                 "route.json", "mapping.yaml", "calibration.yaml"}
    require(set(manifest["artifact_sha256"]) <= artifacts and (not complete or set(manifest["artifact_sha256"]) == artifacts),
            "worker artifact set differs")
    for name, expected in manifest["artifact_sha256"].items():
        relative = worker + name
        read_bytes(root, relative, pins)
        require(pins[relative]["sha256"] == expected, "worker artifact SHA differs: " + name)
    for artifact, logical in (("route.json", "route_file"), ("mapping.yaml", "mapping"), ("calibration.yaml", "calibration")):
        require(manifest["artifact_sha256"].get(artifact) == plan["input_sha256"][logical], "copied input SHA differs")
    require(owner["map"] == manifest["town"] == "C_track_1_0_7"
            and manifest["route_sha256"] == plan["route_sha256"] and number(manifest["wheelbase_m"]) == 2.85,
            "worker map/reference differs")
    require(integer(manifest["state_count"]) == len(native) <= 470, "worker native denominator differs")
    require([row["tick"] for row in native] == list(range(1, len(native) + 1)), "native tick sequence differs")
    require([row["phase"] for row in native] == ["settle"] * min(70, len(native)) + ["hold"] * max(0, len(native) - 70),
            "settle/hold phase sequence differs")
    require([row["tick"] for row in observations] == list(range(1, len(observations) + 1))
            and len(observations) <= len(native), "pre-camera observation denominator differs")
    for observation, row in zip(observations, native):
        require(observation.get("stage") == "OBSERVED_BEFORE_CAMERA" and observation.get("measurement_complete") is False,
                "pre-camera journal stage differs")
        require(all(row.get(key) == value for key, value in observation.items()
                    if key not in ("stage", "measurement_complete", "collision")), "pre-camera/native observations differ")
    require(len(native) <= len(receipts) <= min(470, len(native) + 1), "native/receipt denominators differ")
    require([row["sequence"] for row in receipts] == list(range(1, len(receipts) + 1)), "receipt sequence differs")
    require(len({row["actor_id"] for row in receipts}) <= 1, "multiple ACK actors")
    samples, unavailable = [], []
    for row in native:
        try:
            sample = motion_sample(row)
            require(not unavailable, "motion unavailable after interrupted prefix; no gap bridging")
            if samples:
                interval_record([samples[-1], sample])
            samples.append(sample)
        except (ValueError, KeyError, TypeError) as error:
            unavailable.append({"tick": row["tick"], "reason": str(error)})
        try:
            check_native_protocol(row, receipts[row["tick"] - 1], case["arm"])
        except (ValueError, KeyError, TypeError) as error:
            protocol_issues.append({"tick": row["tick"], "reason": str(error)})
    for receipt in receipts[len(native):]:
        try:
            check_receipt(receipt, case["arm"], receipt["sequence"])
        except (ValueError, KeyError, TypeError) as error:
            protocol_issues.append({"tick": receipt["sequence"], "reason": str(error)})
    motion, intervals = summarize_motion(samples)
    require(not complete or not protocol_issues and not unavailable, "complete worker contains failed native evidence")
    native_by_tick = {row["tick"]: row for row in native}
    require(len({row["tick"] for row in cameras}) == len(cameras), "duplicate camera tick")
    saved = partial = 0
    image_paths = set()
    for row in cameras:
        tick = integer(row["tick"])
        require(tick % 2 == 0 and tick in native_by_tick and row["frame"] == native_by_tick[tick]["frame"], "camera/native frame association differs")
        require(abs(number(row["timestamp"]) - number(native_by_tick[tick]["timestamp"])) <= 5e-7,
                "camera/native timestamp association differs")
        require(type(row["complete"]) is bool and isinstance(row["images"], dict), "camera availability mask differs")
        require(set(row["images"]) <= set(CAMERAS), "unknown camera")
        require(not row["complete"] or set(row["images"]) == set(CAMERAS), "complete camera bundle is incomplete")
        saved += row["complete"]
        partial += not row["complete"]
        for image in row["images"].values():
            relative = worker + image["path"]
            require(relative not in image_paths, "camera file reused across observations")
            image_paths.add(relative)
            payload = read_bytes(root, relative, pins)
            require(pins[relative]["sha256"] == image["sha256"], "camera SHA differs")
            require(abs(number(image["timestamp"]) - number(row["timestamp"])) <= 5e-7, "image timestamp differs")
            decode_image(payload)
    require(integer(manifest["saved_camera_anchor_count"]) == saved
            and integer(manifest["partial_camera_anchor_count"]) == partial, "camera denominator differs")
    if complete:
        require(len(native) == 470 and saved == 235 and partial == 0
                and [row["tick"] for row in cameras] == list(range(2, 471, 2)), "complete worker has incomplete counts")
        require(len(observations) == 470, "complete worker lacks pre-camera native observations")
        require(owner_result is not None and owner_result["exit_code"] == 0, "complete worker lacks successful owned exit")
        require(manifest.get("cleanup_errors") == [] and manifest.get("collision_events") == [], "complete cleanup/collision differs")
        owned = manifest.get("owned_actor_ids")
        require(isinstance(owned, list) and len(owned) == len(set(owned)) == 8
                and all(integer(value) > 0 for value in owned)
                and set(manifest.get("destroy_rpc_confirmed_actor_ids", [])) == set(owned), "owned actor cleanup differs")
        require(all(receipt["actor_id"] == owned[0] for receipt in receipts), "ACK actor is not owned ego")
        check_lifecycle(root, case, plan, pins, native)
    return {"case_id": case["id"], "arm": case["arm"], "status": manifest["status"],
            "owner_exit_code": None if owner_result is None else owner_result["exit_code"],
            "worker_folder": worker.rstrip("/"), "worker_error": manifest.get("error"),
            "worker_safety_failure": manifest.get("safety_failure"), "worker_cleanup_errors": manifest.get("cleanup_errors"),
            "planned_native_state_count": 470, "native_state_count": len(native),
            "measurement_complete_count": sum(row.get("measurement_complete") is True for row in native),
            "phase_counts": dict(Counter(row["phase"] for row in native)), "camera_anchor_count": saved,
            "partial_camera_anchor_count": partial, "decoded_image_count": len(image_paths),
            "motion_sample_count": len(samples), "motion_unavailable": unavailable, "control_receipt_count": len(receipts),
            "pre_camera_observation_count": len(observations),
            "protocol_issues": protocol_issues, "motion": motion, **DENIALS}, samples, cameras, worker


def render_trial(root, output, result, samples, cameras, worker, pins):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    key = result["case_id"]
    time = [(row["timestamp_ns"] - samples[0]["timestamp_ns"]) / 1e9 for row in samples] if samples else []
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for ref, label in (("actor", "Actor API"), ("base", "Virtual base")):
        if samples:
            origin = samples[0][ref]
            axes[0, 0].plot(time, [1000 * math.dist(row[ref], origin) for row in samples], label=label)
            axes[0, 1].plot(time, [math.hypot(*row[ref + "_velocity"]) for row in samples], label=label)
            rows = [interval_record(samples[i:i + 2])["references"][ref] for i in range(len(samples) - 1)]
            axes[1, 1].plot(time[1:], [1000 * row["residual_norm_m"] for row in rows], label=label)
    if samples:
        axes[1, 0].plot(time, [math.degrees(wrapped(row["yaw"] - samples[0]["yaw"])) for row in samples])
        hold = next((time[i] for i, row in enumerate(samples) if row["phase"] == "hold"), None)
        for axis in axes.flat:
            if hold is not None:
                axis.axvline(hold, color="gray", linestyle="--", label="hold starts")
    for axis, title, unit in zip(axes.flat, ("Distance from first observation", "Reported planar speed", "Body yaw from first observation", "50 ms position minus velocity integral"), ("mm", "m/s", "degrees", "mm")):
        axis.set(title=title, xlabel="Seconds from first observation", ylabel=unit)
        axis.grid(alpha=.25)
        if axis.lines:
            axis.legend(fontsize=8)
    figure.suptitle(f"{key}: {result['status']} | measured stationary probe; no data approval")
    figure.savefig(output / f"{key}_measured_traces.png", dpi=135)
    plt.close(figure)
    # HH_260906 - Fixed final scheduled tick; never substitute a prettier or earlier complete bundle.
    final = next((row for row in cameras if row["tick"] == 470), None)
    figure = plt.figure(figsize=(15, 10), constrained_layout=True)
    grid = figure.add_gridspec(3, 3, height_ratios=(1, 1, 1.15))
    for index, camera in enumerate(CAMERAS):
        axis = figure.add_subplot(grid[index // 3, index % 3])
        metadata = None if final is None else final["images"].get(camera)
        if metadata is not None:
            axis.imshow(decode_image(read_bytes(root, worker + metadata["path"], pins)))
        else:
            axis.text(.5, .5, "FINAL TICK NOT RECORDED", ha="center", va="center", transform=axis.transAxes)
        axis.set_title(camera, fontsize=10)
        axis.axis("off")
    axis = figure.add_subplot(grid[2, :])
    if samples:
        anchor = samples[-1]
        yaw = anchor["yaw"]
        extent = 2.0
        for ref, label in (("actor", "Actor API path"), ("base", "Virtual-base path")):
            # HH_260906 - Both references use the final virtual-base origin, preserving the real reference offset.
            xy = np.asarray([[row[ref][j] - anchor["base"][j] for j in range(2)] for row in samples])
            forward = math.cos(yaw) * xy[:, 0] + math.sin(yaw) * xy[:, 1]
            left = -math.sin(yaw) * xy[:, 0] + math.cos(yaw) * xy[:, 1]
            axis.plot(-left, forward, label=label, linewidth=1.5, marker=".", markevery=[-1])
            extent = max(extent, float(max(np.max(np.abs(left)), np.max(np.abs(forward)))) * 1.15)
        axis.scatter([0], [0], marker="^", color="black", label="Final observed virtual base")
        axis.set(xlim=(-extent, extent), ylim=(-extent, extent))
        axis.set_aspect("equal", adjustable="box")
        axis.legend(fontsize=8)
    axis.set(xlabel="Right (m)", ylabel="Forward (m)", title="Full measured trace in final observed ego frame; no predicted trajectory")
    axis.grid(alpha=.25)
    camera_status = f"frame{final['frame']}, {len(final['images'])}/6 cameras" if final else "NOT RECORDED"
    figure.suptitle(f"{key}: {result['status']} | full-FOV camera tick470; {camera_status}")
    figure.savefig(output / f"{key}_six_camera_trace.png", dpi=135)
    plt.close(figure)


def analyze(campaign_root, output_dir, *, render=True):
    root, output = Path(campaign_root), Path(output_dir)
    require(root.is_dir() and not root.is_symlink(), "campaign root missing or symlinked")
    require(not output.exists() and not output.is_symlink(), "fresh output directory required")
    require(not root.resolve().is_relative_to(output.resolve()) and not output.resolve().is_relative_to(root.resolve()),
            "output and input campaign trees must be disjoint")
    for parent in output.parents:
        require(not parent.is_symlink(), "symlinked output parent")
    pins = {}
    plan = strict_json(read_bytes(root, "plan.json", pins))
    validate_plan(plan)
    source_before = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    trials = [validate_trial(root, case, plan, pins) for case in plan["cases"]]
    output.mkdir(parents=True, exist_ok=False)
    for result, samples, cameras, worker in trials:
        if render:
            render_trial(root, output, result, samples, cameras, worker, pins)
    for relative in tuple(pins):
        read_bytes(root, relative, pins, maximum=max(8 * 1024 * 1024, pins[relative]["size_bytes"]))
    require(hashlib.sha256(Path(__file__).read_bytes()).hexdigest() == source_before, "analysis source changed")
    cases = [trial[0] for trial in trials]
    result = {"schema": SCHEMA, "created_at_utc": datetime.now(timezone.utc).isoformat(),
              "campaign_id": plan["campaign_id"], "status": "MEASURED_NOT_ADMITTED" if all(case["status"] == "COMPLETE" for case in cases) else "PARTIAL_OR_FAILED_NOT_ADMITTED",
              "planned_case_count": 4, "complete_case_count": sum(case["status"] == "COMPLETE" for case in cases),
              "native_state_count": sum(case["native_state_count"] for case in cases),
              "camera_anchor_count": sum(case["camera_anchor_count"] for case in cases), "cases": cases,
              "source_sha256": source_before, "input_sha256_before_after": pins, "input_files_unchanged": True,
              "rendered": render, "scope": {**DENIALS,
                  "interval_grouping": "By endpoint phase; crossing intervals retained and counted separately.",
                  "phase_control": "Settle and hold have identical arm-specific controls; the phase boundary is not an actuation change.",
                  "velocity_integral": "Trapezoid over every native50ms interval, including the midpoint for100ms.",
                  "reference": "Actor API world vectors reflected CARLA y-right to ROS y-left; virtual-base velocity retains legacy planar transform.",
                  "interpretation": "Descriptive stationary observations, not a universal noise bound or an explanation of moving-coast failures."}}
    with (output / "summary.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.campaign_root, args.output_dir)
    print(json.dumps({key: result[key] for key in ("status", "complete_case_count", "native_state_count", "camera_anchor_count")}))
    return 0


if __name__ == "__main__":
    main()
