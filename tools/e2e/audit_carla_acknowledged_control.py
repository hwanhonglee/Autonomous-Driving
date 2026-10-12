#!/usr/bin/env python3
"""HH_260906 - Independently audit acknowledged command/frame evidence, never data admission or actuation safety."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

if __package__:
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
# HH_260906 - The new protocol has a separate explicit source pin; the legacy v3 auditor remains unchanged.
REVIEWED_ACK_SOURCES = {
    "scripts/e2e/collect_carla_vad_expert.py": "cccf656f3c865c487b6a0fce6f34b7fc2f32eb682c03b9ce5a308a3f904f05c4",
    "scripts/e2e/carla_goal_stop_profile.py": "2ef5d93665d7500e2f33a650e8abd9f26d2f44f903fd569780aa49db58d68dcf",
}
FROZEN_CAMPAIGN = {
    "source_commit": "cb46f359834f959db819f7cd689427f230031760",
    "collector_sha256": REVIEWED_ACK_SOURCES["scripts/e2e/collect_carla_vad_expert.py"],
    "goal_helper_sha256": REVIEWED_ACK_SOURCES["scripts/e2e/carla_goal_stop_profile.py"],
    "control_transport": "acknowledged_batch", "kinematics_source": "immutable_actor_snapshot",
    "profile": "comfortable_v3", "route_sha256": pilot.ROUTE_SHA,
    "route_length_m": 210.5975914062836, "map": "Town07", "scenario": "straight",
    "vehicle": "vehicle.toyota.prius", "weather": "ClearNoon", "seed": 0, "quality": "Low",
    "nominal_target_speed_kmh": 28.8, "maximum_actual_speed_kmh": 30.0,
    "physics_hz": 20.0, "camera_hz": 10.0, "maximum_total_sim_seconds": 180.0,
    "wall_timeout_seconds": 900, "finish_before_utc": "2026-09-08T01:00:00Z",
    "maximum_attempts_per_revision": 2, "automatic_retry": False,
    "first_output": "town07_straight_calibration/run_001",
    "baseline_outputs": ["../comfortable_goal_stop_v3/town07_straight_calibration/run_001",
                         "../comfortable_goal_stop_v3/town07_straight_calibration/run_002"],
    "normal_lateral_controller_unchanged": True, "normal_longitudinal_profile_unchanged": True,
    "emergency_control_unchanged": True, "quality_limits_unchanged": True,
    "training_data_approved": False, "learned_model_control": False, "remote_training_started": False,
}
CONTROL_FIELDS = ("throttle", "brake", "steer", "hand_brake", "reverse", "gear", "manual_gear_shift")
REASONS = {"pre_capture_bootstrap", "stationary_warmup_start", "initial_governor_failure_abort",
           "initial_drive_control", "driving_end_brake", "alignment_failure_abort", "exception_cleanup_abort",
           *("next_" + phase + "_control" for phase in base.PHASES)}
CARLA_SOURCE_HEAD = "25623bfdb3ecee680eb27b644f8ab3c2ffd4403a"
CARLA_SOURCE_PINS = {
    "LibCarla/source/carla/client/Vehicle.cpp": "b6add7f50b913c44b116e122aa2a8f7c1e75208ce4c114c0f8ac1aa75a6dd759",
    "LibCarla/source/carla/client/detail/Client.cpp": "d4672f5b430cfb7224d1ae83d87125beb49d25ba80da8867ee3ff03d6706d41c",
    "LibCarla/source/carla/client/detail/Simulator.cpp": "67b9eb6e1d692469d6dcbe03ce981385114bc4693d28e6efd4826d4dab519ea5",
    "LibCarla/source/carla/client/detail/Simulator.h": "9ebb453f568356af4d7556adf95719b5692a8979937e62a020a32f52fad484c8",
    "LibCarla/source/carla/client/ActorSnapshot.h": "0661ce9e5bf4789028273ce1cd0ff144e549359d654c1751b2270fcd8d444084",
    "LibCarla/source/carla/rpc/Server.h": "dc40af145d410b994e0cd6f02fd66744a9f24b8671e8c5997467402394585f75",
    "LibCarla/source/carla/rpc/Client.h": "13e74f4b4d45ef771a974964a7f3559ea33906d818ee39c3f95fe8de23dc38bb",
    "PythonAPI/carla/source/libcarla/Client.cpp": "999e69519ca8bc75ae056f27b154d821aebc0f83d4ff9c396c703a17666e8b4f",
    "PythonAPI/carla/source/libcarla/Snapshot.cpp": "65ba5921d3897d942bc7697bbc6af53aa7ed5668fc947dd92f74ba2d428f1f0d",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp": "6c7ca2b804afc7cd169dceb5249a94be866d37b6fc0fa72a95a62f46ff76cb70",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEngine.cpp": "09b600afeb6504e1a83d8f0b2713683487317910bcc0b8f7a588b4b129fb3eb6",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/WorldObserver.cpp": "5a5cd9279bfcd95ab7250a868f7bbd3ce9cfeb3ec1bc6dbc9d032ea4cedb8695",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp": "1244cb0c6d64e619cb34898a4b8976bc2e28c8c6748efabb7ec5eac46c341c44",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.h": "1588342d4f677b43738296698213acc0e38393611dd309b5e164726e1a0a0a8b",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleAIController.cpp": "a0d62018fa4d79ed736542ea926ee51836feee7c2cc7b0ba793d9fc0889cb902",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/MovementComponents/DefaultMovementComponent.cpp": "55b38bf86da4a5e6eb6ebc62ce0f3e6da1b8aa4dcbc0138af36e550436aca42f",
}


def primary_source_evidence():
    """HH_260906 - Preserve the completed read-only source inspection, not a claim of linked server binary identity."""
    return {"schema": "carla.control_transport_source_inspection.v1", "inspection_date": "2026-09-08",
        "local_source_head_at_inspection": CARLA_SOURCE_HEAD, "local_describe_at_inspection": "0.9.15-dev-dirty",
        "source_files": [{"path": name, "sha256": digest, "matched_local_head_bytes_at_inspection": True}
                         for name, digest in CARLA_SOURCE_PINS.items()],
        "executed_server_or_python_binary_match_proven": False,
        "findings": [
            "Vehicle.apply_control sends an asynchronous RPC and does not await command acceptance.",
            "world.tick synchronously sends tick_cue then waits until the cached episode frame reaches the requested frame.",
            "Vehicle.get_control reads the same cached EpisodeState actor data as live kinematic getters; it is not a new server RPC.",
            "C++ ActorSnapshot includes vehicle control, but the Python binding exposes only transform, velocity, angular velocity and acceleration.",
            "apply_batch_sync with one ApplyVehicleControl and False awaits the same server handler without requesting a tick.",
            "Server input acceptance, prephysics control flush, postphysics observer reporting and actual movement filtering are distinct stages.",
            "All old two-row matches are observational associations, not proof of physical command latency; no historical labels were shifted."],
        "limitations": "These exact local primary-source bytes were inspected read-only. Matching version strings do not prove that the installed client or executed server was built from them."}


def full_control(value):
    base.require(isinstance(value, dict) and set(value) == set(CONTROL_FIELDS), "invalid full control field set")
    for name in ("throttle", "brake", "steer"):
        number = base.number(value[name])
        base.require(-1 <= number <= 1 if name == "steer" else 0 <= number <= 1, "control outside normalized range")
    base.require(all(type(value[name]) is bool for name in ("hand_brake", "reverse", "manual_gear_shift"))
                 and type(value["gear"]) is int, "control flag or gear type mismatch")
    return value


def mismatched_controls(observed, requested, *, requested_pair=False):
    observed, requested = full_control(observed), full_control(requested)
    keys = [name for name in ("throttle", "brake", "steer") if abs(observed[name] - requested[name]) > 1e-6]
    keys += [name for name in ("hand_brake", "reverse", "manual_gear_shift") if observed[name] != requested[name]]
    if (requested_pair or requested["manual_gear_shift"]) and observed["gear"] != requested["gear"]:
        keys.append("gear")
    return keys


def recompute_snapshot_state(row, wheelbase):
    """HH_260906 - Independently reproduce the unchanged planar legacy conversion, including its acceleration convention."""
    pose = row["actor_snapshot_transform_carla"]
    base.require(set(pose) == {"x", "y", "z", "roll", "pitch", "yaw"}, "raw snapshot pose must include all six fields")
    pose = {name: base.number(value) for name, value in pose.items()}
    vectors = []
    for name in ("world_velocity_carla", "world_acceleration_carla", "world_angular_velocity_carla_deg_s"):
        values = row[name]
        base.require(isinstance(values, (tuple, list)) and len(values) == 3, "raw snapshot vector must have three coordinates")
        vectors.append([base.number(value) for value in values])
    velocity, acceleration, angular = vectors
    yaw, pitch, half = math.radians(pose["yaw"]), math.radians(pose["pitch"]), wheelbase / 2
    c, s = math.cos(yaw), math.sin(yaw)
    yaw_rate = -math.radians(angular[2])
    return {"x": pose["x"] - half * math.cos(pitch) * c,
        "y": -(pose["y"] - half * math.cos(pitch) * s), "z": pose["z"] - half * math.sin(pitch),
        "yaw": math.atan2(-s, c),
        "vx": c * velocity[0] + s * velocity[1],
        "vy": s * velocity[0] - c * velocity[1] - half * yaw_rate,
        "ax": c * acceleration[0] + s * acceleration[1], "ay": s * acceleration[0] - c * acceleration[1],
        "yaw_rate": yaw_rate}


def analyze_transport(states, receipts, bootstrap, *, wheelbase=2.85):
    """HH_260906 - Separate well-formed failed transport evidence from a coherent complete command/frame chain."""
    flags = {name: True for name in ("nonempty_observation", "all_commands_acknowledged", "command_frame_consistency",
        "all_snapshot_conversions_match", "control_frame_binding", "observed_controls_match_prior_ack",
        "worker_alignment_claims_match", "next_command_links", "phase_command_ledger", "twenty_hz_frame_chain")}
    failures, phase_counts = [], Counter()
    def check(name, passed, identifier, detail):
        if not passed:
            flags[name] = False
            failures.append({"check": name, "identifier": identifier, "detail": detail})
    base.require(isinstance(receipts, list), "receipt journal must be a list")
    actor_id = None
    previous_before = None
    by_sequence = {}
    for sequence, receipt in enumerate(receipts, 1):
        base.require(type(receipt.get("sequence")) is int and receipt["sequence"] == sequence,
                     "receipt journal sequence gap, duplicate or reordering")
        base.require(receipt.get("mode") == "acknowledged_batch" and receipt.get("do_tick") is False
                     and receipt.get("physical_actuation_proven") is False, "receipt protocol or scope mismatch")
        base.require(type(receipt.get("actor_id")) is int and receipt["actor_id"] > 0, "invalid receipt actor ID")
        actor_id = receipt["actor_id"] if actor_id is None else actor_id
        base.require(receipt["actor_id"] == actor_id, "multiple actor IDs in one owned command journal")
        base.require(receipt.get("reason") in REASONS and receipt.get("status") in ("ACKNOWLEDGED", "FAILED")
                     and type(receipt.get("server_accepted")) is bool, "unknown receipt reason/status")
        full_control(receipt["requested_control"])
        before = receipt.get("before_frame")
        if before is not None:
            base.integer(before)
            check("command_frame_consistency", previous_before is None or before >= previous_before, sequence, "command frames decreased")
            previous_before = before
        if receipt["status"] == "ACKNOWLEDGED":
            base.require(before is not None and type(receipt.get("after_ack_frame")) is int
                         and receipt.get("server_accepted") is True and type(receipt.get("response_actor_id")) is int
                         and receipt.get("response_actor_id") == actor_id
                         and "error" not in receipt, "acknowledgement receipt is contradictory")
            check("command_frame_consistency", receipt["after_ack_frame"] == before, sequence, "no-tick batch changed world frame")
            if "expected_before_frame" in receipt:
                base.integer(receipt["expected_before_frame"])
                check("command_frame_consistency", receipt["expected_before_frame"] == before, sequence, "source row frame changed before command")
        else:
            base.require(isinstance(receipt.get("error"), str) and receipt["error"], "failed receipt needs an error witness")
            if receipt["server_accepted"]:
                base.require(type(receipt.get("response_actor_id")) is int and receipt["response_actor_id"] == actor_id,
                             "server-accepted failed receipt lacks a matching actor response")
            check("all_commands_acknowledged", False, sequence, "command receipt reports failure; no fallback pass")
        phase_counts[receipt["reason"]] += 1
        by_sequence[sequence] = receipt
    check("nonempty_observation", bool(states) and bool(bootstrap) and bool(receipts), None, "no complete observed command sequence")
    base.require(not receipts or receipts[0]["reason"] == "pre_capture_bootstrap", "first receipt is not the bootstrap")
    observations = ([] if not bootstrap else [(bootstrap, bootstrap["alignment"], True)]) + [
        (row, row["control_transport"], False) for row in states]
    state_by_frame = {row["frame"]: row for row in states}
    base.require(len(state_by_frame) == len(states), "duplicate state frames")
    control_mismatches, maximum_conversion_error, prior, alignment_failure_count = [], 0.0, None, 0
    for row, alignment, is_bootstrap in observations:
        frame = base.integer(row["frame"])
        timestamp = base.number(row["timestamp"])
        if prior is not None:
            check("twenty_hz_frame_chain", frame == prior[0] + 1 and abs(timestamp - prior[1] - .05) <= 1e-6,
                  frame, "bootstrap/recorded state frame or 20 Hz timestamp gap")
        prior = (frame, timestamp)
        reconstructed = recompute_snapshot_state(row, wheelbase)
        errors = {key: abs(base.number(row[key]) - value) for key, value in reconstructed.items()}
        maximum_conversion_error = max(maximum_conversion_error, max(errors.values()))
        check("all_snapshot_conversions_match", max(errors.values()) <= 1e-9, frame, "raw snapshot does not reproduce legacy state")
        sequence = base.integer(alignment["expected_receipt_sequence"])
        for field in ("control_read_frame_before", "control_read_frame_after", "observed_frame"):
            base.integer(alignment[field])
        base.require(sequence in by_sequence, "observation references a missing receipt")
        receipt = by_sequence[sequence]
        prior_commands = [item for item in receipts if item.get("before_frame") is not None and item["before_frame"] < frame]
        latest = prior_commands[-1] if prior_commands else None
        bound = (latest is receipt and receipt["status"] == "ACKNOWLEDGED" and receipt.get("after_ack_frame") == frame - 1
                 and alignment.get("control_read_frame_before") == alignment.get("control_read_frame_after") == frame)
        check("control_frame_binding", bound, frame, "observation is not bound to the immediately preceding accepted command")
        mismatches = mismatched_controls(row["current_control"], receipt["requested_control"])
        check("observed_controls_match_prior_ack", not mismatches, frame, "current control differs from the prior acknowledged request")
        if mismatches:
            control_mismatches.append({"frame": frame, "receipt_sequence": sequence, "fields": mismatches,
                                      "observed": row["current_control"], "requested": receipt["requested_control"]})
        expected_status = "PASS" if bound and not mismatches else "FAIL"
        alignment_failure_count += int(expected_status == "FAIL")
        check("worker_alignment_claims_match", alignment.get("status") == expected_status
              and alignment.get("frame_binding_pass") is bound and alignment.get("mismatched_fields") == mismatches
              and alignment.get("observed_frame") == frame and alignment.get("absolute_tolerance") == 1e-6
              and alignment.get("expected_command_before_frame") == receipt.get("before_frame")
              and alignment.get("automatic_gear_equality_required") is False
              and alignment.get("physical_actuation_proven") is False and alignment.get("lookback_relabeling_allowed") is False,
              frame, "worker alignment claim differs from independent reconstruction")
        if not is_bootstrap:
            next_sequence = base.integer(alignment["next_command_receipt_sequence"])
            base.require(next_sequence in by_sequence, "state references a missing next-command receipt")
            next_receipt = by_sequence[next_sequence]
            check("next_command_links", next_receipt.get("before_frame") == frame
                  and not mismatched_controls(row["next_control"], next_receipt["requested_control"], requested_pair=True),
                  frame, "next_control does not match its same-frame receipt")
        elif bootstrap.get("recorded_training_state") is not False:
            check("worker_alignment_claims_match", False, frame, "bootstrap wrongly represented as a training state")
    bootstrap_frame = bootstrap.get("frame") if bootstrap else None
    warmup = [row["frame"] for row in states if row["capture_phase"] == "stationary_warmup"]
    driving = [row["frame"] for row in states if row["capture_phase"] == "driving"]
    observed_frames = set(state_by_frame) | ({bootstrap_frame, bootstrap_frame - 1} if bootstrap_frame is not None else set())
    check("phase_command_ledger", phase_counts["pre_capture_bootstrap"] == 1, None, "bootstrap command count differs from one")
    for reason in ("stationary_warmup_start", "initial_drive_control", "driving_end_brake",
                   "initial_governor_failure_abort", "alignment_failure_abort", "exception_cleanup_abort"):
        check("phase_command_ledger", phase_counts[reason] <= 1, reason, "phase boundary or abort command repeated")
    if warmup:
        check("phase_command_ledger", phase_counts["stationary_warmup_start"] == 1, None, "warmup start command missing")
    if driving:
        check("phase_command_ledger", phase_counts["initial_drive_control"] == 1, None, "initial drive command missing")
    for phase in base.PHASES:
        expected_count = sum(row["capture_phase"] == phase and row["control_transport"]["status"] == "PASS" for row in states)
        check("phase_command_ledger", phase_counts["next_" + phase + "_control"] == expected_count,
              phase, "per-state next-command receipt count differs from recorded non-aborted states")
    # HH_260906 - Aggregate counts alone cannot detect an omitted row command replaced by a duplicate on another row.
    per_frame_commands = Counter((receipt.get("before_frame"), receipt["reason"]) for receipt in receipts)
    for row in states:
        reason = "next_" + row["capture_phase"] + "_control"
        check("phase_command_ledger", per_frame_commands[(row["frame"], reason)] == int(row["control_transport"]["status"] == "PASS"),
              row["frame"], "each non-aborted state requires exactly one own-frame next-phase command")
    for receipt in receipts:
        reason, before, requested = receipt["reason"], receipt.get("before_frame"), receipt["requested_control"]
        if receipt["status"] == "ACKNOWLEDGED":
            check("command_frame_consistency", before in observed_frames, receipt["sequence"], "command frame is outside the complete observed tick chain")
        if reason.startswith("next_"):
            phase = reason[len("next_"):-len("_control")]
            valid = before in state_by_frame and state_by_frame[before]["capture_phase"] == phase
        elif reason == "pre_capture_bootstrap":
            valid = bootstrap_frame is None or before == bootstrap_frame - 1
        elif reason == "stationary_warmup_start":
            valid = before == bootstrap_frame
        elif reason == "initial_drive_control":
            valid = before == (warmup[-1] if warmup else bootstrap_frame)
        elif reason == "driving_end_brake":
            valid = bool(driving) and before == driving[-1]
        elif reason == "alignment_failure_abort":
            valid = before in state_by_frame and state_by_frame[before]["control_transport"]["status"] == "FAIL"
        else:
            valid = receipt["sequence"] == len(receipts) or reason == "initial_governor_failure_abort"
        check("phase_command_ledger", valid, receipt["sequence"], "command reason is inconsistent with the phase/frame ledger")
        if reason in ("pre_capture_bootstrap", "stationary_warmup_start", "driving_end_brake",
                      "initial_governor_failure_abort", "alignment_failure_abort", "exception_cleanup_abort"):
            check("phase_command_ledger", requested["throttle"] == requested["steer"] == 0.0 and requested["brake"] == 1.0,
                  receipt["sequence"], "bootstrap/tail/abort is not the declared full-brake command")
    return {"status": "PASS" if all(flags.values()) else "FAIL", "flags": flags,
        "failed_flags": [name for name, passed in flags.items() if not passed], "failures": failures,
        "receipt_count": len(receipts), "acknowledged_count": sum(r["status"] == "ACKNOWLEDGED" for r in receipts),
        "failed_receipt_count": sum(r["status"] == "FAILED" for r in receipts), "command_reason_counts": dict(phase_counts),
        "recorded_state_count": len(states), "bootstrap_observation_count": int(bool(bootstrap)),
        "alignment_failure_count": alignment_failure_count,
        "control_mismatch_count": len(control_mismatches), "control_mismatches": control_mismatches,
        "maximum_legacy_state_reconstruction_absolute_error": maximum_conversion_error,
        "server_binary_identity_proven": False, "physical_actuation_proven": False,
        "notice": "PASS means coherent recorded command acceptance and frame/state evidence only; it does not prove physical actuator timing, driving quality, visual quality or dataset admission."}


def verify_owner_sources(root, ledger):
    """HH_260906 - Verify the new explicit revision without overriding the original async auditor's frozen pins."""
    read = lambda name: pilot.read_json(root, name, ledger)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    base.require(plan.get("source_bytes_archived") is True and plan.get("bounds_source_bytes_archived") is True
                 and set(plan["source_sha256"]) == pilot.SOURCE_NAMES, "ack trial requires exactly ten archived sources")
    base.require(owner.get("source_bytes_unchanged_and_archived") is True and set(owner["source_checks"]) == pilot.SOURCE_NAMES
                 and all(value is True for value in owner["source_checks"].values()), "owned source postcheck failed")
    base.require(plan.get("host") == "127.0.0.1" and plan.get("map") == started.get("map") == "Town07"
                 and plan.get("capture_mode") == "expert" and plan.get("quality") == started.get("quality") == "Low"
                 and plan.get("route_sha256") == pilot.ROUTE_SHA, "ack trial is outside the reviewed Low Town07 expert protocol")
    base.require(owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False,
                 "owner scope must remain expert-only")
    argv = plan["collector_argv"]
    base.require(argv.count("--control-transport") == 1 and argv[argv.index("--control-transport") + 1] == "acknowledged_batch",
                 "acknowledged transport was not explicitly selected")
    base.require({path.relative_to(root / "provenance").as_posix() for path in (root / "provenance").rglob("*")
                  if path.is_file() or path.is_symlink()} == pilot.SOURCE_NAMES, "unexpected or missing source archive path")
    for name, expected in plan["source_sha256"].items():
        raw = pilot.checked_bytes(root, "provenance/" + name, ledger)
        base.require(hashlib.sha256(raw).hexdigest() == expected, "source archive SHA mismatch")
        base.require(raw == pilot.recorded_commit_bytes(plan["source_head_commit"], name), "source archive differs from recorded commit")
        if name in REVIEWED_ACK_SOURCES:
            base.require(expected == REVIEWED_ACK_SOURCES[name], "unreviewed acknowledged protocol source revision")
    pid = base.integer(started["server_pid"])
    base.require(pid > 1 and started["server_pgid"] == pid and started["port"] == plan["port"], "owned startup identity mismatch")
    log = pilot.checked_bytes(root, "server.log", ledger)
    previous = None
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        base.require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
                     and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid
                     and evidence.get("generation_id") == f"expert_{pid}" and evidence.get("host") == plan["host"]
                     and evidence.get("port") == plan["port"] and evidence.get("expected_map") == "Town07", "owned lifecycle mismatch")
        base.require(evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "lifecycle mode mismatch")
        base.require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                     else evidence.get("active_map_basename") == "Town07", "running map or stopped proof mismatch")
        prefix = evidence["server_log"]
        base.require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
                     and hashlib.sha256(log[:prefix["size_bytes"]]).hexdigest() == prefix["sha256"], "lifecycle log prefix SHA mismatch")
        current = datetime.fromisoformat(evidence["checked_at"])
        base.require(current.tzinfo is not None and (previous is None or current > previous), "lifecycle timestamps are out of order")
        previous = current
    return plan, owner


def analyze_camera_alignment(states, cameras):
    """HH_260906 - Check every six-camera bundle against the exact recorded physics frame, not wall-clock FPS."""
    expected = states[::2]
    failures = []
    if [row["frame"] for row in cameras] != [row["frame"] for row in expected]:
        failures.append("camera frame sequence is not the complete every-second-state sequence")
    by_frame = {row["frame"]: row for row in states}
    for camera in cameras:
        frame = base.integer(camera["frame"])
        if frame not in by_frame:
            failures.append(f"camera frame {frame} has no recorded actor state")
            continue
        row = by_frame[frame]
        timestamps = camera.get("source_timestamps", {})
        valid = (tuple(camera.get("camera_order", ())) == base.CAMERAS
                 and set(camera.get("images", {})) == set(base.CAMERAS) and set(timestamps) == set(base.CAMERAS)
                 and camera.get("capture_phase") == row["capture_phase"]
                 and abs(base.number(camera["timestamp"]) - base.number(row["timestamp"])) <= 1e-6)
        valid &= all(abs(base.number(value) - row["timestamp"]) <= 1e-6 for value in timestamps.values())
        valid &= all(camera.get("images", {}).get(name) == f"images/{name}/{frame:08d}.jpg" for name in base.CAMERAS)
        if not valid:
            failures.append(f"camera frame {frame} bundle identity/timestamp/phase mismatch")
    return {"status": "PASS" if states and not failures else "FAIL", "state_count": len(states),
        "camera_anchor_count": len(cameras), "expected_anchor_count": len(expected), "failures": failures,
        "source_timestamp_tolerance_seconds": 1e-6, "camera_pixels_examined": False}


def comparison_configuration(manifest):
    """HH_260906 - Compare declared physical/camera/control settings without exposing local source paths."""
    contract = manifest["capture_contract"]
    return {"capture_world_settings": manifest["runtime"]["capture_world_settings"],
        "camera_rig_canonical_sha256": hashlib.sha256(json.dumps(manifest["cameras"], sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
        "physics_hz": contract["physics_hz"], "camera_hz": contract["camera_hz"],
        "target_speed_kmh": contract["target_speed_kmh"], "spawn_z_offset_m": contract["spawn_z_offset_m"],
        "basic_agent_control": contract["basic_agent_control"], "wheelbase_m": manifest["coordinate_contract"]["wheelbase_m"]}


def validate_campaign_plan(root, async_root):
    """HH_260906 - Bind this single trial to the exact bounded prospective revision, never infer campaign completion."""
    root, async_root = Path(root), Path(async_root)
    campaign, ledger = root.parent.parent, {}
    base.require(root.parent.name == "town07_straight_calibration" and root.name in ("run_001", "run_002")
                 and not root.is_symlink() and not root.parent.is_symlink(), "trial is outside the frozen campaign layout")
    raw = pilot.checked_bytes(campaign, "pilot_plan.json", ledger)
    def unique_object(pairs):
        value = {}
        for key, item in pairs:
            base.require(key not in value, "duplicate prospective plan JSON key")
            value[key] = item
        return value
    plan = json.loads(raw, object_pairs_hook=unique_object)
    base.require(isinstance(plan, dict) and set(plan) == set(FROZEN_CAMPAIGN) | {"comment", "notice", "declared_at_utc"},
                 "unknown or missing prospective plan field")
    for key, expected in FROZEN_CAMPAIGN.items():
        actual = plan[key]
        numeric = type(expected) in (float, int)
        base.require((type(actual) in (float, int) if numeric else type(actual) is type(expected)) and actual == expected,
                     "prospective plan differs from the reviewed revision: " + key)
        if type(expected) is int:
            base.require(type(actual) is int, "prospective integer field type mismatch: " + key)
    base.require(all(isinstance(plan[key], str) and plan[key] for key in ("comment", "notice")), "plan explanation missing")
    def utc(value):
        base.require(isinstance(value, str), "plan or owner timestamp must be a string")
        try:
            result = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError as error:
            raise base.EvidenceError("invalid plan or owner timestamp") from error
        base.require(result.tzinfo is not None and result.utcoffset().total_seconds() == 0, "plan or owner timestamp must be UTC")
        return result
    declared, deadline = utc(plan["declared_at_utc"]), utc(plan["finish_before_utc"])
    base.require(declared < deadline, "prospective declaration is after its deadline")
    for name, expected in zip(("run_001", "run_002"), plan["baseline_outputs"]):
        base.require((campaign / expected).resolve() == (async_root / name).resolve(), "async comparison root differs from prospective plan")
    attempts = sorted(path for path in root.parent.iterdir() if path.name.startswith("run_"))
    names = [path.name for path in attempts]
    base.require(names in (["run_001"], ["run_001", "run_002"]) and root.name in names,
                 "missing, noncontiguous, or excess attempted runs in prospective campaign")
    rows, planned_times, completed_times = [], {}, {}
    for attempt in attempts:
        base.require(attempt.is_dir() and not attempt.is_symlink(), "attempt directory must not be a symlink")
        relative = attempt.relative_to(campaign).as_posix()
        owner_plan = pilot.read_json(campaign, relative + "/owner_plan.json", ledger)
        base.require(owner_plan.get("source_head_commit") == plan["source_commit"]
                     and owner_plan.get("route_sha256") == plan["route_sha256"]
                     and owner_plan.get("quality") == plan["quality"] and owner_plan.get("map") == plan["map"]
                     and owner_plan.get("collector_wall_timeout_sec") == plan["wall_timeout_seconds"]
                     and owner_plan.get("finish_before_utc") == plan["finish_before_utc"], "attempt owner differs from prospective plan")
        base.require(all(owner_plan["source_sha256"].get(name) == digest for name, digest in REVIEWED_ACK_SOURCES.items()),
                     "attempt owner uses a different reviewed protocol source")
        planned = utc(owner_plan["planned_at_utc"])
        planned_times[attempt.name] = planned
        base.require(declared <= planned < deadline, "trial was planned before prospective declaration or after deadline")
        argv = owner_plan["collector_argv"]
        for option, expected in {"--control-transport": "acknowledged_batch", "--physics-hz": 20., "--capture-hz": 10.,
                "--target-speed-kmh": 28.8, "--max-duration-sec": 180., "--weather": "ClearNoon", "--seed": 0.,
                "--goal-stop-profile": "comfortable_v3"}.items():
            base.require(argv.count(option) == 1 and argv.index(option) + 1 < len(argv), "missing or repeated frozen CLI option")
            actual = argv[argv.index(option) + 1]
            base.require(float(actual) == expected if isinstance(expected, float) else actual == expected,
                         "attempt CLI changed a prospective condition: " + option)
        finalized = (attempt / "owner_result.json").exists()
        row = {"trial_id": attempt.name, "finalized_at_audit": finalized, "analyzed_by_this_entrypoint": attempt == root}
        if finalized:
            owner = pilot.read_json(campaign, relative + "/owner_result.json", ledger)
            ready = pilot.read_json(campaign, relative + "/lifecycle/ready.json", ledger)
            stopped = pilot.read_json(campaign, relative + "/lifecycle/stopped.json", ledger)
            base.require(planned < utc(ready["checked_at"]) < utc(stopped["checked_at"])
                         <= utc(owner["completed_at_utc"]) <= deadline, "attempt lifecycle exceeds or contradicts prospective time bounds")
            completed_times[attempt.name] = utc(owner["completed_at_utc"])
            row["owner_exit_code"] = owner["exit_code"]
        base.require(attempt != root or finalized, "INCOMPLETE: selected prospective attempt is not finalized")
        rows.append(row)
    base.require(root.name != "run_002" or rows[0]["finalized_at_audit"], "second attempt began before first attempt finalized")
    review = None
    if len(rows) == 2:
        # HH_260906 - A bounded second attempt requires a dated review bound to the complete failed first-trial bytes.
        base.require(rows[0]["finalized_at_audit"], "second attempt discovered before first attempt finalized")
        review = pilot.read_json(campaign, "run_002_preflight_review.json", ledger)
        expected_review = {"previous_output": "town07_straight_calibration/run_001", "owner_exit_code": 1,
            "source_checks_passed": 10, "lifecycle_ready_after_stopped": "PASS", "independent_transport_status": "PASS",
            "control_mismatch_count": 0, "native_scalar_quality": "FAIL", "collision_count": 0, "lane_invasion_count": 0,
            "next_output": "town07_straight_calibration/run_002", "source_and_parameters_changed": False,
            "maximum_attempts_per_revision": 2, "automatic_retry": False, "training_data_approved": False}
        other_fields = {"comment", "reason", "reviewed_at_utc", "previous_states_sha256", "previous_receipts_sha256",
                        "acknowledged_receipts", "minimum_native_speed_rate_mps2", "minimum_camera_speed_rate_mps2", "goal_distance_m"}
        base.require(isinstance(review, dict) and set(review) == set(expected_review) | other_fields,
                     "unknown or missing second-attempt review field")
        base.require(all(type(review[key]) is type(expected) and review[key] == expected for key, expected in expected_review.items()),
                     "second-attempt review changes frozen scope or first-trial outcome")
        base.require(all(isinstance(review[key], str) and review[key] for key in ("comment", "reason")), "empty review explanation")
        base.require(completed_times["run_001"] <= utc(review["reviewed_at_utc"]) < planned_times["run_002"],
                     "second-attempt review is not between first completion and second planning")
        first = root.parent / "run_001"
        directories = [name for name in ("episode", "episode.partial") if (first / name).exists()]
        base.require(len(directories) == 1, "reviewed first trial must retain exactly one episode directory")
        prefix = "town07_straight_calibration/run_001/" + directories[0] + "/"
        states = pilot.read_json(campaign, prefix + "states.jsonl", ledger, lines=True)
        receipts = pilot.read_json(campaign, prefix + "control_receipts.jsonl", ledger, lines=True)
        manifest = pilot.read_json(campaign, prefix + "manifest.json", ledger)
        base.require(review["previous_states_sha256"] == ledger[prefix + "states.jsonl"]["sha256"]
                     and review["previous_receipts_sha256"] == ledger[prefix + "control_receipts.jsonl"]["sha256"],
                     "second-attempt review references different first-trial bytes")
        measured = analyze_transport(states, receipts, manifest["capture_contract"]["control_transport"]["bootstrap_observation"])
        first_summary, _ = base.summarize_trial(first, None)
        qa = first_summary["independent_qa"]
        native = qa["speed_rate_qa"]["native_20hz"]["by_phase"]["all"]
        camera = qa["speed_rate_qa"]["camera_10hz"]["by_phase"]["all"]
        base.require(measured["status"] == review["independent_transport_status"]
                     and type(review["acknowledged_receipts"]) is int and measured["acknowledged_count"] == review["acknowledged_receipts"]
                     and measured["control_mismatch_count"] == 0 and qa["raw_scalar_quality_clear"] is False
                     and first_summary["owner_exit_code"] == 1 and qa["event_counts"] == {"collision": 0, "lane_invasion": 0},
                     "recorded review outcome contradicts independent first-trial evidence")
        for key, expected in (("minimum_native_speed_rate_mps2", native["minimum_mps2"]),
                              ("minimum_camera_speed_rate_mps2", camera["minimum_mps2"]),
                              ("goal_distance_m", qa["final_driving"]["goal_error_m"])):
            base.require(abs(base.number(review[key]) - expected) <= 1e-9, "review metric differs from complete first-trial evidence")
        for item in first_summary["source_manifest"]:
            pilot.checked_bytes(campaign, "town07_straight_calibration/run_001/" + item["path"], ledger)
    return {"status": "DECLARED_PLAN_AND_RECORDED_EXECUTION_BINDING_VERIFIED", "plan_sha256": ledger["pilot_plan.json"]["sha256"],
        "plan": plan, "discovered_attempts": rows, "discovered_attempt_count": len(rows), "maximum_attempt_count": 2,
        "campaign_completion_claimed": False, "historical_file_creation_time_proven": False,
        "prior_attempt_human_review_separately_verified": False,
        "prior_attempt_review_record_verified": review is not None, "prior_attempt_review_record": review,
        "notice": "This checks recorded declarations, source pins, conditions and timestamps; it cannot prove historical filesystem creation time or replace the required review before a second attempt.",
        "source_manifest": [{"path": name, **item} for name, item in sorted(ledger.items())]}


def audit_trial(root):
    root, ledger = Path(root), {}
    base.require((root / "owner_result.json").is_file(), "INCOMPLETE: active trial has no finalized owner result")
    plan, owner = verify_owner_sources(root, ledger)
    result, timeline = base.summarize_trial(root, None)
    for item in result["source_manifest"]:
        pilot.checked_bytes(root, item["path"], ledger)
    directories = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    base.require(len(directories) == 1, "ack audit requires one finalized or partial episode")
    directory = directories[0]
    manifest = pilot.read_json(root, directory + "/manifest.json", ledger)
    config, transport = manifest["capture_contract"]["goal_stop_profile"], manifest["capture_contract"]["control_transport"]
    base.require(all(config.get(key) == expected and (not isinstance(expected, bool) or type(config.get(key)) is bool)
                     for key, expected in pilot.FROZEN.items()), "ack trial changed the frozen comfortable_v3 profile")
    base.require(manifest["capture_contract"]["target_speed_kmh"] == 28.8
                 and manifest["result"].get("training_data_approved") is False
                 and manifest["coordinate_contract"]["wheelbase_m"] == 2.85, "nominal target, geometry or development boundary changed")
    expected_transport = {"schema": "carla.acknowledged_control_transport.v1", "mode": "acknowledged_batch",
        "single_actor_single_response": True, "implicit_tick": False, "async_fallback_allowed": False,
        "receipt_journal": "control_receipts.jsonl", "receipt_persisted_before_next_tick": True,
        "comparison_absolute_tolerance": 1e-6, "automatic_gear_equality_required": False,
        "label_rewrite_or_lookback_pass_allowed": False, "physical_actuation_proven": False,
        "native_acceleration_definition_changed": False,
        "kinematics_source": "tick-exact immutable WorldSnapshot.find(owned_actor_id)"}
    base.require(all(transport.get(key) == expected and (not isinstance(expected, bool) or type(transport.get(key)) is bool)
                     for key, expected in expected_transport.items()), "recorded acknowledgement protocol differs from reviewed contract")
    states = pilot.read_json(root, directory + "/states.jsonl", ledger, lines=True)
    cameras = pilot.read_json(root, directory + "/camera_frames.jsonl", ledger, lines=True)
    receipts_path = directory + "/control_receipts.jsonl"
    receipts = pilot.read_json(root, receipts_path, ledger, lines=True)
    measured = analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    camera_alignment = analyze_camera_alignment(states, cameras)
    measured["camera_alignment"] = camera_alignment
    measured["flags"]["complete_camera_frame_alignment"] = camera_alignment["status"] == "PASS"
    if camera_alignment["status"] != "PASS":
        measured["failed_flags"].append("complete_camera_frame_alignment")
        measured["status"] = "FAIL"
    declared = manifest["result"].get("control_transport", {})
    early_failure = not declared and not receipts and not states and manifest["status"] == "failed" and owner.get("exit_code") != 0
    if not early_failure:
        base.require(declared.get("receipt_journal_sha256") == ledger[receipts_path]["sha256"], "receipt journal SHA differs from manifest")
        base.require(declared.get("receipt_journal_error") is None and declared.get("physical_actuation_proven") is False,
                     "receipt journal was not finalized or claims physical actuation proof")
        for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
                           ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
            base.require(type(declared.get(field)) is int and declared[field] == measured[key], "worker receipt/alignment counts contradict independent evidence")
    protocol = pilot.analyze_protocol(states, timeline["native_states"]) if timeline else None
    image_hashes, seen = [], set()
    for camera in cameras:
        for name in base.CAMERAS:
            relative = directory + "/" + camera["images"][name]
            base.require(relative not in seen, "camera references reuse an image")
            seen.add(relative)
            raw = pilot.checked_bytes(root, relative, {})
            image_hashes.append({"path": relative, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    image_hashes.sort(key=lambda item: item["path"])
    image_corpus = json.dumps(image_hashes, sort_keys=True, separators=(",", ":")).encode()
    for name, item in ledger.items():
        base.require(base.sha(root / name) == item["sha256"], "finalized input changed during audit")
    for item in image_hashes:
        base.require(base.sha(root / item["path"]) == item["sha256"], "camera image changed during audit")
    base.recheck_bounds_source_archive(root, result["bounds_source_proof"])
    result.update({"status": "TRANSPORT_" + measured["status"] + "_NOT_ADMITTED", "transport_protocol": measured,
        "pilot_protocol": protocol, "training_data_approved": False, "dataset_admission": False,
        "source_head_commit": plan["source_head_commit"], "reviewed_protocol_source_sha256": REVIEWED_ACK_SOURCES,
        "archived_sources_match_recorded_commit": True,
        "comparison_configuration": comparison_configuration(manifest) if "runtime" in manifest else None,
        "failed_before_first_command": early_failure,
        "image_byte_integrity": {"file_count": len(image_hashes), "total_size_bytes": sum(item["size_bytes"] for item in image_hashes),
            "canonical_sorted_file_ledger_sha256": hashlib.sha256(image_corpus).hexdigest(), "pixels_decoded_or_visually_approved": False},
        "source_manifest": [{"path": name, **item} for name, item in sorted(ledger.items())],
        "interpretation": "Combined acknowledged-command plus immutable-measurement protocol comparison, not an isolated physical latency fix. Scalar/profile/visual checks and dataset admission remain separate."})
    return result, image_hashes


def audit_async_reference(root):
    """HH_260906 - Include both original async attempts, never select only the apparently successful one."""
    root = Path(root)
    trials = sorted(path for path in root.iterdir() if path.name.startswith("run_"))
    base.require([path.name for path in trials] == ["run_001", "run_002"], "async reference must include exactly both original attempts")
    results = []
    for path in trials:
        base.require((path / "owner_result.json").is_file(), "INCOMPLETE: async reference attempt is not finalized")
        result = pilot.audit_trial(path)
        directory = "episode" if (path / "episode").is_dir() else "episode.partial"
        manifest = pilot.read_json(path, directory + "/manifest.json", {})
        results.append({"trial_id": result["trial_id"], "status": result["status"],
            "owner_exit_code": result["owner_exit_code"], "raw_quality_candidate": result["raw_quality_candidate"],
            "control_observation_alignment": result["pilot_protocol"]["control_observation_alignment"],
            "goal_reached_independently": result["independent_qa"]["goal_reached_independently"],
            "maximum_measured_speed_kmh": result["independent_qa"]["maximum_measured_speed_kmh"],
            "comparison_configuration": comparison_configuration(manifest),
            "source_head_commit": result["source_head_commit"], "source_manifest": result["source_manifest"]})
    return {"attempt_count": 2, "trials": results, "selection_of_successful_subset": False,
        "interpretation": "All original async attempts are retained. The new arm changes command acknowledgement and observation coherence together, so this is not an isolated actuator-latency experiment."}


def publish_trial(root, output, async_root):
    """HH_260906 - Write a new single-trial audit with the complete old reference, without replacing prior evidence."""
    root, output, async_root = Path(root), Path(output), Path(async_root)
    base.require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(root.resolve())
                 and not output.resolve().is_relative_to(async_root.resolve()),
                 "new audit output must be outside the raw trial and must not already exist")
    prospective = validate_campaign_plan(root, async_root)
    result, images = audit_trial(root)
    reference = audit_async_reference(async_root)
    configuration = result["comparison_configuration"]
    same_configurations = configuration is not None and all(row["comparison_configuration"] == configuration for row in reference["trials"])
    report = {"schema": "portable_e2e.acknowledged_control_independent_audit.v1", "status": "REVIEWED_NOT_ADMITTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "trial": result, "async_reference": reference,
        "prospective_plan_proof": prospective,
        "declared_physical_camera_and_basic_agent_configuration_matches_both_async_attempts": same_configurations,
        "auditor_source_sha256": base.sha(Path(__file__)), "base_reader_source_sha256": base.sha(Path(base.__file__)),
        "original_v3_protocol_auditor_sha256": base.sha(Path(pilot.__file__)),
        "scope": {"single_ack_trial_report_not_campaign_completion": True, "both_async_attempts_included": True,
            "prospective_campaign_plan_verified_by_this_entrypoint": True, "all_image_bytes_hashed": True,
            "camera_pixels_visually_approved": False, "dataset_admission": False, "automatic_promotion": False,
            "model_loaded": False, "live_simulator_access": False, "test_payload_used": False,
            "original_inputs_modified": False, "physical_actuation_proven": False}}
    # HH_260906 - Later reference analysis must not leave a time-of-check gap for earlier source or image evidence.
    for raw_root, entries in [(root, result["source_manifest"]), (root, images), (root.parent.parent, prospective["source_manifest"]),
                             *((async_root / row["trial_id"], row["source_manifest"]) for row in reference["trials"])]:
        for entry in entries:
            raw = pilot.checked_bytes(raw_root, entry["path"], {})
            base.require(hashlib.sha256(raw).hexdigest() == entry["sha256"], "input changed before publication")
    base.require([path.name for path in sorted(root.parent.iterdir()) if path.name.startswith("run_")]
                 == [row["trial_id"] for row in prospective["discovered_attempts"]], "attempt denominator changed before publication")
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("summary.json", report), ("image_hashes.json", images),
                        ("carla_source_evidence.json", primary_source_evidence())):
        (output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text("".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir())))
    return report


def publish_campaign(campaign, output, async_root):
    """HH_260906 - Retain both bounded acknowledged attempts and both original attempts, including every failed capture."""
    campaign, output, async_root = Path(campaign), Path(output), Path(async_root)
    base.require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(campaign.resolve())
                 and not output.resolve().is_relative_to(async_root.resolve()), "new campaign output must be outside both raw input trees")
    roots = [campaign / "town07_straight_calibration" / name for name in ("run_001", "run_002")]
    base.require(all((root / "owner_result.json").is_file() for root in roots), "INCOMPLETE: both bounded attempts must be finalized")
    prospective = validate_campaign_plan(roots[-1], async_root)
    audits = [audit_trial(root) for root in roots]
    reference = audit_async_reference(async_root)
    configurations = [result["comparison_configuration"] for result, _ in audits] + [row["comparison_configuration"] for row in reference["trials"]]
    report = {"schema": "portable_e2e.acknowledged_control_campaign_audit.v1", "status": "COMPLETE_NOT_ADMITTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "acknowledged_attempt_count": 2, "async_attempt_count": 2,
        "acknowledged_trials": [result for result, _ in audits], "async_reference": reference, "prospective_plan_proof": prospective,
        "declared_physical_camera_and_basic_agent_configuration_matches_all_four_attempts": configurations[0] is not None
            and all(value == configurations[0] for value in configurations),
        "auditor_source_sha256": base.sha(Path(__file__)), "base_reader_source_sha256": base.sha(Path(base.__file__)),
        "original_v3_protocol_auditor_sha256": base.sha(Path(pilot.__file__)),
        "scope": {"all_four_attempts_included": True, "maximum_two_acknowledged_attempts_exhausted": True,
            "all_acknowledged_image_bytes_hashed": True, "camera_pixels_visually_approved": False, "dataset_admission": False,
            "automatic_promotion": False, "model_loaded": False, "live_simulator_access": False,
            "test_payload_used": False, "original_inputs_modified": False, "physical_actuation_proven": False},
        "interpretation": "Combined acknowledged transport plus coherent measurement comparison, not an isolated physical latency fix. Current controls are API-reported controls; inherited metric keys containing applied do not prove physical pedal, torque or actuation timing. Protocol PASS is separate from raw scalar, independent XY, visual quality and admission."}
    ledgers = [(campaign, prospective["source_manifest"])]
    ledgers += [(root, result["source_manifest"] + images) for root, (result, images) in zip(roots, audits)]
    ledgers += [(async_root / row["trial_id"], row["source_manifest"]) for row in reference["trials"]]
    for raw_root, entries in ledgers:
        for entry in entries:
            raw = pilot.checked_bytes(raw_root, entry["path"], {})
            base.require(hashlib.sha256(raw).hexdigest() == entry["sha256"], "input changed before campaign publication")
    base.require(sorted(path.name for path in roots[0].parent.iterdir() if path.name.startswith("run_")) == ["run_001", "run_002"],
                 "attempt denominator changed before campaign publication")
    output.mkdir(parents=True, exist_ok=False)
    documents = {"summary.json": report, "carla_source_evidence.json": primary_source_evidence()}
    documents.update({root.name + "_image_hashes.json": images for root, (_, images) in zip(roots, audits)})
    for name, value in documents.items():
        (output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text("".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir())))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("trial_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--campaign", action="store_true", help="Audit exactly both finalized attempts beneath a campaign root.")
    parser.add_argument("--async-root", type=Path, default=ROOT / "artifacts/training/2026-09-08/comfortable_goal_stop_v3/town07_straight_calibration")
    args = parser.parse_args(argv)
    try:
        report = (publish_campaign if args.campaign else publish_trial)(args.trial_root, args.output_dir, args.async_root)
    except (base.EvidenceError, OSError, ValueError, KeyError, TypeError) as error:
        print(f"ACKNOWLEDGED_CONTROL_AUDIT_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "dataset_admission": False,
        "transport_status": [row["transport_protocol"]["status"] for row in report["acknowledged_trials"]]
            if args.campaign else report["trial"]["transport_protocol"]["status"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
