#!/usr/bin/env python3
"""HH_261001 - Independently reconstruct one frozen PI/coast/hold trial; read raw evidence without simulator contact."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_town12_brake_calibration_20261001 as helpers
from scripts.e2e import audit_town12_j15824_capture_20261001 as cleanup
from scripts.e2e import collect_carla_vad_expert as collector
from scripts.e2e import run_owned_town12_j15824_pi_coast_hold_20261001 as frozen_owner


WORKER_SHA256 = "3e3ce381161e91d92ac2f6bd8ddb15230a351374e2bc1e6dd3f4142c68f9e1f3"
CLOSURE_SHA256 = "ac653b0aeb48d5d67ae17f0e0fb7e3b61e0b7a5c4423eae7b2310e67475425b6"
PI, COAST, HOLD = "si_pi_launch_and_stabilize", "zero_pedal_coast", "measured_low_speed_hold"
NORMAL = {PI, COAST, HOLD}
CLEANUP = "termination_service_brake_nontraining"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def same(first, second, field=None):
    """HH_261001 - Float32 actuator tolerance does not alter any speed threshold or native decision."""
    if isinstance(first, dict) and isinstance(second, dict):
        return first.keys() == second.keys() and all(same(first[key], second[key], key) for key in first)
    if isinstance(first, list) and isinstance(second, list):
        return len(first) == len(second) and all(same(a, b, field) for a, b in zip(first, second))
    if isinstance(first, (int, float)) and not isinstance(first, bool) and isinstance(second, (int, float)) and not isinstance(second, bool):
        tolerance = 1e-6 if field in {"throttle", "brake", "steer"} else 1e-12
        return math.isfinite(first) and math.isfinite(second) and abs(first - second) <= tolerance
    return first == second


class IndependentReference:
    """HH_261001 - Re-implement the declared equations and causal transitions; never invoke the live worker's policy."""
    def __init__(self):
        self.integral, self.throttle = 0., 0.
        self.started = self.previous_time = self.previous_frame = None
        self.entry = self.hold_entry = self.last_pi = None
        self.entry_history, self.low_history = [], []
        self.hold_commands = self.hold_observed = 0

    def decision(self, frame, timestamp, speed, original):
        require(math.isfinite(speed) and 0. <= speed <= 2.55, "invalid actual calibration speed/time")
        first = self.previous_time is None
        if not first:
            require(frame == self.previous_frame + 1 and abs(timestamp - self.previous_time - .05) < 1e-5,
                    "non-native policy frame/time cadence")
        require(original["brake"] == 0., "normal planner brake must be capped before replacement")
        require(self.hold_observed < 40, "policy called after bounded hold completion")
        if first:
            self.started = timestamp
        self.previous_frame, self.previous_time = frame, timestamp
        witness = {"frame": frame, "timestamp": timestamp, "raw_actor_planar_speed_mps": speed}
        if self.entry is None:
            require(timestamp - self.started < 20. - 1e-6, "20-second PI entry stabilization timed out")
            self.entry_history = (self.entry_history + [witness])[-10:] if 1.45 <= speed <= 1.55 else []
            if len(self.entry_history) == 10:
                self.entry = {**witness, "preceding_ten_native_observations": self.entry_history.copy()}
        expected = dict(original)
        if self.entry is None:
            dt = 0. if first else .05
            error = 1.5 - speed
            old_i, old_u = self.integral, self.throttle
            trial_i = min(.20, max(0., old_i + .04 * error * dt))
            lower, upper = max(0., old_u - .05 * dt), min(.20, old_u + .02 * dt)
            trial_u = .12 * error + trial_i
            reject = (error > 0. and trial_u > upper) or (error < 0. and trial_u < lower)
            self.integral = old_i if reject else trial_i
            raw_u = .12 * error + self.integral
            self.throttle = min(upper, max(lower, raw_u))
            self.last_pi = {"dt_seconds": dt, "raw_speed_mps": speed, "target_speed_mps": 1.5,
                "error_mps": error, "proportional_contribution": .12 * error,
                "integral_before": old_i, "integral_candidate": trial_i,
                "candidate_unclipped_throttle": trial_u, "integral_update_rejected": reject,
                "integral_after": self.integral, "previous_throttle": old_u,
                "allowed_throttle_lower": lower, "allowed_throttle_upper": upper,
                "unclipped_throttle_after_anti_windup": raw_u, "requested_throttle": self.throttle}
            expected.update(throttle=self.throttle, brake=0.)
            stage = PI
        else:
            if self.hold_entry is None:
                self.low_history = (self.low_history + [witness])[-2:] if speed <= .05 else []
                elapsed = timestamp - self.entry["timestamp"]
                if len(self.low_history) == 2:
                    require(elapsed <= 10. + 1e-6, "hold entry exceeded 10-second coast budget")
                    self.hold_entry = {**witness, "preceding_two_native_observations": self.low_history.copy()}
                else:
                    require(elapsed < 10. - 1e-6, "10-second zero-pedal coast timed out")
            expected["throttle"] = 0.
            if self.hold_entry is None:
                expected["brake"], stage = 0., COAST
            else:
                require(speed <= .05, "actual speed exceeded fixed hold maximum")
                require(self.hold_commands < 40, "bounded hold command count exceeded")
                self.hold_commands += 1
                expected["brake"], stage = 1., HOLD
        return {"stage": stage, "source_frame": frame, "source_timestamp": timestamp,
            "raw_actor_planar_speed_mps": speed, "normal_planner_before_intervention": original,
            "normal_control_before_original_emergency": expected,
            "pi": dict(self.last_pi) if self.last_pi is not None else None,
            "pi_updated_this_tick": stage == PI, "entry": self.entry, "hold_entry": self.hold_entry,
            "hold_command_number": self.hold_commands, "training_data_admitted": False}

    def observation(self, stage, speed):
        if stage == HOLD:
            require(math.isfinite(speed) and speed <= .05, "observed hold speed exceeds .05 m/s")
            require(self.hold_observed + 1 == self.hold_commands, "duplicate or missing hold observation")
            self.hold_observed += 1


def source_audit(trial, plan):
    with frozen_owner.configured_owner(), frozen_owner.inherited.configured_owner():
        paths = frozen_owner.inherited.inherited.diagnostic.source_paths()
    failures = []
    hashes = plan["source_sha256"]
    closure = hashlib.sha256(json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if len(hashes) != 95 or closure != CLOSURE_SHA256 or set(paths) != set(hashes):
        failures.append("frozen 95-source closure differs")
    for name, expected in hashes.items():
        for label, path in (("current", paths.get(name)), ("archived", trial / "provenance" / name)):
            if path is None or not path.is_file() or path.is_symlink() or helpers.digest(path) != expected:
                failures.append(f"{label}: {name}")
    if hashes.get("tools/e2e/calibrate_town12_j15824_pi_coast_hold_20261001.py") != WORKER_SHA256:
        failures.append("frozen worker hash differs")
    return {"status": "PASS" if not failures else "FAIL", "files": len(hashes),
            "closure_sha256": closure, "failures": failures}


def guard_errors(row):
    failures = []
    if not math.isfinite(row["speed_mps"]) or row["speed_mps"] > 2.55:
        failures.append("speed")
    if not math.isfinite(row.get("front_bumper_stop_distance_m", float("nan"))) or row["front_bumper_stop_distance_m"] < 40.:
        failures.append("signal_distance")
    if not math.isfinite(row.get("route_projection", {}).get("progress_m", float("nan"))) or row["route_projection"]["progress_m"] > 30.:
        failures.append("route_extent")
    if row.get("signal_state") not in ("Green", "Yellow", "Red"):
        failures.append("signal_state")
    points = row.get("body_corners", [])
    if len(points) != 8 or any(p.get("lane") != [588, 0, 1] or not math.isfinite(p.get("lane_margin_m", float("nan")))
                               or p["lane_margin_m"] < .10 for p in points):
        failures.append("instantaneous_lane_inset")
    if row.get("free_disk_bytes", 0) < 5 * 1024 ** 3:
        failures.append("disk_floor")
    return failures


def analyze(directory):
    result = json.loads((directory / "result.json").read_text())
    native = helpers.rows(directory / "native_frames.jsonl")
    receipts = helpers.rows(directory / "control_receipts.jsonl")
    decisions = helpers.rows(directory / "normal_decisions.jsonl")
    abort_path = directory / "policy_aborts.jsonl"
    aborts = helpers.rows(abort_path) if abort_path.is_file() else []
    frame_rows = {row["frame"]: row for row in native}
    before = {row["before_frame"]: row for row in receipts}
    command_rows = {row["frame"]: row for row in decisions}
    failures, operational, emergency_rows, reconstructed_rows, replay_rows = [], [], [], [], []
    if len(frame_rows) != len(native) or len(before) != len(receipts) or len(command_rows) != len(decisions):
        failures.append({"kind": "DUPLICATE_FRAME"})
    if len(receipts) != len(decisions) or len(native) != len(receipts):
        failures.append({"kind": "INCOMPLETE_COMMAND_ACK_OBSERVATION", "native": len(native),
                         "ACKs": len(receipts), "commands": len(decisions)})
    for index, row in enumerate(native):
        frame, command = row["frame"], command_rows.get(row["frame"] - 1)
        ack = before.get(frame - 1)
        if (ack is None or ack.get("sequence") != index + 1 or ack.get("status") != "ACKNOWLEDGED"
                or ack.get("server_accepted") is not True or ack.get("do_tick") is not False
                or ack.get("actor_id") != result["actor_id"] or ack.get("response_actor_id") != result["actor_id"]
                or ack.get("after_ack_frame") != frame - 1 or ack.get("expected_before_frame") != frame - 1
                or not helpers.same_control(ack["requested_control"], row["applied_control"])):
            failures.append({"kind": "ACK_ALIGNMENT", "frame": frame})
        if command is None or not helpers.same_control(command["final_control"], ack["requested_control"]):
            failures.append({"kind": "FINAL_COMMAND_ACK", "frame": frame})
        elif not same({key: value for key, value in command.items() if key not in {"frame", "final_control"}}, row["applied_policy"]):
            failures.append({"kind": "COMMAND_OBSERVATION_POLICY", "frame": frame})
        alignment = row.get("control_alignment", {})
        if (alignment.get("status") != "PASS" or alignment.get("observed_frame") != frame
                or alignment.get("control_read_frame_before") != frame or alignment.get("control_read_frame_after") != frame
                or alignment.get("expected_command_before_frame") != frame - 1
                or alignment.get("expected_receipt_sequence") != index + 1):
            failures.append({"kind": "ACK_RECEIPT_FRAME_BINDING", "frame": frame})
        converted = collector.base_link_state(row["actor_snapshot_transform_carla"], row["world_velocity_carla"],
            row["world_acceleration_carla"], row["world_angular_velocity_carla_deg_s"][2], collector.WHEELBASE_M)
        if any(abs(converted[key] - row[key]) > 1e-8 for key in converted):
            failures.append({"kind": "RAW_BASE_CONVERSION", "frame": frame})
        speed = math.hypot(converted["vx"], converted["vy"])
        actor_speed = math.hypot(*row["world_velocity_carla"][:2])
        if abs(speed - row["speed_mps"]) > 1e-8 or abs(actor_speed - row["raw_actor_planar_speed_mps"]) > 1e-8:
            failures.append({"kind": "RAW_SPEED_CONVERSION", "frame": frame})
        rate = None
        if index:
            prior = native[index - 1]
            dt = row["timestamp"] - prior["timestamp"]
            if frame != prior["frame"] + 1 or abs(dt - .05) > 1e-5:
                failures.append({"kind": "NATIVE_CADENCE", "frame": frame})
            if dt > 0:
                rate = (speed - reconstructed_rows[-1]["speed_mps"]) / dt
                if row.get("speed_rate_mps2") is None or abs(rate - row["speed_rate_mps2"]) > 1e-8:
                    failures.append({"kind": "RAW_RATE_CONVERSION", "frame": frame})
        elif row.get("speed_rate_mps2") is not None:
            failures.append({"kind": "FIRST_RATE_NOT_NULL", "frame": frame})
        stage = row["applied_policy"]["stage"]
        if stage not in NORMAL | {"stationary_settle", CLEANUP}:
            failures.append({"kind": "UNDECLARED_STAGE", "frame": frame})
        if stage != CLEANUP:
            operational.extend({"frame": frame, "guard": error} for error in guard_errors(row))
        if any(row.get("events_seen", [])):
            operational.append({"frame": frame, "guard": "event_recorded"})
        if row["applied_policy"].get("original_agent_overrode_normal_control"):
            emergency_rows.append({"frame": frame, "control": row["applied_control"]})
        reconstructed_rows.append({"frame": frame, "timestamp": row["timestamp"], "stage": stage,
            "speed_mps": speed, "actor_speed_mps": actor_speed, "speed_rate_mps2": rate,
            "control": row["applied_control"], "actor_pose": row["actor_snapshot_transform_carla"]})

    reference = IndependentReference()
    normal_commands = [row for row in decisions if row["stage"] in NORMAL]
    for command in normal_commands:
        frame = command["source_frame"]
        source, observed = frame_rows.get(frame), frame_rows.get(frame + 1)
        try:
            require(source is not None and observed is not None, "command source/response raw observation missing")
            speed = math.hypot(*source["world_velocity_carla"][:2])
            require(command["source_timestamp"] == source["timestamp"], "source timestamp differs")
            expected = reference.decision(frame, source["timestamp"], speed, command["normal_planner_before_intervention"])
            stored = {key: value for key, value in command.items()
                      if key not in {"frame", "final_control", "original_agent_overrode_normal_control"}}
            valid = same(expected, stored)
            if not valid:
                failures.append({"kind": "INDEPENDENT_POLICY_REPLAY", "frame": frame,
                    "differing_keys": [key for key in expected if not same(expected[key], stored.get(key), key)]})
            if not command["original_agent_overrode_normal_control"]:
                require(helpers.same_control(command["final_control"], expected["normal_control_before_original_emergency"]),
                        "normal final command differs from independent policy")
                reference.observation(expected["stage"], math.hypot(*observed["world_velocity_carla"][:2]))
            replay_rows.append({"frame": frame, "status": "PASS" if valid else "FAIL", "stage": expected["stage"],
                "integral": reference.integral, "throttle": expected["normal_control_before_original_emergency"]["throttle"]})
        except Exception as error:
            failures.append({"kind": "POLICY_REPLAY_EXCEPTION", "frame": frame, "error": str(error)})
            break
    abort_checks = []
    for abort in aborts:
        frame = abort["source_frame"]
        try:
            source = frame_rows[frame]
            require(frame not in command_rows or command_rows[frame]["stage"] == CLEANUP,
                    "normal command exists after recorded policy abort")
            require(abort["new_command_sent"] is False and abort["last_ack_observation_frame"] == frame,
                    "policy abort not bound to last native observation")
            try:
                reference.decision(frame, source["timestamp"], math.hypot(*source["world_velocity_carla"][:2]),
                                   abort["original_normal_proposal"])
            except ValueError as error:
                require(str(error) in abort["error"], "independent abort reason differs")
                abort_checks.append({"frame": frame, "status": "PASS", "reason": str(error)})
            else:
                raise ValueError("independent reference did not reject recorded abort")
        except Exception as error:
            failures.append({"kind": "ABORT_REPLAY", "frame": frame, "error": str(error)})
    for name, expected in result.get("raw_files_sha256", {}).items():
        if helpers.digest(directory / name) != expected:
            failures.append({"kind": "RAW_HASH_MISMATCH", "file": name})
    if {path.name for path in directory.glob("*.jsonl")} != set(result.get("raw_files_sha256", {})):
        failures.append({"kind": "RAW_HASH_INVENTORY"})
    if not same(result.get("entry"), reference.entry) or not same(result.get("hold_entry"), reference.hold_entry):
        failures.append({"kind": "RESULT_ENTRY_WITNESS"})
    if result.get("hold_observed_ticks") != reference.hold_observed or not same(result.get("final_pi_integral"), reference.integral):
        failures.append({"kind": "RESULT_POLICY_COUNTERS"})
    if result.get("measurement_completed") != (reference.hold_observed == 40):
        failures.append({"kind": "RESULT_COMPLETION_NOT_PHYSICAL"})

    stages = {}
    for stage in dict.fromkeys(row["stage"] for row in reconstructed_rows):
        values = [row for row in reconstructed_rows if row["stage"] == stage]
        rates = [row["speed_rate_mps2"] for row in values if row["speed_rate_mps2"] is not None]
        origin = frame_rows.get(values[0]["frame"] - 1)
        last_raw = frame_rows[values[-1]["frame"]]
        stages[stage] = {"frames": len(values), "first_frame": values[0]["frame"], "last_frame": values[-1]["frame"],
            "observed_seconds_from_source": values[-1]["timestamp"] - origin["timestamp"] if origin else None,
            "speed_range_mps": [min(row["speed_mps"] for row in values), max(row["speed_mps"] for row in values)],
            "rate_range_mps2": [min(rates), max(rates)] if rates else None,
            "decoder_rate_failures": [row for row in values if row["speed_rate_mps2"] is not None and abs(row["speed_rate_mps2"]) > 2.9],
            "runtime_rate_failures": [row for row in values if row["speed_rate_mps2"] is not None and not -6. <= row["speed_rate_mps2"] <= 3.],
            "first_applied_control": values[0]["control"], "last_applied_control": values[-1]["control"],
            "entry_speed_mps": origin["speed_mps"] if origin else None, "exit_speed_mps": values[-1]["speed_mps"],
            "route_progress_m": last_raw["route_projection"]["progress_m"] - origin["route_projection"]["progress_m"] if origin else None,
            "actor_center_dz_m": values[-1]["actor_pose"]["z"] - origin["actor_snapshot_transform_carla"]["z"] if origin else None}
    normal_failures = [row for stage in NORMAL for row in stages.get(stage, {}).get("decoder_rate_failures", [])]
    normal_runtime = [row for stage in NORMAL for row in stages.get(stage, {}).get("runtime_rate_failures", [])]
    expected_quality_status = "FAIL" if normal_failures or normal_runtime or not normal_commands else "PASS"
    if result.get("native_motion_quality", {}).get("status") != expected_quality_status:
        failures.append({"kind": "RESULT_NATIVE_QUALITY"})
    if stages.get("stationary_settle", {}).get("frames") != 40:
        failures.append({"kind": "SETTLE_TICKS"})
    if result.get("effective_target_speed_kmh") != 5.4:
        failures.append({"kind": "NOMINAL_TARGET"})
    return {"evidence_status": "PASS" if not failures else "FAIL", "evidence_failures": failures,
        "original_trial_status": result["status"], "original_error": result.get("error"),
        "actor_id": result.get("actor_id"), "native_frames": len(native), "control_ACKs": len(receipts),
        "independent_policy_replayed": len(replay_rows), "policy_replay_failures": [row for row in replay_rows if row["status"] != "PASS"],
        "abort_replay": abort_checks, "stage_measurements": stages,
        "entry": reference.entry, "hold_entry": reference.hold_entry, "hold_observed_ticks": reference.hold_observed,
        "final_integral_contribution": reference.integral, "measurement_completed": reference.hold_observed == 40,
        "normal_native_motion_quality": expected_quality_status, "normal_decoder_failed_intervals": len(normal_failures),
        "normal_runtime_failed_intervals": len(normal_runtime), "operational_violations": operational,
        "preserved_emergency_observations": emergency_rows, "actor_cleanup_original": result.get("actor_cleanup"),
        "minimum_free_disk_bytes_observed": min((row.get("free_disk_bytes", 0) for row in native), default=None),
        "minimum_body_lane_margin_m": min((p["lane_margin_m"] for row in native if row["applied_policy"]["stage"] != CLEANUP
                                           for p in row.get("body_corners", []) if "lane_margin_m" in p), default=None),
        "training_data_admitted": False}, replay_rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", required=True, type=Path)
    parser.add_argument("--owner-plan-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    require(helpers.digest(args.trial / "owner_plan.json") == args.owner_plan_sha256, "owner plan changed")
    require(not args.output.exists() and not args.output.is_symlink(), "new report path required")
    plan = json.loads((args.trial / "owner_plan.json").read_text())
    owner = json.loads((args.trial / "owner_result.json").read_text())
    result = json.loads((args.trial / "calibration/calibration_result.json").read_text())
    summary, replay = analyze(args.trial / "calibration/candidate_00")
    sources = source_audit(args.trial, plan)
    try:
        observed_cleanup = cleanup.cleanup_observation(args.trial)
    except Exception as error:
        observed_cleanup = {"status": "UNSUPPORTED_OR_INVALID", "error": f"{type(error).__name__}: {error}"}
    effective = result.get("effective_settings", {})
    settings_ok = (effective.get("synchronous_mode") is True and effective.get("fixed_delta_seconds") == .05
                   and effective.get("substepping") is True and effective.get("max_substep_delta_time") == .01
                   and effective.get("max_substeps") == 10 and result.get("settings_restored") is True)
    report = {"status": "DIAGNOSED_NOT_TRAINING", "original_owner_status": owner["status"],
        "owner_result_sha256": helpers.digest(args.trial / "owner_result.json"),
        "owner_plan_sha256": args.owner_plan_sha256, "audit_source_sha256": helpers.digest(Path(__file__)),
        "original_owner_cleanup_flag": owner.get("owned_processes_stopped_and_ports_released"),
        "original_source_assets_unchanged": owner.get("source_assets_unchanged"),
        "sources": sources, "independent_cleanup": observed_cleanup, "trial": summary,
        "settings_and_restoration": {"status": "PASS" if settings_ok else "FAIL", "effective": effective,
                                     "original_settings_restored": result.get("settings_restored")},
        "training_data_admitted": False, "full_body_swept_qualification": False, "signal_stop_safety_proven": False,
        "notice": "Independent equations and causal stage checks; raw labels are unchanged. Normal and termination motion remain separate. No simulator RPC or launch."}
    args.output.mkdir(parents=True)
    with (args.output / "summary.json").open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    with (args.output / "policy_replay.jsonl").open("x") as stream:
        for row in replay:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({"status": report["status"], "evidence": summary["evidence_status"], "sources": sources["status"],
        "cleanup": observed_cleanup["status"], "normal_motion": summary["normal_native_motion_quality"],
        "stages": {name: value["frames"] for name, value in summary["stage_measurements"].items()},
        "report": str(args.output / "summary.json")}, sort_keys=True))
    return 0 if summary["evidence_status"] == sources["status"] == "PASS" and settings_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
