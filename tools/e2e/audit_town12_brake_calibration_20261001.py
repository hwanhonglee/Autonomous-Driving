#!/usr/bin/env python3
"""HH_261001 - Read-only pulse/ACK forensics, including failed levels; no training or stopping-safety admission."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import calibrate_town12_j15824_brake_pulses_20261001 as frozen


WORKER_SHA256 = "16efac0f46d737a3fb7047a3d2e89906df756b10b07105b268880ac617665933"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def same_control(first, second):
    return all(abs(first[key] - second[key]) <= 1e-6 for key in ("throttle", "brake", "steer")) and all(
        first[key] == second[key] for key in ("hand_brake", "reverse", "manual_gear_shift")) and (
        not first["manual_gear_shift"] or first["gear"] == second["gear"])


def analyze_level(directory):
    result_path = directory / "result.json"
    original = json.loads(result_path.read_text()) if result_path.exists() else None
    native = rows(directory / "native_frames.jsonl")
    receipts = rows(directory / "control_receipts.jsonl")
    decisions = rows(directory / "normal_decisions.jsonl")
    by_frame = {row["before_frame"]: row for row in receipts if "before_frame" in row}
    failures, pulse, rates, decoder_bad, stages = [], [], [], [], {}
    for index, row in enumerate(native):
        frame = row["frame"]
        ack = by_frame.get(frame - 1)
        if (ack is None or ack["status"] != "ACKNOWLEDGED" or not ack["server_accepted"]
                or ack["after_ack_frame"] != frame - 1 or not same_control(ack["requested_control"], row["applied_control"])):
            failures.append({"frame": frame, "kind": "ACK_ALIGNMENT"})
        if index:
            previous = native[index - 1]
            dt = row["timestamp"] - previous["timestamp"]
            if frame != previous["frame"] + 1 or abs(dt - .05) > 1e-5:
                failures.append({"frame": frame, "kind": "NATIVE_CADENCE"})
            rate = (row["speed_mps"] - previous["speed_mps"]) / dt
            if abs(rate - row.get("speed_rate_mps2", rate)) > 1e-7:
                failures.append({"frame": frame, "kind": "RECORDED_SPEED_RATE"})
        else:
            rate = None
        stage = row["applied_policy"]["stage"]
        section = stages.setdefault(stage, {"frames": 0, "minimum_speed_mps": math.inf,
            "maximum_speed_mps": -math.inf, "minimum_speed_rate_mps2": None,
            "maximum_speed_rate_mps2": None, "decoder_rate_violations": []})
        section["frames"] += 1
        section["minimum_speed_mps"] = min(section["minimum_speed_mps"], row["speed_mps"])
        section["maximum_speed_mps"] = max(section["maximum_speed_mps"], row["speed_mps"])
        if rate is not None:
            section["minimum_speed_rate_mps2"] = min(rate, section["minimum_speed_rate_mps2"]
                                                    if section["minimum_speed_rate_mps2"] is not None else rate)
            section["maximum_speed_rate_mps2"] = max(rate, section["maximum_speed_rate_mps2"]
                                                    if section["maximum_speed_rate_mps2"] is not None else rate)
            if abs(rate) > 2.9:
                section["decoder_rate_violations"].append({"frame": frame, "speed_mps": row["speed_mps"],
                    "speed_rate_mps2": rate, "applied_control": row["applied_control"]})
        if row["applied_policy"]["stage"] == "measured_brake_pulse":
            pulse.append(row)
            if rate is not None:
                rates.append(rate)
                if abs(rate) > 2.9:
                    decoder_bad.append({"frame": frame, "speed_rate_mps2": rate})
        if row.get("geometry_failure"):
            failures.append({"frame": frame, "kind": "GEOMETRY_UNAVAILABLE", "error": row["geometry_failure"]})
        elif row["applied_policy"]["stage"] != "termination_service_brake_nontraining":
            failures.extend({"frame": frame, "kind": "GEOMETRY_GUARD", "error": message}
                            for message in frozen.scalar_guards(row))
    normal = [row for row in decisions if row["stage"] in {"gentle_launch_and_stabilize", "measured_brake_pulse"}]
    if normal:
        policy = frozen.PulsePolicy(normal[0]["brake_level"])
        for row in normal:
            try:
                proposed = SimpleNamespace(**row["normal_planner_before_intervention"])
                output = policy.apply(proposed, speed=row["measured_speed_mps"],
                    timestamp=row["source_timestamp"], frame=row["source_frame"])
                if (policy.last["stage"] != row["stage"] or policy.last["pulse_command_number"] != row["pulse_command_number"]
                        or not same_control(frozen.collector.control_dict(output), row["normal_control_before_original_emergency"])):
                    failures.append({"frame": row["frame"], "kind": "POLICY_REPLAY"})
                if row["original_agent_overrode_normal_control"]:
                    failures.append({"frame": row["frame"], "kind": "PRESERVED_ORIGINAL_EMERGENCY"})
            except Exception as error:
                failures.append({"frame": row["frame"], "kind": "POLICY_REPLAY_EXCEPTION", "error": str(error)})
                break
    archived_hashes = (original or {}).get("raw_files_sha256", {})
    for name, expected in archived_hashes.items():
        if digest(directory / name) != expected:
            failures.append({"kind": "RAW_HASH_MISMATCH", "file": name})
    summary = {"directory": str(directory), "original_status": (original or {}).get("status", "MISSING"),
        "original_error": (original or {}).get("error"), "actor_id": (original or {}).get("actor_id"),
        "raw_frames": len(native), "control_receipts": len(receipts), "normal_decisions": len(normal),
        "pulse_frames": len(pulse), "pulse_decoder_rate_violations": decoder_bad,
        "stage_measurements": stages,
        "minimum_pulse_speed_rate_mps2": min(rates) if rates else None,
        "maximum_pulse_speed_rate_mps2": max(rates) if rates else None,
        "evidence_failures": failures, "training_data_admitted": False,
        "full_stopping_distance_proven": False}
    if pulse:
        first, last = pulse[0], pulse[-1]
        entry = first["applied_policy"]["entry"]
        elapsed = last["timestamp"] - entry["timestamp"]
        mean = (entry["speed_mps"] - last["speed_mps"]) / elapsed
        summary.update(brake_level=first["applied_policy"]["brake_level"], entry=entry,
            pulse_duration_sec=elapsed, exit_speed_mps=last["speed_mps"], mean_pulse_deceleration_mps2=mean,
            pulse_body_stop_distance_range_m=[min(row["front_bumper_stop_distance_m"] for row in pulse),
                                              max(row["front_bumper_stop_distance_m"] for row in pulse)],
            constant_mean_extrapolated_distance_to_point1_m=(entry["speed_mps"] ** 2 - .1 ** 2) / (2 * mean)
                if mean > 0 else None,
            extrapolation_notice="Algebra only, not observed stopping distance: force may change with speed/grade/gear.")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", required=True, type=Path)
    parser.add_argument("--owner-plan-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    frozen.scout.require(digest(args.trial / "owner_plan.json") == args.owner_plan_sha256, "owner plan differs")
    frozen.scout.require(digest(Path(frozen.__file__)) == WORKER_SHA256, "frozen replay implementation changed")
    frozen.scout.require(not args.output.exists() and not args.output.is_symlink(), "new report path required")
    owner = json.loads((args.trial / "owner_result.json").read_text())
    levels = [analyze_level(path) for path in sorted((args.trial / "calibration").glob("level_*")) if path.is_dir()]
    identities = [level["actor_id"] for level in levels if level["actor_id"] is not None]
    result = {"status": "DIAGNOSED_NOT_TRAINING", "owner_status_preserved": owner["status"],
        "owner_result_sha256": digest(args.trial / "owner_result.json"),
        "owner_plan_sha256": args.owner_plan_sha256, "levels": levels,
        "distinct_spawned_actor_ids": len(identities) == len(set(identities)),
        "complete_four_level_calibration": len(levels) == 4 and all(
            row["original_status"] == "PASS_MEASURED_CALIBRATION" and not row["evidence_failures"] for row in levels),
        "training_data_admitted": False, "signal_stopping_safety_proven": False,
        "notice": "Failed and cleanup native frames remain untouched. Pulse averages cannot establish a low-speed coast/hold profile."}
    args.output.mkdir(parents=True)
    frozen.scout.write_new(args.output / "summary.json", result)
    print(json.dumps({"status": result["status"], "levels": len(levels),
        "complete_four_level_calibration": result["complete_four_level_calibration"],
        "report": str(args.output / "summary.json")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
