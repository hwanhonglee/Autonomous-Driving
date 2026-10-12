#!/usr/bin/env python3
"""HH_261001 - Independent read-only native-stage/pulse replay for v2, without changing failed raw measurements."""

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import audit_town12_brake_calibration_20261001 as helpers
from scripts.e2e import audit_town12_j15824_capture_20261001 as cleanup
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v2_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v2_20261001 as owner_module


WORKER_SHA256 = "3eb142bda626b3c57af203452c8e6ea9240c5f6fe72b655461b8af14f2530661"


def same_record(first, second, field=None):
    """HH_261001 - CARLA stores pedals/steering as float32; keep its existing ACK tolerance only on those fields."""
    if isinstance(first, dict) and isinstance(second, dict):
        return first.keys() == second.keys() and all(same_record(first[key], second[key], key) for key in first)
    if isinstance(first, list) and isinstance(second, list):
        return len(first) == len(second) and all(same_record(a, b, field) for a, b in zip(first, second))
    if field in {"throttle", "brake", "steer"}:
        return math.isfinite(first) and math.isfinite(second) and abs(first - second) <= 1e-6
    return first == second


def source_audit(trial, plan):
    with owner_module.configured_owner(), owner_module.inherited.configured_owner():
        paths = owner_module.inherited.inherited.diagnostic.source_paths()
    failures = []
    if set(paths) != set(plan["source_sha256"]):
        failures.append("source closure keys differ")
    for name, expected in plan["source_sha256"].items():
        for label, path in (("current", paths.get(name)), ("archived", trial / "provenance" / name)):
            if path is None or not path.is_file() or path.is_symlink() or helpers.digest(path) != expected:
                failures.append(f"{label}: {name}")
    return {"status": "PASS" if not failures else "FAIL", "files": len(paths), "failures": failures}


def analyze_level(directory):
    result = json.loads((directory / "result.json").read_text())
    native = helpers.rows(directory / "native_frames.jsonl")
    receipts = helpers.rows(directory / "control_receipts.jsonl")
    decisions = helpers.rows(directory / "normal_decisions.jsonl")
    before = {row.get("before_frame"): row for row in receipts}
    native_frames = {row["frame"]: row for row in native}
    failures, stages, policy_rows, pulses = [], {}, [], []
    if len(before) != len(receipts) or len(native_frames) != len(native):
        failures.append({"kind": "DUPLICATE_FRAME"})
    for index, row in enumerate(native):
        frame = row["frame"]
        ack = before.get(frame - 1)
        if (ack is None or ack.get("status") != "ACKNOWLEDGED" or ack.get("server_accepted") is not True
                or ack.get("actor_id") != result["actor_id"] or ack.get("response_actor_id") != result["actor_id"]
                or ack.get("after_ack_frame") != frame - 1
                or not helpers.same_control(ack["requested_control"], row["applied_control"])):
            failures.append({"kind": "ACK_ALIGNMENT", "frame": frame})
        reconstructed = worker.collector.base_link_state(row["actor_snapshot_transform_carla"], row["world_velocity_carla"],
            row["world_acceleration_carla"], row["world_angular_velocity_carla_deg_s"][2], worker.collector.WHEELBASE_M)
        if any(abs(reconstructed[key] - row[key]) > 1e-8 for key in reconstructed):
            failures.append({"kind": "RAW_SNAPSHOT_CONVERSION", "frame": frame})
        speed = math.hypot(reconstructed["vx"], reconstructed["vy"])
        if abs(speed - row["speed_mps"]) > 1e-8:
            failures.append({"kind": "SPEED_RECONSTRUCTION", "frame": frame})
        rate = None
        if index:
            previous = native[index - 1]
            dt = row["timestamp"] - previous["timestamp"]
            if frame != previous["frame"] + 1 or abs(dt - .05) > 1e-5:
                failures.append({"kind": "NATIVE_CADENCE", "frame": frame})
            if dt > 0:
                rate = (speed - previous["speed_mps"]) / dt
                if row.get("speed_rate_mps2") is None or abs(rate - row["speed_rate_mps2"]) > 1e-8:
                    failures.append({"kind": "SPEED_RATE_RECONSTRUCTION", "frame": frame})
        stage = row["applied_policy"]["stage"]
        stages.setdefault(stage, []).append({"frame": frame, "speed_mps": speed, "speed_rate_mps2": rate,
            "gear": row["applied_control"]["gear"], "control": row["applied_control"]})
        if stage == "measured_brake_pulse":
            pulses.append(row)
        if row.get("geometry_failure"):
            failures.append({"kind": "GEOMETRY_MISSING", "frame": frame, "error": row["geometry_failure"]})
        elif stage != "termination_service_brake_nontraining":
            failures.extend({"kind": "BODY_SIGNAL_GUARD", "frame": frame, "error": error}
                            for error in worker.inherited.scalar_guards(row))
        if any(row.get("events_seen", [])):
            failures.append({"kind": "EVENT_RECORDED", "frame": frame})
    normal = [row for row in decisions if row["stage"] in {"timed_cap_launch_and_stabilize", "measured_brake_pulse"}]
    with worker.configured_worker():
        policy = worker.inherited.PulsePolicy(result["brake_level"])
        for row in normal:
            frame = row["source_frame"]
            state = native_frames.get(frame)
            if state is None or abs(row["source_timestamp"] - state["timestamp"]) > 1e-8:
                failures.append({"kind": "POLICY_SOURCE_FRAME", "frame": frame})
            if state is not None:
                vx, vy, _ = state["world_velocity_carla"]
                if abs(math.hypot(vx, vy) - row["measured_speed_mps"]) > 1e-8:
                    failures.append({"kind": "POLICY_ACTOR_SPEED", "frame": frame})
            try:
                control = policy.apply(SimpleNamespace(**row["normal_planner_before_intervention"]),
                    speed=row["measured_speed_mps"], timestamp=row["source_timestamp"], frame=frame)
                replay = worker.collector.control_dict(control)
                valid = (same_record(policy.last, {key: value for key, value in row.items()
                                         if key not in {"frame", "final_control", "original_agent_overrode_normal_control"}})
                         and helpers.same_control(replay, row["normal_control_before_original_emergency"]))
                if not valid:
                    failures.append({"kind": "POLICY_REPLAY", "frame": frame})
                if row["original_agent_overrode_normal_control"]:
                    failures.append({"kind": "PRESERVED_EMERGENCY", "frame": frame})
                if not helpers.same_control(row["final_control"], before[frame]["requested_control"]):
                    failures.append({"kind": "PROPOSAL_ACK_MISMATCH", "frame": frame})
                policy_rows.append({"frame": frame, "status": "PASS" if valid else "FAIL"})
            except Exception as error:
                failures.append({"kind": "POLICY_REPLAY_EXCEPTION", "frame": frame, "error": str(error)})
                break
    for name, expected in result.get("raw_files_sha256", {}).items():
        if helpers.digest(directory / name) != expected:
            failures.append({"kind": "RAW_HASH_MISMATCH", "file": name})
    summary = {"actor_id": result["actor_id"], "brake_level": result["brake_level"],
        "original_status": result["status"], "original_error": result.get("error"),
        "frames": len(native), "ACKs": len(receipts), "normal_policy_replayed": len(policy_rows),
        "pulse_frames": len(pulses), "stage_measurements": {}, "evidence_failures": failures,
        "cleanup": result["actor_cleanup"], "training_data_admitted": False}
    for stage, values in stages.items():
        rates = [row["speed_rate_mps2"] for row in values if row["speed_rate_mps2"] is not None]
        summary["stage_measurements"][stage] = {"frames": len(values),
            "speed_range_mps": [min(row["speed_mps"] for row in values), max(row["speed_mps"] for row in values)],
            "speed_rate_range_mps2": [min(rates), max(rates)] if rates else None,
            "decoder_rate_violations": [row for row in values if row["speed_rate_mps2"] is not None
                                        and abs(row["speed_rate_mps2"]) > 2.9],
            "runtime_rate_violations": [row for row in values if row["speed_rate_mps2"] is not None
                                        and not -6. <= row["speed_rate_mps2"] <= 3.]}
    if pulses:
        entry, last = pulses[0]["applied_policy"]["entry"], pulses[-1]
        first = native_frames[entry["frame"]]
        duration = last["timestamp"] - entry["timestamp"]
        mean = (entry["speed_mps"] - last["speed_mps"]) / duration
        summary["pulse_measurement"] = {"entry": entry, "exit_speed_mps": last["speed_mps"],
            "duration_sec": duration, "mean_deceleration_mps2": mean,
            "progress_m": last["route_projection"]["progress_m"] - first["route_projection"]["progress_m"],
            "actor_center_dz_m": last["actor_snapshot_transform_carla"]["z"] - first["actor_snapshot_transform_carla"]["z"],
            "first_actor_pitch_deg": first["actor_snapshot_transform_carla"]["pitch"],
            "last_actor_pitch_deg": last["actor_snapshot_transform_carla"]["pitch"],
            "minimum_body_lane_margin_m": min(point["lane_margin_m"] for row in pulses for point in row["body_corners"]),
            "minimum_signal_distance_m": min(row["front_bumper_stop_distance_m"] for row in pulses),
            "constant_mean_extrapolated_stop_distance_m": (entry["speed_mps"] ** 2 - .1 ** 2) / (2 * mean)
                if mean > 0 else None, "extrapolation_is_observed_stopping_distance": False,
            "full_stop_or_low_speed_behavior_proven": False}
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--owner-plan-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    worker.scout.require(helpers.digest(args.trial / "owner_plan.json") == args.owner_plan_sha256, "owner plan changed")
    worker.scout.require(helpers.digest(Path(worker.__file__)) == WORKER_SHA256, "frozen worker changed")
    worker.scout.require(not args.output.exists() and not args.output.is_symlink(), "new report path required")
    plan = json.loads((args.trial / "owner_plan.json").read_text())
    owner = json.loads((args.trial / "owner_result.json").read_text())
    observed_cleanup = cleanup.cleanup_observation(args.trial)
    levels = [analyze_level(path) for path in sorted((args.trial / "calibration").glob("level_*")) if path.is_dir()]
    sources = source_audit(args.trial, plan)
    result = {"status": "DIAGNOSED_NOT_TRAINING", "original_owner_status": owner["status"],
        "original_owner_sha256": helpers.digest(args.trial / "owner_result.json"),
        "owner_plan_sha256": args.owner_plan_sha256, "sources": sources,
        "independent_cleanup": observed_cleanup, "levels": levels,
        "all_four_fresh_actors_observed": len(levels) == len({row["actor_id"] for row in levels}) == 4,
        "training_data_admitted": False, "signal_stop_safety_proven": False,
        "full_stop_behavior_proven": False,
        "policy_replay_control_tolerance": {"fields": ["throttle", "brake", "steer"], "absolute": 1e-6,
            "reason": "CARLA float32 actuator fields versus Python double replay; all other metadata remains exact."},
        "notice": "All stages remain raw. Launch/cleanup violations are not pulse data; two-second pulse trends are not a stopping guarantee."}
    args.output.mkdir(parents=True)
    worker.scout.write_new(args.output / "summary.json", result)
    print(json.dumps({"status": result["status"], "levels": len(levels), "sources": sources["status"],
                      "cleanup": observed_cleanup["status"], "report": str(args.output / "summary.json")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
