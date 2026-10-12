#!/usr/bin/env python3
"""HH_261001 - Replay bound-signal policy decisions against immutable same-frame actor snapshots."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from scripts.e2e import audit_town12_j15824_capture_20261001 as qa
from scripts.e2e import capture_town12_j15824_signal_approach_20261001 as policy


def audit(episode, qualification, carla):
    require = qa.require
    paths = {name: episode / name for name in ("manifest.json", "states.jsonl", "control_receipts.jsonl")}
    paths["signal_approach_controls.jsonl"] = episode.parent / "signal_approach_controls.jsonl"
    paths["scout_result.json"] = episode.parents[1] / "scout/scout_result.json"
    hashes = {name: qa.sha(path) for name, path in paths.items()}
    manifest = json.loads(paths["manifest.json"].read_text())
    scout = json.loads(paths["scout_result.json"].read_text())
    require(qualification["episode"] == str(episode), "QA belongs to another episode")
    prefix = str(episode.relative_to(Path(qualification["trial"])))
    require(all(hashes[name] == qualification["input_sha256"][prefix + "/" + name]
                for name in ("manifest.json", "states.jsonl", "control_receipts.jsonl")), "native metadata changed since offline QA")
    require(manifest["town12_bound_signal_approach_policy"] == policy.policy_record(), "policy manifest differs from executed fixed revision")
    planned = qualification["frozen_trial"]["combined_protocol_revision"]["additional_signal_policy_revision"]
    require(planned == policy.policy_record(), "owner did not prospectively freeze this signal approach policy")
    states = [json.loads(line) for line in paths["states.jsonl"].read_text().splitlines()]
    decisions = [json.loads(line) for line in paths["signal_approach_controls.jsonl"].read_text().splitlines()]
    receipts = [json.loads(line) for line in paths["control_receipts.jsonl"].read_text().splitlines()]
    by_frame = {row["frame"]: row for row in states}
    require(len({row["frame"] for row in decisions}) == len(decisions), "duplicate policy decision frame")
    stop = scout["controlling_signal"]["selected_stop_waypoint"]
    stop_x, stop_y, _ = stop["xyz_carla_m"]
    angle = math.radians(stop["yaw_carla_deg"])
    box, _ = qa.lane._body_box(manifest, carla)
    records, failures = [], []
    for decision in decisions:
        frame = decision["frame"]
        state = by_frame.get(frame)
        if state is None:
            failures.append({"frame": frame, "failure": "policy decision has no committed native frame; consult retained abort snapshot"})
            records.append({"frame": frame, "status": "UNCOMMITTED_ABORT_OR_INVALID", "decision": decision})
            continue
        pose = state["actor_snapshot_transform_carla"]
        transform = carla.Transform(carla.Location(**{key: pose[key] for key in ("x", "y", "z")}),
                                    carla.Rotation(**{key: pose[key] for key in ("roll", "pitch", "yaw")}))
        distance = min((stop_x - vertex.x) * math.cos(angle) + (stop_y - vertex.y) * math.sin(angle)
                       for vertex in box.get_world_vertices(transform))
        light = state["traffic_light_observation"]["town12_signal"]
        active = policy.should_stop(light["signal_state"], distance)
        checks = {
            "frame_time_identity": abs(state["timestamp"] - decision["timestamp"]) <= 1e-6,
            "bound_signal_identity": decision["signal_actor_id"] == scout["controlling_signal"]["actor_id"],
            "signal_state": decision["signal_state"] == light["signal_state"],
            "actual_body_distance": abs(decision["front_bumper_stop_distance_m"] - distance) <= 1e-5,
            "bounded_active_decision": decision["active"] is active,
            "temporary_target": decision["effective_normal_pid_target_speed_kmh"] == (0. if active else decision["planned_target_speed_kmh"]),
            "restoration_declared": decision["target_restored_after_step"] is True,
            "approved_policy": decision["policy_id"] == policy.POLICY_ID,
        }
        hazard = decision["original_basic_agent_red_hazard"]
        emitted = decision["returned_control"]
        checks["original_red_emergency_retained"] = not hazard["detected"] or (
            emitted["throttle"] == 0 and abs(emitted["brake"] - .5) <= 1e-6)
        checks["normal_pedal_caps_retained"] = hazard["detected"] or (
            emitted["brake"] <= .15 + 1e-6 and emitted["throttle"] <= .20 + 1e-6)
        expected_control = SimpleNamespace(**emitted)
        qa.capture.collector.suppress_stopped_brake_steering(expected_control, math.hypot(state["vx"], state["vy"]),
            enabled=manifest["capture_contract"]["stopped_brake_steering_suppressed"])
        control_mismatches = qa.raw.ack_audit.mismatched_controls(state["next_control"], vars(expected_control), requested_pair=True)
        checks["original_return_plus_existing_stopped_guard_matches_command"] = not control_mismatches
        if state["capture_phase"] == "driving":
            checks["embedded_same_frame_policy"] = state["traffic_light_observation"]["town12_signal_approach_policy"] == decision
            checks["original_hazard_witness"] = hazard == light["basic_agent_red_hazard"]
        row = {"frame": frame, "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "actual_body_stop_plane_distance_m": distance, "active": active, "signal_state": light["signal_state"],
            "speed_mps": math.hypot(state["vx"], state["vy"]), "control_mismatches": control_mismatches}
        records.append(row)
        failures.extend({"frame": frame, "failure": name} for name, passed in checks.items() if not passed)
    actual_frames = {row["frame"] for row in decisions}
    expected_frames = {row["before_frame"] for row in receipts if row["reason"] == "initial_drive_control"}
    expected_frames.update(row["frame"] for row in states if row["capture_phase"] == "driving"
                           and row.get("goal_stop", {}).get("termination_reason") is None)
    if actual_frames != expected_frames:
        failures.append({"failure": "policy-decision frame coverage differs from initial/ongoing expert steps",
                         "missing": sorted(expected_frames - actual_frames), "extra": sorted(actual_frames - expected_frames)})
    for name, path in paths.items():
        require(qa.sha(path) == hashes[name], "policy evidence changed during offline replay")
    active = [row for row in records if row.get("active")]
    return {"status": "PASS_POLICY_REPLAY" if decisions and not failures else "FAIL_OR_INCOMPLETE",
        "decision_count": len(decisions), "active_decision_count": len(active),
        "active_signal_state_counts": dict(Counter(row["signal_state"] for row in active)),
        "minimum_active_body_stop_distance_m": min((row["actual_body_stop_plane_distance_m"] for row in active), default=None),
        "minimum_active_measured_speed_mps": min((row["speed_mps"] for row in active), default=None),
        "failures": failures, "source_sha256": hashes, "training_data_admitted": False,
        "scope": "Policy timing/body-state/target/control replay only; full route, strict body/sweep, runtime64, RGB and admission checks remain separate."}, records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    episode, output = args.episode.resolve(strict=True), args.output.absolute()
    qa.require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(episode.parents[1]),
               "new policy replay output must be outside immutable raw trial")
    qualification_sha = qa.sha(args.qualification)
    qualification = json.loads(args.qualification.read_text())
    sys.path[:0] = [str(qa.capture.scout.SOURCE_EGG), str(qa.capture.scout.SOURCE_PLANNER.parents[2])]
    carla, _ = qa.capture.scout.source_api()
    report, rows = audit(episode, qualification, carla)
    report.update(qualification_sha256=qualification_sha, script_sha256=qa.sha(Path(__file__)),
                  policy_source_sha256=qa.sha(Path(policy.__file__)))
    output.mkdir(parents=True, exist_ok=False)
    qa.capture.scout.write_new(output / "summary.json", report)
    with (output / "all_policy_replay_frames.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    print(json.dumps({key: report[key] for key in ("status", "decision_count", "active_decision_count", "failures")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
