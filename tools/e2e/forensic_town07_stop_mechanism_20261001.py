#!/usr/bin/env python3
"""HH_261001 - Audit archived Town07 stops without starting CARLA or a model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "storage/experiments/validation/2026-10-01"
RUNS = {
    "A_new20": "directional_gradient_A_town07_sparse20_v1",
    "B_new20": "directional_gradient_B_town07_sparse20_v1",
    "W1_history20": "warmup_supervision_W1_warmup_anchor_off_town07_v1",
    "W1_history150": "warmup_supervision_W1_town07_long_v1",
}
CONTROL_FIELDS = ("throttle", "brake", "steer")


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def speed_xy(state: dict) -> float:
    return math.hypot(*state["world_velocity_carla"][:2])


def project_decision(row: dict) -> dict:
    """HH_261001 - Discard large raw candidate tensors after extracting causal fields."""
    control = row.get("control") or {}
    selected = row.get("selected") or {}
    speeds = selected.get("speed_mps") or []
    return {
        "source": row.get("control_source"),
        "inference_frame": row.get("inference_frame"),
        "command_frame": row.get("command_frame"),
        "held": row.get("held_between_camera_frames"),
        "desired_speed_mps": control.get("desired_speed_mps"),
        "selected_speed_0p3_mps": speeds[2] if len(speeds) > 2 else None,
        "throttle": control.get("throttle"),
        "brake": control.get("brake"),
        "steer": control.get("steer"),
    }


def audit_run(name: str, dirname: str) -> dict:
    episode = SOURCE / dirname / "episode.partial"
    states_path = episode / "states.jsonl"
    decisions_path = episode / "learned_control.jsonl"
    receipts_path = episode / "control_receipts.jsonl"
    states = list(read_jsonl(states_path))
    decisions = {
        row["ack_receipt_sequence"]: project_decision(row)
        for row in read_jsonl(decisions_path)
        if row.get("ack_receipt_sequence") is not None
    }
    receipts = {row["sequence"]: row for row in read_jsonl(receipts_path)}
    assert states, name
    first_ts = states[0]["timestamp"]
    integrity = {
        "missing_receipt": 0,
        "not_acknowledged": 0,
        "frame_binding_failure": 0,
        "receipt_frame_mismatch": 0,
        "requested_control_mismatch": 0,
        "decision_control_mismatch": 0,
        "decision_frame_mismatch": 0,
        "noncontiguous_state_frame": 0,
    }
    dt_values = []
    for index, state in enumerate(states):
        sequence = state["control_transport"]["expected_receipt_sequence"]
        receipt = receipts.get(sequence)
        decision = decisions.get(sequence)
        if receipt is None:
            integrity["missing_receipt"] += 1
            continue
        if receipt.get("status") != "ACKNOWLEDGED" or not receipt.get("server_accepted"):
            integrity["not_acknowledged"] += 1
        if state["control_transport"].get("frame_binding_pass") is not True:
            integrity["frame_binding_failure"] += 1
        if receipt.get("before_frame") != state["frame"] - 1 or receipt.get("after_ack_frame") != receipt.get("before_frame"):
            integrity["receipt_frame_mismatch"] += 1
        if any(abs(float(receipt["requested_control"][field]) - float(state["current_control"][field])) > 1e-6 for field in CONTROL_FIELDS):
            integrity["requested_control_mismatch"] += 1
        if decision and decision["source"] == "learned_trajectory_tracker" and decision["command_frame"] != receipt.get("before_frame"):
            integrity["decision_frame_mismatch"] += 1
        if decision and decision["source"] == "learned_trajectory_tracker" and any(
            abs(float(receipt["requested_control"][field]) - float(decision[field])) > 1e-6
            for field in CONTROL_FIELDS
        ):
            integrity["decision_control_mismatch"] += 1
        if index:
            previous = states[index - 1]
            if state["frame"] != previous["frame"] + 1:
                integrity["noncontiguous_state_frame"] += 1
            dt_values.append(state["timestamp"] - previous["timestamp"])

    events = []
    for index in range(1, len(states)):
        before, after = states[index - 1:index + 1]
        dt = after["timestamp"] - before["timestamp"]
        if dt <= 0 or not 14 <= after["route_progress_m"] <= 26 or before["vx"] <= 0.7:
            continue
        longitudinal_delta = (after["vx"] - before["vx"]) / dt
        if longitudinal_delta >= -8:
            continue
        sequence = after["control_transport"]["expected_receipt_sequence"]
        receipt = receipts[sequence]
        decision = decisions.get(sequence)
        before_xy = before["world_velocity_carla"][:2]
        acceleration_xy = after["world_acceleration_carla"][:2]
        along_previous_velocity = sum(v * a for v, a in zip(before_xy, acceleration_xy)) / math.hypot(*before_xy)
        follow = states[index:min(len(states), index + 7)]
        following_hold = next((row for row in follow if row["current_control"]["brake"] >= 0.3 and abs(row["vx"]) <= 0.05), None)
        lead = states[max(0, index - 20):index]
        events.append({
            "state_frame": after["frame"],
            "relative_time_s": round(after["timestamp"] - first_ts, 3),
            "route_progress_m": round(after["route_progress_m"], 3),
            "previous_forward_speed_mps": round(before["vx"], 4),
            "observed_forward_speed_mps": round(after["vx"], 4),
            "previous_raw_xy_speed_mps": round(speed_xy(before), 4),
            "observed_raw_xy_speed_mps": round(speed_xy(after), 4),
            "discrete_forward_accel_mps2": round(longitudinal_delta, 4),
            "carla_raw_accel_along_previous_velocity_mps2": round(along_previous_velocity, 4),
            "requested_throttle": round(after["current_control"]["throttle"], 6),
            "requested_brake": round(after["current_control"]["brake"], 6),
            "receipt_sequence": sequence,
            "receipt_status": receipt["status"],
            "receipt_command_frame": receipt["before_frame"],
            "decision": decision,
            "prior_1s_throttle_range": [round(min(row["current_control"]["throttle"] for row in lead), 4), round(max(row["current_control"]["throttle"] for row in lead), 4)],
            "prior_1s_brake_range": [round(min(row["current_control"]["brake"] for row in lead), 4), round(max(row["current_control"]["brake"] for row in lead), 4)],
            "brake_0p3_after_stop_frame": following_hold["frame"] if following_hold else None,
        })

    return {
        "run": name,
        "source_directory": str((SOURCE / dirname).relative_to(ROOT)),
        "source_sha256": {file.name: sha256(file) for file in (states_path, decisions_path, receipts_path)},
        "state_count": len(states),
        "first_and_last_frame": [states[0]["frame"], states[-1]["frame"]],
        "median_sim_tick_s": round(sorted(dt_values)[len(dt_values) // 2], 9),
        "sim_tick_range_s": [round(min(dt_values), 9), round(max(dt_values), 9)],
        "integrity": integrity,
        "candidate_drop_threshold": "prior vx >0.7m/s, delta vx/dt <-8m/s^2, route progress 14..26m",
        "drop_events": events,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", choices=RUNS, action="append", help="Select runs; default is all four archived runs")
    args = parser.parse_args()
    selected = args.run or list(RUNS)
    print(json.dumps({"schema": "portable_e2e.town07_stop_forensic.v1", "runs": [audit_run(name, RUNS[name]) for name in selected]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
