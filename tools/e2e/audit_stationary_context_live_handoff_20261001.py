"""HH_261001 - Preserve counterevidence when comparing TRAIN starts with research handoffs."""

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "storage/experiments/validation/2026-10-01"
ALIAS = ROOT / "docs/assets/validation/2026-10-01/selector_phase_ab/train_stationary_context_alias.json"
RUNS = ("ah_town07_straight_research_v2",
        "selector_phase_S0_uniform_selector_town07_v1",
        "selector_phase_S1_phase_balanced_selector_town07_v1")
FEATURES = {"velocity_x_mps": "vx", "velocity_y_mps": "vy",
            "acceleration_x_mps2": "ax", "acceleration_y_mps2": "ay",
            "yaw_rate_radps": "yaw_rate", "steering_tire_angle_rad": "steering_tire_angle_rad"}
COMMANDS = ("command_left", "command_right", "command_straight", "command_lane_follow",
            "command_change_left", "command_change_right")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def readlines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def handoff_history(states, camera_frames, handoff_frame):
    """HH_261001 - Bind history to actual camera receipts, never assume frame-number parity."""
    frames = [row["frame"] for row in camera_frames if row["frame"] <= handoff_frame][-10:]
    if len(frames) != 10 or frames[-1] != handoff_frame or len(set(frames)) != 10:
        raise ValueError("ten distinct exact camera/state history frames required")
    lookup = {row["frame"]: row for row in states}
    if len(lookup) != len(states) or any(frame not in lookup for frame in frames):
        raise ValueError("ambiguous camera/state association")
    return [lookup[frame] for frame in frames]


def feature_comparison(history, original):
    if len(history) != len(original) or len(history) != 10:
        raise ValueError("aligned ten-anchor comparison required")
    deltas = {name: max(abs(row[field] - ref["current_features"][name])
                       for row, ref in zip(history, original)) for name, field in FEATURES.items()}
    deltas["bias"] = max(abs(1. - ref["current_features"]["bias"]) for ref in original)
    for index, name in enumerate(COMMANDS):
        deltas[name] = max(abs(float(row["command"] == index) - ref["current_features"][name])
                           for row, ref in zip(history, original))
    return dict(max_abs_by_feature=deltas, all_thirteen_features_exact_equal=not any(deltas.values()))


def brief(row, state, first_timestamp):
    return dict(frame=row["inference_frame"], elapsed_since_handoff_s=state["timestamp"]-first_timestamp,
                vx_mps=state["vx"], steering_tire_angle_rad=state["steering_tire_angle_rad"],
                selected_candidate=row["selected"]["candidate_index"],
                selected_speed_at_0_3s_mps=row["selected"]["speed_mps"][2],
                actual_control_proposal={key: row["control"][key] for key in ("throttle", "brake", "steer")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh evidence output required")
    source = json.loads(ALIAS.read_text())
    original = sorted((row for row in source["anchors"] if "town07_town07_straight" in row["episode_id"]
                       and 31 <= row["sequence_index"] <= 40), key=lambda row: row["sequence_index"])
    runs = []
    for run in RUNS:
        directory = BASE / run / "episode.partial"
        paths = [directory / name for name in ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl")]
        states, cameras, commands = map(readlines, paths)
        lookup = {row["frame"]: row for row in states}
        learned = [row for row in commands if row.get("application", {}).get("learned_control_applied")
                   and not row["held_between_camera_frames"]]
        first = learned[0]
        history = handoff_history(states, cameras, first["inference_frame"])
        comparison = feature_comparison(history, original)
        first_time = lookup[first["inference_frame"]]["timestamp"]
        compact = [brief(row, lookup[row["inference_frame"]], first_time) for row in learned]
        transitions = [row for i, row in enumerate(compact)
                       if i == 0 or row["selected_candidate"] != compact[i-1]["selected_candidate"]]
        low = [lookup[row["inference_frame"]] for row in learned
               if abs(lookup[row["inference_frame"]]["vx"]) <= .5]
        runs.append(dict(run=run, source_sha256={str(path.relative_to(ROOT)): sha(path) for path in paths},
            checkpoint_sha256=first["source_checkpoint_sha256"], first_learned=compact[0],
            handoff_history=[dict(frame=row["frame"], **{key: row[key] for key in FEATURES.values()},
                                 command=row["command"]) for row in history],
            original_town07_sequences_31_to_40_feature_comparison=comparison,
            first_18_inferences=compact[:18], selected_candidate_transitions=transitions,
            low_speed_inference_count=len(low), low_speed_abs_tire_angle_max_rad=max(abs(row["steering_tire_angle_rad"]) for row in low),
            actual_full_input_identity_not_established=True))
    result = dict(schema="portable_e2e.stationary_context_handoff_comparison.v1", alias_audit_sha256=sha(ALIAS),
        train_reference=original, runs=runs, neural_forward_passes=0, optimizer_steps=0,
        dev_metrics_computed=False, training_rows_discarded=0, labels_modified=False,
        causal_claims_not_established=["steering history caused stall", "warmup labels caused stall",
                                      "fixed source implies identical simulator startup"],
        training_data_approved=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "runs": len(runs), "neural_forwards": 0}))


if __name__ == "__main__":
    main()
