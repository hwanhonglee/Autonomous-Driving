"""HH_261001 - Audit actual cap binding and same-state actuator differences; no rollout counterfactual claim."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RUNS = ("ah_town07_straight_research_v2", "selector_phase_S0_uniform_selector_town07_v1",
        "selector_phase_S1_phase_balanced_selector_town07_v1")
CACHE = ROOT / "storage/experiments/training/2026-10-01/ah_selector_phase_ab_20261001_v1/frozen_train_cache.npz"
CACHE_SHA = "763dcbb76046317e6cbc9254eec0ee4232bbf01cae4b8407a9f9cf761093e856"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def longitudinal(raw_preview, current, cap):
    """HH_261001 - Exact existing longitudinal arithmetic, independent of unobserved future vehicle responses."""
    desired = min(cap, raw_preview)
    error = desired - max(0., current)
    throttle, brake = min(.5, max(0., .7*error)), min(1., max(0., -.7*error))
    if desired <= .05:
        throttle, brake = 0., max(.3, brake)
    return dict(desired_speed_mps=desired, throttle=throttle, brake=brake)


def trace(directory):
    states, commands, cameras = (rows(directory/name) for name in ("states.jsonl", "learned_control.jsonl", "camera_frames.jsonl"))
    by_frame = {row["frame"]: row for row in states}
    source_indices = {row["frame"]: i for i, row in enumerate(states)}
    learned = [row for row in commands if row.get("application", {}).get("learned_control_applied")
               and not row["held_between_camera_frames"]]
    first_time = by_frame[learned[0]["inference_frame"]]["timestamp"]
    switch_index = next(i for i in range(1, len(learned))
                        if learned[i-1]["selected"]["candidate_index"] == 5 and learned[i]["selected"]["candidate_index"] == 1)
    result = []
    for i, decision in enumerate(learned[:switch_index+1]):
        state = by_frame[decision["inference_frame"]]
        previous = states[source_indices[state["frame"]]-1]
        preview = decision["selected"]["speed_mps"][2]
        capped, cap30 = longitudinal(preview, state["vx"], 2.5), longitudinal(preview, state["vx"], 30/3.6)
        if any(abs(capped[key]-decision["control"][key]) > 1e-9 for key in capped):
            raise ValueError("recorded controller differs from exact capped arithmetic")
        history = [by_frame[camera["frame"]] for camera in cameras if camera["frame"] <= state["frame"]][-10:]
        result.append(dict(frame=state["frame"], elapsed_since_handoff_s=state["timestamp"]-first_time,
            selected=decision["selected"]["candidate_index"], current_vx_mps=state["vx"],
            raw_selected_speed_at_0p3_mps=preview, actual_control=capped, same_state_cap30_control=cap30,
            cap_bound=preview > 2.5+1e-6, throttle_reduction_due_to_cap=cap30["throttle"]-capped["throttle"],
            model_input_ax_mps2=state["ax"], history_ax_mps2=[s["ax"] for s in history],
            history_vx_mps=[s["vx"] for s in history],
            preceding_tick_dvx_dt_mps2=(state["vx"]-previous["vx"])/(state["timestamp"]-previous["timestamp"]),
            control_acknowledged=decision["application"]["acknowledged"]))
    before = result[:-1]
    capped = [row for row in before if row["cap_bound"]]
    changed = [row for row in before if row["throttle_reduction_due_to_cap"] > 1e-9]
    ax_negative = [row for row in before if row["model_input_ax_mps2"] < -.05]
    return dict(run=directory.parent.name, source_sha256={name: sha(directory/name) for name in
        ("states.jsonl", "learned_control.jsonl", "camera_frames.jsonl")}, checkpoint_sha256=learned[0]["source_checkpoint_sha256"],
        first_5_to_1=result[-1], pre_switch_trace=before,
        clipped_preview_inferences_before_switch=len(capped), changed_throttle_inferences_before_switch=len(changed),
        first_cap_binding_s=capped[0]["elapsed_since_handoff_s"] if capped else None,
        first_actual_throttle_difference_s=changed[0]["elapsed_since_handoff_s"] if changed else None,
        first_negative_model_input_ax_s=ax_negative[0]["elapsed_since_handoff_s"] if ax_negative else None,
        maximum_same_state_throttle_reduction=max((row["throttle_reduction_due_to_cap"] for row in before), default=0.),
        unobserved_30cap_rollout_not_inferred=True)


def train_band():
    if sha(CACHE) != CACHE_SHA:
        raise ValueError("frozen original AH TRAIN cache differs")
    cache = np.load(CACHE, allow_pickle=False)
    all_band = set(np.flatnonzero((cache["current"] >= 1.5) & (cache["current"] <= 2.5)).tolist())
    audit = read(ROOT / "docs/assets/validation/2026-10-01/ah_town07_straight_research_v2/train_start_stop_audit.json")
    offsets, index = {}, 0
    for episode in audit["episodes"]:
        offsets[episode["episode_id"]] = index; index += episode["counts"]["all"]
    dataset = ROOT / "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1"
    manifest = read(dataset/"dataset.json")
    found = []
    for reference in manifest["episodes"]:
        path = dataset/reference["manifest"]
        if sha(path) != reference["sha256"]:
            raise ValueError("original source manifest differs")
        episode = read(path)
        if episode["split"] != "train":
            continue
        annotation = path.parent/episode["sample_jsonl"]
        if sha(annotation) != episode["sample_jsonl_sha256"]:
            raise ValueError("original annotation hash differs")
        samples = rows(annotation)
        for row in samples:
            cache_index = offsets[episode["episode_id"]]+row["sequence_index"]
            if cache_index not in all_band:
                continue
            speed = row["ego"]["linear_velocity_base_mps"][0]
            target = row["labels"]["planning"]["speed_mps"]
            if np.float32(speed) != cache["current"][cache_index] or not np.array_equal(np.asarray(target, dtype=np.float32), cache["target_speed"][cache_index]):
                raise ValueError("cached original TRAIN band association differs")
            found.append(dict(index=cache_index, episode_id=episode["episode_id"], sequence_index=row["sequence_index"],
                current_vx_mps=speed, model_input_ax_mps2=row["ego"]["linear_acceleration_base_mps2"][0],
                target_speed_at_0p3_mps=target[2], target_delta_at_0p3_mps=target[2]-speed,
                future_mean_acceleration_first_0p3_mps2=(target[2]-speed)/.3,
                parent_selected=int(cache["logits"][cache_index].argmax()), parent_oracle=int(cache["costs"][cache_index].argmin()),
                history_ax_mps2=[r["ego"]["linear_acceleration_base_mps2"][0] for r in samples[max(0,row["sequence_index"]-9):row["sequence_index"]+1]]))
    if {row["index"] for row in found} != all_band or len(found) != 16:
        raise ValueError("complete16 TRAIN moving-band coverage differs")
    return dict(count=16, cache_npz_sha256=CACHE_SHA, records=found,
        model_input_ax_min_max=[min(r["model_input_ax_mps2"] for r in found), max(r["model_input_ax_mps2"] for r in found)],
        future_mean_acceleration_0p3_min_max=[min(r["future_mean_acceleration_first_0p3_mps2"] for r in found),
                                            max(r["future_mean_acceleration_first_0p3_mps2"] for r in found)],
        target_delta_0p3_mean=statistics.fmean(r["target_delta_at_0p3_mps"] for r in found),
        selected_counts=dict(Counter(r["parent_selected"] for r in found)), neural_forwards=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh audit output required")
    result = dict(schema="portable_e2e.ah_actual_speed_cap_trace.v1",
        controller_source_sha256=sha(ROOT/"learning/portable_e2e/simulation_research.py"),
        runs=[trace(ROOT/"storage/experiments/validation/2026-10-01"/name/"episode.partial") for name in RUNS],
        train_band=train_band(), optimizer_steps=0, live_runs_launched=0, training_data_approved=False,
        cap_binding_is_not_a_causal_stall_proof=True,
        observed_api_acceleration_not_replaced_with_velocity_difference=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")
    print(json.dumps(dict(output=str(args.output), runs=[{key: row[key] for key in
        ("run", "clipped_preview_inferences_before_switch", "changed_throttle_inferences_before_switch", "first_cap_binding_s")}
        for row in result["runs"]], train_band_count=result["train_band"]["count"])))


if __name__ == "__main__":
    main()
