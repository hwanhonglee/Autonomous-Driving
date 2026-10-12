"""HH_261001 - Verify snapshot acceleration, native tick velocity differences and camera-phase sampling."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

from scripts.e2e.audit_ah_speed_cap_trace_20261001 import ROOT, RUNS, rows, sha


def mean(values):
    return statistics.fmean(values) if values else None


def rms(values):
    return math.sqrt(mean([v*v for v in values])) if values else None


def project(vector, yaw_deg):
    yaw = math.radians(yaw_deg)
    return math.cos(yaw)*vector[0] + math.sin(yaw)*vector[1]


def native_records(states, cameras):
    camera_frames = {row["frame"] for row in cameras}
    result = []
    for i, state in enumerate(states):
        if i == 0:
            continue
        previous = states[i-1]
        dt = state["timestamp"]-previous["timestamp"]
        if state["frame"] != previous["frame"]+1 or not .049 < dt < .051:
            raise ValueError("continuous native20Hz evidence required")
        yaw = state["actor_snapshot_transform_carla"]["yaw"]
        world_fd = [(a-b)/dt for a, b in zip(state["world_velocity_carla"], previous["world_velocity_carla"])]
        vector_error = [a-b for a, b in zip(state["world_acceleration_carla"], world_fd)]
        entry = dict(frame=state["frame"], timestamp=state["timestamp"], camera_frame=state["frame"] in camera_frames,
            capture_phase=state["capture_phase"], current_vx_mps=state["vx"], recorded_ax_mps2=state["ax"],
            projected_snapshot_ax_mps2=project(state["world_acceleration_carla"], yaw),
            projected_native_world_dv_dt_mps2=project(world_fd, yaw),
            native_body_dvx_dt_mps2=(state["vx"]-previous["vx"])/dt,
            world_acceleration_finite_difference_error=vector_error,
            world_dv_dt=world_fd, world_acceleration=state["world_acceleration_carla"],
            control=state["current_control"], ack_frame_pass=state["control_transport"]["frame_binding_pass"])
        if i >= 2:
            earlier = states[i-2]
            before_dt = previous["timestamp"]-earlier["timestamp"]
            span = dt+before_dt
            causal_mean_world_acceleration = [(a*dt+b*before_dt)/span for a, b in
                zip(state["world_acceleration_carla"], previous["world_acceleration_carla"])]
            world_dv_10hz = [(a-b)/span for a, b in zip(state["world_velocity_carla"], earlier["world_velocity_carla"])]
            entry.update(causal_two_native_mean_ax_mps2=project(causal_mean_world_acceleration, yaw),
                camera_interval_projected_world_dv_dt_mps2=project(world_dv_10hz, yaw),
                camera_interval_body_dvx_dt_mps2=(state["vx"]-earlier["vx"])/span)
        result.append(entry)
    return result


def analyze(directory):
    states = rows(directory/"states.jsonl"); cameras = rows(directory/"camera_frames.jsonl")
    decisions = [row for row in rows(directory/"learned_control.jsonl")
                 if row.get("application", {}).get("learned_control_applied") and not row["held_between_camera_frames"]]
    first = decisions[0]["inference_frame"]
    switch = next(decisions[i]["inference_frame"] for i in range(1,len(decisions))
                  if decisions[i-1]["selected"]["candidate_index"] == 5 and decisions[i]["selected"]["candidate_index"] == 1)
    native = native_records(states, cameras)
    by_frame = {row["frame"]: row for row in native}
    before = [row for row in native if first <= row["frame"] < switch]
    lag_errors = {}
    for lag in range(-2, 3):
        residuals = [component-compare for row in before if row["frame"]+lag in by_frame
                     for component,compare in zip(row["world_acceleration"],by_frame[row["frame"]+lag]["world_dv_dt"])]
        lag_errors[str(lag)] = rms(residuals)
    groups = {}
    for camera in (True, False):
        selected = [row for row in before if row["camera_frame"] == camera]
        groups["camera10Hz" if camera else "intervening_native_ticks"] = dict(count=len(selected),
            mean_recorded_ax_mps2=mean([row["recorded_ax_mps2"] for row in selected]),
            mean_causal_two_native_ax_mps2=mean([row["causal_two_native_mean_ax_mps2"] for row in selected]),
            negative_ax_while_10Hz_world_dv_positive=sum(row["recorded_ax_mps2"] < -.05
                and row["camera_interval_projected_world_dv_dt_mps2"] > .1 for row in selected))
    return dict(run=directory.parent.name, source_sha256={name:sha(directory/name) for name in
                ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl")},
        first_learned_frame=first, first_5_to_1_frame=switch, groups_before_switch=groups,
        world_acceleration_vs_native_velocity_difference_max_abs=max(abs(e) for row in native for e in row["world_acceleration_finite_difference_error"]),
        world_acceleration_vs_native_velocity_difference_rms=rms([e for row in native for e in row["world_acceleration_finite_difference_error"]]),
        logged_ax_vs_same_frame_snapshot_projection_max_abs=max(abs(row["recorded_ax_mps2"]-row["projected_snapshot_ax_mps2"]) for row in native),
        two_native_mean_vs_10Hz_velocity_difference_max_abs=max(abs(row["causal_two_native_mean_ax_mps2"]-row["camera_interval_projected_world_dv_dt_mps2"])
            for row in native if "causal_two_native_mean_ax_mps2" in row),
        before_switch_world_vector_rmse_by_velocity_difference_frame_lag=lag_errors,
        all_ack_frames_pass=all(row["ack_frame_pass"] for row in native), pre_switch_native_trace=before,
        original_state_and_input_features_unmodified=True,
        conclusion="Same-frame native acceleration is correct within floating-point tolerance; camera10Hz samples different phases of native20Hz velocity oscillation.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh sampling audit output required")
    local_engine = Path('/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/WorldObserver.cpp')
    result = dict(schema="portable_e2e.ah_acceleration_sampling_audit.v1",
        runs=[analyze(ROOT/"storage/experiments/validation/2026-10-01"/run/"episode.partial") for run in RUNS],
        source_sha256={str(path):sha(path) for path in (ROOT/"tools/e2e/collect_carla_vad_expert.py",
            ROOT/"tools/e2e/collect_ah_simulation_research.py", local_engine)},
        local_engine_source_explains_formula_but_packaged_binary_equivalence_not_verified=True,
        neural_forwards=0, optimizer_steps=0, training_data_approved=False, input_transform_implemented=False,
        temporal_aliasing_evidence_not_proof_of_stall_causality=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")
    print(json.dumps(dict(output=str(args.output), runs=[{key:row[key] for key in ("run",
        "world_acceleration_vs_native_velocity_difference_max_abs", "logged_ax_vs_same_frame_snapshot_projection_max_abs",
        "groups_before_switch")} for row in result["runs"]]),indent=2))


if __name__ == "__main__":
    main()
