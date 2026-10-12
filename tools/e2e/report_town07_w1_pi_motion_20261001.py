"""HH_261001 - Create-only ACK-bound motion audit; no simulator, GPU or approval mutation."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import gzip
import hashlib
import json
import math
from pathlib import Path

from portable_e2e.longitudinal_pi_research_20261001 import LongitudinalPIConfig, LongitudinalPIResearch
from scripts.e2e import audit_carla_acknowledged_control as ack


SCHEMA = "portable_e2e.town07_w1_pi_ack_bound_motion.20261001.v1"
PI_SOURCE_SHA256 = "bc15f390398c6b1c2e678b5c90896d4ac4612ea349451b4cdb530d3f0946665b"
ARMS = ("A1_original", "B1_pi_only")
POLICY_ARMS = ("A_original", "B_pi_only")
SCREEN = dict(net_progress_minimum_m=30., unintended_stationary_dwell_limit_s=1.,
              stationary_threshold_mps=.05, cte_p95_maximum_m=.35, cte_absolute_maximum_m=.75,
              collisions_maximum=0, lane_invasions_maximum=0, learned_rejected_commands_maximum=0,
              body_or_swept_lane_clearance_below_point_one_m_maximum=0)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def percentile(values, fraction):
    """HH_261001 - Use explicit linear order-statistic interpolation, not a hidden default."""
    values = sorted(values)
    require(bool(values) and 0 <= fraction <= 1, "nonempty bounded percentile required")
    position = (len(values) - 1) * fraction
    lo, hi = math.floor(position), math.ceil(position)
    return values[lo] + (position - lo) * (values[hi] - values[lo])


def statistics(values):
    values = list(values)
    require(values and all(math.isfinite(value) for value in values), "finite statistic required")
    return dict(count=len(values), minimum=min(values), mean=sum(values) / len(values),
                p50=percentile(values, .5), p95=percentile(values, .95), maximum=max(values))


def near(actual, expected, reason, tolerance=1e-8):
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and set(expected) <= set(actual), reason)
        for key, value in expected.items():
            near(actual[key], value, reason + "/" + key, tolerance)
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        require(not isinstance(actual, bool) and isinstance(actual, (int, float))
                and math.isfinite(actual) and abs(actual - expected) <= tolerance, reason)
    else:
        require(actual == expected, reason)


def stationary_runs(states, threshold):
    """HH_261001 - Retain all near-stationary observations with conservative one-tick bounds."""
    runs, current = [], []

    def close():
        if current:
            runs.append(dict(first_frame=current[0]["frame"], last_frame=current[-1]["frame"],
                             native_observations=len(current),
                             timestamp_span_s=current[-1]["timestamp"] - current[0]["timestamp"],
                             observation_cell_upper_bound_s=len(current) * .05))
            current.clear()

    for state in states:
        speed = math.hypot(*state["world_velocity_carla"][:2])
        if speed <= threshold:
            if current and state["frame"] != current[-1]["frame"] + 1:
                close()
            current.append(state)
        else:
            close()
    close()
    return dict(threshold_mps=threshold, speed_source="native actor world XY velocity norm",
                runs=runs, maximum_observation_cell_upper_bound_s=max(
                    (row["observation_cell_upper_bound_s"] for row in runs), default=0.),
                continuous_time_stationary_proven=False)


def command_audit(states, receipts, commands, arm):
    """HH_261001 - Separate outgoing annotations from next-tick measured effects and replay PI state."""
    require(arm in POLICY_ARMS, "fixed arm required")
    state_by_frame = {row["frame"]: row for row in states}
    receipt_by_sequence = {row["sequence"]: row for row in receipts}
    require(len(state_by_frame) == len(states) and len(receipt_by_sequence) == len(receipts),
            "duplicate native frame or ACK sequence")
    commands_by_sequence = {}
    learned, fresh, held, no_effect = [], [], [], []
    pi = LongitudinalPIResearch()
    previous_fresh = None
    stop_count = 0
    for command in commands:
        sequence = command["ack_receipt_sequence"]
        require(sequence not in commands_by_sequence, "duplicate learned command receipt")
        commands_by_sequence[sequence] = command
        receipt = receipt_by_sequence[sequence]
        require(receipt["server_accepted"] is True and receipt["status"] == "ACKNOWLEDGED"
                and receipt["do_tick"] is False and receipt["physical_actuation_proven"] is False,
                "command lacks its single accepted native ACK")
        if command["control_source"] != "learned_trajectory_tracker":
            pi.reset()
            continue
        require(command["application"]["learned_control_applied"] is True
                and command["application"]["acknowledged"] is True
                and command["application"]["vehicle_control_approved"] is False
                and command["emergency_reason"] is None, "learned command application differs")
        frame = command["command_frame"]
        state = state_by_frame[frame]
        require(receipt["before_frame"] == receipt["after_ack_frame"] == frame
                and state["control_transport"]["next_command_receipt_sequence"] == sequence,
                "outgoing learned ACK not bound to current state")
        near(state["next_control"], receipt["requested_control"], "next control/ACK", 1e-6)
        for key in ("throttle", "brake", "steer", "reverse", "hand_brake"):
            near(receipt["requested_control"][key], command["control"][key], "proposal/ACK " + key, 1e-6)
        effect = state_by_frame.get(frame + 1)
        if effect is None:
            no_effect.append(sequence)
        else:
            require(effect["control_transport"]["expected_receipt_sequence"] == sequence,
                    "next native snapshot is not bound to learned ACK")
            # HH_261001 - Preserve the transport's original automatic-gear exemption, not a new tolerance.
            expected_measured = dict(receipt["requested_control"])
            if expected_measured["manual_gear_shift"] is False:
                expected_measured.pop("gear")
            near(effect["current_control"], expected_measured, "ACK/measured control", 1e-6)
        audit = command["longitudinal_pi_research"]
        near(audit["gains"], asdict(LongitudinalPIConfig()), "PI gains")
        require(audit["arm"] == arm and audit["actuator_acknowledged"] is False
                and audit["camera_policy_timestep_s"] == .1
                and audit["held_control_native_timestep_s"] == .05,
                "PI policy scope or cadence differs")
        for key in ("feedforward_added", "throttle_floor_added", "selected_speed_or_path_changed",
                    "steering_changed", "stop_branch_changed", "emergency_authority_changed",
                    "control_cadence_changed"):
            require(audit[key] is False, "PI changed protected behavior " + key)
        original = audit["original_normal_tracker_control"]
        proposed = audit["proposed_normal_tracker_control"]
        near(command["control"], proposed, "audited policy return")
        for key in set(original) - {"throttle", "brake"}:
            near(proposed[key], original[key], "protected tracker field " + key)
        if command["held_between_camera_frames"]:
            require(previous_fresh is not None and frame == previous_fresh["command_frame"] + 1
                    and command["inference_frame"] == previous_fresh["inference_frame"],
                    "held native command lacks its real camera inference")
            for key in ("raw_output", "selected", "control", "longitudinal_pi_research", "total_seconds"):
                require(command[key] == previous_fresh[key], "held command changed/reintegrated " + key)
            held.append(command)
        else:
            require(command["inference_frame"] == frame
                    and audit["camera_policy_timestamp_ns"] == round(state["timestamp"] * 1e9),
                    "fresh inference/current observed frame differs")
            if previous_fresh:
                require(frame == previous_fresh["command_frame"] + 2
                        and abs(audit["camera_policy_timestamp_ns"]
                                - previous_fresh["longitudinal_pi_research"]["camera_policy_timestamp_ns"]
                                - 100_000_000) <= 1_000, "fresh camera policy cadence differs")
            desired = original["desired_speed_mps"]
            if arm == "B_pi_only":
                near(audit["observed_speed_mps"] if desired > .05 else state["vx"],
                     state["vx"], "PI uses current measured forward base-frame velocity")
                request, expected = pi.update(desired, state["vx"])
                near(audit, expected, "PI independent replay")
                if request is not None:
                    near(proposed, request, "PI throttle/brake replay")
                else:
                    near(proposed, original, "original stop return")
            else:
                require(audit["integral_before"] == audit["integral_after"] == 0.,
                        "A proportional arm carried an integral")
                near(proposed, original, "A unchanged original tracker")
            near(audit["committed_integral_after"], pi.integral_error, "committed PI state")
            stop_count += desired <= .05
            fresh.append(command)
            previous_fresh = command
        learned.append(command)
    effect_states = [row for row in states if commands_by_sequence.get(
        row["control_transport"]["expected_receipt_sequence"], {}).get("control_source")
        == "learned_trajectory_tracker"]
    require(learned and fresh, "learned command/effect evidence absent")
    return dict(learned_commands=learned, fresh_commands=fresh, effect_states=effect_states,
                summary=dict(learned_acknowledged_commands=len(learned), real_camera_inferences=len(fresh),
                             held_native_commands=len(held), ack_matched_observed_effect_states=len(effect_states),
                             terminal_ack_without_following_snapshot_sequences=no_effect,
                             held_commands_exactly_unchanged=True, pi_replay_passed=True,
                             integral_updates_per_real_camera_only=True, observed_model_stop_targets=stop_count,
                             actual_emergency_commands=sum(row["control_source"] == "emergency_brake" for row in commands),
                             original_model_stop_branch_exercised=stop_count > 0,
                             original_emergency_branch_exercised=any(
                                 row["control_source"] == "emergency_brake" for row in commands),
                             actuator_ack_record_is_separate_from_policy_proposal=True,
                             pi_observation_source="unaltered current base-frame velocity_x_mps; not planar norm"))


def motion_diagnostics(states):
    """HH_261001 - Do not relabel finite differences as the native actor acceleration API."""
    diagnostics = []
    for previous, state in zip(states, states[1:]):
        dt = state["timestamp"] - previous["timestamp"]
        require(state["frame"] == previous["frame"] + 1 and abs(dt - .05) <= 1e-6,
                "motion diagnostic requires contiguous 20 Hz snapshot observations")
        planar = math.hypot(*state["world_velocity_carla"][:2])
        old_planar = math.hypot(*previous["world_velocity_carla"][:2])
        diagnostics.append(dict(frame=state["frame"], timestamp=state["timestamp"],
            native_rotated_actor_ax_mps2=state["ax"], native_rotated_actor_ay_mps2=state["ay"],
            finite_difference_base_vx_mps2=(state["vx"] - previous["vx"]) / dt,
            finite_difference_actor_planar_speed_mps2=(planar - old_planar) / dt,
            native_ax_sample_difference_jerk_mps3=(state["ax"] - previous["ax"]) / dt))
    for previous, row in zip(diagnostics, diagnostics[1:]):
        dt = row["timestamp"] - previous["timestamp"]
        row["finite_difference_actor_planar_speed_jerk_mps3"] = (
            row["finite_difference_actor_planar_speed_mps2"]
            - previous["finite_difference_actor_planar_speed_mps2"]) / dt
    return diagnostics


def analyze_run(run, body_root, arm):
    """HH_261001 - Rebind derived body proof to actual failed-source bytes and all archived code."""
    episode = run / "episode.partial"
    require(episode.is_dir() and not (run / "episode").exists(), "one retained failed episode required")
    files = [episode / name for name in ("manifest.json", "states.jsonl", "route.json", "control_receipts.jsonl",
              "camera_frames.jsonl", "active_map_opendrive.xodr", "learned_control.jsonl", "learned_summary.json")]
    files += [run / name for name in ("owner_plan.json", "owner_result.json", "declared_plan.json",
              "owner_diagnostic_result.json", "natural_port_release_receipt.json", "native_guard.jsonl")]
    files += [body_root / name for name in ("summary.json", "all_native_body_sweeps.jsonl.gz",
              "all_candidate_nine_bounds.jsonl.gz")]
    plan = json.loads((run / "declared_plan.json").read_text())
    owner_plan = json.loads((run / "owner_plan.json").read_text())
    require(sha(run / "declared_plan.json") == owner_plan["declaration_sha256"], "declaration pin differs")
    for key, expected in SCREEN.items():
        require(plan["prospective_motion_screen"][key] == expected, "prospective screen changed " + key)
    for relative, expected in plan["source_sha256"].items():
        archived = run / "source" / relative
        require(sha(archived) == expected, "archived source closure mismatch " + relative)
        files.append(archived)
    manifest = json.loads((episode / "manifest.json").read_text())
    require(manifest["status"] == "failed" and manifest["training_data_approved"] is False
            and manifest["vehicle_control_approved"] is False, "failed/unapproved source must remain unchanged")
    body_report = json.loads((body_root / "summary.json").read_text())
    for path, expected in body_report["source_sha256"].items():
        require(sha(Path(path)) == expected, "body audit input changed " + path)
    states, receipts, commands = [rows(episode / name) for name in
                                  ("states.jsonl", "control_receipts.jsonl", "learned_control.jsonl")]
    capture = manifest["capture_contract"]
    require(capture["physics_hz"] == 20 and capture["camera_hz"] == 10,
            "declared native/camera frequencies differ")
    transport = ack.analyze_transport(states, receipts, capture["control_transport"]["bootstrap_observation"], wheelbase=2.85)
    require(transport["status"] == "PASS", "native acknowledged-control reconstruction failed")
    audited = command_audit(states, receipts, commands, arm)
    learned, effects = audited["learned_commands"], audited["effect_states"]
    first_frame = learned[0]["command_frame"]
    state_by_frame = {row["frame"]: row for row in states}
    baseline = state_by_frame[first_frame]
    require(effects and effects[0]["frame"] == first_frame + 1,
            "first learned observed effect must follow, not precede, the handoff ACK")
    interval = [baseline, *effects]
    require(len({row["frame"] for row in interval}) == len(interval), "motion interval duplicates snapshot")
    progress = effects[-1]["route_progress_m"] - baseline["route_progress_m"]
    dwell = stationary_runs(effects, SCREEN["stationary_threshold_mps"])
    cte = statistics(abs(row["route_cte_m"]) for row in effects)
    with gzip.open(body_root / "all_native_body_sweeps.jsonl.gz", "rt") as stream:
        geometry = [json.loads(line) for line in stream]
    selected_geometry = [row for row in geometry if row["frame"] in {s["frame"] for s in effects}]
    require(len(selected_geometry) == len(effects), "body proof does not cover every learned observed effect")
    clearances, violations = [], 0
    for row in selected_geometry:
        values = [row["sampled_body"]["inset_10cm_lane"], row["previous_sweep"]["regions"]["inset_10cm_lane"]]
        for value in values:
            violations += not value["area_gate_clear"]
            if value["minimum_boundary_clearance_m"] is not None:
                clearances.append(value["minimum_boundary_clearance_m"])
    rejected = sum(row.get("control_source") == "emergency_brake" for row in commands)
    collisions = sum(len(row["collision"]) for row in effects)
    invasions = sum(len(row["lane_invasion"]) for row in effects)
    checks = dict(net_progress=progress >= 30., stationary=dwell["maximum_observation_cell_upper_bound_s"] < 1.,
                  cte_p95=cte["p95"] <= .35, cte_absolute=cte["maximum"] <= .75,
                  collision=collisions == 0, lane_invasion=invasions == 0,
                  learned_rejection=rejected == 0, body_and_sweeps_10cm_inset=violations == 0)
    cameras = rows(episode / "camera_frames.jsonl")
    image_paths = [episode / value for row in cameras for value in row["images"].values()]
    require(len(image_paths) == len(set(image_paths)) == len(cameras) * 6, "six real unique camera files required")
    files += image_paths
    diagnostics = motion_diagnostics(interval)
    fields = ("native_rotated_actor_ax_mps2", "native_rotated_actor_ay_mps2", "finite_difference_base_vx_mps2",
              "finite_difference_actor_planar_speed_mps2", "native_ax_sample_difference_jerk_mps3",
              "finite_difference_actor_planar_speed_jerk_mps3")
    summary = dict(arm=arm, original_manifest_status=manifest["status"], original_capture_error=manifest["error"],
        original_owner_result=json.loads((run / "owner_result.json").read_text()),
        original_owner_diagnostic=json.loads((run / "owner_diagnostic_result.json").read_text()),
        declaration_sha256=sha(run / "declared_plan.json"), source_checkpoint_sha256=owner_plan["source_checkpoint_sha256"],
        model_state_sha256=owner_plan["model_state_sha256"],
        additional_model_optimizer_updates_in_controller_experiment=owner_plan["model_optimizer_updates"],
        controller_only_experiment_updates_not_entire_weight_training_history=True,
        source_closure_archived_file_count=len(plan["source_sha256"]), native_state_count=len(states), camera_bundle_count=len(cameras),
        unique_real_camera_image_count=len(image_paths), native_ack_transport=transport, command_audit=audited["summary"],
        outgoing_learned_annotated_state_count=sum(row.get("research_control_source") == "learned_trajectory_tracker" for row in states),
        handoff_baseline_frame=first_frame, first_ack_matched_effect_frame=effects[0]["frame"], final_observed_frame=effects[-1]["frame"],
        elapsed_learned_motion_s=effects[-1]["timestamp"] - baseline["timestamp"],
        motion_net_progress_m=progress, cte_absolute_m=cte, stationary=dwell,
        measured_actor_planar_speed_mps=statistics(math.hypot(*row["world_velocity_carla"][:2]) for row in effects),
        observed_collisions=collisions, observed_lane_invasions=invasions, observed_learned_rejections=rejected,
        learned_effect_body_geometry=dict(sampled_count=len(selected_geometry), incoming_sweep_count=len(selected_geometry),
            inset_10cm_violations=violations, minimum_inset_interior_clearance_m=min(clearances)),
        all_400_snapshot_body_geometry=body_report["body_geometry"],
        selected_candidate_nine_bound_diagnostic=body_report["candidate_nine_bound_diagnostic"]["selected_candidates"],
        inference_and_policy_seconds=statistics(row["total_seconds"] for row in audited["fresh_commands"]),
        inference_deadline_violations=sum(row["total_seconds"] > .1 for row in audited["fresh_commands"]),
        sensor_to_ack_latency_measured=False, sensor_to_ack_latency_reason="No wall-clock camera arrival/ACK timestamp pair was recorded; simulated timestamps and inference duration cannot prove this latency.",
        acceleration_and_jerk={key: statistics(row[key] for row in diagnostics if key in row) for key in fields},
        kinematics_notice="Native ax/ay are unchanged actor snapshot acceleration rotated into local axes, not a verified physical COM/rear-axle acceleration. vx/vy use the legacy rear-shift velocity convention; rotating-frame finite differences and native acceleration are distinct. Jerk is a sample-difference diagnostic, not a sensor-measured physical jerk.",
        prospective_motion_screen=dict(status="PASS_PRECURSOR_ONLY" if all(checks.values()) else "FAIL_PRECURSOR_RETAINED",
            checks=checks, frozen_thresholds=SCREEN, prospective=True, official_RTE_01_pass=False,
            feature_approval=False, independent_replication=False, route_completion=False),
        interval_notice="Motion begins at the snapshot before the first learned ACK and ends at the final ACK-matched observed effect. Outgoing source annotations are not relabelled as prior actuator effects; the terminal ACK without a following tick is disclosed.")
    return summary, diagnostics, files


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, required=True)
    parser.add_argument("--body-audits", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    require(not args.output.exists(), "create-only output required")
    import portable_e2e.longitudinal_pi_research_20261001 as pi_module
    require(sha(Path(pi_module.__file__)) == PI_SOURCE_SHA256, "frozen PI replay implementation changed")
    # HH_261001 - Freeze bytes before parsing; a concurrently edited raw file cannot acquire a new matching pin.
    source_files = [Path(__file__), Path(pi_module.__file__), Path(ack.__file__)]
    for run_name, body_name in zip(ARMS, ("A_body", "B_body")):
        run = args.runs / run_name
        for directory in (run / "episode.partial", run / "source", run / "source_external",
                          args.body_audits / body_name):
            source_files += [path for path in directory.rglob("*") if path.is_file()]
        source_files += [path for path in run.iterdir() if path.is_file() and path.suffix in (".json", ".jsonl")]
    initial_hashes = {str(path): sha(path) for path in source_files}
    summaries, diagnostics, files = [], {}, [Path(__file__), Path(pi_module.__file__), Path(ack.__file__)]
    for run_name, arm, body_name in zip(ARMS, POLICY_ARMS, ("A_body", "B_body")):
        summary, per_state, source_files = analyze_run(args.runs / run_name, args.body_audits / body_name, arm)
        summaries.append(summary)
        diagnostics[arm] = per_state
        files += source_files
    require(summaries[0]["declaration_sha256"] == summaries[1]["declaration_sha256"]
            and summaries[0]["source_checkpoint_sha256"] == summaries[1]["source_checkpoint_sha256"]
            and summaries[0]["model_state_sha256"] == summaries[1]["model_state_sha256"],
            "A/B declaration, weights or model state differ")
    require(all(str(path) in initial_hashes for path in files), "analysis read an unpinned source")
    hashes = initial_hashes
    report = dict(schema=SCHEMA, status="COMPLETE_RESEARCH_DIAGNOSTIC_NOT_FEATURE_APPROVAL",
        arms=summaries, paired_attempts=2, attempts_per_arm=1, ABBA=False,
        net_progress_improvement_m=summaries[1]["motion_net_progress_m"] - summaries[0]["motion_net_progress_m"],
        training_data_approved=False, vehicle_control_approved=False, official_RTE_01_pass=False,
        independent_replication=False, original_failures_retained=True, raw_inputs_modified=False,
        physical_continuous_time_proven=False, source_sha256=hashes)
    require(all(sha(Path(path)) == digest for path, digest in hashes.items()), "source changed during analysis")
    args.output.mkdir(parents=True)
    with (args.output / "summary.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    for arm, per_state in diagnostics.items():
        with gzip.open(args.output / (arm + "_motion.jsonl.gz"), "xt") as stream:
            for row in per_state:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
    print(json.dumps({"status": report["status"], "arms": [{key: value[key] for key in
          ("arm", "motion_net_progress_m", "prospective_motion_screen", "command_audit")}
          for value in summaries]}, indent=2))


if __name__ == "__main__":
    main()
