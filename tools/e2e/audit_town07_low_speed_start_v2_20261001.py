"""HH_261001 - Retain and audit every v2 expert future, all images and exact original nine bounds; no admission."""

import argparse
import gzip
import hashlib
import inspect
import json
from pathlib import Path

from scripts.e2e import audit_town07_low_speed_start_20261001 as previous
from scripts.e2e.collect_town07_low_speed_start_v2_20261001 import PROFILE


raw, geometry, targets = previous.raw, previous.geometry, previous.targets
sha, rows = previous.sha, previous.rows


def summarize_windows(futures):
    assessed = [row["discrete_bound_diagnostic"] for row in futures if "discrete_bound_diagnostic" in row]
    return dict(raw_anchors=len(futures), full_windows=sum(row.get("valid_future_points") == 64 for row in futures),
                bound_assessed=len(assessed), nine_bounds=targets.summarize_samples(assessed, 64))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for module in (raw, geometry, targets):
        if sha(Path(module.__file__)) != previous.PINS[Path(module.__file__).name]:
            raise ValueError("original raw future audit helper source differs")
    episode = args.run / "episode"
    if not episode.exists():
        episode = args.run / "episode.partial"
    manifest = json.loads((episode / "manifest.json").read_text())
    profile = manifest["capture_contract"]["low_speed_start_research"]["profile"]
    expected = dict(profile_id=PROFILE, normal_brake_cap=0., desired_deceleration_mps2=.5,
                    low_speed_zero_steering_mps=.75, goal_tolerance_m=1., stopped_speed_mps=.1, hold_seconds=2.)
    if any(profile.get(key) != value for key, value in expected.items()):
        raise ValueError("exact combined v2 expert protocol required")
    states, cameras = rows(episode / "states.jsonl"), rows(episode / "camera_frames.jsonl")
    commands = rows(episode / "expert_launch_control.jsonl")
    route = json.loads((episode / "route.json").read_text())
    hashes = {name: sha(episode / name) for name in
              ("manifest.json", "states.jsonl", "camera_frames.jsonl", "route.json", "expert_launch_control.jsonl")}
    timeline = raw.measured_timeline(states)
    pixel_audit, image_rows, anchor_rows = raw.camera_audit(episode, manifest, cameras, timeline)
    future_summary, future_rows = geometry.geometry_futures(timeline, cameras, route, anchor_rows, args.run.name)
    state_by_frame = {row["frame"]: row for row in states}
    low = [row for row in future_rows if row["capture_phase"] == "driving"
           and row.get("valid_future_points") == 64
           and abs(row["current_recorded_velocity_base_mps"][0]) <= .5]
    initial = [row for row in low if state_by_frame[row["frame"]]["route_progress_m"] < 2.]
    phases = {phase: summarize_windows([row for row in future_rows if row["capture_phase"] == phase])
              for phase in sorted({row["capture_phase"] for row in future_rows})}
    source_paths = {Path(__file__).resolve(), Path(previous.__file__).resolve(),
                    Path(raw.__file__).resolve(), Path(geometry.__file__).resolve(), Path(targets.__file__).resolve()}
    functions = (raw.measured_timeline, raw.future_state, raw.camera_audit, geometry.geometry_futures,
                 targets.audit_sample, targets.summarize_samples, raw.adapter._relative_xy_yaw,
                 raw.adapter._interpolate_state, raw.contract._canonical_route_in_base)
    source_paths.update(Path(inspect.getsourcefile(function)).resolve() for function in functions)
    source_proof = dict(files={str(path): sha(path) for path in sorted(source_paths)},
        functions={function.__module__ + "." + function.__name__: hashlib.sha256(inspect.getsource(function).encode()).hexdigest()
                   for function in functions})
    summary = dict(schema="carla.town07_low_speed_start_raw_audit.v2", source_hashes=source_proof,
        input_sha256=hashes, capture_status=manifest["status"], capture_error=manifest.get("error"),
        original_scalar_quality=manifest.get("result", {}).get("goal_stop_quality"),
        full_nine_bound_metrics=list(targets.METRICS), all_raw_futures=future_summary,
        phase_windows=phases, low_speed_driving=summarize_windows(low), initial_start=summarize_windows(initial),
        initial_start_definition="driving anchor with collector route progress <2m and abs(vx)<=0.5; no row is removed",
        camera_pixel_audit=pixel_audit, state_count=len(states),
        observed_ack_alignment_failures=sum(row.get("control_transport", {}).get("status") != "PASS" for row in states),
        logged_driving_commands=len(commands), acknowledged_commands=sum(row["ack_receipt"]["server_accepted"] for row in commands),
        commands_with_current_normal_policy=sum(row["normal_policy"] is not None for row in commands),
        low_speed_steering_interventions=sum(row["normal_policy"] is not None
            and row["normal_policy"]["low_speed_steering_intervention"] for row in commands),
        changed_by_downstream_basic_agent_or_stopped_steer_guard=sum(row["normal_policy"] is not None
            and row["normal_policy"]["profile_normal_control"] != row["final_basic_agent_proposal"] for row in commands),
        collision_events=sum(len(row["collision"]) for row in states),
        lane_invasion_events=sum(len(row["lane_invasion"]) for row in states),
        warmup_used_as_successful_start=False, restart_demonstration=False, learned_control=False,
        combined_protocol_not_controlled_ab=True, causal_resolution_claimed=False,
        raw_labels_rewritten=False, common10_samples_created=False, training_data_approved=False,
        limitation="Every raw window retained; nine unchanged necessary bounds, scalar QA and ACK checks do not constitute admission.")
    if any(sha(episode / name) != digest for name, digest in hashes.items()):
        raise ValueError("raw recording changed during independent analysis")
    args.output.mkdir(parents=True, exist_ok=False)
    for filename, records in (("all_raw_futures.jsonl.gz", future_rows), ("all_image_checks.jsonl.gz", image_rows)):
        with gzip.open(args.output / filename, "wt") as stream:
            for row in records:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
    with (args.output / "summary.json").open("x") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key: value for key, value in summary.items()
                     if key not in ("source_hashes", "original_scalar_quality", "camera_pixel_audit")}, indent=2))


if __name__ == "__main__":
    main()
