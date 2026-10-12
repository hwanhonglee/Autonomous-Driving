"""HH_261001 - Audit every raw expert start/stop future with unchanged nine bounds; never admit or rewrite data."""

import argparse
import gzip
import hashlib
import inspect
import json
from pathlib import Path

from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_carla_turn_launch_raw_geometry as geometry
from scripts.e2e import audit_portable_target_feasibility as targets


PINS = {"audit_carla_raw_pre_admission.py": "0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00",
    "audit_carla_turn_launch_raw_geometry.py": "32f1cd04669afc665d28cdf7040ee1517505a6543c3a420806a1816db06fbe49",
    "audit_portable_target_feasibility.py": "8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for module in (raw, geometry, targets):
        if sha(Path(module.__file__)) != PINS[Path(module.__file__).name]:
            raise ValueError("original raw future audit helper source differs")
    episode = args.run / "episode"
    if not episode.exists():
        episode = args.run / "episode.partial"
    manifest = json.loads((episode / "manifest.json").read_text())
    if manifest["capture_contract"]["low_speed_start_research"]["profile"]["profile_id"] != "town07_low_speed_start_20261001_v1":
        raise ValueError("exact fixed expert low-speed start profile required")
    states, cameras = rows(episode / "states.jsonl"), rows(episode / "camera_frames.jsonl")
    commands = rows(episode / "expert_launch_control.jsonl")
    route = json.loads((episode / "route.json").read_text())
    hashes = {name: sha(episode / name) for name in
              ("manifest.json", "states.jsonl", "camera_frames.jsonl", "route.json", "expert_launch_control.jsonl")}
    timeline = raw.measured_timeline(states)
    pixel_audit, image_rows, anchor_rows = raw.camera_audit(episode, manifest, cameras, timeline)
    future_summary, future_rows = geometry.geometry_futures(timeline, cameras, route, anchor_rows, args.run.name)
    start_rows = [row for row in future_rows if row["capture_phase"] == "driving"
                  and row.get("valid_future_points") == 64
                  and abs(row["current_recorded_velocity_base_mps"][0]) <= .5]
    start_diagnostics = [row["discrete_bound_diagnostic"] for row in start_rows if "discrete_bound_diagnostic" in row]
    source_paths = {Path(__file__).resolve(), Path(raw.__file__).resolve(), Path(geometry.__file__).resolve(), Path(targets.__file__).resolve()}
    functions = (raw.measured_timeline, raw.future_state, raw.camera_audit, geometry.geometry_futures,
                 targets.audit_sample, targets.summarize_samples, raw.adapter._relative_xy_yaw,
                 raw.adapter._interpolate_state, raw.contract._canonical_route_in_base)
    source_paths.update(Path(inspect.getsourcefile(function)).resolve() for function in functions)
    source_proof = dict(files={str(path): sha(path) for path in sorted(source_paths)},
                        functions={function.__module__ + "." + function.__name__: hashlib.sha256(inspect.getsource(function).encode()).hexdigest()
                                   for function in functions})
    summary = dict(schema="carla.town07_low_speed_start_raw_audit.v1", source_hashes=source_proof,
        input_sha256=hashes, capture_status=manifest["status"], capture_error=manifest.get("error"),
        original_scalar_quality=manifest.get("result", {}).get("goal_stop_quality"),
        full_nine_bound_metrics=list(targets.METRICS), all_raw_futures=future_summary,
        low_speed_driving_full_future_count=len(start_rows), low_speed_bound_assessed_count=len(start_diagnostics),
        low_speed_driving_nine_bounds=targets.summarize_samples(start_diagnostics, 64),
        camera_pixel_audit=pixel_audit, state_count=len(states),
        observed_ack_alignment_failures=sum(row.get("control_transport", {}).get("status") != "PASS" for row in states),
        logged_driving_commands=len(commands), acknowledged_commands=sum(row["ack_receipt"]["server_accepted"] for row in commands),
        commands_with_current_normal_policy=sum(row["normal_policy"] is not None for row in commands),
        changed_by_downstream_basic_agent_or_stopped_steer_guard=sum(
            row["normal_policy"] is not None and row["normal_policy"]["profile_normal_control"] != row["final_basic_agent_proposal"]
            for row in commands),
        collision_events=sum(len(row["collision"]) for row in states),
        lane_invasion_events=sum(len(row["lane_invasion"]) for row in states),
        warmup_used_as_successful_start=False, restart_demonstration=False, learned_control=False,
        raw_labels_rewritten=False, common10_samples_created=False, training_data_approved=False,
        limitation="Nine necessary discrete future bounds and source/ACK checks are reported; no threshold is weakened and none constitutes dataset admission.")
    if any(sha(episode / name) != digest for name, digest in hashes.items()):
        raise ValueError("raw recording changed during independent analysis")
    args.output.mkdir(parents=True, exist_ok=False)
    with gzip.open(args.output / "all_raw_futures.jsonl.gz", "wt") as stream:
        for row in future_rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    with gzip.open(args.output / "all_image_checks.jsonl.gz", "wt") as stream:
        for row in image_rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    with (args.output / "summary.json").open("x") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key: value for key, value in summary.items() if key not in ("source_hashes", "original_scalar_quality", "camera_pixel_audit")}, indent=2))


if __name__ == "__main__":
    main()
