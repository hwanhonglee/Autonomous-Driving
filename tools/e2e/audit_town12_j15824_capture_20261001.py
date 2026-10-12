#!/usr/bin/env python3
"""HH_261001 - Offline, read-only raw Town12 qualification; never admit or filter data."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

from portable_e2e import runtime_contract
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_future_actor_lane_containment as lane
from scripts.e2e import capture_town12_j15824_spawn_lift_20261001 as capture


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_section(function, *args):
    try:
        return function(*args)
    except Exception as error:
        return {"status": "UNSUPPORTED_OR_INVALID", "error": f"{type(error).__name__}: {error}"}


def cleanup_observation(trial):
    """HH_261001 - Observe process-group extinction and all three TCP/UDP ports; never signal anything."""
    started_path, stopped_path = trial / "owner_started.json", trial / "lifecycle/stopped.json"
    started, stopped = json.loads(started_path.read_text()), json.loads(stopped_path.read_text())
    pid, pgid = int(started["editor_pid"]), int(started["editor_pgid"])
    require(pid == pgid and pid > 1 and stopped["owner_pid"] == stopped["owner_pgid"] == pid
            and stopped["generation_id"] == f"town12_source_probe_{pid}", "owner PID/PGID generation binding mismatch")
    members = []
    for directory in Path("/proc").iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields = (directory / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == pgid:
                members.append({"pid": int(directory.name), "state": fields[0], "start_time_ticks": int(fields[19])})
        except (FileNotFoundError, ProcessLookupError):
            continue
    ports = []
    for protocol in ("tcp", "tcp6", "udp", "udp6"):
        for line in (Path("/proc/net") / protocol).read_text().splitlines()[1:]:
            fields = line.split()
            if int(fields[1].rsplit(":", 1)[1], 16) in (2100, 2101, 2102):
                ports.append({"protocol": protocol, "local_address_hex": fields[1], "state_hex": fields[3], "inode": fields[9]})
    owner_absent = not Path(f"/proc/{pid}").exists()
    return {"status": "PASS_CURRENTLY_EXTINCT" if owner_absent and not members and not ports else "FAIL",
        "checked_at_utc": datetime.now(timezone.utc).isoformat(), "editor_pid": pid, "editor_pgid": pgid,
        "generation_id": stopped["generation_id"], "owner_process_absent": owner_absent,
        "remaining_group_members": members, "bound_tcp_udp_port_rows": ports,
        "port_triplet": [2100, 2101, 2102], "read_only": True, "processes_signalled": False,
        "owner_started_sha256": sha(started_path), "original_stopped_receipt_sha256": sha(stopped_path),
        "original_stopped_receipt_status": stopped["status"],
        "interpretation": "New observation only; it does not rewrite the original owner cleanup flag or prove its earlier return value was correct."}


def continuous_motion(states):
    """HH_261001 - Keep every uninterrupted >0.5 m/s driving run; never stitch across stops/gaps."""
    runs, current = [], []
    def finish():
        if current:
            runs.append({"first_frame": current[0]["frame"], "last_frame": current[-1]["frame"],
                "sample_count": len(current), "duration_s": current[-1]["timestamp"] - current[0]["timestamp"],
                "minimum_speed_mps": min(math.hypot(row["vx"], row["vy"]) for row in current),
                "maximum_speed_mps": max(math.hypot(row["vx"], row["vy"]) for row in current)})
            current.clear()
    for row in states:
        moving = row["capture_phase"] == "driving" and math.hypot(row["vx"], row["vy"]) > .5
        contiguous = not current or (row["frame"] == current[-1]["frame"] + 1
            and abs(row["timestamp"] - current[-1]["timestamp"] - .05) <= 5e-7)
        if not moving or not contiguous:
            finish()
        if moving:
            current.append(row)
    finish()
    longest = max((row["duration_s"] for row in runs), default=0.)
    return {"status": "PASS" if longest >= 30 else "FAIL", "threshold_mps_strictly_greater_than": .5,
        "required_continuous_seconds": 30, "longest_continuous_seconds": longest, "all_runs": runs,
        "all_driving_speed_range_kmh": [min((math.hypot(r["vx"], r["vy"]) * 3.6 for r in states
            if r["capture_phase"] == "driving"), default=0), max((math.hypot(r["vx"], r["vy"]) * 3.6 for r in states
            if r["capture_phase"] == "driving"), default=0)], "thirty_kmh_qualification_claimed": False}


def frozen_contract(plan, manifest, route, scout_receipt):
    trial = plan["signal_expert_trial"]
    args = capture.collector.parse_args(trial["collector_arguments"])
    actual = manifest["capture_contract"]
    checks = {
        "twenty_kmh": trial["target_speed_kmh"] == args.target_speed_kmh == actual["target_speed_kmh"] == 20,
        "native_and_camera_clocks": args.physics_hz == actual["physics_hz"] == 20 and args.capture_hz == actual["camera_hz"] == 10,
        "spawn_height_as_planned": args.spawn_z_offset_m == actual["spawn_z_offset_m"],
        "signals_observed_not_ignored": args.observe_traffic_lights and trial["traffic_lights_ignored"] is False,
        "map_load_disabled": args.allow_map_load is False and actual["client_map_loading_allowed"] is False,
        "source_xodr": args.expected_live_xodr_sha256 == scout_receipt["xodr_sha256"] == capture.scout.XODR_SHA256,
        "same_run_scout": scout_receipt["status"] == "PASS" and scout_receipt["hold_wall_seconds"] >= 10,
        "route": capture.scout.route_digest(route["route"]) == capture.scout.ROUTE_SHA256,
        "acknowledged_transport": args.control_transport == actual["control_transport"]["mode"] == "acknowledged_batch",
        "camera_specs_as_planned": manifest["cameras"] == trial["camera_contract"]["applied_specs"],
        "full_goal_and_tail": manifest.get("result", {}).get("goal_reached") is True
            and manifest.get("result", {}).get("capture_phases", {}).get("stationary_tail", {}).get("state_count") == 130,
        "no_collision_or_lane_invasion_events": manifest.get("result", {}).get("collision_event_count") == 0
            and manifest.get("result", {}).get("lane_invasion_event_count") == 0,
    }
    runtime_plan = manifest.get("town12_signal_trial", {}).get("runtime_plan_checks", [])
    checks["runtime_lane_sequence_and_polyline"] = bool(runtime_plan) and all(
        row.get("runtime_center_polyline_sha256") == "c26405dad67d929faa1a45463a3c9ce9890b921fc85222a8e017bfc525ef349c"
        and row.get("lane_sequence") == [[588, 0, 1], [15838, 0, 1], [15851, 0, 1], [270, 0, 1]]
        for row in runtime_plan)
    return {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "failed_checks": [name for name, passed in checks.items() if not passed]}


def camera_audit(episode, manifest, cameras, timeline):
    applied = capture.camera_contract()["applied_specs"]
    require(manifest.get("cameras") == applied, "actual camera geometry/appearance differs from the fixed explicit variant")
    alignment = raw.ack_audit.analyze_camera_alignment([row for _, row in timeline], cameras)
    by_frame = {row["frame"]: (stamp, row) for stamp, row in timeline}
    anchors, images, references, inodes = [], [], set(), set()
    for camera in cameras:
        frame, stamp = camera["frame"], raw.adapter._timestamp_ns(camera["timestamp"], "camera timestamp")
        source = by_frame.get(frame)
        timestamps = camera.get("source_timestamps", {})
        same = (source is not None and source[0] == stamp and source[1]["capture_phase"] == camera["capture_phase"]
            and set(timestamps) == set(raw.contract.CAMERA_ORDER)
            and all(abs(raw.adapter._timestamp_ns(value, "sensor timestamp") - stamp) <= 1 for value in timestamps.values()))
        anchors.append({"frame": frame, "timestamp_ns": stamp, "capture_phase": camera["capture_phase"],
                        "same_recorded_frame_and_timestamp": same})
        require(tuple(camera["camera_order"]) == raw.contract.CAMERA_ORDER
            and set(camera["images"]) == set(raw.contract.CAMERA_ORDER), "six-camera role coverage changed")
        for name, relative in camera["images"].items():
            require(relative == f"images/{name}/{frame:08d}.jpg" and relative not in references,
                    "reused or misplaced camera image reference")
            path = raw.contract._safe_file(episode, relative, "raw camera image")
            inode = (path.stat().st_dev, path.stat().st_ino)
            require(inode not in inodes, "camera image inode is reused")
            references.add(relative)
            inodes.add(inode)
            images.append({**raw.decode_image(episode, relative, (640, 360)), "camera": name,
                "frame": frame, "capture_phase": camera["capture_phase"], "referenced": True})
    found = {str(path.relative_to(episode)) for path in (episode / "images").rglob("*.jpg")}
    for relative in sorted(found - references):
        images.append({**raw.decode_image(episode, relative, (640, 360)), "referenced": False})
    passing = (alignment["status"] == "PASS" and bool(images) and references == found
               and all(row["decoded"] for row in images) and all(row["same_recorded_frame_and_timestamp"] for row in anchors))
    return {"status": "PASS" if passing else "FAIL", "alignment": alignment,
        "all_jpeg_count": len(images), "decoded_jpeg_count": sum(row["decoded"] for row in images),
        "unreferenced_jpeg_count": len(found - references), "camera_contract": capture.camera_contract(),
        "appearance_visual_review": "PENDING: decoding and calibration do not prove visible roads or correct lighting"}, images, anchors


def runtime_windows(timeline, cameras, future_rows):
    stamps = [stamp for stamp, _ in timeline]
    by_frame = {row["frame"]: (stamp, row) for stamp, row in timeline}
    target_rows = {row["frame"]: row for row in future_rows}
    records = []
    for camera in cameras:
        frame = camera["frame"]
        row = {"frame": frame, "capture_phase": camera["capture_phase"], "status": "NOT_EVALUATED"}
        diagnostic = target_rows[frame]
        if diagnostic.get("valid_points") != 64:
            records.append({**row, "status": "INCOMPLETE_OR_CONTEXT_ONLY", "disposition": diagnostic["disposition"]})
            continue
        stamp, ego = by_frame[frame]
        xy, speeds, yaws = [], [], []
        for index in range(64):
            future, mode = raw.future_state(timeline, stamps, stamp + (index + 1) * 100_000_000)
            require(future is not None, "full future classification disagrees with actual measured state")
            relative, yaw = raw.adapter._relative_xy_yaw(ego, future)
            xy.append(relative)
            speeds.append(math.hypot(future["vx"], future["vy"]))
            yaws.append(yaw)
        steps = diagnostic["diagnostic"]["steps"]
        violated = sorted({name for step in steps for name, value in step["metrics"].items() if value[1]})
        row.update(nine_metric_violations=violated, nine_metrics_pass=not violated,
            current_longitudinal_speed_mps=ego["vx"], minimum_relative_yaw_rad=min(yaws),
            strong_right_45deg=min(yaws) <= -math.pi / 4,
            definition="At least 45 degrees clockwise measured actor yaw within the exact 6.4 s future; no label clipping.")
        try:
            # HH_261001 - Duplicate the same measured trajectory into six slots; numerical runtime defaults stay unchanged.
            runtime_contract.validate_and_select_trajectory([xy for _ in range(6)], [speeds for _ in range(6)],
                [0.] * 6, current_speed_mps=float(ego["vx"]))
            row["status"] = "PASS"
        except runtime_contract.ContractError as error:
            row.update(status="FAIL", runtime_failure=str(error))
        row["strong_right_nine_metric_runtime64_candidate"] = (row["strong_right_45deg"] and not violated and row["status"] == "PASS")
        records.append(row)
    return {"status": "DIAGNOSED", "all_anchor_count": len(records),
        "runtime_status_counts": dict(Counter(row["status"] for row in records)),
        "strong_right_45deg_full64_count": sum(row.get("strong_right_45deg", False) for row in records),
        "strong_right_nine_metric_runtime64_candidate_count": sum(row.get("strong_right_nine_metric_runtime64_candidate", False) for row in records),
        "runtime_gate_defaults": asdict(runtime_contract.RuntimeGateConfig()),
        "all_failing_and_incomplete_windows_retained": True}, records


def signal_audit(states, signals, manifest, scout_receipt, carla):
    require(states and signals, "no observed signal/body frames")
    by_frame = {row["frame"]: row for row in signals}
    require(len(by_frame) == len(signals), "duplicate signal journal frames")
    box, _ = lane._body_box(manifest, carla)
    stop = scout_receipt["controlling_signal"]["selected_stop_waypoint"]
    stop_x, stop_y, _ = stop["xyz_carla_m"]
    angle = math.radians(stop["yaw_carla_deg"])
    previous, crossed, observations, failures = None, False, [], []
    for state in states:
        observed = by_frame.get(state["frame"])
        require(observed is not None, "a native state has no same-frame controlling-signal record")
        require(observed == state["traffic_light_observation"]["town12_signal"], "embedded and separate signal journals disagree")
        require(abs(state["timestamp"] - observed["timestamp"]) < 1e-6, "signal/body timestamps disagree")
        pose = state["actor_snapshot_transform_carla"]
        transform = carla.Transform(carla.Location(**{key: pose[key] for key in ("x", "y", "z")}),
                                    carla.Rotation(**{key: pose[key] for key in ("roll", "pitch", "yaw")}))
        vertices = box.get_world_vertices(transform)
        distance = min((stop_x - point.x) * math.cos(angle) + (stop_y - point.y) * math.sin(angle) for point in vertices)
        current = {**observed, "front_bumper_stop_distance_m": distance,
                   "speed_mps": math.sqrt(sum(value * value for value in state["world_velocity_carla"]))}
        crossed, failure = capture.base.signal_verdict(current, previous, crossed)
        if (observed["basic_agent_ignore_traffic_lights"] or observed["signal_opendrive_id"] != capture.scout.SIGNAL_ID
                or observed["signal_actor_id"] != scout_receipt["controlling_signal"]["actor_id"]):
            failure = "controlling light identity or ignored-light policy changed"
        if failure or observed.get("failure"):
            failures.append({"frame": state["frame"], "failure": failure or observed["failure"]})
        observations.append({"frame": state["frame"], "signal_state": observed["signal_state"],
            "minimum_any_body_corner_stop_plane_distance_m": distance, "recorded_front_corner_distance_m": observed["front_bumper_stop_distance_m"],
            "crossed": crossed, "failure": failure, "basic_agent_red_hazard": observed["basic_agent_red_hazard"]})
        previous = current
    extras = sorted(set(by_frame) - {row["frame"] for row in states})
    return {"status": "PASS_TRIGGER_BOUNDARY_ONLY" if not failures and crossed and not extras else "FAIL_OR_INCOMPLETE",
        "crossing_seen": crossed, "failures": failures, "extra_uncommitted_signal_frames": extras,
        "observations": observations, "signal_state_counts": dict(Counter(row["signal_state"] for row in signals)),
        "painted_stop_line_compliance_proven": False,
        "limitation": "Trigger-volume stop boundary only; painted stop-line alignment remains separate evidence."}


def run(trial: Path, output: Path, expected_owner_plan_sha256: str) -> dict:
    trial, output = trial.resolve(strict=True), output.absolute()
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(trial),
            "fresh audit output must be outside immutable raw trial")
    ledger = {}
    source_before = {str(Path(module.__file__).resolve()): sha(Path(module.__file__).resolve())
        for name, module in list(sys.modules.items()) if name.startswith(("scripts.e2e.", "portable_e2e."))
        and getattr(module, "__file__", None) and Path(module.__file__).suffix == ".py"}
    source_before[str(Path(__file__).resolve())] = sha(Path(__file__).resolve())
    def read(relative, *, lines=False, optional=False):
        path = trial / relative
        if optional and not path.exists():
            return [] if lines else None
        ledger[relative] = sha(path)
        return [json.loads(line) for line in path.read_text().splitlines()] if lines else json.loads(path.read_text())
    plan, owner = read("owner_plan.json"), read("owner_result.json")
    require(ledger["owner_plan.json"] == expected_owner_plan_sha256, "owner plan hash mismatch")
    cleanup = cleanup_observation(trial)
    require(cleanup["status"] == "PASS_CURRENTLY_EXTINCT", "owner process/group or one of its three TCP/UDP ports is still present")
    require(owner.get("source_bytes_unchanged_and_archived") is True, "owner frozen-source verification failed")
    for name, expected in plan["source_sha256"].items():
        require(sha(trial / "provenance" / name) == expected, f"frozen source archive changed: {name}")
        ledger[f"provenance/{name}"] = expected
    candidates = [path for path in (trial / "capture/episode", trial / "capture/episode.partial") if path.is_dir()]
    require(len(candidates) <= 1, "multiple complete/partial episodes are ambiguous")
    episode = candidates[0] if candidates else trial / "capture/episode"
    prefix = str(episode.relative_to(trial))
    manifest = read(prefix + "/manifest.json") if candidates else {
        "status": "NO_EPISODE", "error": owner.get("capture", {}).get("error"), "result": {}}
    route = read(prefix + "/route.json") if candidates else read("scout/route.json")
    states = read(prefix + "/states.jsonl", lines=True, optional=True)
    cameras = read(prefix + "/camera_frames.jsonl", lines=True, optional=True)
    receipts = read(prefix + "/control_receipts.jsonl", lines=True, optional=True)
    signals = read("capture/signal_frames.jsonl", lines=True, optional=True)
    scout_receipt = read("scout/scout_result.json")
    report = {"schema": "town12.j15824.raw_offline_qualification.20261001.v1", "status": "DIAGNOSED_NOT_ADMITTED",
        "owner_status": owner["status"], "episode_status": manifest["status"], "raw_state_count": len(states),
        "original_owner_cleanup_flag": owner.get("owned_processes_stopped_and_ports_released"),
        "independent_later_cleanup_observation": cleanup,
        "camera_bundle_count": len(cameras), "manifest_error": manifest.get("error"),
        "raw_capture_result": manifest.get("result"), "trial": str(trial), "episode": str(episode),
        "owner_plan_sha256": expected_owner_plan_sha256, "frozen_trial": plan["signal_expert_trial"],
        "training_data_admitted": False, "simulator_contacted": False, "raw_files_modified": False,
        "window_filtering_or_deletion_performed": False,
        "admission_blockers": ["Versioned explicit postprocess-enabled dataset adapter required; legacy adapter rejects this appearance.",
            "Visual RGB review pending; pixels decoding does not prove realistic road appearance.",
            "Prospective admitted-anchor membership and split admission are not fixed by this raw collection plan.",
            "Painted stop-line alignment is not proven by the controlling trigger boundary.",
            "Any map-vs-old-data A/B also changes postprocess appearance; map-only causality is unsupported."]}
    require(capture.scout.route_digest(route["route"]) == capture.scout.ROUTE_SHA256, "captured route changed")
    report["frozen_plan_requirements"] = checked_section(frozen_contract, plan, manifest, route, scout_receipt)
    report["continuous_motion"] = continuous_motion(states)
    images, future_rows, runtime_rows = [], [], []
    if states:
        timeline = raw.measured_timeline(states)
        bootstrap = manifest["capture_contract"]["control_transport"].get("bootstrap_observation")
        report["acknowledged_controls"] = checked_section(raw.ack_audit.analyze_transport, states, receipts, bootstrap)
        report["snapshot_kinematics"] = checked_section(raw.snapshot_kinematics, states, 2.85)
        camera_result = checked_section(camera_audit, episode, manifest, cameras, timeline)
        if isinstance(camera_result, tuple):
            report["cameras"], images, anchors = camera_result
            future_result = checked_section(raw.measured_future_audit, timeline, cameras, route, anchors)
            if isinstance(future_result, tuple):
                report["nine_motion_bounds"], future_rows = future_result
                report["runtime64"], runtime_rows = runtime_windows(timeline, cameras, future_rows)
            else:
                report["nine_motion_bounds"] = future_result
        else:
            report["cameras"] = camera_result
        # HH_261001 - Offline source Map/geometry only; no Client, World, socket or simulator process is created.
        sys.path[:0] = [str(capture.scout.SOURCE_EGG), str(capture.scout.SOURCE_PLANNER.parents[2])]
        carla, _ = capture.scout.source_api()
        report["body_and_bounded_sweep"] = checked_section(lambda: lane.audit_episode(episode, carla_module=carla))
        report["signals"] = checked_section(signal_audit, states, signals, manifest, scout_receipt, carla)
    else:
        report["admission_blockers"].insert(0, "No recorded actor states or camera payload; this is not training data.")
    for name in ("frozen_plan_requirements", "acknowledged_controls", "cameras", "body_and_bounded_sweep"):
        if report.get(name, {}).get("status") != "PASS":
            report["admission_blockers"].append(f"{name} has not passed.")
    if report["continuous_motion"]["status"] != "PASS":
        report["admission_blockers"].append("No uninterrupted >0.5 m/s driving run lasting 30 seconds.")
    if report.get("runtime64", {}).get("strong_right_nine_metric_runtime64_candidate_count", 0) == 0:
        report["admission_blockers"].append("No measured strong-right full64 window passes all nine bounds and the exact runtime gate.")
    if report.get("signals", {}).get("status") != "PASS_TRIGGER_BOUNDARY_ONLY":
        report["admission_blockers"].append("Full-body controlling-signal trigger-boundary compliance has not passed.")
    report["technical_candidate_before_unresolved_admission_requirements"] = (manifest["status"] == "complete"
        and all(report.get(name, {}).get("status") == "PASS" for name in ("frozen_plan_requirements", "acknowledged_controls", "cameras", "body_and_bounded_sweep", "continuous_motion"))
        and report.get("signals", {}).get("status") == "PASS_TRIGGER_BOUNDARY_ONLY"
        and report.get("runtime64", {}).get("strong_right_nine_metric_runtime64_candidate_count", 0) > 0)
    for image in images:
        require(sha(episode / image["path"]) == image["sha256"], "raw JPEG changed during audit")
    for relative, expected in ledger.items():
        require(sha(trial / relative) == expected, "raw metadata or frozen source changed during audit")
    for path, expected in source_before.items():
        require(sha(Path(path)) == expected, "audit source changed while running")
    report["input_sha256"] = ledger
    report["audit_source_sha256"] = {str(Path(module.__file__).resolve()): sha(Path(module.__file__).resolve())
        for name, module in list(sys.modules.items()) if name.startswith(("scripts.e2e.", "portable_e2e."))
        and getattr(module, "__file__", None) and Path(module.__file__).suffix == ".py"}
    report["audit_source_sha256"][str(Path(__file__).resolve())] = sha(Path(__file__).resolve())
    output.mkdir(parents=True, exist_ok=False)
    capture.scout.write_new(output / "summary.json", report)
    capture.scout.write_new(output / "independent_cleanup_observation.json", cleanup)
    for filename, rows in (("jpeg_audit.jsonl", images), ("all_future_windows.jsonl", future_rows), ("all_runtime64_windows.jsonl", runtime_rows)):
        with (output / filename).open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--owner-plan-sha256", required=True)
    args = parser.parse_args(argv)
    report = run(args.trial, args.output, args.owner_plan_sha256)
    print(json.dumps({key: report[key] for key in ("status", "raw_state_count", "camera_bundle_count",
        "technical_candidate_before_unresolved_admission_requirements", "training_data_admitted", "admission_blockers")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
