#!/usr/bin/env python3
"""HH_260906 - Diagnose immutable raw expert pixels and measured futures without producing a training dataset."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace
import warnings

from PIL import Image, ImageFile

from portable_e2e import contract
from scripts.e2e import audit_carla_acknowledged_control as ack_audit
from scripts.e2e import audit_carla_brake_free_goal_stop as brake_free_audit
from scripts.e2e import audit_carla_comfortable_v3_trial as control_audit
from scripts.e2e import audit_portable_target_feasibility as targets
from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e import summarize_carla_goal_stop_trials as scalar

SCHEMA = "carla_expert.raw_pre_admission_diagnostic.v1"
ROOT = Path(__file__).resolve().parents[2]
PHASES = ("stationary_warmup", "driving", "stationary_tail")
FUNCTIONS = (adapter._causal_state, adapter._interpolate_state, adapter._relative_xy_yaw,
    adapter._rig_document, contract._canonical_route_in_base, targets.audit_sample,
    targets.summarize_samples, scalar.summarize_trial, scalar.analyze_native,
    control_audit.analyze_protocol, ack_audit.analyze_transport, ack_audit.analyze_camera_alignment,
    ack_audit.recompute_snapshot_state, ack_audit.audit_trial,
    brake_free_audit.verify_owner_sources, brake_free_audit.analyze_protocol, brake_free_audit.audit_trial)


def require(condition, message):
    scalar.require(condition, message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def source_identity():
    # HH_260906 - Whole-file and function-source pins describe executed diagnostics without claiming a clean commit.
    files = {Path(__file__).resolve(), contract.DEFAULT_CONTRACT_PATH.resolve(),
        ROOT / "portable_e2e/model.py", ROOT / "portable_e2e/runtime_contract.py"}
    files.update(Path(inspect.getsourcefile(function)).resolve() for function in FUNCTIONS)
    return {"files": {str(path.relative_to(ROOT)): scalar.sha(path) for path in sorted(files)},
        "functions": {f"{f.__module__}.{f.__name__}": digest(inspect.getsource(f).encode()) for f in FUNCTIONS}}


def read_metadata(root, relative, ledger, *, lines=False):
    return control_audit.read_json(root, relative, ledger, lines=lines)


def measured_timeline(states):
    # HH_260906 - Keep incomplete/failing episodes readable; reject ambiguous clocks and never bridge missing native ticks.
    result, previous_phase = [], 0
    for state in states:
        stamp = adapter._timestamp_ns(state.get("timestamp"), "raw state timestamp")
        frame = adapter._integer(state.get("frame"), "raw state frame", minimum=0)
        require(not result or stamp > result[-1][0], "duplicate or reversed raw state time")
        require(not result or frame > result[-1][1]["frame"], "duplicate or reversed raw frame")
        for key in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate"):
            adapter._number(state.get(key), f"raw state {key}")
        require(state.get("capture_phase") in PHASES, "unknown native phase")
        phase = PHASES.index(state["capture_phase"])
        require(phase >= previous_phase, "native phases move backwards")
        previous_phase = phase
        result.append((stamp, state))
    return tuple(result)


def protocol_kind(manifest):
    # HH_260906 - Dispatch only three independently reviewed protocols, never arbitrary profile or source overrides.
    capture = manifest.get("capture_contract", {})
    profile = capture.get("goal_stop_profile", {}).get("profile_id")
    if profile == "comfortable_v3" and "control_transport" not in capture:
        return "comfortable_v3_legacy_async"
    transport = capture.get("control_transport")
    require(isinstance(transport, dict) and transport.get("mode") == "acknowledged_batch"
        and profile in ("comfortable_v3", "comfortable_v4"), "unreviewed raw profile/control transport")
    return profile + "_acknowledged_batch"


def reviewed_protocol(root, manifest, states, native_rows, ledger):
    kind = protocol_kind(manifest)
    if kind == "comfortable_v3_legacy_async":
        return control_audit.analyze_protocol(states, native_rows), {}
    # HH_260906 - The selected auditor verifies its own literal source pins, archived commits, configuration and receipt journal.
    auditor = ack_audit if kind == "comfortable_v3_acknowledged_batch" else brake_free_audit
    result = auditor.audit_trial(root)[0]
    for item in result["source_manifest"]:
        raw = control_audit.checked_bytes(root, item["path"], ledger)
        require(digest(raw) == item["sha256"], "reviewed transport metadata changed before diagnostic")
    return result["pilot_protocol"], {"reviewed_protocol": kind,
        "transport_protocol": result["transport_protocol"],
        "reviewed_protocol_scope": "Per-trial frozen source, configuration and receipt proof. Campaign preregistration and before-second-attempt review are separate; this diagnostic does not replace them.",
        "snapshot_kinematics": snapshot_kinematics(states, manifest["coordinate_contract"]["wheelbase_m"])}


def snapshot_kinematics(states, wheelbase):
    # HH_260906 - Reconstruct all original fields from actual full actor snapshots without replacing any recorded label.
    require(states and wheelbase == 2.85, "snapshot diagnostic requires the reviewed nonempty 2.85 m rig")
    fields = ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate")
    maxima = {key: 0.0 for key in fields}
    mismatches, phase_counts, observations = [], Counter(), []
    largest_pitch_effect, interval_maxima = 0.0, {"center_velocity_mps": 0.0, "body_yaw_rate_rad_s": 0.0}
    for index, row in enumerate(states):
        reconstructed = ack_audit.recompute_snapshot_state(row, wheelbase)
        residuals = {key: abs(adapter._number(row.get(key), "recorded snapshot " + key) - reconstructed[key]) for key in fields}
        for key, value in residuals.items():
            maxima[key] = max(maxima[key], value)
        if any(value > 1e-6 for value in residuals.values()):
            mismatches.append({"frame": row["frame"], "absolute_residuals": residuals})
        pose = row["actor_snapshot_transform_carla"]
        yaw, pitch = math.radians(pose["yaw"]), math.radians(pose["pitch"])
        center = [pose["x"], -pose["y"], pose["z"]]
        pitch_effect = wheelbase / 2 * abs(1 - math.cos(pitch))
        largest_pitch_effect = max(largest_pitch_effect, pitch_effect)
        item = {"frame": row["frame"], "capture_phase": row["capture_phase"], "timestamp_ns":
            adapter._timestamp_ns(row["timestamp"], "snapshot timestamp"), "actor_center_ros_xyz_m": center,
            "recorded_rear_ros_xyz_m": [row[key] for key in ("x", "y", "z")],
            "body_yaw_ros_rad": -yaw, "actor_pitch_carla_rad": pitch,
            "actor_roll_carla_rad": math.radians(pose["roll"]),
            "zero_pitch_proxy_planar_position_error_m": pitch_effect,
            "endpoint_center_velocity_ros_mps": [row["world_velocity_carla"][0], -row["world_velocity_carla"][1], row["world_velocity_carla"][2]],
            "endpoint_body_yaw_rate_ros_rad_s": reconstructed["yaw_rate"],
            "maximum_conversion_absolute_residual": max(residuals.values())}
        if index:
            previous = observations[-1]
            dt = (item["timestamp_ns"] - previous["timestamp_ns"]) / 1e9
            require(dt > 0, "snapshot interval timestamp must increase")
            contiguous = row["frame"] == previous["frame"] + 1 and abs(dt - .05) <= 5e-7
            item["previous_interval_contiguous_20hz"] = contiguous
            if contiguous:
                mean_velocity = [(now - old) / dt for now, old in zip(center, previous["actor_center_ros_xyz_m"])]
                delta_yaw = math.atan2(math.sin(item["body_yaw_ros_rad"] - previous["body_yaw_ros_rad"]),
                    math.cos(item["body_yaw_ros_rad"] - previous["body_yaw_ros_rad"]))
                item.update(interval_mean_center_velocity_ros_mps=mean_velocity, interval_mean_body_yaw_rate_ros_rad_s=delta_yaw / dt)
                interval_maxima["center_velocity_mps"] = max(interval_maxima["center_velocity_mps"],
                    math.dist(mean_velocity, item["endpoint_center_velocity_ros_mps"]))
                interval_maxima["body_yaw_rate_rad_s"] = max(interval_maxima["body_yaw_rate_rad_s"],
                    abs(delta_yaw / dt - item["endpoint_body_yaw_rate_ros_rad_s"]))
        phase_counts[row["capture_phase"]] += 1
        observations.append(item)
    return {"state_count": len(states), "state_counts_by_phase": dict(phase_counts),
        "coordinate_conversion_status": "FAIL" if mismatches else "RECORDED_FORMULA_MATCH",
        "absolute_comparison_tolerance": 1e-6, "maximum_absolute_residual_by_field": maxima,
        "mismatched_state_count": len(mismatches), "mismatches": mismatches,
        "maximum_zero_pitch_proxy_planar_position_error_m": largest_pitch_effect,
        "maximum_interval_mean_vs_endpoint_difference": interval_maxima, "observations": observations,
        "interpretation": "Uses observed full actor XYZ/pitch/roll/yaw and world vectors, not the earlier zero-pitch center proxy. Formula agreement does not prove the declared rear-axle offset matches physical wheel geometry. Interval averages and endpoint velocities are different observables; their difference is not an error verdict or proof of actuator delay. No label or acceleration convention is changed."}


def future_state(timeline, stamps, timestamp):
    # HH_260906 - Reuse the adapter interpolation exactly, but only on one proven contiguous native bracket.
    index = bisect_right(stamps, timestamp)
    if index and stamps[index - 1] == timestamp:
        return adapter._interpolate_state(timeline[index - 1:index], timestamp), "exact_native_state"
    if not index or index == len(stamps):
        return None, "episode_end"
    before, after = timeline[index - 1], timeline[index]
    if after[1]["frame"] != before[1]["frame"] + 1 or abs(after[0] - before[0] - 50_000_000) > 500:
        return None, "sensor_gap"
    return adapter._interpolate_state(timeline[index - 1:index + 1], timestamp), "linear_native_bracket"


def decode_image(episode, relative, expected_size):
    # HH_260906 - Decode every pixel under fixed size/byte limits; corruption is reported rather than skipped.
    path = contract._safe_file(episode, relative, "raw JPEG")
    payload = contract._read_regular_file_bounded(path, contract.MAX_JPEG_FILE_BYTES, "raw JPEG")
    result = {"path": relative, "sha256": digest(payload), "size_bytes": len(payload),
        "decoded": False, "expected_size": list(expected_size) if expected_size else None}
    try:
        require(ImageFile.LOAD_TRUNCATED_IMAGES is False, "truncated image acceptance must remain disabled")
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(BytesIO(payload)) as picture:
                require(picture.format == "JPEG", "raw camera payload is not JPEG")
                require(picture.size == (640, 360), "raw pixel dimensions exceed the reviewed rig")
                require(expected_size is None or picture.size == expected_size, "raw pixel dimensions differ from rig")
                picture.load()
                require(picture.mode == "RGB", "raw JPEG must decode into RGB")
                result.update(decoded=True, width=picture.width, height=picture.height, mode=picture.mode,
                    decoded_pixel_sha256=digest(picture.tobytes()))
    except (OSError, ValueError, scalar.EvidenceError, Image.DecompressionBombWarning, Image.DecompressionBombError) as error:
        result["failure"] = str(error)
    return result


def camera_audit(episode, manifest, cameras, timeline):
    rig = adapter._rig_document(SimpleNamespace(manifest=manifest))
    effective_contract = contract.load_contract()
    contract._validate_rig(rig, effective_contract, Path("diagnostic_only_rig"))
    rig_sizes = {entry["name"]: (entry["width_px"], entry["height_px"]) for entry in rig["cameras"]}
    by_frame = {state["frame"]: (stamp, state) for stamp, state in timeline}
    references, anchor_records, seen_inodes, image_records = set(), [], set(), []
    for index, camera in enumerate(cameras):
        frame = adapter._integer(camera.get("frame"), "camera frame", minimum=0)
        require(tuple(camera.get("camera_order", ())) == contract.CAMERA_ORDER
            and tuple(camera.get("images", {})) == contract.CAMERA_ORDER
            and tuple(camera.get("source_timestamps", {})) == contract.CAMERA_ORDER, "camera order or six-role coverage mismatch")
        stamp = adapter._timestamp_ns(camera["timestamp"], "camera timestamp")
        native = by_frame.get(frame)
        times = [adapter._timestamp_ns(camera["source_timestamps"][name], "sensor timestamp") for name in contract.CAMERA_ORDER]
        ok = (native is not None and native[0] == stamp and native[1]["capture_phase"] == camera.get("capture_phase")
              and max(times) - min(times) <= 1 and all(abs(t - stamp) <= 1 for t in times)
              and camera.get("jpeg_quality") == manifest["capture_contract"]["jpeg_quality"]
              and abs(adapter._number(camera.get("timestamp_span_sec"), "camera time span")
                      - (max(times) - min(times)) / 1e9) <= 1e-9)
        anchor_records.append({"camera_index": index, "frame": frame, "timestamp_ns": stamp,
            "capture_phase": camera.get("capture_phase"), "same_recorded_frame_and_timestamp": ok})
        for name, relative in camera["images"].items():
            require(relative == f"images/{name}/{frame:08d}.jpg" and relative not in references,
                "raw image reference is reused or not bound to its camera/frame")
            references.add(relative)
            path = contract._safe_file(episode, relative, "raw camera image")
            inode = (path.stat().st_dev, path.stat().st_ino)
            require(inode not in seen_inodes, "raw image inode is reused")
            seen_inodes.add(inode)
            record = decode_image(episode, relative, rig_sizes[name])
            record.update(camera=name, camera_index=index, frame=frame, capture_phase=camera.get("capture_phase"), referenced=True)
            image_records.append(record)
    found = {str(path.relative_to(episode)) for path in (episode / "images").rglob("*")
             if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg")}
    for relative in sorted(found - references):
        image_records.append({**decode_image(episode, relative, None), "referenced": False, "capture_phase": "unreferenced"})
    require(references <= found, "referenced camera images were not found in original image inventory")
    expected_frames = [state["frame"] for _, state in timeline[::2]]
    complete = [row["frame"] for row in cameras] == expected_frames
    return {"image_count": len(image_records), "referenced_image_count": len(references),
        "unreferenced_jpeg_count": len(found - references), "decoded_count": sum(r["decoded"] for r in image_records),
        "decode_failure_count": sum(not r["decoded"] for r in image_records),
        "camera_anchor_count": len(cameras), "camera_frames_cover_entire_native_recording": complete,
        "timestamp_or_phase_mismatch_count": sum(not r["same_recorded_frame_and_timestamp"] for r in anchor_records),
        "rig": rig, "by_phase": dict(Counter(r["capture_phase"] for r in image_records)),
        "all_pixel_and_metadata_checks_clear": complete and bool(image_records) and all(r["decoded"] for r in image_records)
            and not found - references and all(r["same_recorded_frame_and_timestamp"] for r in anchor_records)}, image_records, anchor_records


def measured_future_audit(timeline, cameras, route, anchor_records):
    effective_contract = contract.load_contract()
    points = tuple((float(p["x"]), float(p["y"])) for p in route["route"])
    length = float(route["route_length_m"])
    stamps = [t for t, _ in timeline]
    require(stamps, "future diagnostic requires at least one measured native state")
    tail_start = next((t for t, r in timeline if r["capture_phase"] == "stationary_tail"), None)
    records, audited = [], []
    interpolation_counts = Counter()
    for camera, metadata in zip(cameras, anchor_records):
        phase = camera["capture_phase"]
        record = {"frame": camera["frame"], "capture_phase": phase, "anchor_timestamp_ns": metadata["timestamp_ns"]}
        if phase == "stationary_tail":
            records.append({**record, "disposition": "tail_label_context_only", "valid_points": 0})
            continue
        require(phase in ("stationary_warmup", "driving"), "unknown future-anchor phase")
        if not metadata["same_recorded_frame_and_timestamp"]:
            records.append({**record, "disposition": "invalid_camera_state_binding", "valid_points": 0})
            continue
        anchor = metadata["timestamp_ns"]
        ego_ns, ego = adapter._causal_state(timeline, anchor)
        require(ego_ns == anchor, "diagnostic anchor must have its actual same-frame native observation")
        yaw = float(ego["yaw"])
        _, goal, arc = contract._canonical_route_in_base(points,
            position_m=(ego["x"], ego["y"], ego["z"]), orientation_xyzw=[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)],
            contract=effective_contract, context="raw diagnostic navigation")
        planning = {"available": True, "dt_s": .1, "positions_base_xy_m": [], "yaw_rad": [], "speed_mps": [],
            "valid": [], "target_timestamp_ns": [], "invalid_reason": []}
        missing = None
        for i in range(64):
            timestamp = anchor + (i + 1) * 100_000_000
            future, mode = future_state(timeline, stamps, timestamp) if missing is None else (None, missing)
            if future is None:
                missing = mode
                values = ([None, None], None, None, False, None, mode)
            else:
                xy, relative_yaw = adapter._relative_xy_yaw(ego, future)
                values = (xy, relative_yaw, math.hypot(future["vx"], future["vy"]), True, timestamp, None)
                interpolation_counts[mode] += 1
            for key, value in zip(("positions_base_xy_m", "yaw_rad", "speed_mps", "valid", "target_timestamp_ns", "invalid_reason"), values):
                planning[key].append(value)
        valid = sum(planning["valid"])
        disposition = "full_64_point_anchor" if valid == 64 else "incomplete_future_retained"
        record.update(disposition=disposition, valid_points=valid, unavailable_reason=missing,
            valid_mask=planning["valid"], invalid_reasons=planning["invalid_reason"])
        if valid:
            # HH_260906 - This transient mapping is never exported as a Common10 sample or accepted by a training loader.
            sample = {"sample_id": f"raw-frame:{camera['frame']}", "anchor_timestamp_ns": anchor,
                "ego": {"linear_velocity_base_mps": [ego["vx"], ego["vy"], 0.]},
                "navigation": {"goal_base_m": list(goal), "route_anchor_arc_m": arc}, "labels": {"planning": planning}}
            measured = targets.audit_sample(sample, route_length_m=length,
                episode_start_ns=stamps[0], episode_end_ns=stamps[-1], tail_start_ns=tail_start)
            measured.update(episode_id="single_raw_trial", capture_phase=phase)
            audited.append(measured)
            record["diagnostic"] = measured
        records.append(record)
    full = [row for row in audited if row["valid_points"] == 64]
    summary = {"camera_anchor_count": len(cameras), "disposition_counts": dict(Counter(r["disposition"] for r in records)),
        "full_anchor_counts_by_phase": dict(Counter(row["capture_phase"] for row in full)),
        "interpolation_counts": dict(interpolation_counts), "full_64_point_anchors": targets.summarize_samples(full, 64),
        "all_available_prefixes": {str(horizon): targets.summarize_samples(audited, horizon) for horizon in targets.HORIZONS},
        "by_anchor_phase": {phase: targets.summarize_samples([r for r in full if r["capture_phase"] == phase], 64)
            for phase in ("stationary_warmup", "driving")},
        "interpretation": "Necessary discrete XY/speed bounds, not a sufficient feasibility proof; yaw is a proxy. Overlapping horizon windows are not independent events. Original warmup anchors remain included, tail anchors remain context-only; unavailable futures are not extrapolated."}
    return summary, records


def audit_trial(root, expected_owner_sha):
    root, ledger = Path(root), {}
    plan = read_metadata(root, "owner_plan.json", ledger)
    require(ledger["owner_plan.json"]["sha256"] == expected_owner_sha, "owner-plan SHA mismatch")
    require(plan.get("capture_mode") == "expert" and plan.get("bounds_source_bytes_archived") is True,
        "only owned expert captures with archived bounds are in scope")
    owner = read_metadata(root, "owner_result.json", ledger)
    require(owner.get("source_bytes_unchanged_and_archived") is True
        and set(owner.get("source_checks", {})) == set(plan["source_sha256"])
        and all(value is True for value in owner["source_checks"].values()), "execution source-after proof failed")
    for name, expected in plan["source_sha256"].items():
        require(digest(control_audit.checked_bytes(root, f"provenance/{name}", ledger)) == expected, "archived execution source SHA mismatch")
    original, raw_timeline = scalar.summarize_trial(root, None)
    require(original["raw_data_available"] and raw_timeline is not None, "no actual native payload available for this diagnostic")
    episode_name = next(name for name in ("episode", "episode.partial") if (root / name).is_dir())
    episode = root / episode_name
    manifest = read_metadata(root, f"{episode_name}/manifest.json", ledger)
    protocol_kind(manifest)
    route = read_metadata(root, f"{episode_name}/route.json", ledger)
    states = read_metadata(root, f"{episode_name}/states.jsonl", ledger, lines=True)
    cameras = read_metadata(root, f"{episode_name}/camera_frames.jsonl", ledger, lines=True)
    for item in original["source_manifest"]:
        control_audit.checked_bytes(root, item["path"], ledger)
    for path in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py"):
        require(ledger[f"provenance/{path}"]["sha256"] == scalar.sha(ROOT / path),
            "diagnostic motion source differs from archived capture source; explicit reviewed mapping required")
    timeline = measured_timeline(states)
    alignment, reviewed = reviewed_protocol(root, manifest, states, raw_timeline["native_states"], ledger)
    pixels, image_rows, anchor_rows = camera_audit(episode, manifest, cameras, timeline)
    future, future_rows = measured_future_audit(timeline, cameras, route, anchor_rows)
    for item in image_rows:
        require(scalar.sha(episode / item["path"]) == item["sha256"], "raw JPEG changed during diagnostic")
    for path, pin in ledger.items():
        require(scalar.sha(root / path) == pin["sha256"], "raw metadata changed during diagnostic")
    scalar.recheck_bounds_source_archive(root, original["bounds_source_proof"])
    return {"trial_id": root.name, "original_episode_directory": episode_name,
        "original_owner_exit_code": original["owner_exit_code"], "original_capture_status": manifest["status"],
        "original_training_data_approved": manifest["result"].get("training_data_approved"),
        "original_development_only": manifest["result"].get("development_only"),
        "raw_scalar_quality": original["independent_qa"], "raw_scalar_candidate_only": original["raw_quality_candidate"],
        "control_protocol": alignment, "camera_pixels": pixels, "measured_future": future,
        "training_data_approved": False, "status": "DIAGNOSED_NOT_ADMITTED",
        "source_manifest": [{"path": p, **v} for p, v in sorted(ledger.items())], **reviewed}, image_rows, future_rows


def run(trials, output, expected_script_sha):
    output = Path(output)
    require(scalar.sha(Path(__file__)) == expected_script_sha, "diagnostic script SHA mismatch")
    require(trials and len({name for name, _, _ in trials}) == len(trials), "distinct explicitly named trials required")
    require(len({Path(root).resolve() for _, root, _ in trials}) == len(trials), "same trial cannot be counted twice")
    require(not output.exists() and not output.is_symlink()
        and all(not output.resolve().is_relative_to(Path(root).resolve()) for _, root, _ in trials), "fresh output must be outside raw inputs")
    sources, reports, images, futures = source_identity(), [], [], []
    for name, root, expected in trials:
        require(name and all(c.isalnum() or c in "_-" for c in name), "unsafe diagnostic trial name")
        report, image_rows, future_rows = audit_trial(root, expected)
        report["diagnostic_trial_name"] = name
        reports.append(report)
        images.extend({"trial": name, **row} for row in image_rows)
        futures.extend({"trial": name, **row} for row in future_rows)
    for (_, root, _), report in zip(trials, reports):
        for source in report["source_manifest"]:
            require(scalar.sha(Path(root) / source["path"]) == source["sha256"], "input changed during full batch")
        episode = Path(root) / report["original_episode_directory"]
        trial_images = [row for row in images if row["trial"] == report["diagnostic_trial_name"]]
        actual_names = {str(p.relative_to(episode)) for p in (episode / "images").rglob("*")
                        if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg")}
        require(actual_names == {row["path"] for row in trial_images}, "JPEG inventory changed during full batch")
        for row in trial_images:
            require(scalar.sha(contract._safe_file(episode, row["path"], "raw JPEG recheck")) == row["sha256"],
                "raw JPEG changed during full batch")
    require(source_identity() == sources, "diagnostic source changed during execution")
    summary = {"schema": SCHEMA, "status": "DIAGNOSED_NOT_ADMITTED", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "trial_count": len(reports), "trials": reports, "source_identity": sources,
        "total_jpeg_count": len(images), "total_decoded_jpeg_count": sum(row["decoded"] for row in images),
        "scope": {"training_data_approved": False, "common10_dataset_written": False, "source_flags_modified": False,
            "model_loaded": False, "model_inference": False, "training": False, "live_simulator_access": False,
            "test_payload_read": False, "all_referenced_jpeg_pixels_decoded_or_failure_retained": True,
            "original_warmup_anchor_policy_preserved": True, "missing_future_extrapolated": False},
        "limitations": ["Successful pixel decode does not prove realistic road materials or correct sensor placement.",
            "Raw speed/goal QA, control-record alignment and future XY necessary bounds remain separate.",
            "This diagnostic never authorizes data admission or changes any captured approval marker."]}
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    for filename, rows in (("jpeg_audit.jsonl", images), ("future_anchor_audit.jsonl", futures)):
        with (output / filename).open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text("".join(f"{scalar.sha(p)}  {p.name}\n" for p in sorted(output.iterdir())))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", nargs=3, metavar=("NAME", "RAW_TRIAL_ROOT", "OWNER_PLAN_SHA256"), action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-script-sha256", required=True)
    args = parser.parse_args(argv)
    run(args.trial, args.output_dir, args.expected_script_sha256)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
