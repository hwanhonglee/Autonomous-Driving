#!/usr/bin/env python3
"""HH_260906 - Diagnose all frozen launch-matrix raw futures without conversion, training, source changes or admission."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import inspect
import json
import math
from pathlib import Path
import re

from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_carla_turn_launch_matrix as matrix

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "carla_expert.turn_launch_raw_geometry_diagnostic.v1"
RAW_HELPER_SHA256 = "0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00"
# HH_260906 - This independently reviewed new-protocol auditor has no runtime source-pin override.
MATRIX_AUDITOR_SHA256 = "2f1013cdaa8b2d94329d2dce6ca5a903c772dcd5d4145e35b88c2d3422859eab"
EXECUTION_COMMIT = "aa1abbbbef03accf14479fd8caec1e3492a548c5"
PLAN_SHA256 = "075051366ec76e1a96c83fa2f7f678acaa72bd122e89cd02ef37714892f19267"
ORDER = ("turn_launch_015_v1", "turn_launch_013_v1", "turn_launch_014_v1", "turn_launch_012_v1",
         "turn_launch_012_v1", "turn_launch_014_v1", "turn_launch_013_v1", "turn_launch_015_v1")
require = raw.require
PURE_FUNCTIONS = (raw.measured_timeline, raw.future_state, raw.camera_audit, raw.snapshot_kinematics,
    raw.adapter._causal_state, raw.adapter._relative_xy_yaw, raw.adapter._interpolate_state,
    raw.contract._canonical_route_in_base, raw.targets.audit_sample, raw.targets.summarize_samples)
AUDIT_FUNCTIONS = (matrix.audit_campaign, matrix.audit_trial, matrix.validate_plan_document,
    matrix.verify_owner_sources, matrix.analyze_protocol, matrix.initialization_audit.analyze_initialization)


def source_identity():
    """HH_260906 - Bind explicit new-protocol auditing and unchanged pure math, never override historical V4 source pins."""
    require(raw.scalar.sha(Path(raw.__file__)) == RAW_HELPER_SHA256, "original raw diagnostic helper SHA changed")
    require(re.fullmatch(r"[0-9a-f]{64}", MATRIX_AUDITOR_SHA256) is not None
            and raw.scalar.sha(Path(matrix.__file__)) == MATRIX_AUDITOR_SHA256,
            "matrix auditor is not the independently reviewed frozen source")
    require(matrix.COMMIT == EXECUTION_COMMIT and matrix.ORIGINAL_PLAN_SHA == PLAN_SHA256,
            "matrix execution revision or prospective plan changed")
    proof = raw.source_identity()
    paths = {Path(__file__).resolve(), Path(matrix.__file__).resolve()}
    paths.update(Path(inspect.getsourcefile(function)).resolve() for function in PURE_FUNCTIONS + AUDIT_FUNCTIONS)
    paths.update(Path(module.__file__).resolve() for module in
                 (matrix.turn, matrix.initialization_audit, matrix.ack, matrix.v4, matrix.pilot, matrix.base))
    proof["files"].update({p.relative_to(ROOT).as_posix(): raw.scalar.sha(p) for p in paths})
    proof["functions"].update({f"{f.__module__}.{f.__name__}": raw.digest(inspect.getsource(f).encode()) for f in PURE_FUNCTIONS + AUDIT_FUNCTIONS})
    return proof


def native_gaps(timeline):
    """HH_260906 - Report every missing or off-grid native interval, even when its next 100 ms endpoint exists."""
    return [{"from_frame": a[1]["frame"], "to_frame": b[1]["frame"], "from_timestamp_ns": a[0],
             "to_timestamp_ns": b[0], "dt_ns": b[0] - a[0]}
            for a, b in zip(timeline, timeline[1:])
            if b[1]["frame"] != a[1]["frame"] + 1 or abs(b[0] - a[0] - 50_000_000) > 500]


def geometry_futures(timeline, cameras, route, anchor_rows, case_id):
    """HH_260906 - Retain every warmup/driving anchor and all tail context under a diagnostic-only, non-Common10 schema."""
    require(len(cameras) == len(anchor_rows), "camera/anchor diagnostic denominators differ")
    require(bool(timeline), "no native observations for future geometry")
    stamps = [t for t, _ in timeline]
    gaps = native_gaps(timeline)
    route_points = tuple((float(p["x"]), float(p["y"])) for p in route["route"])
    length = float(route["route_length_m"])
    tail_start = next((t for t, r in timeline if r["capture_phase"] == "stationary_tail"), None)
    rows, assessed, interpolations = [], [], Counter()
    effective_contract = raw.contract.load_contract()
    for camera, metadata in zip(cameras, anchor_rows):
        phase, anchor = camera["capture_phase"], metadata["timestamp_ns"]
        item = {"case_id": case_id, "frame": camera["frame"], "capture_phase": phase,
                "anchor_timestamp_ns": anchor, "training_data_approved": False}
        if phase == "stationary_tail":
            rows.append({**item, "disposition": "tail_label_context_only", "valid_future_points": 0})
            continue
        require(phase in ("stationary_warmup", "driving"), "unknown future anchor phase")
        if not metadata["same_recorded_frame_and_timestamp"]:
            rows.append({**item, "disposition": "invalid_camera_state_binding", "valid_future_points": 0})
            continue
        ego_ns, ego = raw.adapter._causal_state(timeline, anchor)
        require(ego_ns == anchor and ego["frame"] == camera["frame"], "anchor lacks its exact native frame")
        planning = {"available": True, "dt_s": .1, "positions_base_xy_m": [], "yaw_rad": [], "speed_mps": [],
                    "valid": [], "target_timestamp_ns": [], "invalid_reason": []}
        modes, missing = [], None
        for index in range(64):
            timestamp = anchor + (index + 1) * 100_000_000
            crossed = [g for g in gaps if g["from_timestamp_ns"] < timestamp and g["to_timestamp_ns"] > anchor]
            if missing is None and crossed:
                missing = "sensor_gap"
                item["first_uncovered_native_interval"] = crossed[0]
            future, mode = raw.future_state(timeline, stamps, timestamp) if missing is None else (None, missing)
            if future is None:
                missing = mode
                values = ([None, None], None, None, False, None, mode)
            else:
                xy, yaw = raw.adapter._relative_xy_yaw(ego, future)
                values = (xy, yaw, math.hypot(future["vx"], future["vy"]), True, timestamp, None)
                interpolations[mode] += 1
            modes.append(mode)
            for key, value in zip(("positions_base_xy_m", "yaw_rad", "speed_mps", "valid", "target_timestamp_ns", "invalid_reason"), values):
                planning[key].append(value)
        count = sum(planning["valid"])
        item.update(disposition="full_64_point_anchor" if count == 64 else "incomplete_future_retained",
            valid_future_points=count, unavailable_reason=missing, valid_mask=planning["valid"],
            invalid_reasons=planning["invalid_reason"], future_state_association=modes,
            diagnostic_future_xy_m=planning["positions_base_xy_m"],
            diagnostic_body_yaw_delta_rad=planning["yaw_rad"],
            diagnostic_endpoint_planar_speed_mps=planning["speed_mps"],
            diagnostic_target_timestamp_ns=planning["target_timestamp_ns"],
            current_recorded_velocity_base_mps=[ego["vx"], ego["vy"], 0.],
            future_crosses_stationary_tail=tail_start is not None and any(t is not None and t >= tail_start for t in planning["target_timestamp_ns"]))
        if count:
            try:
                yaw = float(ego["yaw"])
                _, goal, arc = raw.contract._canonical_route_in_base(route_points,
                    position_m=(ego["x"], ego["y"], ego["z"]),
                    orientation_xyzw=[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)],
                    contract=effective_contract, context="raw turn diagnostic navigation")
                # HH_260906 - This ephemeral mapping never leaves this function as a Common10 sample or TrainingExample.
                temporary = {"sample_id": f"{case_id}:raw-frame:{camera['frame']}", "anchor_timestamp_ns": anchor,
                    "ego": {"linear_velocity_base_mps": [ego["vx"], ego["vy"], 0.]},
                    "navigation": {"goal_base_m": list(goal), "route_anchor_arc_m": arc}, "labels": {"planning": planning}}
                diagnostic = raw.targets.audit_sample(temporary, route_length_m=length,
                    episode_start_ns=stamps[0], episode_end_ns=stamps[-1], tail_start_ns=tail_start)
                diagnostic.update(episode_id=case_id, capture_phase=phase)
                item["discrete_bound_diagnostic"] = diagnostic
                item["navigation_diagnostic"] = {"status": "AVAILABLE", "global_nearest_route_arc_m": arc,
                    "goal_base_xy_m": list(goal), "collector_route_progress_m": ego.get("route_progress_m"),
                    "notice": "Canonical nearest-route projection is not the collector's contiguous progress state and is not silently substituted for it."}
                assessed.append(diagnostic)
            except raw.contract.ContractError as error:
                # HH_260906 - A failed route/target interpretation retains its raw XY and remains unassessed, never excluded as a successful anchor.
                item["bound_diagnostic_unavailable"] = {"error_type": type(error).__name__, "message": str(error)}
        rows.append(item)
    full = [s for s in assessed if s["valid_points"] == 64]
    summary = {"camera_anchor_count": len(cameras), "anchor_count_by_phase": dict(Counter(r["capture_phase"] for r in rows)),
        "disposition_counts": dict(Counter(r["disposition"] for r in rows)), "native_gap_count": len(gaps), "native_gaps": gaps,
        "available_64_point_anchor_count": sum(r.get("valid_future_points") == 64 for r in rows),
        "bound_assessed_64_point_anchor_count": len(full),
        "bound_unassessed_anchor_count": sum("bound_diagnostic_unavailable" in r for r in rows),
        "interpolation_counts": dict(interpolations),
        "all_available_prefixes": {str(h): raw.targets.summarize_samples(assessed, h) for h in raw.targets.HORIZONS},
        "full_64_point_anchors": raw.targets.summarize_samples(full, 64),
        "by_anchor_phase": {p: {str(h): raw.targets.summarize_samples([s for s in assessed if s["capture_phase"] == p], h)
                                for h in raw.targets.HORIZONS} for p in ("stationary_warmup", "driving")},
        "tail_crossing_anchor_count": sum(r.get("future_crosses_stationary_tail") is True for r in rows),
        "training_data_approved": False,
        "definitions": {"horizons_seconds": [1., 3., 6.4], "decoder_initial_speed": "clamped longitudinal vx, not planar speed",
            "speed_label": "Endpoint measured planar velocity; differs from XY interval-average speed.",
            "xy_bounds": "Necessary discrete speed/acceleration/curvature/lateral bounds with initial path heading zero, not sufficient reachability.",
            "body_yaw": "Vehicle yaw-label heading-envelope proxy only; not decoder displacement heading or a proof of XY impossibility.",
            "stationary_heading": "Steps <=0.0001m remain unassessed by the unchanged heading diagnostic, not counted as passing.",
            "native_gaps": "A horizon becomes an unavailable contiguous prefix at any missing/off-grid native20Hz interval; exact100ms endpoints do not override source incompleteness.",
            "denominators": "All warmup/driving camera anchors retained; tail anchors context only; overlapping windows are not independent maneuvers.",
            "not_solved": "Route-relative slip/reachability, physical reference-point identity, content realism, safety and dataset admission."}}
    return summary, rows


def snapshot_diagnostic(states, wheelbase):
    """HH_260906 - Preserve observed full3D vectors while naming CARLA's reference point without asserting its physical center."""
    result = raw.snapshot_kinematics(states, wheelbase)
    for source, row in zip(states, result["observations"]):
        for old, new in (("actor_center_ros_xyz_m", "actor_api_reference_ros_xyz_m"),
                         ("endpoint_center_velocity_ros_mps", "endpoint_actor_api_velocity_ros_mps"),
                         ("interval_mean_center_velocity_ros_mps", "interval_mean_actor_api_velocity_ros_mps")):
            if old in row:
                row[new] = row.pop(old)
        row["source_actor_snapshot_transform_carla"] = dict(source["actor_snapshot_transform_carla"])
        for key in ("world_velocity_carla", "world_acceleration_carla", "world_angular_velocity_carla_deg_s"):
            row["source_" + key] = list(source[key])
        row["original_recorded_planar_state"] = {key: source[key] for key in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate")}
    differences = result["maximum_interval_mean_vs_endpoint_difference"]
    differences["actor_api_velocity_mps"] = differences.pop("center_velocity_mps")
    result["physical_reference_point_identity_proven"] = False
    return result


def diagnose_trial(root, reviewed, reviewed_images, timeline, case):
    """HH_260906 - Require the new matrix's independently source-bound result, retain scalar failures, and never dispatch the old V4 auditor."""
    require(reviewed.get("reviewed_execution_commit") == EXECUTION_COMMIT
            and reviewed.get("all_eleven_archives_match_owner_and_reviewed_commits") is True,
            "trial lacks the exact independent matrix source proof")
    ledger = {}
    for entry in reviewed["source_manifest"]:
        payload = raw.control_audit.checked_bytes(root, entry["path"], ledger)
        require(raw.digest(payload) == entry["sha256"], "reviewed raw metadata changed")
    for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py"):
        require(ledger["provenance/" + name]["sha256"] == raw.scalar.sha(ROOT / name),
                "diagnostic bounds source differs from archived execution source")
    directories = [p for p in (root / "episode", root / "episode.partial") if p.is_dir()]
    if not reviewed.get("raw_data_available"):
        require(not directories and not timeline and not reviewed_images, "no-payload case contains unexplained measurements")
        return {"case_id": case["case_id"], "status": "NO_RAW_PAYLOAD_RETAINED", "training_data_approved": False}, [], [], []
    require(len(directories) == 1, "ambiguous finalized/partial raw episode")
    episode = directories[0]
    prefix = episode.name + "/"
    read = lambda name, lines=False: raw.read_metadata(root, prefix + name, ledger, lines=lines)
    manifest, route = read("manifest.json"), read("route.json")
    require(manifest["result"].get("training_data_approved") is False and manifest["result"].get("development_only") is True,
            "original explicit development denial was changed")
    require(manifest["capture_contract"]["goal_stop_profile"]["profile_id"] == case["profile"], "case/profile mismatch")
    states, cameras = read("states.jsonl", True), read("camera_frames.jsonl", True)
    if not states:
        require(not cameras and not reviewed_images and timeline is None
                and manifest["status"] == "failed" and reviewed["owner_exit_code"] != 0,
                "empty native capture contradicts successful or camera evidence")
        require(not any(p.is_file() for p in (episode / "images").rglob("*")), "empty native capture has unaccounted image payload")
        return {"case_id": case["case_id"], "profile": case["profile"], "replicate": case["replicate"],
            "status": "NO_NATIVE_OBSERVATIONS_RETAINED", "raw_native_state_count": 0,
            "original_capture_status": manifest["status"], "original_owner_exit_code": reviewed["owner_exit_code"],
            "raw_scalar_quality": reviewed["independent_qa"], "training_data_approved": False,
            "source_manifest": [{"path": p, **v} for p, v in sorted(ledger.items())]}, [], [], []
    require(timeline is not None and len(timeline["native_states"]) == len(states), "independent native denominator changed")
    measured = raw.measured_timeline(states)
    pixels, images, anchor_rows = raw.camera_audit(episode, manifest, cameras, measured)
    expected_images = {item["path"]: (item["sha256"], item["size_bytes"]) for item in reviewed_images}
    actual_images = {prefix + item["path"]: (item["sha256"], item["size_bytes"]) for item in images}
    require(actual_images == expected_images, "matrix and geometry JPEG inventories differ")
    future, anchors = geometry_futures(measured, cameras, route, anchor_rows, case["case_id"])
    snapshot = snapshot_diagnostic(states, manifest["coordinate_contract"]["wheelbase_m"])
    observations = snapshot.pop("observations")
    report = {"case_id": case["case_id"], "profile": case["profile"], "replicate": case["replicate"],
        "status": "DIAGNOSED_NOT_ADMITTED", "original_episode_directory": episode.name,
        "original_capture_status": manifest["status"], "original_owner_exit_code": reviewed["owner_exit_code"],
        "original_training_data_approved": False, "original_development_only": True,
        "raw_native_state_count": len(states), "raw_native_state_counts_by_phase": dict(Counter(s["capture_phase"] for s in states)),
        "raw_scalar_quality": reviewed["independent_qa"], "pilot_protocol": reviewed["pilot_protocol"],
        "transport_protocol": reviewed["transport_protocol"], "initialization_protocol": reviewed["initialization_protocol"],
        "alternate_10_hz_offsets_diagnostic_only": reviewed.get("alternate_10_hz_offsets_diagnostic_only"),
        "camera_pixels": pixels, "snapshot_kinematics": snapshot, "measured_future": future,
        "source_manifest": [{"path": p, **v} for p, v in sorted(ledger.items())],
        "training_data_approved": False, "common10_dataset_written": False, "labels_modified": False,
        "full_future_xy_admission": False, "qualification_30_kph": "NOT_CLAIMED"}
    return report, images, anchors, observations


def recheck_inputs(campaign, matrix_report, audits):
    """HH_260906 - Recheck the whole batch after all later analyses, including original failed trials and all image inventory bytes."""
    for entry in matrix_report["source_manifest"]:
        require(raw.scalar.sha(campaign / entry["path"]) == entry["sha256"], "matrix declaration/review changed during raw analysis")
    for entry in matrix_report.get("historical_source_manifest", []):
        require(raw.scalar.sha(ROOT / entry["path"]) == entry["sha256"], "historical proof changed during raw analysis")
    for case in matrix_report["cases"]:
        root = campaign / case["output"]
        require(root.exists() == (case["status"] != "NOT_RUN"), "matrix attempt inventory changed during raw analysis")
        require((root / "owner_result.json").is_file() == (case["status"] == "FINALIZED"),
                "matrix finalization changed during raw analysis")
    for root, (trial, images, _) in audits:
        for entry in trial["source_manifest"] + images:
            require(raw.scalar.sha(raw.contract._safe_file(root, entry["path"], "raw final recheck")) == entry["sha256"],
                    "raw/source/image changed during full geometry batch")
        directories = [p for p in (root / "episode", root / "episode.partial") if p.is_dir()]
        if trial.get("raw_data_available"):
            require(len(directories) == 1, "raw directory inventory changed")
            found = {p.relative_to(root).as_posix() for p in (directories[0] / "images").rglob("*")
                     if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg")}
            require(found == {x["path"] for x in images}, "raw JPEG inventory changed during full geometry batch")


def run(campaign, output, expected_script_sha256):
    """HH_260906 - A new diagnostic folder is the only output; all eight declared outcomes remain visible and never become approval."""
    campaign, output = Path(campaign).resolve(), Path(output)
    require(raw.scalar.sha(Path(__file__)) == expected_script_sha256, "new diagnostic script SHA mismatch")
    require(not output.exists() and not output.is_symlink()
            and not output.resolve().is_relative_to(campaign)
            and not output.resolve().is_relative_to(ROOT / "datasets"),
            "fresh diagnostic output must be outside all raw campaign inputs and existing datasets")
    sources = source_identity()
    independent, audits = matrix.audit_campaign(campaign)
    require(all(sources["files"].get(path) == digest for path, digest in independent.get("audit_source_sha256", {}).items()),
            "matrix dependency source changed or is not bound by the diagnostic source proof")
    require(all(not output.resolve().is_relative_to((ROOT / e["path"]).parent)
                for e in independent.get("historical_source_manifest", [])),
            "diagnostic output must be outside historical raw input directories")
    cases = independent["cases"]
    require(independent.get("schema") == "portable_e2e.turn_launch_matrix_audit.v1"
            and independent.get("reviewed_execution_commit") == EXECUTION_COMMIT
            and independent.get("prospective_plan_sha256") == PLAN_SHA256
            and independent.get("all_planned_cases_retained") is True
            and independent.get("planned_cases") == 8 and len(cases) == 8
            and tuple(c["profile"] for c in cases) == ORDER
            and [c["sequence"] for c in cases] == list(range(1, 9)), "independent complete fixed-case matrix contract missing")
    finalized = [c for c in cases if c["status"] == "FINALIZED"]
    require(len(audits) == len(finalized) == independent["finalized_cases"], "independent case denominator differs")
    by_root = {Path(root).resolve(): values for root, values in audits}
    require(len(by_root) == len(audits), "duplicate independent raw trial")
    reports, images, anchors, snapshots = [], [], [], []
    for case in cases:
        if case["status"] != "FINALIZED":
            require(case["status"] in ("NOT_RUN", "INCOMPLETE"), "unknown case disposition")
            reports.append({"case_id": case["case_id"], "profile": case["profile"], "replicate": case["replicate"],
                            "status": case["status"], "training_data_approved": False})
            continue
        root = (campaign / case["output"]).resolve()
        require(root in by_root and root.is_relative_to(campaign), "independent case root does not match declaration")
        trial, image_ledger, timeline = by_root[root]
        require(trial == case["audit"], "case summary differs from independent trial evidence")
        report, im, an, sn = diagnose_trial(root, trial, image_ledger, timeline, case)
        reports.append(report)
        images.extend({"case_id": case["case_id"], **r} for r in im)
        anchors.extend(an)
        snapshots.extend({"case_id": case["case_id"], **r} for r in sn)
    recheck_inputs(campaign, independent, audits)
    require(source_identity() == sources, "diagnostic source changed during execution")
    complete = independent["status"] == "AUDITED_NOT_ADMITTED" and len(finalized) == 8
    summary = {"schema": SCHEMA, "status": "DIAGNOSED_NOT_ADMITTED" if complete else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "source_identity": sources,
        "planned_case_count": 8, "finalized_case_count": len(finalized), "all_planned_cases_retained": True,
        "cases": reports, "independent_matrix_audit": independent,
        "total_jpeg_count": len(images), "total_decoded_jpeg_count": sum(r["decoded"] for r in images),
        "scope": {"training_data_approved": False, "dataset_admission": False, "common10_dataset_written": False,
            "source_flags_modified": False, "labels_modified": False, "training": False, "model_loaded": False,
            "model_inference": False, "live_simulator_access": False, "test_payload_read": False,
            "all_failed_and_unrun_cases_retained": True, "native20hz_quality_replaced_by_10hz": False,
            "wall_timing_independently_reconstructed": False, "automatic_winner_selection": False},
        "limitations": ["Raw scalar quality, discrete XY necessary bounds and vehicle-yaw proxies are distinct; none grants admission.",
            "Pixel decode and source hashes do not prove realistic road materials, correct physical sensor placement or public-road safety.",
            "All warmup/driving anchors are retained; tail is future context only. Overlapping windows are not independent maneuvers.",
            "The immutable-source training denial is intentional: this diagnostic never invokes conversion or emits loader-compatible examples."]}
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(matrix.v4.sanitize(summary), indent=2, allow_nan=False) + "\n")
    for name, rows in (("jpeg_audit.jsonl", images), ("future_geometry_audit.jsonl", anchors), ("native_snapshot_audit.jsonl", snapshots)):
        with (output / name).open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text("".join(f"{raw.scalar.sha(p)}  {p.name}\n" for p in sorted(output.iterdir())))
    return summary


def main(argv=None):
    """HH_260906 - Explicit research diagnostics have no conversion, admission or source-pin override switches."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-script-sha256", required=True)
    args = parser.parse_args(argv)
    result = run(args.campaign, args.output_dir, args.expected_script_sha256)
    return 2 if result["status"] == "INCOMPLETE" else 0


if __name__ == "__main__":
    raise SystemExit(main())
