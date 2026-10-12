#!/usr/bin/env python3
"""HH_260906 - Audit train/val target motion against the unchanged discrete physical decoder envelope."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import (
    ContractError, DEFAULT_CONTRACT_PATH, _iter_jsonl, _read_json_and_sha256, _safe_file,
    contract_fingerprint, validate_contract,
)
from portable_e2e.model import (
    PHYSICAL_MAXIMUM_ACCELERATION_MPS2, PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,
    PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2, PHYSICAL_MAXIMUM_ROUTE_SLIP_RAD,
    PHYSICAL_MAXIMUM_SPEED_MPS, PHYSICAL_TIME_STEP_S,
)

SCHEMA = "portable_e2e.target_feasibility_audit.v1"
HORIZONS = (10, 30, 64)
ROUNDING_TOLERANCE = 1.0e-6
HEADING_MINIMUM_STEP_M = 1.0e-4
NEAR_GOAL_BUFFER_M = 2.5
METRICS = (
    "speed_limit", "speed_acceleration", "speed_deceleration", "xy_speed_limit",
    "xy_acceleration", "xy_deceleration", "xy_curvature", "xy_lateral_acceleration",
    "yaw_label_heading_envelope",
)


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def _number(value, name):
    _require(not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value),
             f"{name} must be a finite number")
    return float(value)


def _integer(value, name):
    _require(isinstance(value, int) and not isinstance(value, bool) and value >= 0,
             f"{name} must be a nonnegative integer")
    return value


def _point(value, name, size=2):
    _require(isinstance(value, list) and len(value) == size, f"{name} has the wrong vector size")
    return tuple(_number(item, name) for item in value)


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_sample(sample, *, route_length_m, episode_start_ns, episode_end_ns, tail_start_ns=None):
    """HH_260906 - Keep speed-label, XY-only and vehicle-yaw proxy diagnostics separate."""
    token = sample.get("sample_id")
    _require(isinstance(token, str) and token, "sample_id must be nonempty")
    anchor = _integer(sample.get("anchor_timestamp_ns"), "anchor_timestamp_ns")
    _require(episode_start_ns <= anchor <= episode_end_ns, "sample anchor crosses episode bounds")
    velocity = _point(sample["ego"]["linear_velocity_base_mps"], "current velocity", 3)
    current_planar = math.hypot(*velocity[:2])
    # HH_260906 - The actual model starts from clamped longitudinal vx, not planar velocity magnitude.
    initial_speed = max(0.0, min(velocity[0], PHYSICAL_MAXIMUM_SPEED_MPS))
    navigation = sample["navigation"]
    goal = _point(navigation["goal_base_m"], "goal_base_m")
    anchor_arc = _number(navigation["route_anchor_arc_m"], "route_anchor_arc_m")
    _require(0.0 <= anchor_arc <= route_length_m + ROUNDING_TOLERANCE, "anchor arc is outside the episode route")
    stopping_distance = current_planar ** 2 / (2.0 * PHYSICAL_MAXIMUM_ACCELERATION_MPS2)
    anchor_region = "near_goal" if route_length_m - anchor_arc <= stopping_distance + NEAR_GOAL_BUFFER_M else "interior"
    planning = sample["labels"]["planning"]
    _require(planning.get("available") is True, "planning targets must be available")
    _require(abs(_number(planning.get("dt_s"), "dt_s") - PHYSICAL_TIME_STEP_S) <= 1.0e-12,
             "target dt_s differs from the physical decoder")
    names = ("positions_base_xy_m", "speed_mps", "yaw_rad", "valid", "target_timestamp_ns", "invalid_reason")
    _require(all(isinstance(planning.get(name), list) and len(planning[name]) == 64 for name in names),
             "target arrays must each contain 64 points")
    previous_speed, previous_geo_speed = initial_speed, initial_speed
    previous_xy, previous_heading, previous_yaw, previous_time = (0.0, 0.0), 0.0, 0.0, anchor
    invalid_started, steps = False, []
    for index, valid in enumerate(planning["valid"]):
        _require(isinstance(valid, bool), "target valid mask must be Boolean")
        if not valid:
            invalid_started = True
            _require(planning["positions_base_xy_m"][index] == [None, None] and
                     all(planning[name][index] is None for name in ("speed_mps", "yaw_rad", "target_timestamp_ns")),
                     "invalid target values must remain null")
            _require(planning["invalid_reason"][index] in ("episode_end", "sensor_gap", "label_unavailable"),
                     "invalid target requires a known reason")
            continue
        _require(not invalid_started, "target valid mask must be a contiguous prefix")
        xy = _point(planning["positions_base_xy_m"][index], "target XY")
        speed = _number(planning["speed_mps"][index], "target speed")
        _require(speed >= 0, "target speed cannot be negative")
        yaw = _number(planning["yaw_rad"][index], "target yaw")
        timestamp = _integer(planning["target_timestamp_ns"][index], "target timestamp")
        _require(timestamp == anchor + round((index + 1) * PHYSICAL_TIME_STEP_S * 1.0e9),
                 "target timestamp is not on the declared 100 ms future grid")
        _require(timestamp <= episode_end_ns and planning["invalid_reason"][index] is None,
                 "valid target crosses episode bounds or declares an invalid reason")
        dt = (timestamp - previous_time) / 1.0e9
        acceleration = (speed - previous_speed) / dt
        displacement = (xy[0] - previous_xy[0], xy[1] - previous_xy[1])
        distance = math.hypot(*displacement)
        geo_speed = distance / dt
        geo_acceleration = (geo_speed - previous_geo_speed) / dt
        curvature = lateral = None
        if distance > HEADING_MINIMUM_STEP_M:
            heading = math.atan2(displacement[1], displacement[0])
            curvature = abs(_wrap(heading - previous_heading)) / distance
            lateral = curvature * max(previous_geo_speed, geo_speed) ** 2
            previous_heading = heading
        # HH_260906 - Vehicle yaw is only a proxy for decoder path heading; do not call its mismatch proof of XY impossibility.
        yaw_delta = abs(_wrap(yaw - previous_yaw))
        yaw_limit = min(PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,
            PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2 / max(previous_speed, speed, 1.0e-6) ** 2) * speed * dt
        yaw_assessed = speed * dt > HEADING_MINIMUM_STEP_M
        entries = {
            "speed_limit": (speed, speed > PHYSICAL_MAXIMUM_SPEED_MPS + ROUNDING_TOLERANCE),
            "speed_acceleration": (acceleration, acceleration > PHYSICAL_MAXIMUM_ACCELERATION_MPS2 + ROUNDING_TOLERANCE),
            "speed_deceleration": (acceleration, acceleration < -PHYSICAL_MAXIMUM_ACCELERATION_MPS2 - ROUNDING_TOLERANCE),
            "xy_speed_limit": (geo_speed, geo_speed > PHYSICAL_MAXIMUM_SPEED_MPS + ROUNDING_TOLERANCE),
            "xy_acceleration": (geo_acceleration, geo_acceleration > PHYSICAL_MAXIMUM_ACCELERATION_MPS2 + ROUNDING_TOLERANCE),
            "xy_deceleration": (geo_acceleration, geo_acceleration < -PHYSICAL_MAXIMUM_ACCELERATION_MPS2 - ROUNDING_TOLERANCE),
            "xy_curvature": (curvature, curvature is not None and curvature > PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M + ROUNDING_TOLERANCE),
            "xy_lateral_acceleration": (lateral, lateral is not None and lateral > PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2 + ROUNDING_TOLERANCE),
            "yaw_label_heading_envelope": (yaw_delta if yaw_assessed else None, yaw_assessed and yaw_delta > yaw_limit + ROUNDING_TOLERANCE),
        }
        future_goal_distance = math.dist(goal, xy)
        future_braking_distance = max(previous_speed, speed) ** 2 / (2.0 * PHYSICAL_MAXIMUM_ACCELERATION_MPS2)
        steps.append({"index": index, "target_timestamp_ns": timestamp,
            "episode_relative_100ms_tick": round((timestamp - episode_start_ns) / 1.0e8),
            "phase": "stationary_tail" if tail_start_ns is not None and timestamp >= tail_start_ns else "pre_tail" if tail_start_ns is not None else "unknown",
            "future_region": "near_goal" if future_goal_distance <= future_braking_distance + NEAR_GOAL_BUFFER_M else "interior",
            "dt_s": dt, "metrics": entries, "speed_xy_absolute_error_mps": abs(geo_speed - speed),
            "absolute_yaw_rate_radps": yaw_delta / dt,
            "measured_planar_initial_acceleration_mps2": (speed - current_planar) / dt if index == 0 else None})
        previous_xy, previous_speed, previous_geo_speed = xy, speed, geo_speed
        previous_yaw, previous_time = yaw, timestamp
    _require(steps, "sample must have at least one valid target")
    return {"sample_id": token, "anchor_region": anchor_region, "steps": steps,
        "valid_points": len(steps), "invalid_points": 64 - len(steps),
        "raw_longitudinal_speed_mps": velocity[0], "decoder_initial_speed_mps": initial_speed,
        "measured_planar_speed_mps": current_planar,
        "all_valid_target_speeds_at_most_0p1_mps": all(value <= 0.1 for value in planning["speed_mps"][:len(steps)])}


def summarize_samples(samples, horizon):
    """HH_260906 - Count overlapping anchor windows and unique rounded target ticks separately."""
    selected = [(sample, sample["steps"][:horizon]) for sample in samples]
    summary = {"sample_count": len(samples), "horizon_s": horizon * PHYSICAL_TIME_STEP_S,
        "horizon_eligible_anchor_count": sum(sample["valid_points"] >= horizon for sample in samples),
        "valid_step_count": sum(len(steps) for _, steps in selected),
        "invalid_step_count": sum(horizon - len(steps) for _, steps in selected), "metrics": {}}
    for metric in METRICS:
        assessed, violations, anchors, eligible_anchors = [], [], set(), set()
        phase_counts, region_counts = {}, {}
        phase_anchors = {}
        for sample, steps in selected:
            for step in steps:
                value, failed = step["metrics"][metric]
                if value is None:
                    continue
                witness = {"sample_id": sample["sample_id"], "point_index": step["index"],
                    "target_timestamp_ns": step["target_timestamp_ns"], "value": value}
                assessed.append(witness)
                if failed:
                    violations.append((sample["episode_id"], step["episode_relative_100ms_tick"]))
                    anchors.add(sample["sample_id"])
                    if sample["valid_points"] >= horizon:
                        eligible_anchors.add(sample["sample_id"])
                    phase_counts[step["phase"]] = phase_counts.get(step["phase"], 0) + 1
                    region_counts[step["future_region"]] = region_counts.get(step["future_region"], 0) + 1
                    phase_anchors.setdefault(step["phase"], set()).add(sample["sample_id"])
        summary["metrics"][metric] = {"assessed_step_count": len(assessed), "violating_step_count": len(violations),
            "violating_anchor_count": len(anchors), "horizon_eligible_violating_anchor_count": len(eligible_anchors),
            "unique_episode_relative_100ms_violation_tick_count": len(set(violations)),
            "violating_steps_by_capture_phase": phase_counts, "violating_steps_by_future_goal_proximity": region_counts,
            "violating_anchors_by_capture_phase": {key: len(value) for key, value in phase_anchors.items()},
            "minimum": min(assessed, key=lambda value: value["value"]) if assessed else None,
            "maximum": max(assessed, key=lambda value: value["value"]) if assessed else None}
    all_steps = [step for _, steps in selected for step in steps]
    summary["speed_xy_consistency_max_absolute_error_mps"] = max(
        (step["speed_xy_absolute_error_mps"] for step in all_steps), default=None)
    summary["maximum_absolute_vehicle_yaw_rate_radps"] = max(
        (step["absolute_yaw_rate_radps"] for step in all_steps), default=None)
    return summary


def _group(samples):
    return {"sample_count": len(samples), "all_valid_target_speeds_at_most_0p1_mps_count":
        sum(sample["all_valid_target_speeds_at_most_0p1_mps"] for sample in samples),
        "horizons": {f"{horizon / 10:.1f}s": summarize_samples(samples, horizon) for horizon in HORIZONS}}


def audit_dataset(root, expected_manifest_sha256):
    """HH_260906 - Read split-identifying metadata but never open held-out test samples, images or routes."""
    root = Path(root).resolve(strict=True)
    manifest, manifest_sha = _read_json_and_sha256(root / "dataset.json")
    _require(manifest_sha == expected_manifest_sha256, "dataset manifest SHA-256 mismatch")
    contract, contract_file_sha = _read_json_and_sha256(DEFAULT_CONTRACT_PATH)
    validate_contract(contract)
    # HH_260906 - Dataset manifests bind the canonical contract, while the evidence also records source-file bytes.
    contract_sha = contract_fingerprint(contract)
    _require(manifest.get("contract_sha256") == contract_sha, "dataset contract SHA-256 mismatch")
    _require(manifest.get("contract_id") == contract.get("contract_id") == "common_10hz_v1", "unexpected dataset contract")
    inputs = [{"path": "dataset.json", "sha256": manifest_sha}]
    splits, episodes, skipped, seen_ids = {"train": [], "val": []}, [], [], set()
    for reference in manifest["episodes"]:
        episode_path = _safe_file(root, reference["manifest"], "episode manifest")
        episode, episode_sha = _read_json_and_sha256(episode_path)
        _require(episode_sha == reference["sha256"], "episode manifest SHA-256 mismatch")
        episode_id, split = episode["episode_id"], episode["split"]
        _require(episode_id == reference["episode_id"] and episode_id not in seen_ids, "duplicate or mismatched episode identity")
        seen_ids.add(episode_id)
        _require(split in ("train", "val", "test"), "unknown episode split")
        inputs.append({"path": str(episode_path.relative_to(root)), "sha256": episode_sha})
        if split == "test":
            skipped.append({"episode_id": episode_id, "sample_data_opened": False})
            continue
        route_path = _safe_file(episode_path.parent, episode["route_geometry_file"], "route geometry")
        route, route_sha = _read_json_and_sha256(route_path)
        _require(route_sha == episode["route_geometry_sha256"], "route geometry SHA-256 mismatch")
        points = [_point(point, "route point") for point in route["polyline_m"]]
        _require(len(points) >= 2, "route needs at least two points")
        length = sum(math.dist(left, right) for left, right in zip(points, points[1:]))
        _require(length > 0.0, "route length must be positive")
        inputs.append({"path": str(route_path.relative_to(root)), "sha256": route_sha})
        provenance = episode["source_provenance"]
        config_path = _safe_file(episode_path.parent, provenance["collection_config_file"], "collection config")
        config, config_sha = _read_json_and_sha256(config_path)
        _require(config_sha == provenance["collection_config_sha256"], "collection config SHA-256 mismatch")
        inputs.append({"path": str(config_path.relative_to(root)), "sha256": config_sha})
        phases = config.get("native_result", {}).get("capture_phases", {})
        tail_first = phases.get("stationary_tail", {}).get("first_timestamp")
        tail_ns = None if tail_first is None else round(_number(tail_first, "tail first_timestamp") * 1.0e9)
        sample_path = _safe_file(episode_path.parent, episode["sample_jsonl"], "train/val sample JSONL")
        samples = []
        previous_anchor = -1
        for _, sample in _iter_jsonl(sample_path, expected_sha256=episode["sample_jsonl_sha256"]):
            _require(sample.get("episode_id") == episode_id, "sample episode identity mismatch")
            _require(_integer(sample.get("sequence_index"), "sample sequence") == len(samples),
                     "sample sequence must be contiguous from zero")
            row = audit_sample(sample, route_length_m=length,
                episode_start_ns=_integer(episode["start_timestamp_ns"], "episode start"),
                episode_end_ns=_integer(episode["end_timestamp_ns"], "episode end"), tail_start_ns=tail_ns)
            _require(sample["anchor_timestamp_ns"] > previous_anchor, "sample anchors must increase")
            previous_anchor = sample["anchor_timestamp_ns"]
            row["episode_id"] = episode_id
            samples.append(row)
        _require(len(samples) == _integer(episode["sample_count"], "episode sample count") and samples,
                 "episode sample count mismatch")
        _require(len({row["sample_id"] for row in samples}) == len(samples), "duplicate sample IDs")
        inputs.append({"path": str(sample_path.relative_to(root)), "sha256": episode["sample_jsonl_sha256"]})
        splits[split].extend(samples)
        episodes.append({"episode_id": episode_id, "split": split, "map_id": episode.get("map_id"),
            "site_id": episode.get("site_id"), "route_length_m": length, "stationary_tail_first_timestamp_ns": tail_ns,
            "capture_accounting": episode.get("capture_accounting", {}), "overall": _group(samples),
            "by_anchor_goal_proximity": {region: _group([sample for sample in samples if sample["anchor_region"] == region])
                for region in ("near_goal", "interior")}})
    _require(all(splits.values()), "audit requires nonempty train and val splits")
    _require(len({row["sample_id"] for samples in splits.values() for row in samples}) == sum(map(len, splits.values())),
             "sample IDs overlap between train/val episodes")
    # HH_260906 - Rehash only files actually read; this does not validate or expose held-out sample contents.
    for item in inputs:
        _require(_sha(root / item["path"]) == item["sha256"], "audit input changed during analysis")
    return {"schema": SCHEMA, "status": "TARGET_ENVELOPE_AUDIT_COMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "dataset_id": manifest["dataset_id"],
        "dataset_manifest_sha256": manifest_sha, "contract_sha256": contract_sha,
        "contract_file_sha256": contract_file_sha,
        "audit_script_sha256": _sha(Path(__file__)), "model_source_sha256": _sha(REPO / "portable_e2e/model.py"),
        "input_manifest": inputs, "splits": {split: _group(samples) for split, samples in splits.items()},
        "episodes": episodes, "skipped_test_episodes": skipped,
        "scope": {"training_labels_opened": True, "validation_labels_opened": True,
            "test_sample_data_opened": False, "test_episode_metadata_read_to_identify_split": bool(skipped),
            "images_opened": False, "model_inference_run": False, "labels_modified": False,
            "vehicle_control_approved": False, "full_dataset_contract_validation_run": False},
        "physical_decoder_limits": {"timestep_s": PHYSICAL_TIME_STEP_S, "speed_mps": PHYSICAL_MAXIMUM_SPEED_MPS,
            "acceleration_and_deceleration_mps2": PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
            "curvature_rad_per_m": PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,
            "lateral_acceleration_mps2": PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2,
            "route_slip_rad": PHYSICAL_MAXIMUM_ROUTE_SLIP_RAD},
        "definitions": {
            "speed_acceleration": "Current clamped longitudinal vx to first target speed, then consecutive valid target speed differences divided by actual target timestamp delta.",
            "xy_acceleration": "Origin-to-first and consecutive target XY distances divided by timestamp delta, then differences from decoder initial speed. These are necessary XY-only scalar bounds for exact discrete decoder representability, not a sufficient feasibility test.",
            "xy_curvature_and_lateral": "Wrapped segment heading change divided by segment distance, times max(entry/current XY-derived speed)^2 for lateral acceleration. Initial heading is zero.",
            "yaw_label_heading_envelope": "Vehicle yaw-label delta compared with decoder heading-step bound based on target speed; proxy only because vehicle yaw and displacement direction can differ.",
            "heading_unassessed": f"XY distance or target-speed distance <= {HEADING_MINIMUM_STEP_M} m is unassessed for the corresponding heading diagnostic, not counted as passing.",
            "rounding_tolerance": ROUNDING_TOLERANCE,
            "anchor_goal_proximity": f"Remaining route arc <= current planar speed^2/(2*decoder acceleration bound)+{NEAR_GOAL_BUFFER_M} m.",
            "future_goal_proximity": f"Goal-to-future-target Euclidean distance <= max(entry/target speed)^2/(2*decoder acceleration bound)+{NEAR_GOAL_BUFFER_M} m; analytical grouping, not a safety gate.",
            "stationary_tail_phase": "Timestamp at or after native capture stationary_tail.first_timestamp; phase name does not imply the vehicle was already stationary.",
            "denominators": "Cumulative valid prefixes through 1/3/6.4 s. Horizon-eligible anchors are counted separately; overlapping windows are not independent physical events.",
            "unique_ticks": "Distinct episode ID and nearest episode-relative 100 ms target tick, not distinct braking maneuvers.",
        },
        "limitations": ["This audits labels against a particular bounded discrete decoder, not vehicle physics or public-road safety.",
            "Speed-label violations alone do not prove XY infeasibility; separate XY-derived tests are reported.",
            "Sensor/integration timing, endpoint-speed versus interval-average-speed conventions and near-stationary pose noise can affect these diagnostics.",
            "Yaw proxy violations are not direct XY impossibility proofs; route-relative slip/reachability is not solved by this audit.",
            "No test samples, model predictions, image validation, label rewrite, filtering or automatic promotion are performed."]}


def main(argv=None):
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--dataset-manifest-sha256", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        _require(not args.output_json.exists() and not args.output_json.is_symlink(), "output already exists")
        _require(not args.output_json.resolve().is_relative_to(args.dataset.resolve()),
                 "output must be outside the read-only dataset directory")
        report = audit_dataset(args.dataset, args.dataset_manifest_sha256)
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        with args.output_json.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
    except (ContractError, OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(f"TARGET_FEASIBILITY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "train_samples": report["splits"]["train"]["sample_count"],
        "val_samples": report["splits"]["val"]["sample_count"], "test_sample_data_opened": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
