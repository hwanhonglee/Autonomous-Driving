#!/usr/bin/env python3
"""HH_261001 - Audit real retained W1 moving-brake attempts without inference or CARLA."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMPAIGN = ROOT / "storage/experiments/validation/2026-10-01/town07_w1_moving_brake_exploratory_v2"
DEFAULT_PLAN = DEFAULT_CAMPAIGN.with_name("town07_w1_moving_brake_exploratory_plan_v2.json")
ARMS = ("A_original", "B_suppress_normal_moving_brake")
CHECKPOINT = "b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76"
MODEL_STATE = "ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f"
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
MAX_ROWS, MAX_BYTES = 1000, 128 * 1024**2
STATIC_MONTAGE_SECONDS = 5.0
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular input file required: " + str(path))
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read(path):
    digest(path)
    require(Path(path).stat().st_size <= MAX_BYTES, "bounded JSON input required")
    return json.loads(Path(path).read_text())


def rows(path):
    digest(path)
    require(Path(path).stat().st_size <= MAX_BYTES, "bounded JSONL input required")
    result = []
    with Path(path).open() as stream:
        for line in stream:
            result.append(json.loads(line))
            require(len(result) <= MAX_ROWS, "bounded native journal required")
    return result


def number(value, lower=-math.inf, upper=math.inf):
    return type(value) in (int, float) and math.isfinite(value) and lower <= value <= upper


def valid_control(control):
    return isinstance(control, dict) and all(number(control.get(key), -1.0 if key == "steer" else 0.0, 1.0)
        for key in ("throttle", "brake", "steer")) and all(type(control.get(key)) is bool
        for key in ("reverse", "hand_brake", "manual_gear_shift")) and type(control.get("gear")) is int


def audit_control_decision(decision, arm):
    """HH_261001 - Compare serialized control fields and recompute intervention flags."""
    if arm not in ARMS:
        return ["undeclared_arm"]
    if decision.get("control_source") == "expert_startup":
        return [] if decision.get("learned_control_proposed") is False else ["expert_claimed_learned"]
    audit = decision.get("moving_brake_ab")
    if not isinstance(audit, dict):
        return ["missing_intervention_audit"]
    issues = []
    returned = decision.get("control_source") == "learned_trajectory_tracker" and decision.get("learned_control_proposed") is True and decision.get("emergency_reason") is None
    if audit.get("arm") != arm or audit.get("normal_control_returned") is not returned:
        issues.append("arm_or_normal_return_flag")
    original = audit.get("original_normal_tracker_control")
    expected = None
    changed = False
    if original is not None:
        if not isinstance(original, dict) or not number(original.get("desired_speed_mps"), 0.0, 2.5) or not number(original.get("brake"), 0.0, 1.0):
            return issues + ["malformed_original_tracker_control"]
        # HH_261001 - The original policy serializes tuples to lists before this journal.
        expected = json.loads(json.dumps(original, allow_nan=False))
        moving = original["desired_speed_mps"] > 0.05
        if arm == ARMS[1] and moving:
            expected["brake"] = 0.0
        changed = expected["brake"] != original["brake"]
        flags = {"normal_tracker_called": True, "moving_target": moving, "stop_speed_threshold_mps": 0.05,
                 "desired_speed_mps": original["desired_speed_mps"], "original_normal_brake": original["brake"],
                 "proposed_normal_brake": expected["brake"], "normal_brake_suppression_proposed": changed}
        if any(audit.get(key) != value or isinstance(value, bool) and type(audit.get(key)) is not bool for key, value in flags.items()):
            issues.append("intervention_metadata")
        if returned and decision.get("control") != expected:
            issues.append("another_tracker_field_changed")
    elif audit.get("normal_tracker_called") is not False or returned or audit.get("normal_brake_suppression_proposed") is not False:
        issues.append("missing_original_tracker_proposal")
    if audit.get("normal_brake_suppression_returned") is not (returned and changed):
        issues.append("returned_suppression_flag")
    if any(audit.get(key, False) is not False for key in ("feedforward_added", "integral_added", "control_cadence_changed", "stop_branch_changed", "emergency_authority_changed")):
        issues.append("expanded_intervention")
    if "moving_brake_ab" in decision.get("control", {}) or "normal_brake_suppression_returned" in decision.get("control", {}):
        issues.append("audit_inside_actuator_control")
    return issues


def check_state_receipt_binding(states, receipts, actor_id):
    issues = Counter()
    lookup = {receipt["sequence"]: receipt for receipt in receipts}
    if [receipt["sequence"] for receipt in receipts] != list(range(1, len(receipts) + 1)):
        issues["receipt_sequence"] += 1
    for receipt in receipts:
        if not (receipt.get("status") == "ACKNOWLEDGED" and receipt.get("server_accepted") is True
                and receipt.get("actor_id") == receipt.get("response_actor_id") == actor_id
                and receipt.get("before_frame") == receipt.get("after_ack_frame")
                and receipt.get("expected_before_frame", receipt.get("before_frame")) == receipt.get("before_frame")
                and receipt.get("do_tick") is False):
            issues["receipt_acceptance_or_frame"] += 1
        if not valid_control(receipt.get("requested_control")):
            issues["invalid_requested_control"] += 1
    for index, state in enumerate(states):
        alignment = state.get("control_transport", state.get("control_alignment", {}))
        receipt = lookup.get(alignment.get("expected_receipt_sequence"))
        if type(state["frame"]) is not int or not number(state["timestamp"]):
            issues["invalid_frame_timestamp"] += 1
        if index and (state["frame"] != states[index-1]["frame"]+1 or abs(state["timestamp"]-states[index-1]["timestamp"]-0.05)>1e-6):
            issues["native_cadence"] += 1
        if not receipt:
            issues["missing_receipt"] += 1
            continue
        if not (receipt["before_frame"] == receipt["after_ack_frame"] == state["frame"]-1
                and alignment.get("expected_command_before_frame") == state["frame"]-1
                and alignment.get("observed_frame") == alignment.get("control_read_frame_before") == alignment.get("control_read_frame_after") == state["frame"]
                and alignment.get("status") == "PASS" and alignment.get("frame_binding_pass") is True):
            issues["state_command_frame_binding"] += 1
        observed, requested = state["current_control"], receipt["requested_control"]
        if not valid_control(observed):
            issues["invalid_observed_control"] += 1
        if valid_control(observed) and valid_control(requested):
            if any(abs(observed[key]-requested[key]) > 1e-6 for key in ("throttle", "brake", "steer")):
                issues["pedal_readback"] += 1
            if any(observed[key] != requested[key] for key in ("reverse", "hand_brake", "manual_gear_shift")):
                issues["flag_readback"] += 1
            if requested["manual_gear_shift"] and observed["gear"] != requested["gear"]:
                issues["manual_gear_readback"] += 1
    return {"issue_counts": dict(issues), "state_count": len(states), "receipt_count": len(receipts)}


def summarize_states(states, decisions):
    if not states:
        return {"state_count": 0, "observed_native_span_seconds": None, "nominal_native_coverage_seconds": 0.0,
                "full_twenty_second_native_count": False, "net_route_progress_m": None}
    require(all(number(row["timestamp"]) and number(row["vx"]) and number(row["route_progress_m"]) for row in states), "finite summary measurements required")
    rates = [(right["vx"]-left["vx"])/(right["timestamp"]-left["timestamp"]) for left, right in zip(states, states[1:])]
    moving, stop = False, None
    for state in states:
        moving = moving or state.get("capture_phase") == "driving" and state["vx"] > 0.5
        if moving and abs(state["vx"]) <= 0.05:
            stop = state["timestamp"]-states[0]["timestamp"]
            break
    inferred = {}
    for decision in decisions:
        if type(decision.get("inference_frame")) is int:
            inferred.setdefault(decision["inference_frame"], decision)
    latencies = [row["total_seconds"] for row in inferred.values() if number(row.get("total_seconds"))]
    # HH_261001 - p95 uses nearest rank, index ceil(0.95*n)-1, without interpolation.
    latency_p95 = sorted(latencies)[math.ceil(0.95*len(latencies))-1] if latencies else None
    world_speeds = [math.hypot(row["world_velocity_carla"][0], row["world_velocity_carla"][1])
                    for row in states if "world_velocity_carla" in row]
    return {"state_count": len(states), "observed_native_span_seconds": states[-1]["timestamp"]-states[0]["timestamp"],
        "nominal_native_coverage_seconds": len(states)*0.05, "full_twenty_second_native_count": len(states)==400,
        "net_route_progress_m": states[-1]["route_progress_m"]-states[0]["route_progress_m"],
        "final_route_progress_m": states[-1]["route_progress_m"], "final_speed_mps": states[-1]["vx"],
        "maximum_forward_speed_mps": max(row["vx"] for row in states),
        "maximum_world_planar_speed_mps": max(world_speeds) if len(world_speeds)==len(states) else None,
        "signed_rate_below_minus_8_mps2_count": sum(rate < -8.0 for rate in rates),
        "minimum_signed_rate_mps2": min(rates, default=None), "maximum_abs_signed_rate_mps2": max(map(abs, rates), default=None),
        "first_speed_at_or_below_0p05_after_motion_s": stop, "first_stop_requires_prior_driving_speed_above_mps": 0.5,
        "maximum_abs_route_cte_m": max(abs(row["route_cte_m"]) for row in states),
        "collision_events": sum(len(row.get("collision", [])) for row in states), "lane_invasion_events": sum(len(row.get("lane_invasion", [])) for row in states),
        "unique_inference_frames_count": len(inferred), "retained_control_decision_count": len(decisions),
        "acknowledged_learned_command_count": sum(row.get("application", {}).get("learned_control_applied") is True for row in decisions),
        "normal_suppression_returned_command_count": sum(row.get("moving_brake_ab", {}).get("normal_brake_suppression_returned") is True for row in decisions),
        "normal_suppression_unique_inference_count": sum(row.get("moving_brake_ab", {}).get("normal_brake_suppression_returned") is True for row in inferred.values()),
        "control_source_counts": dict(Counter(row.get("control_source") for row in decisions)),
        "inference_seconds": {"median": statistics.median(latencies) if latencies else None,
            "p95": latency_p95, "p95_convention": "nearest rank: sorted[ceil(0.95*n)-1], no interpolation",
            "sample_count": len(latencies), "maximum": max(latencies, default=None)},
        "current_automatic_gear_counts": dict(Counter(row["current_control"]["gear"] for row in states)),
    }


def export_tensor_identity(declared):
    """HH_261001 - Hash data-only NPZ tensors on CPU; construct no model and run no inference."""
    import numpy as np
    weights = next(Path(name) for name in declared["asset_sha256"] if name.endswith(".npz"))
    metadata = read(weights.with_suffix(".json"))
    require(metadata["source_checkpoint_sha256"] == CHECKPOINT and metadata["source_checkpoint_schema"] == declared["checkpoint_schema"], "W1 metadata identity differs")
    result = hashlib.sha256()
    with np.load(weights, allow_pickle=False) as arrays:
        require(set(arrays.files) == set(metadata["tensors"]), "export tensor keys differ")
        for name in sorted(arrays.files):
            value = arrays[name]
            require(value.dtype == np.float32 and np.isfinite(value).all() and list(value.shape) == metadata["tensors"][name]["shape"], "finite export tensor ABI required")
            header = json.dumps([name, "torch.float32", list(value.shape)], sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
            result.update(len(header).to_bytes(8, "big")); result.update(header); result.update(np.ascontiguousarray(value).tobytes())
    require(result.hexdigest() == MODEL_STATE == declared["model_state_sha256"], "actual exported W1 tensor identity differs")
    return {"checkpoint_sha256": CHECKPOINT, "actual_data_only_tensor_sha256": result.hexdigest(), "new_inferences": 0, "new_models_constructed": 0}


def source_checks(run, owner_plan, declared, plan_sha):
    issues, local = [], dict(owner_plan["source_sha256"])
    for name, pinned in declared["source_sha256"].items():
        if name in local and local[name] != pinned:
            issues.append("conflicting_local_pin:"+name)
        local[name] = pinned
    for name, pinned in local.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "safe relative source path required")
        if digest(ROOT/relative) != pinned or digest(run/"source"/relative) != pinned:
            issues.append("local_source:"+name)
    for name, pinned in declared["external_agent_sha256"].items():
        if digest(name) != pinned or digest(run/"source_external"/Path(name).relative_to(CARLA_ROOT)) != pinned:
            issues.append("external_source:"+name)
    if digest(run/"declared_plan.json") != plan_sha:
        issues.append("archived_declaration")
    return {"matches": not issues, "issues": issues, "local_file_count": len(local), "external_file_count": len(declared["external_agent_sha256"])}


def sample_archive_checks(run, decisions, states, maximum_bytes):
    """HH_261001 - Verify retained sparse input arrays; never reconstruct unrecorded inputs."""
    import numpy as np
    directory = run/"sampled_exact_runtime_inputs"
    if not directory.is_dir():
        return {"present": False, "issue_counts": {}, "sample_count": 0}
    issues = Counter()
    files = list(directory.iterdir())
    require(all(path.is_file() and not path.is_symlink() for path in files), "regular sample archive files required")
    total = sum(path.stat().st_size for path in files)
    if total > maximum_bytes:
        issues["sample_directory_allowance"] += 1
    index = rows(directory/"index.jsonl")
    if [row["sequence"] for row in index] != list(range(len(index))):
        issues["sample_sequence"] += 1
    times = {row["frame"]:int(round(row["timestamp"]*1e9)) for row in states}
    for row in index:
        relative = Path(row["tensor_file"])
        require(not relative.is_absolute() and ".." not in relative.parts, "safe sampled tensor filename required")
        path = directory/relative
        if digest(path) != row["tensor_file_sha256"]:
            issues["sample_npz_hash"] += 1
        with np.load(path, allow_pickle=False) as arrays:
            if set(arrays.files) != set(row["tensor_sha256"]):
                issues["sample_tensor_keys"] += 1
            for key in arrays.files:
                value = np.ascontiguousarray(arrays[key])
                header = json.dumps(dict(shape=list(value.shape), dtype=str(value.dtype)), sort_keys=True).encode()
                if not np.isfinite(value).all() or hashlib.sha256(header+b"\0"+value.tobytes()).hexdigest() != row["tensor_sha256"].get(key):
                    issues["sample_tensor_content"] += 1
        matches = [decision for decision in decisions if times.get(decision.get("command_frame")) == row["timing"]["now_ns"]]
        if not any(decision.get("raw_output") is not None and hashlib.sha256(json.dumps(decision["raw_output"],sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()==row["raw_output_sha256"] for decision in matches):
            issues["sample_decision_raw_output_link"] += 1
        if row["source_checkpoint_sha256"] != CHECKPOINT:
            issues["sample_checkpoint_identity"] += 1
    return {"present":True, "issue_counts":dict(issues), "sample_count":len(index), "directory_bytes":total,
            "maximum_bytes":maximum_bytes, "index_sha256":digest(directory/"index.jsonl"), "all_inputs_retained":False}


def audit_trial(run, spec, declared, plan_sha):
    owner = read(run/"owner_diagnostic_result.json"); plan = read(run/"owner_plan.json")
    episode = run/"episode.partial" if (run/"episode.partial").is_dir() else run/"episode"
    manifest = read(episode/"manifest.json")
    states, cameras, decisions, receipts = (rows(episode/name) if (episode/name).is_file() else [] for name in ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl"))
    issues = Counter()
    actor_id = manifest.get("capture_contract", {}).get("actor_body_geometry", {}).get("actor_id", receipts[0]["actor_id"] if receipts else None)
    issues.update(check_state_receipt_binding(states, receipts, actor_id)["issue_counts"])
    archives = source_checks(run, plan, declared, plan_sha)
    if not archives["matches"]:
        issues["source_hashes"] += 1
    # HH_261001 - Audit the actual episode map/route bytes, beyond packaged asset checks.
    route_expected = next(pinned for name, pinned in declared["asset_sha256"].items() if name.endswith("/route.json"))
    map_expected = next(pinned for name, pinned in declared["asset_sha256"].items() if name.endswith("/Town07.xodr"))
    if digest(episode/"route.json") != route_expected:
        issues["archived_episode_route"] += 1
    if digest(episode/"active_map_opendrive.xodr") != map_expected:
        issues["archived_active_map"] += 1
    endpoint = read(run/"worker_endpoint_identity.json")
    if not (endpoint["trial"] == plan["trial"] == owner["trial"] == spec
            and endpoint["source_checkpoint_sha256"] == plan["source_checkpoint_sha256"] == owner["source_checkpoint_sha256"] == CHECKPOINT
            and endpoint["model_state_sha256"] == plan["model_state_sha256"] == MODEL_STATE
            and endpoint["declaration_sha256"] == plan["declaration_sha256"] == owner["declaration_sha256"] == plan_sha):
        issues["runtime_identity"] += 1
    by_frame, by_receipt = {row["frame"]: row for row in states}, {row["sequence"]: row for row in receipts}
    for state in states:
        if not all(number(state.get(key)) for key in ("timestamp", "route_progress_m", "route_cte_m", "vx")):
            issues["nonfinite_native_state"] += 1
        if "world_velocity_carla" in state:
            velocity = state["world_velocity_carla"]; yaw = math.radians(state["actor_snapshot_transform_carla"]["yaw"])
            if not all(number(value) for value in velocity) or abs(math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1]-state["vx"])>1e-9:
                issues["raw_forward_speed"] += 1
    for decision in decisions:
        issues.update(audit_control_decision(decision, spec["arm"]))
        if decision.get("source_checkpoint_sha256") != CHECKPOINT:
            issues["decision_checkpoint"] += 1
        receipt = by_receipt.get(decision.get("ack_receipt_sequence"))
        if not receipt:
            issues["decision_missing_ack"] += 1
            continue
        if "command_frame" in decision and receipt["before_frame"] != decision["command_frame"]:
            issues["decision_command_frame"] += 1
        if decision.get("control_source") != "expert_startup":
            proposed = decision["control"]
            if any(abs(proposed[key]-receipt["requested_control"][key])>1e-6 for key in ("throttle", "brake", "steer")):
                issues["decision_pedal_ack"] += 1
        if decision.get("application", {}).get("acknowledged") is not True:
            issues["decision_application_ack"] += 1
        if decision.get("learned_control_proposed") is True:
            frame = decision["inference_frame"]
            if frame not in by_frame:
                issues["inference_frame_absent"] += 1
            selected, raw = decision["selected"], decision["raw_output"]
            index = selected["candidate_index"]
            if not (decision["selection_policy"] == "raw_argmax_fail_closed" and index == max(range(12), key=lambda item: raw["candidate_logits"][item])
                    and selected["speed_mps"] == raw["candidate_speed"][index] and selected["xy_base_m"] == raw["candidate_xy"][index]
                    and decision["candidate_gates"][index]["passed"] is True):
                issues["raw_selection_or_gate"] += 1
    image_hashes = hashlib.sha256()
    from PIL import Image
    decoded_images = 0
    for camera in cameras:
        if camera["frame"] not in by_frame or camera["timestamp"] != by_frame[camera["frame"]]["timestamp"] or tuple(camera["camera_order"]) != CAMERAS:
            issues["camera_state_binding"] += 1
        if camera["timestamp_span_sec"] != 0.0:
            issues["camera_skew"] += 1
        if set(camera["images"]) != set(CAMERAS) or any(camera["source_timestamps"].get(name) != camera["timestamp"] for name in CAMERAS):
            issues["camera_source_timestamp_or_membership"] += 1
        for name in CAMERAS:
            relative = Path(camera["images"][name]); require(not relative.is_absolute() and ".." not in relative.parts, "safe camera reference required")
            image_hashes.update(str(relative).encode()); image_hashes.update(digest(episode/relative).encode())
            try:
                with Image.open(episode/relative) as image:
                    require(image.size == (640,360) and image.format == "JPEG", "original camera image ABI differs")
                    image.load()
                    decoded_images += 1
            except (OSError, ValueError, Image.DecompressionBombError):
                issues["original_camera_decode"] += 1
    if cameras and [row["frame"] for row in cameras] != [row["frame"] for row in states[::2]]:
        issues["camera_cadence"] += 1
    sample_audit = sample_archive_checks(run, decisions, states, declared["sample_directory_maximum_bytes"])
    issues.update(sample_audit["issue_counts"])
    summary = summarize_states(states, decisions)
    cleaned = owner.get("port_triplet_released") is True and not owner.get("owner_cleanup_errors")
    if not cleaned:
        issues["owner_cleanup"] += 1
    full = len(states)==400 and len(cameras)==200 and not issues
    initial = {key: states[0][key] for key in ("x", "y", "z", "yaw", "vx", "route_progress_m")} if states else None
    first_learned = next((row for row in decisions if row.get("application", {}).get("learned_control_applied") is True), None)
    handoff = by_frame.get(first_learned.get("command_frame")) if first_learned else None
    handoff_fields = {key: handoff[key] for key in ("x", "y", "z", "yaw", "vx", "route_progress_m")} if handoff else None
    core = {name: digest(episode/name) for name in ("manifest.json", "states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl") if (episode/name).is_file()}
    result = {"trial_id": spec["trial_id"], "arm": spec["arm"], "run": str(run), "owner_status": owner["status"],
        "original_capture_error": owner.get("original_capture_error"), "integrity_issue_counts": dict(issues), "source_archive_audit": archives,
        "cleanup_verified": cleaned, "full_400_state_200_camera_capture": full, "camera_count": len(cameras), "original_jpeg_count": len(cameras)*6,
        "summary": summary, "initial_recorded_state": initial, "first_learned_handoff_state": handoff_fields,
        "source_checkpoint_sha256": CHECKPOINT, "model_state_sha256": MODEL_STATE, "input_sha256": core,
        "sampled_exact_input_audit": sample_audit,
        "original_jpeg_path_and_content_sha256": image_hashes.hexdigest(), "whole_body_clearance_proven": False,
        "original_jpeg_decode_checked_count": decoded_images,
        "bounded_sweep_clearance_proven": False, "training_data_approved": False, "vehicle_control_approved": False}
    return result, {"states": states, "cameras": cameras, "decisions": decisions, "episode": episode, "route": read(episode/"route.json") if (episode/"route.json").is_file() else None}


def analyze(campaign, plan_path, expected_plan_sha, allow_single=False):
    require(digest(plan_path)==expected_plan_sha, "exact declared plan SHA required")
    declared = read(plan_path)
    require(declared["checkpoint_sha256"] == CHECKPOINT and declared["model_state_sha256"] == MODEL_STATE, "fixed W1 identity required")
    specs = declared["trial_order"]
    require(len(specs)==2 and [spec["arm"] for spec in specs]==list(ARMS) and [spec["index"] for spec in specs]==[0, 1], "fixed exploratory A then B required")
    asset_issues = [name for name, pinned in declared["asset_sha256"].items() if digest(name)!=pinned]
    require(not asset_issues, "declared assets changed: "+str(asset_issues))
    tensor_identity = export_tensor_identity(declared)
    results, bundles = [], {}
    for spec in specs:
        run = Path(campaign)/spec["trial_id"]
        if not (run/"owner_diagnostic_result.json").is_file():
            require(allow_single, "both retained terminal attempt results required")
            continue
        result, bundle = audit_trial(run, spec, declared, expected_plan_sha)
        results.append(result); bundles[spec["trial_id"]] = bundle
    require(results, "at least one terminal retained attempt required")
    comparisons = {}
    if len(results)==2:
        for label in ("initial_recorded_state", "first_learned_handoff_state"):
            a, b = (trial[label] for trial in results)
            comparisons[label+"_B_minus_A"] = {key:b[key]-a[key] for key in a} if a and b else None
    return {"schema":"portable_e2e.town07_moving_brake_postrun_report.20261001.v1", "analyzed_at_utc":datetime.now(timezone.utc).isoformat(),
        "declared_plan_sha256":expected_plan_sha, "declared_experiment_schema":declared["schema"], "reporter_sha256":digest(Path(__file__)),
        "trial_count":len(results), "both_terminal_attempts_present":len(results)==2, "tensor_identity":tensor_identity,
        "all_integrity_checks_pass":all(not trial["integrity_issue_counts"] for trial in results),
        "both_full_twenty_second_captures":len(results)==2 and all(trial["full_400_state_200_camera_capture"] for trial in results),
        "matched_start_observations":comparisons, "matching_observations_prove_internal_state":False, "trials":results,
        "exploratory":True, "ABBA":False, "independent_replication":False, "model_promoted":False,
        "training_data_approved":False, "vehicle_control_approved":False,
        "interpretation_limits":["Partial attempts are retained and have no full twenty-second outcome; no trimming or favorable retry is performed.",
            "Native first-to-last span is count minus one intervals; nominal count/20 coverage is reported separately.",
            "The first near-stop excludes setup zeros and requires prior driving vx>0.5 m/s.",
            "Observed input/history and branch equality do not prove equal internal gear, contact or wheel torque.",
            "ACK proves accepted frame-bound commands/readback, not independently measured physical actuation.",
            "A/B use the same W1 weights; changing brake can change subsequent visited states, camera inputs and predictions.",
            "One exploratory attempt per arm is not a replicated causal or safety approval.",
            "Actual CARLA camera images are not an Autoware screen; any 2 Hz GIF is sampled playback, not the recorded 10 Hz stream."]}, bundles


def fresh_output(output, campaign):
    output, campaign = Path(output), Path(campaign).resolve()
    require(not output.exists() and not output.is_symlink(), "fresh create-only report output required")
    require(output.resolve()!=campaign and campaign not in output.resolve().parents, "output must be outside retained campaign")
    output.mkdir(parents=True, exist_ok=False)
    return output


def plot_measured(result, bundles, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for trial in result["trials"]:
        states = bundles[trial["trial_id"]]["states"]
        if not states:
            continue
        times = [row["timestamp"]-states[0]["timestamp"] for row in states]
        label = trial["trial_id"]+(" (partial)" if not trial["full_400_state_200_camera_capture"] else " (400 native states)")
        color = "#1565c0" if trial["arm"]==ARMS[0] else "#d95f02"
        axes[0].plot(times, [row["vx"] for row in states], label=label, color=color)
        axes[1].plot(times, [row["route_progress_m"]-states[0]["route_progress_m"] for row in states], color=color)
        axes[2].plot(times, [row["current_control"]["throttle"] for row in states], color=color, label=trial["trial_id"]+" throttle")
        axes[2].plot(times, [row["current_control"]["brake"] for row in states], color=color, linestyle="--", label=trial["trial_id"]+" brake")
    axes[0].set_ylabel("forward vx (m/s)"); axes[0].legend(fontsize=8)
    axes[1].set_ylabel("net route progress (m)")
    axes[2].set(ylabel="actual control readback", xlabel="simulation seconds since first retained native state"); axes[2].legend(fontsize=8)
    for axis in axes:
        axis.grid(alpha=.25)
    figure.suptitle("Measured W1 moving-brake attempts; retained partial traces stop at their last state")
    figure.tight_layout(); figure.savefig(output/"native_control_progress.png", dpi=140); plt.close(figure)
    figure, axes = plt.subplots(1, max(1,len(result["trials"])), figsize=(11,6), squeeze=False)
    for column, trial in enumerate(result["trials"]):
        bundle = bundles[trial["trial_id"]]; states = bundle["states"]; by_frame = {row["frame"]:row for row in states}
        decision = next((row for row in bundle["decisions"] if row.get("application",{}).get("learned_control_applied") is True), None)
        axis=axes[0,column]
        if not decision or decision["inference_frame"] not in by_frame:
            axis.text(.5,.5,"No acknowledged learned handoff",ha="center",transform=axis.transAxes); continue
        origin=by_frame[decision["inference_frame"]]; c,s=math.cos(origin["yaw"]),math.sin(origin["yaw"])
        def local(row):
            dx,dy=row["x"]-origin["x"],row["y"]-origin["y"]
            return c*dx+s*dy,-s*dx+c*dy
        observed=[local(row) for row in states if origin["timestamp"]<=row["timestamp"]<=origin["timestamp"]+3]
        route=[local(row) for row in bundle["route"]["route"]] if bundle["route"] else []
        predicted=decision["selected"]["xy_base_m"]
        for values,label,color in ((route,"pinned route","gray"),(predicted,"selected predicted trajectory","#9c27b0"),(observed,"actual next 3 s observed states","#00897b")):
            if values:axis.plot([p[1] for p in values],[p[0] for p in values],label=label,color=color)
        axis.plot(0,0,"ko",label="recorded ego reference");axis.set(xlim=(-5,5),ylim=(-2,20),xlabel="left (m)",ylabel="forward (m)",title=trial["trial_id"]+" first learned handoff")
        axis.set_aspect("equal");axis.grid(alpha=.25);axis.legend(fontsize=7)
    figure.tight_layout();figure.savefig(output/"ego_route_predicted_observed.png",dpi=140);plt.close(figure)


def camera_playback(result, bundles, output, include_gif):
    from PIL import Image, ImageDraw
    def montage(elapsed, sampled=False):
        canvas=Image.new("RGB",(960,700),(20,24,31));draw=ImageDraw.Draw(canvas)
        mode = "sampled at 2 Hz" if sampled else "measured post-handoff snapshot"
        draw.text((8,8),f"Actual recorded CARLA cameras | {mode} | t={elapsed:.1f} s | not Autoware UI",fill="white")
        for column,trial in enumerate(result["trials"]):
            bundle=bundles[trial["trial_id"]];states=bundle["states"];cameras=bundle["cameras"];x0=column*480
            draw.text((x0+8,32),trial["trial_id"],fill="white")
            if not states or not cameras or elapsed>states[-1]["timestamp"]-states[0]["timestamp"]+1e-6:
                draw.text((x0+20,95),"No recorded state/camera at this time",fill="white");continue
            camera=min(cameras,key=lambda row:abs(row["timestamp"]-states[0]["timestamp"]-elapsed))
            draw.text((x0+8,50),f"native frame {camera['frame']}; actual t={camera['timestamp']-states[0]['timestamp']:.3f}s",fill="white")
            for index,name in enumerate(CAMERAS):
                x=x0+(index%3)*160;y=76+(index//3)*150
                with Image.open(bundle["episode"]/camera["images"][name]) as original:
                    image=original.convert("RGB");image.thumbnail((158,120));canvas.paste(image,(x,y+18))
                draw.text((x+2,y),name,fill="white")
            # HH_261001 - This numeric context panel is analysis, not simulator or Autoware UI.
            panel=Image.new("RGB",(480,285),(27,33,42));pen=ImageDraw.Draw(panel)
            origin=next(row for row in states if row["frame"]==camera["frame"])
            c,s=math.cos(origin["yaw"]),math.sin(origin["yaw"])
            def centered(row):
                dx,dy=row["x"]-origin["x"],row["y"]-origin["y"]
                return 240-11*(-s*dx+c*dy),142-11*(c*dx+s*dy)
            route=bundle["route"]["route"] if bundle["route"] else []
            traveled=[row for row in states if row["timestamp"]<=origin["timestamp"]]
            for values,color in ((route,"#89939e"),(traveled,"#4db6ac")):
                if len(values)>1:pen.line([centered(row) for row in values],fill=color,width=2)
            plans=[row for row in bundle["decisions"] if row.get("application",{}).get("learned_control_applied") is True and row.get("command_frame",math.inf)<origin["frame"]]
            if plans:
                selected=plans[-1];prior_origin=next(row for row in states if row["frame"]==selected["inference_frame"])
                pc,ps=math.cos(prior_origin["yaw"]),math.sin(prior_origin["yaw"])
                world=[dict(x=prior_origin["x"]+pc*x-ps*y,y=prior_origin["y"]+ps*x+pc*y) for x,y in selected["selected"]["xy_base_m"]]
                if len(world)>1:pen.line([centered(row) for row in world],fill="#ce93d8",width=2)
                pen.text((5,266),f"Last ACKed selected plan: inference frame {selected['inference_frame']}",fill="white")
            pen.ellipse((236,138,244,146),fill="#ffeb3b");pen.line((240,142,240,125),fill="#ffeb3b",width=2)
            pen.text((5,5),"Analysis: ego reference centered; forward up / left left",fill="white")
            pen.text((5,22),"route gray / traveled green / selected plan purple",fill="white")
            canvas.paste(panel,(x0,400))
        return canvas
    montage(STATIC_MONTAGE_SECONDS).save(output/"actual_six_camera_montage.png")
    if include_gif:
        frames=[montage(index*.5, sampled=True) for index in range(40)]
        frames[0].save(output/"actual_cameras_sampled_2hz.gif",save_all=True,append_images=frames[1:],duration=500,loop=0,optimize=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--campaign",type=Path,default=DEFAULT_CAMPAIGN);parser.add_argument("--plan",type=Path,default=DEFAULT_PLAN)
    parser.add_argument("--plan-sha256",required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--allow-single-retained",action="store_true",help="report one infrastructure/physical failure without calling it an A/B comparison")
    parser.add_argument("--plot",action="store_true");parser.add_argument("--camera-montage",action="store_true");parser.add_argument("--gif",action="store_true")
    args=parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink(),"fresh create-only report output required")
    result,bundles=analyze(args.campaign,args.plan,args.plan_sha256,args.allow_single_retained)
    if args.plot:
        import matplotlib  # noqa: F401 - Verify installed renderer before output creation.
    if args.camera_montage or args.gif:
        from PIL import Image  # noqa: F401
    output=fresh_output(args.output,args.campaign)
    if args.plot:plot_measured(result,bundles,output)
    if args.camera_montage or args.gif:camera_playback(result,bundles,output,args.gif)
    with (output/"comparison.json").open("x") as stream:
        json.dump(result,stream,sort_keys=True,indent=2,allow_nan=False);stream.write("\n")
    table=["| Arm | Native states / cameras | Measured span s | Net progress m | First stop s | Shocks < -8 | Full 400/200 capture |","| --- | ---: | ---: | ---: | ---: | ---: | --- |"]
    for trial in result["trials"]:
        summary=trial["summary"]
        table.append(f"| {trial['trial_id']} | {summary['state_count']} / {trial['camera_count']} | {summary['observed_native_span_seconds']} | {summary['net_route_progress_m']} | {summary.get('first_speed_at_or_below_0p05_after_motion_s')} | {summary.get('signed_rate_below_minus_8_mps2_count')} | {trial['full_400_state_200_camera_capture']} |")
    with (output/"comparison_table.md").open("x") as stream:stream.write("\n".join(table)+"\n\nActual retained records; one attempt per arm, no training or vehicle approval.\n")
    print(json.dumps({"output":str(output),"trial_count":result["trial_count"],"all_integrity_checks_pass":result["all_integrity_checks_pass"],"both_full_twenty_second_captures":result["both_full_twenty_second_captures"]}))
    return 0 if result["all_integrity_checks_pass"] else 1


if __name__=="__main__":
    raise SystemExit(main())
