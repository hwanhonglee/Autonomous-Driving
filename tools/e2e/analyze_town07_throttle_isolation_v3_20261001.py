#!/usr/bin/env python3
"""HH_261001 - Independently audit retained Town07 throttle trials without CARLA."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = ROOT / "storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v3.json"
DEFAULT_CAMPAIGN = ROOT / "storage/experiments/validation/2026-10-01/town07_throttle_isolation_v3"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
TOLERANCE = 1e-6
MAX_ROWS = 1000
MAX_JSON_BYTES = 8 * 1024**2


def sha256(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"regular input file required: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_JSON_BYTES:
        raise ValueError(f"bounded regular JSON input required: {path}")
    return json.loads(path.read_text())


def read_rows(path: Path) -> list[dict]:
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_JSON_BYTES:
        raise ValueError(f"bounded regular JSONL input required: {path}")
    rows = []
    with path.open() as stream:
        for line in stream:
            rows.append(json.loads(line))
            if len(rows) > MAX_ROWS:
                raise ValueError(f"native row budget exceeded: {path}")
    return rows


def reconstruct_state(row: dict) -> dict[str, float]:
    """HH_261001 - Match the actual yaw-only velocity formula and pitched pose shift."""
    transform = row["actor_snapshot_transform_carla"]
    yaw, pitch = math.radians(transform["yaw"]), math.radians(transform["pitch"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    velocity, acceleration = row["world_velocity_carla"], row["world_acceleration_carla"]
    yaw_rate = -math.radians(row["world_angular_velocity_carla_deg_s"][2])
    return {
        "x": transform["x"] - 1.425 * math.cos(pitch) * cosine,
        "y": -transform["y"] + 1.425 * math.cos(pitch) * sine,
        "z": transform["z"] - 1.425 * math.sin(pitch),
        "yaw": math.atan2(math.sin(-yaw), math.cos(-yaw)),
        "vx": cosine * velocity[0] + sine * velocity[1],
        "vy": sine * velocity[0] - cosine * velocity[1] - 1.425 * yaw_rate,
        "ax": cosine * acceleration[0] + sine * acceleration[1],
        "ay": sine * acceleration[0] - cosine * acceleration[1],
        "yaw_rate": yaw_rate,
    }


def project_route(points: list[dict], x: float, y: float, previous: float) -> tuple[float, float]:
    """HH_261001 - Recompute the bounded monotonic route projection from raw geometry."""
    best = None
    lower, upper = max(0.0, previous - 3.0), min(points[-1]["distance_m"], previous + 80.0)
    for index, (first, second) in enumerate(zip(points, points[1:])):
        if second["distance_m"] < lower or first["distance_m"] > upper:
            continue
        dx, dy = second["x"] - first["x"], second["y"] - first["y"]
        squared = dx * dx + dy * dy
        fraction = max(0.0, min(1.0, ((x - first["x"]) * dx + (y - first["y"]) * dy) / squared)) if squared > 1e-9 else 0.0
        error = math.hypot(x - first["x"] - fraction * dx, y - first["y"] - fraction * dy)
        progress = first["distance_m"] + fraction * (second["distance_m"] - first["distance_m"])
        candidate = error, -progress, index, progress
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("no eligible route segment")
    return max(previous, best[3]), best[0]


def metrics(branch: dict, intervention: list[dict]) -> dict | None:
    if not intervention:
        return None
    interval = [branch, *intervention]
    rates = [(right["vx"] - left["vx"]) / (right["timestamp"] - left["timestamp"])
             for left, right in zip(interval, interval[1:])]
    return {
        "branch_frame": branch["frame"], "branch_progress_m": branch["route_progress_m"],
        "branch_speed_mps": branch["vx"], "observed_ticks": len(intervention),
        "observed_seconds": intervention[-1]["timestamp"] - branch["timestamp"],
        "signed_rate_below_minus_8_mps2_count": sum(rate < -8.0 for rate in rates),
        "minimum_signed_rate_mps2": min(rates),
        "net_route_progress_m": intervention[-1]["route_progress_m"] - branch["route_progress_m"],
        # HH_261001 - The pinned worker uses |vx|, although the prose shorthand says vx.
        "first_speed_at_or_below_0p05_s": next((row["timestamp"] - branch["timestamp"] for row in intervention if abs(row["vx"]) <= 0.05), None),
        "final_speed_mps": intervention[-1]["vx"],
    }


def check_ack(row: dict, receipt: dict | None) -> list[str]:
    problems = []
    alignment = row.get("control_alignment", row.get("alignment"))
    if receipt is None:
        return ["missing_receipt"]
    if not (receipt["status"] == "ACKNOWLEDGED" and receipt["server_accepted"] is True
            and receipt["actor_id"] == receipt["response_actor_id"]):
        problems.append("acknowledgement")
    if not (receipt["before_frame"] == receipt["after_ack_frame"] == row["frame"] - 1):
        problems.append("receipt_frame_binding")
    if receipt.get("expected_before_frame", receipt["before_frame"]) != receipt["before_frame"]:
        problems.append("expected_command_frame")
    if not (alignment["observed_frame"] == alignment["control_read_frame_before"] == alignment["control_read_frame_after"] == row["frame"]):
        problems.append("control_read_frame_binding")
    observed, requested = row["current_control"], receipt["requested_control"]
    if any(not math.isfinite(observed[key]) or not math.isfinite(requested[key]) or abs(observed[key] - requested[key]) > TOLERANCE for key in ("throttle", "brake", "steer")):
        problems.append("pedal_readback")
    if any(observed[key] != requested[key] for key in ("hand_brake", "reverse", "manual_gear_shift")):
        problems.append("flag_readback")
    if requested["manual_gear_shift"] and observed["gear"] != requested["gear"]:
        problems.append("manual_gear_readback")
    if alignment["status"] != ("FAIL" if problems else "PASS"):
        problems.append("stored_alignment_status")
    return problems


def safety_failures(row: dict, previous_frame: int, alignment_ok: bool, minimum_free: int) -> list[str]:
    failures = []
    if row["frame"] != previous_frame + 1:
        failures.append("native_frame_discontinuity")
    if not math.isfinite(row["vx"]) or not math.isfinite(row["route_cte_m"]):
        failures.append("nonfinite_state")
    if abs(row["vx"]) > 2.7:
        failures.append("maximum_speed")
    if abs(row["route_cte_m"]) > 1.0:
        failures.append("route_cte")
    if row["collision_count"] or row["lane_invasion_count"]:
        failures.append("observed_event")
    if row["traffic_light_hazard"]["hazard"]:
        failures.append("traffic_light_hazard")
    if not alignment_ok:
        failures.append("control_alignment")
    free = row["observed_free_disk_bytes"]
    if type(free) is not int or free < minimum_free:
        failures.append("disk_guard")
    return failures


def audit_records(rows: list[dict], receipts: list[dict], bootstrap: dict, spec: dict,
                  points: list[dict], minimum_free: int) -> tuple[dict, list[dict]]:
    """Audit measurement integrity separately from an expected retained safety abort."""
    problems: Counter = Counter()
    max_error: Counter = Counter()
    lookup = {receipt["sequence"]: receipt for receipt in receipts}
    if [receipt["sequence"] for receipt in receipts] != list(range(1, len(receipts) + 1)):
        problems["receipt_sequence"] += 1
    for receipt in receipts:
        if not (receipt["status"] == "ACKNOWLEDGED" and receipt["server_accepted"] is True
                and receipt["actor_id"] == receipt["response_actor_id"]
                and receipt["before_frame"] == receipt["after_ack_frame"]):
            problems["all_receipts_acknowledged"] += 1
        if receipt.get("do_tick", False) is not False:
            problems["receipt_advanced_world"] += 1
    raw_guard_rows = []
    previous_frame, previous_timestamp, previous_progress = None, None, 0.0
    for row in [bootstrap, *rows]:
        numeric_values = [row["timestamp"], row["route_progress_m"], *row["actor_snapshot_transform_carla"].values(),
                          *row["world_velocity_carla"], *row["world_acceleration_carla"], *row["world_angular_velocity_carla_deg_s"]]
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in numeric_values):
            raise ValueError(f"nonfinite or invalid raw numeric input at frame {row['frame']}")
        if type(row["frame"]) is not int:
            raise ValueError("integer native frame required")
        reconstructed = reconstruct_state(row)
        for key, value in reconstructed.items():
            error = abs(value - row[key])
            max_error[key] = max(max_error[key], error)
            if not math.isfinite(value) or not math.isfinite(row[key]) or error > 1e-9:
                problems["raw_state_reconstruction"] += 1
        progress, cte = project_route(points, row["x"], row["y"], previous_progress)
        previous_progress = progress
        max_error["route_progress_m"] = max(max_error["route_progress_m"], abs(progress - row["route_progress_m"]))
        if abs(progress - row["route_progress_m"]) > 1e-9:
            problems["route_projection"] += 1
        alignment = row.get("control_alignment", row.get("alignment"))
        ack_problems = check_ack(row, lookup.get(alignment["expected_receipt_sequence"]))
        problems.update(ack_problems)
        if previous_frame is not None:
            if row["frame"] != previous_frame + 1:
                problems["native_frame_cadence"] += 1
            if abs(row["timestamp"] - previous_timestamp - 0.05) > TOLERANCE:
                problems["native_timestamp_cadence"] += 1
            if abs(cte - row["route_cte_m"]) > 1e-9:
                problems["route_cte_projection"] += 1
            expected = safety_failures(row, previous_frame, not ack_problems, minimum_free)
            # HH_261001 - Warmup final guards are separately recounted below.
            stored = [failure for failure in row["safety_failures"] if not failure.startswith("warmup_")]
            if stored != expected:
                problems["stored_safety_guard"] += 1
            if expected:
                raw_guard_rows.append({"frame": row["frame"], "phase": row["phase"],
                                       "intervention_tick": row["intervention_tick"],
                                       "vx": row["vx"], "failures": expected})
        previous_frame, previous_timestamp = row["frame"], row["timestamp"]
    warmup = [row for row in rows if row["phase"] == "stationary_warmup"]
    prefixes = [row for row in rows if row["phase"] == "prefix"]
    branches = [row for row in rows if row["phase"] == "branch"]
    intervention = [row for row in rows if row["phase"] == "intervention"]
    if [row["warmup_tick"] for row in warmup] != list(range(1, 71)):
        problems["warmup_tick_sequence"] += 1
    warmup_audit = None
    if warmup:
        final = warmup[-1]
        drift = final["route_progress_m"] - bootstrap["route_progress_m"]
        displacement = math.hypot(final["x"] - bootstrap["x"], final["y"] - bootstrap["y"])
        warmup_audit = {"bootstrap_progress_m": bootstrap["route_progress_m"], "final_progress_m": final["route_progress_m"],
                        "progress_drift_m": drift, "planar_displacement_m": displacement, "final_vx_mps": final["vx"]}
        if not (0.0 <= bootstrap["route_progress_m"] <= 0.5 and 0.0 <= final["route_progress_m"] <= 0.5
                and abs(final["vx"]) <= 0.05 and abs(drift) <= 0.25 and displacement <= 0.25):
            problems["warmup_final_guard"] += 1
        if any(row["current_control"]["brake"] != 1.0 or row["current_control"]["throttle"] != 0.0 for row in warmup):
            problems["warmup_control"] += 1
    if len(branches) != 1:
        problems["branch_count"] += 1
    branch = branches[0] if len(branches) == 1 else None
    if [row["intervention_tick"] for row in intervention] != list(range(1, len(intervention) + 1)) or len(intervention) > 60:
        problems["intervention_tick_sequence"] += 1
    if branch:
        if not (spec["site_progress_m"] <= branch["route_progress_m"] <= spec["site_progress_m"] + 0.25 and 2.35 <= branch["vx"] <= 2.55):
            problems["branch_admission"] += 1
        prior = rows[rows.index(branch) - 1]
        if prior["route_progress_m"] >= spec["site_progress_m"]:
            problems["first_site_crossing"] += 1
        expected_phases = ["stationary_warmup"] * len(warmup) + ["prefix"] * len(prefixes) + ["branch"] + ["intervention"] * len(intervention)
        if [row["phase"] for row in rows] != expected_phases:
            problems["phase_order"] += 1
    for row in intervention:
        control = row["current_control"]
        if not (abs(control["throttle"] - spec["throttle"]) <= TOLERANCE and control["brake"] == control["steer"] == 0.0
                and not control["hand_brake"] and not control["reverse"] and not control["manual_gear_shift"]):
            problems["intervention_control"] += 1
        if branch and row["frame"] - branch["frame"] != row["intervention_tick"]:
            problems["intervention_frame_binding"] += 1
    observed = metrics(branch, intervention) if branch else None
    common = metrics(branch, intervention[:20]) if branch and len(intervention) >= 20 else None
    shock_events, acceleration_errors, along_errors = [], [], []
    if branch:
        for left, right in zip([branch, *intervention], intervention):
            dt = right["timestamp"] - left["timestamp"]
            rate = (right["vx"] - left["vx"]) / dt
            velocity = left["world_velocity_carla"]
            norm = math.hypot(velocity[0], velocity[1])
            raw = right["world_velocity_carla"]
            acceleration = right["world_acceleration_carla"]
            along_rate = ((raw[0] - velocity[0]) * velocity[0] + (raw[1] - velocity[1]) * velocity[1]) / (dt * norm) if norm else None
            along_acceleration = (acceleration[0] * velocity[0] + acceleration[1] * velocity[1]) / norm if norm else None
            acceleration_errors.append(abs(rate - right["ax"]))
            if along_rate is not None:
                along_errors.append(abs(rate - along_rate))
            if rate < -8.0:
                shock_events.append({"frame": right["frame"], "previous_frame": left["frame"], "signed_rate_mps2": rate,
                                     "world_along_rate_mps2": along_rate, "world_along_acceleration_mps2": along_acceleration,
                                     "vx_before_mps": left["vx"], "vx_after_mps": right["vx"], "current_control": right["current_control"]})
    branch_fields = None
    if branch:
        branch_fields = {key: branch[key] for key in ("frame", "timestamp", "x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate",
                        "route_progress_m", "route_cte_m", "current_control", "world_velocity_carla", "world_acceleration_carla")}
    return {
        "integrity_issue_counts": dict(problems), "raw_reconstruction_max_abs_error": dict(max_error),
        "phase_counts": dict(Counter(row["phase"] for row in rows)), "native_state_count": len(rows), "receipt_count": len(receipts),
        "receipt_reason_counts": dict(Counter(receipt["reason"] for receipt in receipts)), "warmup": warmup_audit,
        "branch_start_state": branch_fields, "observed_window": observed, "exploratory_common_1s": common,
        "recomputed_guard_rows": raw_guard_rows, "shock_events": shock_events,
        "observed_intervention_gear_counts": dict(Counter(row["current_control"]["gear"] for row in intervention)),
        "raw_acceleration_crosscheck": {"max_abs_signed_rate_minus_ax_mps2": max(acceleration_errors, default=None),
                                       "max_abs_signed_rate_minus_world_along_rate_mps2": max(along_errors, default=None)},
    }, [branch, *intervention] if branch else []


def source_audit(run: Path, plan: dict, owner_plan: dict, plan_path: Path, repository: Path) -> dict:
    issues = []
    local = dict(owner_plan["source_sha256"])
    for name, expected in plan["source_sha256"].items():
        if name in local and local[name] != expected:
            issues.append("conflicting_owner_pin:" + name)
        local[name] = expected
    for name, expected in local.items():
        if sha256(repository / name) != expected or sha256(run / "source" / name) != expected:
            issues.append("local_source:" + name)
    for name, expected in plan["external_agent_sha256"].items():
        relative = Path(name).relative_to(CARLA_ROOT)
        if sha256(Path(name)) != expected or sha256(run / "source_external" / relative) != expected:
            issues.append("external_source:" + name)
    if sha256(run / "declared_plan.json") != sha256(plan_path):
        issues.append("declared_plan_archive")
    if sha256(run / "episode.partial/active_map_opendrive.xodr") != plan["active_opendrive_sha256"]:
        issues.append("active_map_archive")
    return {"matches": not issues, "issues": issues, "local_archive_file_count": len(local),
            "external_archive_file_count": len(plan["external_agent_sha256"])}


def analyze_trial(run: Path, spec: dict, plan: dict, plan_path: Path, points: list[dict], repository: Path) -> tuple[dict, list[dict]]:
    episode = run / "episode.partial"
    manifest, owner, frozen = (read_json(path) for path in (episode / "manifest.json", run / "owner_result.json", run / "throttle_result.json"))
    rows, receipts = read_rows(episode / "states.jsonl"), read_rows(episode / "control_receipts.jsonl")
    result, plot_rows = audit_records(rows, receipts, manifest["bootstrap_observation"], spec, points, plan["resource_guard"]["running_free_bytes"])
    raw_identity = manifest["trial"] == spec and manifest["fixed_plan_sha256"] == sha256(plan_path)
    archives = source_audit(run, plan, read_json(run / "owner_plan.json"), plan_path, repository)
    issues = result["integrity_issue_counts"]
    if not raw_identity:
        issues["raw_trial_identity"] = 1
    if not archives["matches"]:
        issues["source_archive_hashes"] = 1
    if manifest["native_states"] != len(rows):
        issues["manifest_state_count"] = 1
    cleanup_ok = not owner["cleanup_errors"] and owner["ports_released"] is True
    if not cleanup_ok:
        issues["owned_cleanup"] = 1
    expected_identity_flag = raw_identity and isinstance(manifest.get("summary"), dict) and manifest["summary"].get("trial_id") == spec["trial_id"]
    if frozen["worker_identity_matches"] != expected_identity_flag:
        issues["frozen_identity_flag_semantics"] = 1
    if frozen["source_hashes_match"] is not True:
        issues["frozen_source_audit"] = 1
    actor_id = manifest["actor_body_geometry"]["actor_id"]
    if any(receipt["actor_id"] != actor_id for receipt in receipts):
        issues["owned_actor_receipt_identity"] = 1
    if receipts:
        last_receipt = receipts[-1]
        expected_reason = "post_intervention_cleanup_brake" if manifest["status"] == "FINISHED_DIAGNOSTIC_ONLY" else "failure_cleanup_full_brake"
        command = last_receipt["requested_control"]
        if not (last_receipt["reason"] == expected_reason and last_receipt["before_frame"] == rows[-1]["frame"]
                and command["throttle"] == command["steer"] == 0.0 and command["brake"] == 1.0):
            issues["post_window_cleanup_receipt"] = 1
    abort_guard_matches = None
    if result["recomputed_guard_rows"]:
        last_guard = result["recomputed_guard_rows"][-1]
        expected_failure = "CollectionError: fixed safety abort: " + ",".join(last_guard["failures"])
        abort_guard_matches = len(result["recomputed_guard_rows"]) == 1 and last_guard["frame"] == rows[-1]["frame"] and manifest["failure"] == expected_failure
        if not abort_guard_matches:
            issues["retained_abort_guard_explanation"] = 1
    observed = result["observed_window"]
    for container in (manifest, frozen):
        stored = container.get("summary")
        if isinstance(stored, dict):
            for key, value in (observed or {}).items():
                if key == "observed_seconds":
                    continue
                other = stored.get(key)
                if other != value and not (isinstance(value, (int, float)) and isinstance(other, (int, float)) and abs(value - other) < 1e-9):
                    issues["stored_summary_mismatch"] = issues.get("stored_summary_mismatch", 0) + 1
    complete = bool(observed and observed["observed_ticks"] == 60 and not issues and not result["recomputed_guard_rows"]
                    and manifest["status"] == frozen["status"] == "FINISHED_DIAGNOSTIC_ONLY" and owner["collector_exit_code"] == 0)
    if manifest["status"] == "FINISHED_DIAGNOSTIC_ONLY" and not complete:
        issues["claimed_completed_window_invalid"] = 1
    result.update({
        "trial_id": spec["trial_id"], "fixed_index": spec["index"], "site_progress_m": spec["site_progress_m"],
        "arm": spec["arm"], "throttle": spec["throttle"], "retained_run_path": str(run), "worker_status": manifest["status"],
        "frozen_owner_status": frozen["status"], "failure": manifest["failure"], "raw_plan_trial_identity_matches": raw_identity,
        "frozen_wrapper_worker_identity_matches": frozen["worker_identity_matches"],
        "frozen_wrapper_identity_note": "The frozen wrapper also requires a summary dictionary; a censored safety abort has no summary.",
        "source_archive_audit": archives, "cleanup_verified": cleanup_ok,
        "retained_abort_guard_matches_failure": abort_guard_matches,
        "completed_valid_3s_window": complete, "summary_3s": observed if complete else None,
        "censored_before_3s": bool(observed and observed["observed_ticks"] < 60),
        "physics_applied": manifest["physics"]["applied"], "final_observed_event_counts": manifest.get("final_observed_event_counts"),
        "input_sha256": {name: sha256(path) for name, path in {"states": episode / "states.jsonl", "controls": episode / "control_receipts.jsonl",
            "manifest": episode / "manifest.json", "owner_result": run / "owner_result.json", "throttle_result": run / "throttle_result.json"}.items()},
    })
    return result, plot_rows


def compare_sites(trials: list[dict]) -> list[dict]:
    sites = []
    for site in (16, 25):
        selected = [trial for trial in trials if trial["site_progress_m"] == site]
        starts = [trial["branch_start_state"] for trial in selected if trial["branch_start_state"]]
        ranges = {key: max(start[key] for start in starts) - min(start[key] for start in starts)
                  for key in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate", "route_progress_m", "route_cte_m")} if starts else None
        comparison = {"site_progress_m": site, "fixed_arm_order": [trial["arm"] for trial in selected], "branch_state_ranges": ranges,
                      "branch_state_identity_proves_mechanical_internal_state": False,
                      "valid_3s_ab_comparison_available": len(selected) == 4 and all(trial["completed_valid_3s_window"] for trial in selected),
                      "exploratory_common_1s": {"posthoc_exploratory": True, "includes_observed_guard_trigger_tick_when_present": True,
                        "available_for_all_four_trials": len(selected) == 4 and all(trial["exploratory_common_1s"] and not trial["integrity_issue_counts"] for trial in selected),
                        "trials": [{"trial_id": trial["trial_id"], "arm": trial["arm"], "metrics": trial["exploratory_common_1s"]} for trial in selected]}}
        if comparison["exploratory_common_1s"]["available_for_all_four_trials"]:
            arm_means = {}
            for arm in ("A", "B"):
                members = [trial["exploratory_common_1s"] for trial in selected if trial["arm"] == arm]
                arm_means[arm] = {key: sum(member[key] for member in members) / len(members)
                                  for key in ("net_route_progress_m", "final_speed_mps", "signed_rate_below_minus_8_mps2_count")}
            comparison["exploratory_common_1s"]["arm_means"] = arm_means
            comparison["exploratory_common_1s"]["B_minus_A"] = {key: arm_means["B"][key] - arm_means["A"][key] for key in arm_means["A"]}
        sites.append(comparison)
    return sites


def analyze_campaign(campaign: Path, plan_path: Path, repository: Path = ROOT) -> tuple[dict, dict]:
    plan = read_json(plan_path)
    specs = plan["trial_order"]
    expected_ids = [f"site{site:02d}_{name}" for site in (16, 25) for name in ("A1", "B1", "B2", "A2")]
    if ([spec["arm"] for spec in specs] != list("ABBAABBA") or [spec["site_progress_m"] for spec in specs] != [16] * 4 + [25] * 4
            or [spec["trial_id"] for spec in specs] != expected_ids or [spec["index"] for spec in specs] != list(range(8))
            or any(spec["throttle"] != {"A": 0.01, "B": 0.20}[spec["arm"]] for spec in specs)):
        raise ValueError("the fixed eight-trial ABBA plan is required")
    for spec in specs:
        run = campaign / spec["trial_id"]
        if not (run / "owner_result.json").is_file() or not (run / "throttle_result.json").is_file():
            raise ValueError(f"all eight terminal trials are required; missing {spec['trial_id']}")
        frozen = read_json(run / "throttle_result.json")
        manifest = read_json(run / "episode.partial/manifest.json")
        if frozen["status"] not in ("FINISHED_DIAGNOSTIC_ONLY", "FAILED_RETAINED") or manifest["status"] not in ("FINISHED_DIAGNOSTIC_ONLY", "FAILED_RETAINED"):
            raise ValueError(f"terminal worker/study status required: {spec['trial_id']}")
    asset_issues = [name for name, expected in plan["asset_sha256"].items() if sha256(Path(name)) != expected]
    route_path = next(Path(name) for name in plan["asset_sha256"] if name.endswith("/route.json"))
    points = read_json(route_path)["route"]
    trials, plot_data = [], {}
    for spec in specs:
        trial, plot_rows = analyze_trial(campaign / spec["trial_id"], spec, plan, plan_path, points, repository)
        if asset_issues:
            trial["integrity_issue_counts"]["declared_asset_hashes"] = len(asset_issues)
            trial["completed_valid_3s_window"], trial["summary_3s"] = False, None
        trials.append(trial)
        plot_data[spec["trial_id"]] = plot_rows
    sites = compare_sites(trials)
    result = {
        "schema": "carla.town07_throttle_isolation_independent_analysis.v3.20261001",
        "analyzed_at_utc": datetime.now(timezone.utc).isoformat(), "declared_plan_sha256": sha256(plan_path),
        "analyzer_sha256": sha256(Path(__file__)), "trial_count": len(trials), "all_eight_terminal_trials_retained": True,
        "asset_hash_issues": asset_issues, "all_raw_integrity_checks_pass": not asset_issues and all(not trial["integrity_issue_counts"] for trial in trials),
        "completed_3s_trial_count": sum(trial["completed_valid_3s_window"] for trial in trials),
        "censored_trial_count": sum(trial["censored_before_3s"] for trial in trials), "trials": trials, "site_comparisons": sites,
        "all_predeclared_3s_ab_comparisons_available": all(site["valid_3s_ab_comparison_available"] for site in sites),
        "training_data_approved": False, "vehicle_control_approved": False, "model_promoted": False,
        "interpretation_limits": [
            "Partial safety-aborted records do not replace the predeclared three-second outcome; their summary_3s is null.",
            "The common one-second comparison is posthoc exploratory and includes the measured guard-trigger tick when present.",
            "The first near-stop uses absolute forward speed <=0.05 m/s, matching the pinned worker.",
            "Equal recorded branch states do not establish equal internal gear, wheel torque, tire/contact or actuator state.",
            "ACK and control readback establish delivery and frame alignment, not independent physical actuation.",
            "Physics substeps are inherited; collision/lane event delivery is asynchronous and does not prove absence of road contact.",
            "The records provide simulation timestamps; per-tick wall-clock actuation or rendering latency is not independently measured.",
            "Two repetitions per arm and site are diagnostic evidence, not statistical safety or generalization approval.",
            "No camera images, model inferences, training admission, throttle-floor policy change or real-vehicle approval follows.",
        ],
    }
    return result, plot_data


def plot_actual_records(result: dict, plot_data: dict, target: Path) -> None:
    # HH_261001 - These are measured native traces, never fabricated simulator images.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(12, 7), sharex="col")
    for trial in result["trials"]:
        rows = plot_data[trial["trial_id"]]
        if not rows:
            continue
        column = 0 if trial["site_progress_m"] == 16 else 1
        branch_time = rows[0]["timestamp"]
        times = [row["timestamp"] - branch_time for row in rows]
        color = "#1f77b4" if trial["arm"] == "A" else "#d95f02"
        style = "-" if trial["trial_id"].endswith("1") else "--"
        label = trial["trial_id"] + (" (safety abort)" if trial["censored_before_3s"] else " (3 s)")
        axes[0, column].plot(times, [row["vx"] for row in rows], style, color=color, label=label)
        axes[1, column].step(times, [row["current_control"]["throttle"] for row in rows], where="pre", linestyle=style, color=color)
        if trial["censored_before_3s"]:
            axes[0, column].plot(times[-1], rows[-1]["vx"], "x", color=color, markersize=9)
    for column, site in enumerate((16, 25)):
        axes[0, column].axhline(2.7, color="gray", linestyle=":", label="fixed speed guard 2.7")
        axes[0, column].set(title=f"Town07 site {site} m: actual retained native states", ylabel="forward vx (m/s)")
        axes[0, column].legend(fontsize=8)
        axes[1, column].set(xlabel="simulation seconds after branch", ylabel="current throttle readback", xlim=(0, 3.05))
        for row in range(2):
            axes[row, column].grid(alpha=0.25)
    figure.suptitle("A=0.01, B=0.20; safety-aborted traces stop at their last measured tick")
    figure.tight_layout()
    figure.savefig(target, dpi=150)
    plt.close(figure)


def write_artifacts(output: Path, result: dict, plot_data: dict, include_plot: bool) -> None:
    """HH_261001 - Create a new result directory; never overwrite a retained run."""
    if output.exists() or output.is_symlink():
        raise ValueError("fresh create-only analysis output required")
    protected = DEFAULT_CAMPAIGN.resolve()
    if output.resolve() == protected or protected in output.resolve().parents:
        raise ValueError("analysis output must be outside retained trial directories")
    if include_plot:
        import matplotlib  # noqa: F401 - Fail before creating output if plotting is unavailable.
    output.mkdir(parents=True, exist_ok=False)
    if include_plot:
        plot_actual_records(result, plot_data, output / "native_vx_throttle.png")
    with (output / "analysis.json").open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plot", action="store_true", help="use already installed matplotlib for measured trace PNG")
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh create-only analysis output required")
    result, plot_data = analyze_campaign(args.campaign, args.plan)
    if args.output.resolve() == args.campaign.resolve() or args.campaign.resolve() in args.output.resolve().parents:
        raise ValueError("analysis output must be outside retained trial directories")
    write_artifacts(args.output, result, plot_data, args.plot)
    print(json.dumps({"output": str(args.output), "all_raw_integrity_checks_pass": result["all_raw_integrity_checks_pass"],
                      "completed_3s_trial_count": result["completed_3s_trial_count"], "censored_trial_count": result["censored_trial_count"],
                      "all_predeclared_3s_ab_comparisons_available": result["all_predeclared_3s_ab_comparisons_available"]}))
    return 0 if result["all_raw_integrity_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
