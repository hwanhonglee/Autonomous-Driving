#!/usr/bin/env python3
"""HH_260906 - Audit and publish the frozen six-case v3 partial failure without hiding its collision."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_goal_stop_trials as base
from scripts.e2e import summarize_carla_low_speed_response as pedal

COMMIT = "cdf65a8d0effdc5075fad77d164d0f1305ebee28"
PARTIAL = "actuation.partial"
SOURCES = pedal.SOURCES + ("scripts/e2e/carla_low_speed_response_matrix.py",)
PHASES = ("settle", "prepare", "throttle_hold", "brake_hold", "coast_hold", "launch_prepare", "post_handoff_ramp")
require, number = base.require, base.number


def matrix():
    """HH_260906 - Independently freeze all six planned cases, including the three never executed."""
    cases = [{"case_id": f"{index:02d}_coast_45s_repeat_{index:02d}", "kind": "coast", "level": 0.0,
              "repeat": index, "hold_seconds": 45.0, "ramp_rate_per_second": None} for index in (1, 2)]
    for threshold in (.5, 1.):
        for rate in (.05, .10):
            cases.append({"case_id": f"{len(cases) + 1:02d}_handoff_{threshold:.1f}_ramp_{rate:.2f}",
                "kind": "two_stage_ramp", "level": .4, "repeat": 1, "hold_seconds": 12.,
                "ramp_rate_per_second": rate, "handoff_speed_mps": threshold, "launch_throttle": .15,
                "launch_maximum_seconds": 8., "measured_speed_stop_mps": 30. / 3.6})
    return cases


def verify_denominator(manifest, directories):
    """HH_260906 - This actual failed campaign cannot be relabelled as six successful or three planned cases."""
    cases = matrix()
    statuses = ["complete", "complete", "failed"] + ["not_run_after_failure"] * 3
    require(manifest.get("matrix_id") == "low_speed_v3" and manifest.get("status") == "failed", "expected frozen v3 partial failure")
    require(manifest.get("matrix") == cases, "six-case planned matrix mismatch")
    require(manifest.get("case_ledger") == [{"case_id": case["case_id"], "status": status}
            for case, status in zip(cases, statuses)], "all-six failure denominator mismatch")
    require([item["case_id"] for item in manifest.get("completed_cases", [])] == [case["case_id"] for case in cases[:2]]
            and all(item["status"] == "complete" for item in manifest["completed_cases"]), "completed denominator mismatch")
    require(set(directories) == {case["case_id"] for case in cases[:3]}, "missing partial case or unexpected not-run payload")


def speed(row):
    """HH_260906 - Keep the recorded rear-reference planar velocity definition, not an acceleration-channel proxy."""
    return math.hypot(number(row["vx"]), number(row["vy"]))


def phase_names(rows):
    """HH_260906 - Preserve the archived report's original four base phases and present v3 phases."""
    return PHASES[:4] + tuple(phase for phase in PHASES[4:] if any(row["phase"] == phase for row in rows))


def rates(rows, bounds):
    """HH_260906 - Recompute all intervals and phase-boundary witnesses at native20 and both derived10 offsets."""
    output = {}
    for name, sampled, stride in (("native_20hz", rows, 1), ("derived_10hz_offset_0", rows[::2], 2), ("derived_10hz_offset_1", rows[1::2], 2)):
        pairs = []
        for first, second in zip(sampled, sampled[1:]):
            dt = number(second["timestamp"]) - number(first["timestamp"])
            require(abs(dt - .05 * stride) < 1e-4 and second["frame"] - first["frame"] == stride, "raw cadence/frame stride mismatch")
            initial, final = speed(first), speed(second)
            pairs.append({"from_frame": first["frame"], "to_frame": second["frame"], "from_phase": first["phase"],
                "to_phase": second["phase"], "from_speed_mps": initial, "to_speed_mps": final, "dt_sec": dt,
                "speed_rate_mps2": (final - initial) / dt})
        phases = {}
        for phase in ("all", *phase_names(rows)):
            selected = pairs if phase == "all" else [pair for pair in pairs if pair["to_phase"] == phase]
            values = [pair["speed_rate_mps2"] for pair in selected]
            item = {"interval_count": len(values), "minimum_speed_rate_mps2": min(values) if values else None,
                    "maximum_speed_rate_mps2": max(values) if values else None,
                    "phase_boundary_intervals": [pair for pair in selected if pair["from_phase"] != pair["to_phase"]]}
            for kind in ("physical_decoder", "runtime_speed_rate_gate"):
                limit = bounds[kind]
                failed = [pair for pair in selected if pair["speed_rate_mps2"] > limit["maximum_acceleration_mps2"] + 1e-9
                          or pair["speed_rate_mps2"] < -limit["maximum_deceleration_mps2"] - 1e-9]
                item[kind] = {"violation_count": len(failed), "violation_intervals": failed}
            phases[phase] = item
        output[name] = {"sample_count": len(sampled), "cadence_violation_count": 0, "frame_stride_violation_count": 0, "phases": phases}
    return output


def coast_observations(rows, recorded):
    """HH_260906 - Recompute observed release-to-dwell distance; it is not a catalog goal-position pass."""
    index = next(index for index, row in enumerate(rows) if row["phase"] == "coast_hold")
    origin, coast = rows[index - 1], rows[index:]
    start = first_below = first_dwell = None
    longest = 0.
    for row in coast:
        if speed(row) <= .1:
            start = row if start is None else start
            first_below = row if first_below is None else first_below
            duration = row["timestamp"] - start["timestamp"]
            longest = max(longest, duration)
            if duration >= 2. and first_dwell is None:
                first_dwell = row
        else:
            start = None
    require(recorded["maximum_speed_mps"] == .1 and recorded["required_continuous_seconds"] == 2.
            and recorded["sample_count"] == len(coast) and recorded["measurement_valid"] is True, "coast stop observation settings mismatch")
    require(recorded["first_below_threshold_frame"] == (None if first_below is None else first_below["frame"])
            and recorded["first_verified_dwell_frame"] == (None if first_dwell is None else first_dwell["frame"])
            and recorded["verified_dwell_observed"] == (first_dwell is not None)
            and recorded["longest_observed_dwell_seconds"] == longest, "coast dwell differs from continuous raw observations")
    def point(row):
        return None if row is None else {"frame": row["frame"], "seconds_after_release": row["timestamp"] - origin["timestamp"],
            "travel_after_release_m": row["travel_m"] - origin["travel_m"],
            "route_arc_after_release_m": row["route_progress_m"] - origin["route_progress_m"], "speed_mps": speed(row)}
    if first_dwell is not None:
        for field, raw_field in (("first_verified_dwell_timestamp", "timestamp"), ("first_verified_dwell_travel_m", "travel_m"),
                                ("first_verified_dwell_route_progress_m", "route_progress_m")):
            require(recorded[field] == first_dwell[raw_field], "coast dwell coordinate mismatch")
    return {"release_speed_mps": speed(origin), "first_at_or_below_0p1": point(first_below),
            "first_verified_continuous_2s_dwell": point(first_dwell), "final_observation": point(coast[-1]),
            "longest_observed_dwell_seconds": longest, "goal_position_tested": False, "normal_brake_applied": False}


def verify_case(case, report, rows, bounds, expected_status):
    """HH_260906 - Validate the exact controls and retain the final collision sample beyond last-success metadata."""
    require(report["case"] == case and report["status"] == expected_status and report["matrix_id"] == "low_speed_v3"
            and report["training_data"] is False and report["schema"] == "carla.low_speed_response_calibration.v1", "case identity/status mismatch")
    require(report["cleanup"]["completed"] is True and report["cleanup"]["errors"] == [], "owned case cleanup incomplete")
    require(rows and len(rows) == report["state_count"], "raw state count mismatch")
    counts = {phase: sum(row["phase"] == phase for row in rows) for phase in phase_names(rows)}
    require(counts == report["phase_counts"] and counts["settle"] == 70, "raw phase denominator mismatch")
    if case["kind"] == "coast":
        require(1 <= counts["prepare"] <= 300 and counts["coast_hold"] == 900, "coast duration mismatch")
        expected_phases = ["settle"] * 70 + ["prepare"] * counts["prepare"] + ["coast_hold"] * 900
    else:
        require(1 <= counts["launch_prepare"] <= 160 and 1 <= counts["post_handoff_ramp"] <= 240, "handoff duration invalid")
        expected_phases = ["settle"] * 70 + ["launch_prepare"] * counts["launch_prepare"] + ["post_handoff_ramp"] * counts["post_handoff_ramp"]
    require([row["phase"] for row in rows] == expected_phases, "phase ordering changed")
    post_index = 0
    for row in rows:
        base.integer(row["frame"])
        for name in ("timestamp", "x", "y", "travel_m", "route_progress_m", "route_cte_m"):
            number(row[name])
        require(0 <= row["travel_m"] <= 80 and 0 <= row["route_cte_m"] <= 3 and row["vx"] >= -.1 and speed(row) <= 10, "unexpected extra safety failure")
        throttle, brake = (0., 1.) if row["phase"] == "settle" else (.3, 0.) if row["phase"] == "prepare" else (
            (.15, 0.) if row["phase"] == "launch_prepare" else (0., 0.))
        if row["phase"] == "post_handoff_ramp":
            throttle = min(.4, .15 + case["ramp_rate_per_second"] * (post_index + 1) / 20.)
            post_index += 1
        for field in ("requested_control", "applied_control"):
            control = base.control(row[field])
            require(abs(control["throttle"] - throttle) <= 1e-6 and abs(control["brake"] - brake) <= 1e-6
                    and abs(control["steer"]) <= 1e-6 and all(row[field][name] is False for name in ("hand_brake", "reverse", "manual_gear_shift")), "command differs from frozen v3 matrix")
    collisions = [event for row in rows for event in row["collision"]]
    require(collisions == report["collision_events"], "collision events missing from raw samples")
    require(report["post_despawn_empty_world_frame"] == rows[-1]["frame"] + 1, "post-despawn frame proof mismatch")
    measured = rates(rows, bounds)
    require(measured == report["motion_analysis"]["measurements"], "stored rates or boundary/violation witnesses differ from all raw samples")
    require(report["maximum_speed_mps"] == max(map(speed, rows)) and report["final_speed_mps"] == speed(rows[-1]), "stored speed extrema mismatch")
    result = {"case": case, "execution_status": expected_status, "state_count": len(rows), "phase_counts": counts,
              "first_frame": rows[0]["frame"], "last_frame": rows[-1]["frame"], "final_speed_mps": speed(rows[-1]),
              "maximum_speed_mps": max(map(speed, rows)), "final_travel_m": rows[-1]["travel_m"],
              "maximum_cte_m": max(row["route_cte_m"] for row in rows), "measurements": measured,
              "collision_events": collisions, "whole_case_training_quality_pass": False}
    if case["kind"] == "coast":
        require(not collisions, "unexpected coast collision")
        prepared = [row for row in rows if row["phase"] == "prepare"]
        require(speed(prepared[-1]) >= 3 and all(speed(row) < 3 for row in prepared[:-1]), "coast did not start at first measured crossing")
        require(report["coast_entry_frame"] == prepared[-1]["frame"] and report["coast_entry_speed_mps"] == speed(prepared[-1]), "coast entry metadata mismatch")
        result["coast_observations"] = coast_observations(rows, report["coast_stop_observation"])
    else:
        require(expected_status == "failed" and report["safety_failure"] == "collision" and report["error"] == "CalibrationError: collision", "failed handoff reason changed")
        require(len(collisions) == 1 and collisions[0]["other_actor_type"] == "static.vegetation" and collisions[0]["frame"] == rows[-1]["frame"], "expected final static-vegetation collision missing")
        launched = [row for row in rows if row["phase"] == "launch_prepare"]
        require(speed(launched[-1]) >= case["handoff_speed_mps"] and all(speed(row) < case["handoff_speed_mps"] for row in launched[:-1]), "handoff was not first measured crossing")
        metadata = report["two_stage"]
        require(metadata["handoff_frame"] == launched[-1]["frame"] and metadata["handoff_speed_mps"] == speed(launched[-1])
                and metadata["launch_ticks"] == len(launched) and metadata["post_handoff_ticks"] == post_index - 1
                and metadata["last_frame"] == rows[-2]["frame"] and metadata["last_speed_mps"] == speed(rows[-2])
                and metadata["status"] == "post_handoff_ramp" and metadata["reached_measured_speed_stop"] is False, "last-success handoff metadata does not match retained failing tick")
        result["partial_handoff_notice"] = (f"The metadata counts successful ticks only ({metadata['post_handoff_ticks']}); "
            f"the raw phase has{post_index} including the final collision tick. All{post_index} are analyzed.")
        result["collision_observation"] = {"frame": rows[-1]["frame"], "seconds_after_first_state": rows[-1]["timestamp"] - rows[0]["timestamp"],
            "travel_m": rows[-1]["travel_m"], "cte_m": rows[-1]["route_cte_m"], "speed_before_mps": speed(rows[-2]),
            "speed_after_mps": speed(rows[-1]), "native_speed_rate_mps2": (speed(rows[-1]) - speed(rows[-2])) / (rows[-1]["timestamp"] - rows[-2]["timestamp"])}
    return result


def verify_trial(root):
    """HH_260906 - Bind partial evidence to all ten archived sources, historical bounds, and stopped owner proof."""
    root = Path(root).resolve()
    ledger = []
    read = lambda name, **kwargs: base.read_file(root, name, ledger, **kwargs)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    require(plan["source_head_commit"] == COMMIT and plan["source_worktree_status"] == "", "unexpected execution commit or dirty sources")
    require(owner["exit_code"] == 1 and plan["capture_mode"] == owner["capture_mode"] == "actuation-response"
            and plan["learned_model_control"] is owner["learned_model_control"] is False, "failed measurement-only ownership mismatch")
    require(set(plan["source_sha256"]) == set(SOURCES) == set(owner["source_checks"])
            and owner["source_bytes_unchanged_and_archived"] is True and plan["source_bytes_archived"] is True
            and all(value is True for value in owner["source_checks"].values()), "source archive/postcheck denominator mismatch")
    require({path.relative_to(root / "provenance").as_posix() for path in (root / "provenance").rglob("*") if path.is_file() or path.is_symlink()} == set(SOURCES), "unexpected source archive contents")
    for name in SOURCES:
        raw = pedal.read_bytes(root, "provenance/" + name, ledger)
        committed = subprocess.check_output(["git", "show", f"{COMMIT}:{name}"], cwd=ROOT)
        require(hashlib.sha256(raw).hexdigest() == plan["source_sha256"][name] and raw == committed, "archived source differs from recorded hash or exact clean commit")
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and plan["host"] == "127.0.0.1"
            and plan["port"] == started["port"] and plan["map"] == started["map"] == "Town07" and plan["quality"] == "Low", "owned simulator identity mismatch")
    server_log = pedal.read_bytes(root, "server.log", ledger)
    previous = None
    for stage in ("ready", "after_capture", "stopped"):
        proof = read(f"lifecycle/{stage}.json")
        require(proof["status"] == "PASS" and proof["stage"] == stage and proof["read_only"] is True and proof["error"] is None
                and proof["owner_pid"] == proof["owner_pgid"] == pid and proof["generation_id"] == f"expert_{pid}"
                and proof["host"] == plan["host"] and proof["port"] == plan["port"] and proof["expected_map"] == "Town07", "lifecycle identity/status mismatch")
        log = proof["server_log"]
        require(0 < log["size_bytes"] <= len(server_log) and hashlib.sha256(server_log[:log["size_bytes"]]).hexdigest() == log["sha256"], "lifecycle log prefix mismatch")
        if stage == "stopped":
            require(proof["mode"] == "stopped" and proof["port_released"] is True and proof["owner_process_state"] is None, "owner was not fully stopped")
        else:
            require(proof["mode"] == "running" and proof["active_map_basename"] == "Town07", "wrong active map")
        now = datetime.fromisoformat(proof["checked_at"])
        require(previous is None or now >= previous, "lifecycle times not ordered")
        previous = now
    require(datetime.fromisoformat(owner["completed_at_utc"]) >= previous, "owner result predates stopped evidence")
    manifest = read(f"{PARTIAL}/manifest.json")
    verify_denominator(manifest, [path.name for path in (root / PARTIAL).iterdir() if path.is_dir()])
    require(manifest["schema"] == "carla.low_speed_response_calibration.v1" and manifest["training_data"] is False
            and manifest["error"] == "CalibrationError: collision" and manifest["client_map_loading_allowed"] is False
            and manifest["workspace_ownership"]["verified_exclusive"] is True and manifest["cleanup"] == {"completed": True, "errors": []}, "partial failure/restoration scope mismatch")
    require(manifest["limits"] == {"maximum_travel_m": 80., "maximum_cte_m": 3., "maximum_speed_mps": 10., "maximum_reverse_speed_mps": .1}, "safety limits changed")
    require(manifest["physics_hz"] == 20. and manifest["vehicle_type"] == "vehicle.toyota.prius" and manifest["wheelbase_m"] == 2.85
            and manifest["spawn_z_offset_m"] == .5 and manifest["weather"] == "ClearNoon", "vehicle/physics scenario changed")
    contract = manifest["identification_matrix_contract"]
    require(contract["matrix_id"] == "low_speed_v3" and contract["cases"] == matrix() and contract["coast_hold_seconds"] == 45.
            and contract["coast_hold_ticks"] == 900 and contract["two_stage_policy"]["launch_maximum_ticks"] == 160
            and contract["two_stage_policy"]["maximum_post_handoff_ticks"] == 240, "frozen v3 phase contract mismatch")
    require(all(contract[key] is False for key in ("independent_route_claim", "manual_gear_changes", "physics_parameter_changes", "training_data", "automatic_quality_promotion")), "unexpected promotion or physics change")
    argv = plan["collector_argv"]
    require(argv.count("--matrix") == 1 and argv[argv.index("--matrix") + 1] == "low_speed_v3", "owner did not select v3")
    settings = manifest["runtime"]["capture_world_settings"]
    require(settings["synchronous_mode"] is True and settings["fixed_delta_seconds"] == .05 and settings["substepping"] is True
            and settings["max_substep_delta_time"] * settings["max_substeps"] >= .05, "actual synchronous physics settings invalid")
    route = pedal.read_bytes(root, f"{PARTIAL}/route.json", ledger)
    require(hashlib.sha256(route).hexdigest() == manifest["route_sha256"] == plan["route_sha256"] == started["route_sha256"], "route hash mismatch")
    require(set(manifest["source_sha256"]) == {"calibrate_carla_low_speed_response.py", "collect_carla_vad_expert.py", "carla_goal_stop_profile.py", "carla_low_speed_response_matrix.py"}, "worker source denominator changed")
    for name, expected in manifest["source_sha256"].items():
        require(plan["source_sha256"]["scripts/e2e/" + name] == expected, "worker hash differs from owner pin")
    bounds, bounds_proof = base.resolve_bounds_source(root, manifest["bounds"], plan)
    summaries, raw_cases, physics, actors, last_frame = [], {}, None, [], None
    for index, case in enumerate(matrix()[:3]):
        report = read(f"{PARTIAL}/{case['case_id']}/report.json")
        report_sha = ledger[-1]["sha256"]
        if index < 2:
            require(report_sha == manifest["completed_cases"][index]["report_sha256"], "completed report SHA mismatch")
        require(report["motion_analysis"]["bounds"] == manifest["bounds"], "case bounds changed")
        rows = read(f"{PARTIAL}/{case['case_id']}/states.jsonl", jsonl=True)
        require(ledger[-1]["sha256"] == report["states_sha256"], "raw states SHA mismatch")
        measured = verify_case(case, report, rows, bounds, "complete" if index < 2 else "failed")
        measured.update(raw_sha256=report["states_sha256"], report_sha256=report_sha)
        require(last_frame is None or rows[0]["frame"] > last_frame, "case actor generations overlap")
        last_frame = report["post_despawn_empty_world_frame"]
        actors.append(report["actor_id"])
        if physics is None:
            physics = report["vehicle_physics"]
        require(report["vehicle_physics"] == physics, "actual vehicle physics differ across cases")
        summaries.append(measured)
        raw_cases[case["case_id"]] = rows
    require(len(set(actors)) == 3, "fresh actor not used for each executed case")
    require(pedal.canonical_sha(physics["values"]) == "329e0d373a6a9c219f5694ec964f3551f32325d2fdf09b2a650fb7d73f0b0c8d", "physics differs from v1/v2 actual baseline")
    for item in ledger:
        require(base.sha(root / item["path"]) == item["sha256"], "evidence changed during audit")
    base.recheck_bounds_source_archive(root, bounds_proof)
    result = {"schema": "portable_e2e.low_speed_v3_partial_audit.v1", "evidence_verification": "PASS", "campaign_status": "FAILED_COLLISION",
        "planned_case_count": 6, "completed_case_count": 2, "failed_case_count": 1, "not_run_case_count": 3,
        "case_ledger": manifest["case_ledger"], "raw_state_count": sum(len(rows) for rows in raw_cases.values()),
        "source_manifest": ledger, "base_source_commit": COMMIT, "archived_source_sha256": plan["source_sha256"],
        "source_archive_matches_commit_and_owner_postcheck": True, "owner_exit_code": 1,
        "lifecycle": {"ready": "PASS", "after_capture": "PASS", "stopped": "PASS"},
        "scalar_bounds": bounds, "bounds_source_proof": bounds_proof, "actual_runtime": manifest["runtime"],
        "vehicle_physics": physics, "physics_identical_across_executed_cases_and_v1_v2": True,
        "route_sha256": manifest["route_sha256"], "cases": summaries,
        "scope": {"all_recorded_states_and_collisions_retained": True, "both_derived10_offsets": True,
                  "cameras_collected": False, "training_data": False, "learned_model_control": False, "automatic_promotion": False, "goal_position_pass_claimed": False},
        "limitations": ["Two completed coast measurements still retain preparation acceleration violations; phase-only success is not whole-case quality.",
            "The handoff case collided with static vegetation under steer=0; its full collision impulse is included, not treated as a successful route or30kph cruise.",
            "Three later planned cases were not run and have no invented metrics.",
            "Continuous low-speed dwell and release distance are observed on one initial condition, not goal-position completion or independent-route evidence.",
            "Speed finite differences are scalar checks, not full XY/decoder/runtime feasibility. First state has no preceding measured rate.",
            "The failing case has319 raw states: derived10 offset0 includes final collision frame5120; offset1 ends at frame5119 by its fixed parity, not by sample filtering.",
            "Plots show actual raw measurements, not simulator screenshots; no camera footage exists for this direct-pedal experiment."]}
    return result, raw_cases, json.loads(route)


def render(result, raw_cases, route, output):
    """HH_260906 - Render actual samples at1920x1080, including startup, phase boundaries and final collision."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), dpi=120)
    for case in result["cases"][:2]:
        rows = raw_cases[case["case"]["case_id"]]
        release_index = next(index for index, row in enumerate(rows) if row["phase"] == "coast_hold") - 1
        release = rows[release_index]
        times = [row["timestamp"] - release["timestamp"] for row in rows]
        style = "-" if case is result["cases"][0] else "--"
        axes[0, 0].plot(times, list(map(speed, rows)), style, label=case["case"]["case_id"])
        axes[1, 0].plot(times, [row["travel_m"] - release["travel_m"] for row in rows], style)
        coast = rows[release_index + 1:]
        axes[0, 1].plot(times[release_index + 1:], list(map(speed, coast)), style)
        if case is result["cases"][0]:
            for label, selected in (("native20", rows), ("derived10 offset0", rows[::2]), ("derived10 offset1", rows[1::2])):
                axes[1, 1].plot([b["timestamp"] - release["timestamp"] for b in selected[1:]],
                    [(speed(b) - speed(a)) / (b["timestamp"] - a["timestamp"]) for a, b in zip(selected, selected[1:])], label=label, linewidth=.9)
    axes[0, 0].set(title="Entire retained history, including preparation", ylabel="Planar speed (m/s)")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set(title="Zero-pedal coast: low-speed detail (not goal-position PASS)", ylabel="Planar speed (m/s)", ylim=(0, .4))
    axes[0, 1].axhline(.1, color="red", linestyle=":", label="stop speed <=0.10")
    observation = result["cases"][0]["coast_observations"]["first_verified_continuous_2s_dwell"]
    axes[0, 1].axvline(observation["seconds_after_release"], color="green", linestyle=":", label=f"first2s dwell {observation['seconds_after_release']:.2f}s")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].set(title=f"Observed first2s dwell: {observation['travel_after_release_m']:.4f}m after release", ylabel="Travel relative to coast release (m)")
    axes[1, 1].set(title="All-history scalar rates: preparation violation retained", ylabel="Speed finite difference (m/s²)")
    for bound in (-2.9, 2.9):
        axes[1, 1].axhline(bound, color="red", linestyle=":")
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel("Simulation seconds after zero-pedal release")
        axis.grid(alpha=.25)
    fig.suptitle("V3 partial campaign | 2 completed coast cases / 1 collision failure / 3 not run", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(output / "01_extended_coast_observed_stop.png", metadata={"Description": "HH_260906 - Actual measured data; no generated simulator image."})
    plt.close(fig)
    case = result["cases"][2]
    rows = raw_cases[case["case"]["case_id"]]
    origin = rows[0]["timestamp"]
    times = [row["timestamp"] - origin for row in rows]
    collision_time = times[-1]
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), dpi=120)
    axes[0, 0].plot(times, list(map(speed, rows)), label="Actual planar speed")
    axes[0, 0].set(title="Entire failed case: no collision sample removed", ylabel="Speed (m/s)")
    for label, selected in (("native20", rows), ("derived10 offset0", rows[::2]), ("derived10 offset1", rows[1::2])):
        axes[1, 0].plot([b["timestamp"] - origin for b in selected[1:]],
            [(speed(b) - speed(a)) / (b["timestamp"] - a["timestamp"]) for a, b in zip(selected, selected[1:])], label=label, linewidth=1.)
    axes[1, 0].set(title=f"Final collision rate {case['collision_observation']['native_speed_rate_mps2']:.2f}m/s² retained", ylabel="Speed finite difference (m/s²)")
    for bound in (-2.9, 2.9):
        axes[1, 0].axhline(bound, color="red", linestyle=":")
    axes[1, 0].legend(fontsize=8)
    for name in ("throttle", "brake", "steer"):
        axes[0, 1].plot(times, [row["applied_control"][name] for row in rows], label=name)
    axes[0, 1].set(title="API-reported pedals/steer: zero steering", ylabel="Normalized command")
    axes[0, 1].legend(fontsize=8)
    route_points = [point for point in route["route"] if point["distance_m"] <= 45.]
    x0, y0 = rows[0]["x"], rows[0]["y"]
    axes[1, 1].plot([point["x"] - x0 for point in route_points], [point["y"] - y0 for point in route_points], "--", label="Catalog centerline first45m")
    axes[1, 1].plot([row["x"] - x0 for row in rows], [row["y"] - y0 for row in rows], label="Actual rear-reference path")
    axes[1, 1].scatter([rows[-1]["x"] - x0], [rows[-1]["y"] - y0], color="red", marker="x", s=90, label="Collision (vegetation)")
    axes[1, 1].set(title=f"Collision at travel{case['final_travel_m']:.3f}m / CTE{rows[-1]['route_cte_m']:.3f}m", xlabel="Relative map X (m)", ylabel="Relative map Y (m)")
    axes[1, 1].axis("equal")
    axes[1, 1].legend(fontsize=8)
    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.axvline(collision_time, color="red", linestyle=":")
        for first, second in zip(rows, rows[1:]):
            if first["phase"] != second["phase"]:
                axis.axvline(second["timestamp"] - origin, color="gray", linestyle="--", alpha=.5)
        axis.set_xlabel("Simulation seconds after first retained state")
    for axis in axes.flat:
        axis.grid(alpha=.25)
    fig.suptitle("V3 case03 FAILED | Last successful metadata frame5119; collision frame5120 is retained", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(output / "02_handoff_collision_full_history.png", metadata={"Description": "HH_260906 - Actual measured data; final collision retained."})
    plt.close(fig)


def sanitize(value):
    """HH_260906 - Publish relative provenance and aggregate measurements, never account paths or endpoints."""
    if isinstance(value, str):
        value = re.sub(r"/home/[^/\s]+/autoware_e2e(?=/|$)", "${REPO_ROOT}", value)
        value = re.sub(r"/home/[^/\s]+", "${USER_HOME}", value)
        return re.sub(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", "${HOST}", value)
    if isinstance(value, list):
        return [sanitize(item) for item in value]
    if isinstance(value, dict):
        return {key: sanitize(item) for key, item in value.items()}
    return value


def publish(root, output):
    """HH_260906 - Create one new category only after verification, without touching previous failures or public versions."""
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "publication category already exists")
    result, raw_cases, route = verify_trial(root)
    result["publication_notice"] = "Independently recomputed aggregate view with raw-source digests; raw full states remain private. PNGs are actual-data plots, not CARLA/RViz screenshots."
    output.mkdir(parents=True)
    def write(name, text):
        with (output / name).open("x", encoding="utf-8") as stream:
            stream.write(text)
    write("summary.json", json.dumps(sanitize(result), indent=2, allow_nan=False) + "\n")
    render(result, raw_cases, route, output)
    provenance = {"schema": "portable_e2e.low_speed_v3_publication.v1", "raw_root": Path(root).resolve().relative_to(ROOT).as_posix(),
        "raw_evidence": result["source_manifest"], "executed_base_commit": COMMIT, "executed_archived_sources": result["archived_source_sha256"],
        "audit_source_sha256": {str(Path(name).resolve().relative_to(ROOT)): base.sha(Path(name)) for name in (__file__, base.__file__, pedal.__file__)},
        "public_payload_sha256": {path.name: base.sha(path) for path in sorted(output.iterdir())},
        "notice": "Public summary is a recomputed metadata view, not an exact copy of raw JSON. Images are exact output from the recorded plotting code; no camera capture existed."}
    write("provenance.json", json.dumps(sanitize(provenance), indent=2, allow_nan=False) + "\n")
    observation = result["cases"][0]["coast_observations"]
    dwell, final = observation["first_verified_continuous_2s_dwell"], observation["final_observation"]
    write("README.md", f"""<!-- HH_260906 - Preserve a failed six-case identification campaign without promotion or sample filtering. -->
# 확장 타력 주행 및 가속 전환: 충돌로 중단된 3차 계측

실험 전체 결과는 **실패**입니다. 예정6건 중 완료2건, 충돌실패1건, 미실행3건입니다. `owner exit=1`이며 실행 소스 보존과 서버 ready/after/stopped 검증은 모두 통과했습니다. 이는 성공 주행이나 학습 데이터 승인 결과가 아닙니다.

| 예정 실험 | 실행 결과 |
|---|---|
| 01–02: 45초 무제동 타력 주행 | 계측 완료. 출발 준비 구간의 가속도 위반은 유지 |
| 03: 실제0.5m/s 이후 +0.05/s 스로틀 램프 | 정적 식생 충돌로 실패 |
| 04: 실제0.5m/s 이후 +0.10/s | 이전 실패로 미실행 |
| 05–06: 실제1.0m/s 이후 +0.05/+0.10/s | 이전 실패로 미실행 |

두 타력 주행은 동일 초기 조건의 반복입니다. 실제3.039646m/s에서 페달을 모두 놓은 후, 처음으로0.10m/s 이하가2초 연속 관측된 시점은{dwell['seconds_after_release']:.2f}초, 주행거리는{dwell['travel_after_release_m']:.4f}m입니다. 45초 마지막 관측은{final['travel_after_release_m']:.4f}m / {final['speed_mps']:.5f}m/s입니다. 이는 관측된 정지 거리이며 **목표 위치 정지 통과나 독립 경로 검증이 아닙니다**. 타력 구간에는 속도 변화율±2.9m/s² 위반이 없지만, 보존된 출발 준비 구간에는 위반이 있습니다.

![실측 타력 주행과 전체 준비 구간](01_extended_coast_observed_stop.png)

03번은 조향0으로 진행하다37.552m 부근에서 `static.vegetation`과 충돌했습니다. CTE약1.605m는3m 제한보다 작았지만 충돌 검사는 별도로 실패했습니다. 최종 프레임5120의 속도6.583→3.207m/s 급변도 삭제하지 않았습니다. 메타데이터의212틱은 성공한 가속 전환 틱만 의미하며, 실제 원시 구간213틱에는 충돌 틱이 포함됩니다. 30kph 순항 성공으로 해석하면 안 됩니다.

03번 전체 원시 샘플은319개입니다. 고정 다운샘플링상10Hz offset0에는 최종 충돌 프레임5120이 포함되고, offset1은5119에서 끝납니다. offset1만 선택해 충돌 영향이 없었다고 결론내리지 않습니다.

페달·조향 그래프는 CARLA API가 보고한 제어 값입니다. 해당 값이 물리 서브스텝에 실제 적용된 시점까지 입증하는 자료는 아닙니다.

![충돌을 포함한 전체 속도·페달·실제 궤적](02_handoff_collision_full_history.png)

계측 조건: Town07 기존 직진 경로, Toyota Prius, ClearNoon, Low 렌더링, 물리20Hz, 각 실험 신규 차량, 준비3.5초. 물리 설정 변경·수동 기어·학습 모델 제어·카메라 촬영은 없습니다. 모든 원시 샘플과 구간 경계를 포함해20Hz 및 두 가지10Hz 다운샘플링 위치를 독립 재계산했습니다. 10Hz는 카메라 측정이 아닙니다. 속도 변화율만 검사한 결과이지 전체 XY/디코더/실행 안전성 검증은 아닙니다.

[재계산 결과](summary.json), [원본·소스 해시와 공개 자료 출처](provenance.json), [공개 파일 SHA256](SHA256SUMS).
원본은 저장소의 `artifacts/training/2026-09-08/low_speed_response_v3/run_001/actuation.partial`에 그대로 보존했습니다. 실행 기준 커밋은 `{COMMIT}`이며10개 실행 소스의 보관 바이트와 사후 동일성을 검증했습니다. 공개 JSON은 원본 전체 복사본이 아닌 재계산·경로 비식별 메타데이터입니다. PNG는 실측 자료의 그래프이며 시뮬레이터 화면을 만들어낸 이미지가 아닙니다.

재현: `python3 scripts/e2e/summarize_carla_low_speed_response_v3.py --trial artifacts/training/2026-09-08/low_speed_response_v3/run_001 --output <새로운_출력_폴더>`. 이미 존재하는 폴더는 덮어쓰지 않습니다. 모델/검증/테스트 데이터셋에는 아무것도 추가하지 않았습니다.
""")
    write("SHA256SUMS", "".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir()) if path.is_file()))
    return result


def main(argv=None):
    """HH_260906 - The CLI only reads completed evidence and creates a new aggregate publication directory."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = publish(args.trial, args.output)
    print(json.dumps({key: result[key] for key in ("evidence_verification", "campaign_status", "planned_case_count", "completed_case_count", "failed_case_count", "not_run_case_count", "raw_state_count")}, indent=2))


if __name__ == "__main__":
    main()
