#!/usr/bin/env python3
"""HH_260906 - Verify the declared complete v1/v2 pedal measurements, never training acceptance."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_goal_stop_trials as base

PHASES = ("settle", "prepare", "throttle_hold", "brake_hold")
SOURCES = tuple("scripts/e2e/" + name for name in (
    "run_owned_carla_expert_trial.sh", "run_carla_map.sh", "process_group_cleanup.sh", "workspace_runtime_lock.sh",
    "probe_carla_server.py", "env.sh", "collect_carla_vad_expert.py", "carla_goal_stop_profile.py", "calibrate_carla_low_speed_response.py"))
require, number = base.require, base.number


def matrix(matrix_id="low_speed_v1"):
    # HH_260906 - Independently freeze both known plans; do not import the evolving live collector's matrix.
    if matrix_id == "low_speed_v2":
        cases = []
        for kind, repeats, level, seconds in (("coast", 2, 0.0, 20.0), ("throttle", 3, 0.15, 8.0)):
            for repeat in range(1, repeats + 1):
                label = "coast" if kind == "coast" else "throttle_0.15"
                cases.append({"case_id": f"{len(cases) + 1:02d}_{label}_repeat_{repeat:02d}", "kind": kind,
                              "level": level, "repeat": repeat, "hold_seconds": seconds, "ramp_rate_per_second": None})
        for rate in (0.01, 0.025, 0.05, 0.10):
            cases.append({"case_id": f"{len(cases) + 1:02d}_throttle_ramp_{rate:.3f}_per_second", "kind": "throttle_ramp",
                          "level": 0.4, "repeat": 1, "hold_seconds": 8.0, "ramp_rate_per_second": rate})
        return cases
    require(matrix_id == "low_speed_v1", "unknown pedal measurement matrix")
    cases = []
    for kind, levels in (("throttle", (0.05, 0.10, 0.15, 0.20, 0.30, 0.40)),
                         ("brake", (0.02, 0.04, 0.06, 0.08, 0.10, 0.12))):
        for level in levels:
            cases.append({"case_id": f"{len(cases) + 1:02d}_{kind}_{level:.2f}", "kind": kind, "level": level})
    return cases


def identification_contract():
    # HH_260906 - These constants describe the archived second plan, including ramps that never reach the cap.
    return {
        "schema": "carla.low_speed_response_matrix.v1", "matrix_id": "low_speed_v2", "cases": matrix("low_speed_v2"),
        "coast_prepare": {"throttle": 0.3, "measured_speed_threshold_mps": 3.0, "maximum_seconds": 15.0,
                          "entry_speed_is_recorded_not_assumed_exact": True},
        "settle_seconds": 3.5, "physics_hz": 20.0,
        "ramp_policy": {"initial_throttle": 0.0,
            "command_formula": "min(0.40, rate_per_second * (zero_based_hold_tick + 1) / 20.0)",
            "hold_ticks": 160, "rates_per_second": [0.01, 0.025, 0.05, 0.1],
            "final_commanded_throttles": [0.08, 0.2, 0.4, 0.4], "all_ramps_reach_cap": False},
        "constant_launch_repeat_count": 3, "coast_repeat_count": 2, "fresh_vehicle_each_case": True,
        "independent_route_claim": False, "manual_gear_changes": False, "physics_parameter_changes": False,
        "simultaneous_throttle_and_brake": False, "training_data": False, "automatic_quality_promotion": False,
        "notice": "Repeated initial conditions identify launch and zero-pedal behavior; no chosen profile is declared safe by this matrix alone."}


def phase_names(rows):
    return PHASES + tuple(name for name in ("coast_hold", "throttle_ramp") if any(row["phase"] == name for row in rows))


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def read_bytes(root, name, ledger):
    path = root / name
    require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root.resolve()), "unsafe or missing binary/source evidence")
    raw = path.read_bytes()
    ledger.append({"path": name, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    return raw


def summarize_rates(rows, bounds):
    """HH_260906 - Independently recalculate both decimations, retaining all startup and stop impulses."""
    result = {}
    for name, selected, stride in (("native_20hz", rows, 1), ("derived_10hz_offset_0", rows[::2], 2), ("derived_10hz_offset_1", rows[1::2], 2)):
        values = []
        for first, second in zip(selected, selected[1:]):
            dt = second["timestamp"] - first["timestamp"]
            require(dt > 0 and abs(dt - 0.05 * stride) < 1.0e-4 and second["frame"] - first["frame"] == stride,
                    "measured cadence/frame stride differs from the fixed plan")
            initial, speed = math.hypot(first["vx"], first["vy"]), math.hypot(second["vx"], second["vy"])
            rate = (speed - initial) / dt
            values.append({"from_frame": first["frame"], "to_frame": second["frame"],
                "from_phase": first["phase"], "to_phase": second["phase"], "dt_sec": dt,
                "from_speed_mps": initial, "to_speed_mps": speed, "speed_rate_mps2": rate,
                "timestamp": second["timestamp"], "applied_control": second["applied_control"]})
        phases = {}
        for phase in ("all", *phase_names(rows)):
            subset = values if phase == "all" else [item for item in values if item["to_phase"] == phase]
            rates = [item["speed_rate_mps2"] for item in subset]
            details = {"interval_count": len(subset), "minimum_speed_rate_mps2": min(rates) if rates else None,
                       "maximum_speed_rate_mps2": max(rates) if rates else None,
                       "phase_boundary_intervals": [item for item in subset if item["from_phase"] != item["to_phase"]]}
            for kind in ("physical_decoder", "runtime_speed_rate_gate"):
                threshold = bounds[kind]
                failed = [item for item in subset if item["speed_rate_mps2"] > threshold["maximum_acceleration_mps2"] + 1.0e-9
                          or item["speed_rate_mps2"] < -threshold["maximum_deceleration_mps2"] - 1.0e-9]
                details[kind] = {"violation_count": len(failed), "violation_intervals": failed}
            phases[phase] = details
        result[name] = {"sample_count": len(selected), "cadence_violation_count": 0, "frame_stride_violation_count": 0,
                        "phases": phases}
    return result


def verify_recorded_rates(recorded, recalculated):
    """HH_260906 - Check stored calculations, but publish independently recomputed values and controls."""
    require(set(recorded) == set(recalculated), "stored cadence set mismatch")
    for cadence, recomputed in recalculated.items():
        original = recorded[cadence]
        require(set(original["phases"]) == set(recomputed["phases"]), "stored phase accounting mismatch")
        require(all(original[key] == recomputed[key] for key in ("sample_count", "cadence_violation_count", "frame_stride_violation_count")),
                "stored sample/cadence accounting mismatch")
        for phase, measured in recomputed["phases"].items():
            old = original["phases"][phase]
            for field in ("interval_count", "minimum_speed_rate_mps2", "maximum_speed_rate_mps2"):
                expected, actual = measured[field], old[field]
                require(actual == expected if expected is None or field == "interval_count" else
                        abs(number(actual) - expected) <= 1.0e-9, "stored rate summary mismatch")
            for kind in ("physical_decoder", "runtime_speed_rate_gate"):
                require(old[kind]["violation_count"] == measured[kind]["violation_count"], "stored violation count mismatch")
                require(len(old[kind]["violation_intervals"]) == len(measured[kind]["violation_intervals"]), "stored violation interval count mismatch")
                for first, second in zip(old[kind]["violation_intervals"], measured[kind]["violation_intervals"]):
                    require(all(first[key] == second[key] for key in ("from_frame", "to_frame", "from_phase", "to_phase"))
                            and abs(number(first["speed_rate_mps2"]) - second["speed_rate_mps2"]) <= 1.0e-9,
                            "stored violation witness differs from raw states")


def verify_case(case, report, rows, bounds, matrix_id="low_speed_v1"):
    require(case in matrix(matrix_id), "case is outside the declared immutable matrix")
    require(report.get("matrix_id", "low_speed_v1") == matrix_id, "case matrix identity mismatch")
    require(report.get("case") == case and report.get("status") == "complete" and report.get("training_data") is False,
            "case report is not the declared completed measurement")
    require(report.get("cleanup", {}).get("completed") is True and report["cleanup"].get("errors") == []
            and report.get("collision_events") == [], "case cleanup/collision proof failed")
    require(rows and report["state_count"] == len(rows), "case state count mismatch")
    physics = report.get("vehicle_physics", {})
    require(physics.get("schema") == "carla.vehicle_physics_snapshot.v1" and isinstance(physics.get("values"), dict),
            "numeric vehicle physics snapshot is missing")
    values = physics["values"]
    require(number(values["mass"]) > 0 and number(values["max_rpm"]) > 0
            and isinstance(values.get("wheels"), list) and len(values["wheels"]) == 4,
            "vehicle physics mass, engine or wheel count is invalid")
    for wheel in values["wheels"]:
        require(number(wheel["radius"]) > 0 and number(wheel["max_brake_torque"]) >= 0
                and number(wheel["tire_friction"]) >= 0, "wheel physics is invalid")
    require(report.get("post_despawn_empty_world_frame") == rows[-1]["frame"] + 1, "post-despawn frame proof mismatch")
    phases = [row["phase"] for row in rows]
    counts = {phase: phases.count(phase) for phase in phase_names(rows)}
    if case["kind"] in ("throttle", "throttle_ramp"):
        expected = ["settle"] * 70 + ["throttle_hold" if case["kind"] == "throttle" else "throttle_ramp"] * 160
    else:
        require(1 <= counts["prepare"] <= 300, "brake/coast preparation duration is invalid")
        expected = ["settle"] * 70 + ["prepare"] * counts["prepare"] + (
            ["brake_hold"] * 300 if case["kind"] == "brake" else ["coast_hold"] * 400)
    require(phases == expected and report["phase_counts"] == counts, "planned phase sequence/duration mismatch")
    for index, row in enumerate(rows):
        base.integer(row["frame"])
        for key in ("timestamp", "vx", "vy", "x", "y", "travel_m", "route_cte_m"):
            number(row[key])
        require(0 <= row["travel_m"] <= 80 and 0 <= row["route_cte_m"] <= 3 and row["vx"] >= -0.1
                and math.hypot(row["vx"], row["vy"]) <= 10 and row.get("collision") == [], "raw case violates its declared safety bounds")
        throttle, brake = (0.0, 1.0) if row["phase"] == "settle" else (0.3, 0.0) if row["phase"] == "prepare" else (
            (case["level"], 0.0) if row["phase"] == "throttle_hold" else (0.0, case["level"]))
        if row["phase"] == "throttle_ramp":
            throttle, brake = min(0.4, case["ramp_rate_per_second"] * (index - 70 + 1) / 20.0), 0.0
        elif row["phase"] == "coast_hold":
            throttle, brake = 0.0, 0.0
        for field in ("requested_control", "applied_control"):
            command = base.control(row[field])
            require(abs(command["throttle"] - throttle) <= 1.0e-6 and abs(command["brake"] - brake) <= 1.0e-6
                    and abs(command["steer"]) <= 1.0e-6 and row[field].get("reverse") is False
                    and row[field].get("hand_brake") is False and row[field].get("manual_gear_shift") is False,
                    "raw pedal command differs from the fixed matrix")
    entry = None
    if case["kind"] in ("brake", "coast"):
        prepared = [row for row in rows if row["phase"] == "prepare"]
        entry = math.hypot(prepared[-1]["vx"], prepared[-1]["vy"])
        require(entry >= 3.0 and all(math.hypot(row["vx"], row["vy"]) < 3.0 for row in prepared[:-1]), "brake entry was not the first measured 3 m/s crossing")
        require(report[case["kind"] + "_entry_frame"] == prepared[-1]["frame"]
                and abs(number(report[case["kind"] + "_entry_speed_mps"]) - entry) <= 1.0e-9,
                "reported brake entry differs from actual raw entry")
    maximum = max(math.hypot(row["vx"], row["vy"]) for row in rows)
    final = math.hypot(rows[-1]["vx"], rows[-1]["vy"])
    require(abs(number(report["maximum_speed_mps"]) - maximum) <= 1.0e-9 and abs(number(report["final_speed_mps"]) - final) <= 1.0e-9,
            "stored final/maximum speed mismatch")
    rates = summarize_rates(rows, bounds)
    verify_recorded_rates(report["motion_analysis"]["measurements"], rates)
    result = {**case, "state_count": len(rows), "phase_counts": counts, "maximum_speed_mps": maximum, "final_speed_mps": final,
            "brake_entry_speed_mps": entry if case["kind"] == "brake" else None, "actor_id": base.integer(report["actor_id"]),
            "vehicle_physics_canonical_sha256": canonical_sha(report["vehicle_physics"]), "measurements": rates}
    if matrix_id == "low_speed_v2":
        result.update(coast_entry_speed_mps=entry if case["kind"] == "coast" else None,
                      final_commanded_throttle=number(rows[-1]["requested_control"]["throttle"]),
                      final_commanded_brake=number(rows[-1]["requested_control"]["brake"]))
    return result


def verify_trial(root):
    root = Path(root).resolve()
    ledger = []
    def read(name, **kwargs):
        return base.read_file(root, name, ledger, **kwargs)
    plan, started, owner = read("owner_plan.json"), read("owner_started.json"), read("owner_result.json")
    # HH_260906 - Preserve the v1 evidence ordering while selecting only an explicitly recognized archive contract.
    initial_manifest = base.read_file(root, "actuation/manifest.json", [])
    matrix_id = initial_manifest.get("matrix_id", "low_speed_v1")
    expected_matrix = matrix(matrix_id)
    sources = SOURCES + (("scripts/e2e/carla_low_speed_response_matrix.py",) if matrix_id == "low_speed_v2" else ())
    require(owner.get("exit_code") == 0 and owner.get("capture_mode") == plan.get("capture_mode") == "actuation-response"
            and owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False,
            "owned calibration did not finalize successfully in measurement-only mode")
    require(owner.get("source_bytes_unchanged_and_archived") is True and plan.get("source_bytes_archived") is True
            and set(plan["source_sha256"]) == set(sources) and set(owner["source_checks"]) == set(sources)
            and all(value is True for value in owner["source_checks"].values()), "source freeze/postcheck proof is incomplete")
    for name in sources:
        raw = read_bytes(root, "provenance/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == plan["source_sha256"][name], "archived executed source hash mismatch")
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and started["map"] == plan["map"] == "Town07"
            and plan["host"] == "127.0.0.1" and started["port"] == plan["port"], "owned startup identity mismatch")
    server_log = read_bytes(root, "server.log", ledger)
    prior_time = datetime.fromisoformat(started["started_at_utc"])
    for stage in ("ready", "after_capture", "stopped"):
        proof = read(f"lifecycle/{stage}.json")
        require(proof.get("status") == "PASS" and proof.get("stage") == stage and proof.get("read_only") is True
                and proof.get("error") is None and proof["owner_pid"] == proof["owner_pgid"] == pid
                and proof["generation_id"] == f"expert_{pid}" and proof["host"] == plan["host"]
                and proof["port"] == plan["port"] and proof["expected_map"] == "Town07", "scoped lifecycle proof mismatch")
        log = proof["server_log"]
        require(0 < log["size_bytes"] <= len(server_log) and hashlib.sha256(server_log[:log["size_bytes"]]).hexdigest() == log["sha256"],
                "lifecycle server-log prefix hash mismatch")
        if stage == "stopped":
            require(proof.get("mode") == "stopped" and proof.get("port_released") is True and proof.get("owner_process_state") is None,
                    "simulator lacks complete stopped proof")
        else:
            require(proof.get("mode") == "running" and proof.get("active_map_basename") == "Town07", "wrong running simulator map")
        now = datetime.fromisoformat(proof["checked_at"])
        if stage != "ready":
            require(now >= prior_time, "lifecycle evidence timestamps are out of order")
        prior_time = now
    require(datetime.fromisoformat(owner["completed_at_utc"]) >= prior_time, "owner result predates stopped proof")
    manifest = read("actuation/manifest.json")
    require(manifest == initial_manifest, "matrix manifest changed during review")
    require(manifest.get("schema") == "carla.low_speed_response_calibration.v1" and manifest.get("status") == "complete"
            and manifest.get("training_data") is False and manifest.get("client_map_loading_allowed") is False
            and manifest.get("workspace_ownership", {}).get("verified_exclusive") is True,
            "calibration manifest is incomplete or outside measurement scope")
    require(manifest["matrix"] == expected_matrix and manifest["case_ledger"] == [{"case_id": case["case_id"], "status": "complete"} for case in expected_matrix],
            "all twelve/nine planned cases must be completed, without omission or replacement")
    require([item["case_id"] for item in manifest["completed_cases"]] == [case["case_id"] for case in expected_matrix]
            and all(item["status"] == "complete" for item in manifest["completed_cases"]), "completed-case denominator mismatch")
    require({p.name for p in (root / "actuation").iterdir() if p.is_dir()} == {case["case_id"] for case in expected_matrix}, "unexpected or missing case directory")
    phase_contract = {"settle_seconds": 3.5, "throttle_hold_seconds": 8.0, "prepare_maximum_seconds": 15.0,
        "prepare_throttle": 0.3, "prepare_measured_speed_mps": 3.0, "brake_hold_seconds": 15.0}
    if matrix_id == "low_speed_v2":
        declared = identification_contract()
        require(manifest.get("identification_matrix_contract") == declared, "identification matrix contract differs from frozen v2")
        phase_contract.update(coast_hold_seconds=20.0, throttle_ramp_seconds=8.0, throttle_ramp_definition=declared["ramp_policy"])
        argv = plan["collector_argv"]
        require(argv.count("--matrix") == 1 and argv[argv.index("--matrix") + 1] == "low_speed_v2",
                "owner argv does not explicitly select the v2 matrix")
    require(manifest["phase_contract"] == phase_contract, "planned timing changed")
    require(manifest["limits"] == {"maximum_travel_m": 80.0, "maximum_cte_m": 3.0, "maximum_speed_mps": 10.0, "maximum_reverse_speed_mps": 0.1}, "planned safety limits changed")
    require(manifest["physics_hz"] == 20.0 and manifest["vehicle_type"] == "vehicle.toyota.prius"
            and manifest["role_name"] == "autoware_e2e_low_speed_calibration" and manifest["wheelbase_m"] == 2.85
            and manifest["spawn_z_offset_m"] == 0.5 and manifest["weather"] == "ClearNoon", "fixed vehicle or environment changed")
    require(manifest["cleanup"].get("completed") is True and manifest["cleanup"].get("errors") == [], "world restoration reported failure")
    settings = manifest["runtime"]["capture_world_settings"]
    require(settings["synchronous_mode"] is True and settings["fixed_delta_seconds"] == 0.05 and settings["substepping"] is True
            and settings["max_substep_delta_time"] * settings["max_substeps"] >= 0.05, "actual physics timing is incompatible")
    route = read_bytes(root, "actuation/route.json", ledger)
    require(hashlib.sha256(route).hexdigest() == manifest["route_sha256"] == started["route_sha256"] == plan["route_sha256"], "route hash differs across owner and calibration")
    calibrator_sources = {"calibrate_carla_low_speed_response.py", "collect_carla_vad_expert.py", "carla_goal_stop_profile.py"}
    if matrix_id == "low_speed_v2":
        calibrator_sources.add("carla_low_speed_response_matrix.py")
    require(set(manifest["source_sha256"]) == calibrator_sources, "calibrator source denominator mismatch")
    for name, expected in manifest["source_sha256"].items():
        require(plan["source_sha256"].get("scripts/e2e/" + name) == expected, "calibrator source differs from frozen owner source")
    bounds = base.source_bounds()
    require(all(manifest["bounds"][key] == value for key, value in bounds.items()), "physical scalar bounds/source changed")
    cases, raw_cases, physics, prior_frame = [], {}, None, None
    for case, reference in zip(expected_matrix, manifest["completed_cases"]):
        case_id = case["case_id"]
        report = read(f"actuation/{case_id}/report.json")
        require(ledger[-1]["sha256"] == reference["report_sha256"], "case report hash mismatch")
        require(report["motion_analysis"].get("bounds") == manifest["bounds"], "case scalar bounds differ from the frozen manifest")
        rows = read(f"actuation/{case_id}/states.jsonl", jsonl=True)
        require(ledger[-1]["sha256"] == report["states_sha256"], "raw state hash mismatch")
        measured = verify_case(case, report, rows, bounds, matrix_id)
        require(prior_frame is None or rows[0]["frame"] > prior_frame, "new case overlaps previous actor generation frames")
        prior_frame = report["post_despawn_empty_world_frame"]
        if physics is None:
            physics = report["vehicle_physics"]
        require(report["vehicle_physics"] == physics, "vehicle physics changed between cases")
        cases.append(measured)
        raw_cases[case_id] = rows
    require(len({case["actor_id"] for case in cases}) == len(expected_matrix), "each case requires a distinct fresh vehicle actor")
    for item in ledger:
        require(base.sha(root / item["path"]) == item["sha256"], "input changed during batch review")
    entry_kind = "brake" if matrix_id == "low_speed_v1" else "coast"
    entries = [case[entry_kind + "_entry_speed_mps"] for case in cases if case["kind"] == entry_kind]
    result = {"schema": "portable_e2e.pedal_response_summary.v1", "status": ("TWELVE" if matrix_id == "low_speed_v1" else "NINE") + "_MEASUREMENTS_VERIFIED_NOT_PROMOTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "summarizer_source_sha256": base.sha(Path(__file__)),
        "supporting_reader_source_sha256": base.sha(Path(base.__file__)), "source_manifest": ledger,
        "planned_and_included_case_count": len(expected_matrix), "cases": cases, "scalar_bounds": bounds, "vehicle_physics": physics,
        "vehicle_physics_identical_across_all_cases": True,
        entry_kind + "_entry_speed_mps": {"minimum": min(entries), "maximum": max(entries), "observed_range": max(entries) - min(entries), "threshold_not_exact_target_mps": 3.0},
        "executed_sources": {"base_git_head": plan["source_head_commit"], "worktree_was_dirty": bool(plan["source_worktree_status"]),
            "source_sha256": plan["source_sha256"], "archived_bytes_and_postchecks_verified": True,
            "notice": "Base HEAD is not claimed to contain the executed uncommitted scripts; their archived exact hashes are authoritative."},
        "actual_runtime": manifest["runtime"], "scope": {"all_settle_start_stop_samples_retained": True, "phase_boundaries_retained": True,
            "speed_threshold_filter_applied": False, "ten_hz_is_two_derived_offsets_not_cameras": True,
            "cameras_collected": False, "training_data": False, "test_dataset_used": False, "learned_model_control": False, "automatic_promotion": False},
        "limitations": ["These are fixed-pedal simulator measurements, not autonomous-driving tests or accepted training data.",
            "One spawn/vehicle/weather and one run per level do not establish a universally safe pedal value.",
            "Derived10Hz traces include both offsets; neither is a camera recording.",
            "Physics settings are equal across cases but engine/wheel RPM, filtered inputs and tire sticky states are not observed.",
            "Scalar violations remain visible even at near-zero speed; no startup or stop samples are discarded."]}
    if matrix_id == "low_speed_v2":
        result.update(matrix_id=matrix_id, identification_matrix_contract=identification_contract())
        result["limitations"][0] = "These are simulator coast, fixed-pedal and ramp measurements, not autonomous-driving tests or accepted training data."
        result["limitations"][1] = "One spawn/vehicle/weather and repeated identical conditions do not establish a universally safe pedal profile."
    return result, raw_cases


def draw_charts(result, raw_cases, output):
    # HH_260906 - Align each response to its first commanded hold interval while keeping the preceding measured state.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if result.get("matrix_id") == "low_speed_v2":
        draw_identification_charts(result, raw_cases, output, plt)
        return
    for kind, filename in (("throttle", "01_constant_throttle_startup.png"), ("brake", "02_constant_brake_response.png")):
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        for case in result["cases"]:
            if case["kind"] != kind:
                continue
            rows = raw_cases[case["case_id"]]
            start = next(i for i, row in enumerate(rows) if row["phase"] == kind + "_hold")
            selected = rows[start - 1:]
            origin = rows[start - 1]["timestamp"]
            label = f"{kind}={case['level']:.2f}"
            if kind == "brake":
                label += f" | entry {case['brake_entry_speed_mps']:.4f} m/s"
            line = axes[0].plot([row["timestamp"] - origin for row in selected], [math.hypot(row["vx"], row["vy"]) for row in selected], label=label)[0]
            axes[1].plot([row["timestamp"] - origin for row in selected[1:]],
                [(math.hypot(b['vx'], b['vy']) - math.hypot(a['vx'], a['vy'])) / (b['timestamp'] - a['timestamp']) for a, b in zip(selected, selected[1:])], color=line.get_color(), alpha=0.85)
        axes[0].set_ylabel("Measured speed (m/s)")
        axes[0].legend(fontsize=9, ncol=2)
        axes[1].set_ylabel("Actual 20 Hz speed rate (m/s²)")
        axes[1].set_xlabel("Seconds after preceding state / first constant-pedal interval")
        axes[1].axhline(2.9, color="black", linestyle="--")
        axes[1].axhline(-2.9, color="black", linestyle="--", label="Unchanged decoder +/-2.9")
        axes[1].legend(fontsize=9)
        for axis in axes:
            axis.grid(alpha=0.18)
        fig.suptitle(f"Constant {kind} response | 6 fixed levels, actual measurements", fontweight="bold")
        fig.text(0.5, 0.015, "No low-speed filtering. Hold views retain entry prefix; full settle/preparation/hold phases remain in JSON. No cameras or learned driving.", ha="center", fontsize=9)
        fig.tight_layout(rect=(0, 0.045, 1, 0.95))
        fig.savefig(output / filename, dpi=150)
        plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for axis, kind, metric in ((axes[0], "throttle", "maximum_speed_rate_mps2"), (axes[1], "brake", "minimum_speed_rate_mps2")):
        cases = [case for case in result["cases"] if case["kind"] == kind]
        for offset, (cadence, label) in enumerate((("native_20hz", "Actual20Hz"), ("derived_10hz_offset_0", "Derived10Hz offset0"), ("derived_10hz_offset_1", "Derived10Hz offset1"))):
            bars = axis.bar([i + (offset - 1) * 0.25 for i in range(6)], [case['measurements'][cadence]['phases'][kind + '_hold'][metric] for case in cases], width=0.24, label=label)
            axis.bar_label(bars, fmt="%.1f", padding=3, fontsize=7)
        axis.axhline(2.9 if kind == "throttle" else -2.9, color="black", linestyle="--")
        axis.set_xticks(range(6), [f"{case['level']:.2f}" for case in cases])
        axis.set_xlabel(f"Constant normalized {kind}")
        axis.set_ylabel("Maximum acceleration (m/s²)" if kind == "throttle" else "Minimum acceleration (m/s²)")
        axis.grid(axis="y", alpha=0.18)
    axes[0].legend(fontsize=8)
    fig.suptitle("Startup and low-speed stop peaks remain visible at both derived10Hz offsets", fontweight="bold")
    fig.text(0.5, 0.02, "Hold-phase intervals include the preceding phase state. Derived10Hz is not camera capture. No universal pedal recommendation or training acceptance.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.07, 1, 0.93))
    fig.savefig(output / "03_peak_rates_by_pedal_and_cadence.png", dpi=150)
    plt.close(fig)


def draw_identification_charts(result, raw_cases, output, plt):
    """HH_260906 - Plot all nine actual responses, disclosing overlapping repeats and unfinished coast stops."""
    for kind, filename, title in (
        ("coast", "01_zero_pedal_coast_repeats.png", "Zero-pedal coast | two fresh-vehicle repeats, complete 20 s hold"),
        ("throttle", "02_constant_throttle_repeats.png", "Constant throttle 0.15 | three fresh-vehicle repeats, complete 8 s hold"),
        ("throttle_ramp", "03_gradual_throttle_ramps.png", "Four declared throttle ramps | actual commands, speeds and speed rates")):
        ramp = kind == "throttle_ramp"
        fig, axes = plt.subplots(3 if ramp else 2, 1, figsize=(12, 10 if ramp else 8), sharex=True)
        speed_axis, rate_axis = axes[-2:]
        for index, case in enumerate(item for item in result["cases"] if item["kind"] == kind):
            rows = raw_cases[case["case_id"]]
            phase = "throttle_ramp" if ramp else kind + "_hold"
            start = next(i for i, row in enumerate(rows) if row["phase"] == phase)
            selected = rows[start - 1:]
            times = [row["timestamp"] - selected[0]["timestamp"] for row in selected]
            speeds = [math.hypot(row["vx"], row["vy"]) for row in selected]
            label = (f"rate={case['ramp_rate_per_second']:.3f}/s | final pedal={case['final_commanded_throttle']:.2f}" if ramp
                     else f"repeat {case['repeat']}" + (f" | entry={case['coast_entry_speed_mps']:.4f} m/s" if kind == "coast" else ""))
            style = "-" if ramp else ("-", "--", ":")[index]
            line = speed_axis.plot(times, speeds, linestyle=style, label=label, linewidth=1.6)[0]
            rates = [(b - a) / (t1 - t0) for a, b, t0, t1 in zip(speeds, speeds[1:], times, times[1:])]
            rate_axis.plot(times[1:], rates, color=line.get_color(), linestyle=style, linewidth=1.2)
            if ramp:
                # HH_260906 - Each recorded command acted on the interval ending at its timestamp, hence pre steps.
                axes[0].step(times, [row["requested_control"]["throttle"] for row in selected], where="pre", color=line.get_color(), label=label)
        speed_axis.set_ylabel("Measured speed (m/s)")
        speed_axis.legend(fontsize=9, ncol=2 if ramp else 1)
        rate_axis.set_ylabel("Actual 20 Hz speed rate (m/s²)")
        rate_axis.set_xlabel("Seconds after preceding measured state / first hold interval")
        rate_axis.axhline(2.9, color="black", linestyle="--", label="Unchanged decoder +/-2.9 m/s²")
        rate_axis.axhline(-2.9, color="black", linestyle="--")
        rate_axis.legend(fontsize=9)
        if ramp:
            axes[0].set_ylabel("Commanded throttle [0, 1]")
            axes[0].set_ylim(-0.015, 0.43)
        for axis in axes:
            axis.grid(alpha=0.18)
        fig.suptitle(title, fontweight="bold")
        footer = ("Coast did not fully stop by 20 s. Preparation exceeds +2.9 and remains in JSON; overlapping repeats are not independent routes." if kind == "coast"
                  else "Identical-condition repeats overlap; the narrow +2.9 margin is not a robustness or autonomous-driving claim." if kind == "throttle"
                  else "Rate 0.010/s stays effectively stationary. Other ramps retain startup peaks; no speed-rate clipping or low-speed filtering is applied.")
        fig.text(0.5, 0.025, footer + "\nActual measurements, no cameras. JSON retains every phase and both derived10Hz offsets.", ha="center", fontsize=9)
        fig.tight_layout(rect=(0, 0.075, 1, 0.95))
        fig.savefig(output / filename, dpi=150)
        plt.close(fig)


def publish(root, output):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "output already exists")
    require(not output.resolve().is_relative_to(Path(root).resolve()), "output must be outside original trial")
    result, raw_cases = verify_trial(root)
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    draw_charts(result, raw_cases, output)
    (output / "SHA256SUMS").write_text("".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir())))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("owned_trial_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = publish(args.owned_trial_root, args.output_dir)
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        print(f"PEDAL_SUMMARY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result["status"], "included_cases": result["planned_and_included_case_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
