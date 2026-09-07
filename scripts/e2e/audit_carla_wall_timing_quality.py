#!/usr/bin/env python3
"""HH_260906 - Independently audit the frozen Low/Epic timing comparison without importing the measured timing helper."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

if __package__:
    from . import audit_carla_acknowledged_control as ack
    from . import audit_carla_brake_free_goal_stop as v4
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_acknowledged_control as ack
    import audit_carla_brake_free_goal_stop as v4
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "3a069256046dfb8e06a8d068678dc3f88ed3d1c0"
SOURCES = pilot.SOURCE_NAMES | {"scripts/e2e/carla_wall_timing.py"}
STAGES = ("world_tick_snapshot", "observation_control", "camera_queue_wait", "jpeg_encode_write", "control_rpc")
PHASES, CAMERAS = base.PHASES, base.CAMERAS
require, integer, number = base.require, base.integer, base.number
ROW_FIELDS = {"sequence", "tick_index", "phase", "camera_expected", "start_ns", "start_utc", "end_ns", "end_utc", "total_ns",
    "frame", "sim_timestamp", "observation_wall_ns", "camera_recorded_wall_ns", "state_recorded", "camera_recorded", "status",
    "error_type", "instrumentation_errors", "unattributed_ns", "stages"}
PLAN_FIXED = {
    "source_commit": COMMIT, "schema": "portable_e2e.wall_timing_quality_plan.v1",
    "ordered_arms": [{"quality": "Low", "output": "low/run_001"}, {"quality": "Epic", "output": "epic/run_001"}],
    "maximum_attempts_per_quality": 1, "automatic_retry": False, "profile": "comfortable_v4",
    "control_transport": "acknowledged_batch", "wall_timing": True, "quality_is_only_between_arm_configuration_change": True,
    "route_sha256": v4.ROUTE_SHA, "route_length_m": 210.5975914062836, "map": "Town07", "scenario": "straight",
    "vehicle": "vehicle.toyota.prius", "weather": "ClearNoon", "seed": 0, "physics_hz": 20, "camera_hz": 10,
    "nominal_target_speed_kmh": 28.8, "maximum_actual_speed_kmh": 30, "maximum_total_sim_seconds": 180,
    "wall_timeout_seconds": 900, "finish_before_utc": "2026-09-08T01:00:00Z", "normal_brake_cap": 0,
    "normal_lateral_controller_unchanged": True, "emergency_control_unchanged": True, "quality_limits_unchanged": True,
    "mapping_sha256": "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
    "calibration_sha256": "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea",
    "training_data_approved": False, "learned_model_control": False, "remote_training_started": False,
    "baseline_campaign": "../brake_free_goal_stop_v4",
}


def recorded_bytes(commit, relative):
    """HH_260906 - Extend only this new protocol's Git allowlist to its timing helper; historical auditors remain unchanged."""
    require(isinstance(commit, str) and re.fullmatch(r"[a-f0-9]{40}", commit) is not None and relative in SOURCES, "unsafe recorded source identity")
    environment = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(["git", "-c", "protocol.allow=never", "show", commit + ":" + relative], cwd=ROOT,
        env=environment, timeout=10, capture_output=True, check=False)
    require(result.returncode == 0, "reviewed source history unavailable locally; no fetch attempted")
    return result.stdout


def distribution(values):
    """HH_260906 - Recompute nearest-rank wall latency statistics from every measured span, including failed spans."""
    values = sorted(values)
    if not values:
        return dict(count=0, mean_ms=None, p50_ms=None, p95_ms=None, p99_ms=None, maximum_ms=None)
    return dict(count=len(values), mean_ms=sum(values) / len(values) / 1e6,
        p50_ms=values[max(0, math.ceil(.5 * len(values)) - 1)] / 1e6,
        p95_ms=values[max(0, math.ceil(.95 * len(values)) - 1)] / 1e6,
        p99_ms=values[max(0, math.ceil(.99 * len(values)) - 1)] / 1e6, maximum_ms=values[-1] / 1e6)


def equal(actual, expected):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and set(actual) == set(expected) and all(equal(actual[k], value) for k, value in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(equal(a, b) for a, b in zip(actual, expected))
    if type(expected) in (int, float):
        return type(actual) in (int, float) and math.isfinite(actual) and math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-9)
    return type(actual) is type(expected) and actual == expected


def validate_plan(plan):
    extra = {"declared_at_utc", "source_hashes", "capture_flags", "comment", "expected_risk", "retention"}
    require(isinstance(plan, dict) and set(plan) == set(PLAN_FIXED) | extra, "unexpected timing campaign plan fields")
    require(all(v4.strict_equal(plan[k], value) for k, value in PLAN_FIXED.items()), "plan differs from frozen timing revision")
    require(all(isinstance(plan[k], str) and plan[k] for k in ("comment", "expected_risk", "retention")), "missing plan explanation")
    require(v4.utc(plan["declared_at_utc"]) < v4.utc(plan["finish_before_utc"]), "plan declared after its deadline")
    require(isinstance(plan["source_hashes"], dict) and set(plan["source_hashes"]) == SOURCES, "plan requires exact eleven sources")
    for name, digest in plan["source_hashes"].items():
        require(re.fullmatch(r"[a-f0-9]{64}", digest or "") is not None
            and hashlib.sha256(recorded_bytes(COMMIT, name)).hexdigest() == digest, "plan source differs from reviewed timing commit")
    flags = plan["capture_flags"]
    require(isinstance(flags, list) and all(isinstance(value, str) for value in flags)
            and flags.count("--wall-timing") == 1, "plan must explicitly enable timing once")
    projected = {"port": 2100, "route_path": "route.json", "collector_argv": [str(ROOT / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100",
                 *(value for value in flags if value != "--wall-timing")]}
    # HH_260906 - Project only the new Boolean timing flag; the unchanged V4 CLI validator retains every control and rig constraint.
    v4.validate_cli(projected, ROOT)


def span_valid(span, *, camera=False):
    expected = {"status", "start_ns", "end_ns", "duration_ns", "camera" if camera else "cameras"}
    require(isinstance(span, dict) and set(span) == expected, "invalid timing span field set")
    status = span["status"]
    require(status in ("COMPLETE", "FAILED", "INVALID_TIMING", "NOT_REACHED", "NOT_SCHEDULED", "PARTIAL"), "unknown timing span status")
    if status in ("NOT_REACHED", "NOT_SCHEDULED"):
        return span["start_ns"] is span["end_ns"] is None and span["duration_ns"] == (0 if status == "NOT_SCHEDULED" else None)
    for name in ("start_ns", "end_ns", "duration_ns"):
        if span[name] is not None:
            integer(span[name])
    return (span["start_ns"] is not None and span["end_ns"] is not None and span["duration_ns"] is not None
        and span["end_ns"] >= span["start_ns"] and span["duration_ns"] == span["end_ns"] - span["start_ns"])


def analyze_timing(rows, states, cameras):
    """HH_260906 - Recompute all journaled durations, stage/camera coverage, and wall throughput independently of the timing worker."""
    flags = {key: True for key in ("nonempty_attempts", "every_attempt_complete", "ordered_complete_stage_spans",
        "exact_camera_child_spans", "observation_and_append_markers", "unchanged_frame_timestamp_binding",
        "exact_state_camera_phase_coverage", "consecutive_attempts_and_frames", "no_overlap_or_clock_reversal",
        "no_continuation_after_failed_attempt", "camera_schedule_exact", "native_sim_cadence", "camera_sim_cadence")}
    issues = []
    def check(flag, passed, sequence):
        if not passed:
            flags[flag] = False
            issues.append({"check": flag, "sequence": sequence})
    observed, recorded, recorded_cameras = [], [], []
    last_end, last_phase, failed_seen = None, -1, False
    check("nonempty_attempts", bool(rows), None)
    for i, row in enumerate(rows):
        require(isinstance(row, dict) and set(row) == ROW_FIELDS, "invalid timing row fields")
        require(type(row["sequence"]) is int and row["sequence"] == i + 1 and type(row["tick_index"]) is int,
                "timing journal sequence gap, duplicate, or invalid tick index")
        require(row["phase"] in PHASES and row["status"] in ("COMPLETE", "PARTIAL_OR_FAILED", "INVALID_TIMING"), "unknown timing phase or row status")
        require(all(type(row[name]) is bool for name in ("camera_expected", "state_recorded", "camera_recorded")), "timing markers must be Boolean")
        require(isinstance(row["instrumentation_errors"], list) and all(isinstance(value, str) for value in row["instrumentation_errors"]), "invalid instrumentation error ledger")
        require(row["error_type"] is None or isinstance(row["error_type"], str), "invalid timing error type")
        integer(row["start_ns"])
        v4.utc(row["start_utc"])
        if row["end_utc"] is not None:
            v4.utc(row["end_utc"])
        for name in ("end_ns", "total_ns", "unattributed_ns", "frame", "observation_wall_ns", "camera_recorded_wall_ns"):
            if row[name] is not None:
                integer(row[name])
        duration_ok = row["end_ns"] is not None and row["end_ns"] >= row["start_ns"] and row["total_ns"] == row["end_ns"] - row["start_ns"]
        check("no_overlap_or_clock_reversal", duration_ok and (last_end is None or row["start_ns"] >= last_end), i + 1)
        last_end = row["end_ns"]
        check("no_continuation_after_failed_attempt", not failed_seen, i + 1)
        failed_seen |= row["status"] != "COMPLETE"
        phase_index = PHASES.index(row["phase"])
        check("consecutive_attempts_and_frames", row["tick_index"] == i and phase_index >= last_phase, i + 1)
        last_phase = phase_index
        check("camera_schedule_exact", row["camera_expected"] == (i % 2 == 0), i + 1)
        require(isinstance(row["stages"], dict) and set(row["stages"]) == set(STAGES), "missing or extra timing stages")
        stage_end, stage_sum, full, saw_missing = row["start_ns"], 0, True, False
        for name in STAGES:
            span = row["stages"][name]
            valid = span_valid(span)
            if span["duration_ns"] is not None:
                stage_sum += span["duration_ns"]
            if name in ("camera_queue_wait", "jpeg_encode_write") and not row["camera_expected"]:
                check("ordered_complete_stage_spans", valid and span["status"] == "NOT_SCHEDULED" and span["cameras"] == [], i + 1)
                continue
            if span["status"] == "NOT_REACHED":
                saw_missing, full = True, False
            else:
                check("ordered_complete_stage_spans", not saw_missing and valid and span["start_ns"] >= stage_end
                      and row["end_ns"] is not None and span["end_ns"] <= row["end_ns"], i + 1)
                if span["end_ns"] is not None:
                    stage_end = span["end_ns"]
                full &= span["status"] == "COMPLETE"
            require(isinstance(span["cameras"], list), "camera span ledger must be a list")
            if name in ("camera_queue_wait", "jpeg_encode_write"):
                expected_names = list(CAMERAS)[:len(span["cameras"])]
                child_names, child_end = [], span["start_ns"]
                for child in span["cameras"]:
                    child_ok = span_valid(child, camera=True)
                    child_names.append(child["camera"])
                    check("exact_camera_child_spans", child_ok and child_end is not None and child["start_ns"] >= child_end
                          and span["end_ns"] is not None and child["end_ns"] <= span["end_ns"], i + 1)
                    child_end = child["end_ns"]
                    full &= child["status"] == "COMPLETE"
                check("exact_camera_child_spans", child_names == expected_names and len(child_names) <= 6
                      and (span["status"] != "COMPLETE" or len(child_names) == 6), i + 1)
            else:
                check("exact_camera_child_spans", not span["cameras"], i + 1)
        check("ordered_complete_stage_spans", row["total_ns"] is not None and stage_sum <= row["total_ns"]
              and row["unattributed_ns"] == row["total_ns"] - stage_sum, i + 1)
        if row["frame"] is not None:
            number(row["sim_timestamp"])
            observed.append(row)
            span = row["stages"]["world_tick_snapshot"]
            check("observation_and_append_markers", row["observation_wall_ns"] is not None and span["start_ns"] is not None
                  and span["end_ns"] is not None and span["start_ns"] <= row["observation_wall_ns"] <= span["end_ns"], i + 1)
        else:
            check("observation_and_append_markers", row["sim_timestamp"] is row["observation_wall_ns"] is None
                  and not row["state_recorded"] and not row["camera_recorded"], i + 1)
        if row["state_recorded"]:
            recorded.append(row)
            check("observation_and_append_markers", row["frame"] is not None and row["stages"]["observation_control"]["status"] in ("COMPLETE", "FAILED"), i + 1)
        if row["camera_recorded"]:
            recorded_cameras.append(row)
            span = row["stages"]["jpeg_encode_write"]
            check("observation_and_append_markers", row["state_recorded"] and row["camera_expected"] and row["camera_recorded_wall_ns"] is not None
                  and span["start_ns"] is not None and span["end_ns"] is not None
                  and span["start_ns"] <= row["camera_recorded_wall_ns"] <= span["end_ns"], i + 1)
        else:
            check("observation_and_append_markers", row["camera_recorded_wall_ns"] is None, i + 1)
        complete = duration_ok and full and row["state_recorded"] and row["camera_recorded"] == row["camera_expected"]
        check("every_attempt_complete", complete and row["status"] == "COMPLETE" and row["error_type"] is None and not row["instrumentation_errors"], i + 1)
    check("exact_state_camera_phase_coverage", len(recorded) == len(states) and len(recorded_cameras) == len(cameras), None)
    for timed, source in zip(recorded, states):
        check("unchanged_frame_timestamp_binding", timed["frame"] == source["frame"] and timed["sim_timestamp"] == source["timestamp"]
              and timed["phase"] == source["capture_phase"], timed["sequence"])
    for timed, source in zip(recorded_cameras, cameras):
        check("unchanged_frame_timestamp_binding", timed["frame"] == source["frame"] and timed["sim_timestamp"] == source["timestamp"]
              and timed["phase"] == source["capture_phase"], timed["sequence"])
    for left, right in zip(observed, observed[1:]):
        check("consecutive_attempts_and_frames", right["frame"] == left["frame"] + 1, right["sequence"])
        check("native_sim_cadence", abs(right["sim_timestamp"] - left["sim_timestamp"] - .05) <= 1e-6, right["sequence"])
    for left, right in zip(recorded_cameras, recorded_cameras[1:]):
        check("camera_sim_cadence", right["frame"] == left["frame"] + 2 and abs(right["sim_timestamp"] - left["sim_timestamp"] - .1) <= 1e-6, right["sequence"])
    def rate(samples, key):
        if len(samples) < 2 or samples[-1][key] is None or samples[0][key] is None:
            return None
        delta = samples[-1][key] - samples[0][key]
        return (len(samples) - 1) * 1e9 / delta if delta > 0 else None
    totals = [row["total_ns"] for row in rows if row["total_ns"] is not None]
    sim_deltas = [b["sim_timestamp"] - a["sim_timestamp"] for a, b in zip(observed, observed[1:])]
    wall_span = observed[-1]["observation_wall_ns"] - observed[0]["observation_wall_ns"] if len(observed) > 1 else None
    ratio = (observed[-1]["sim_timestamp"] - observed[0]["sim_timestamp"]) * 1e9 / wall_span if wall_span and wall_span > 0 else None
    metrics = {"attempt_count": len(rows), "complete_attempt_count": sum(row["status"] == "COMPLETE" for row in rows),
        "failed_or_incomplete_attempt_count": sum(row["status"] != "COMPLETE" for row in rows),
        "first_utc": rows[0]["start_utc"] if rows else None, "last_utc": rows[-1]["end_utc"] if rows else None,
        "phase_counts": {phase: {"attempts": sum(row["phase"] == phase for row in rows),
            "states": sum(row["phase"] == phase for row in recorded), "camera_anchors": sum(row["phase"] == phase for row in recorded_cameras)} for phase in PHASES},
        "native_wall_observation_hz": rate(observed, "observation_wall_ns"), "camera_bundle_wall_completion_hz": rate(recorded_cameras, "camera_recorded_wall_ns"),
        "simulation_seconds_per_wall_second": ratio, "observed_native_sim_dt_min_seconds": min(sim_deltas) if sim_deltas else None,
        "observed_native_sim_dt_max_seconds": max(sim_deltas) if sim_deltas else None, "total_tick": distribution(totals),
        "stages": {name: distribution([row["stages"][name]["duration_ns"] for row in rows if row["stages"][name]["duration_ns"] is not None
            and row["stages"][name]["status"] != "NOT_SCHEDULED"]) for name in STAGES},
        "per_camera": {name: {camera: distribution([child["duration_ns"] for row in rows for child in row["stages"][name]["cameras"]
            if child["camera"] == camera and child["duration_ns"] is not None]) for camera in CAMERAS} for name in ("camera_queue_wait", "jpeg_encode_write")},
        "observation_control_plus_rpc": distribution([sum(row["stages"][name]["duration_ns"] for name in ("observation_control", "control_rpc"))
            for row in rows if all(row["stages"][name]["duration_ns"] is not None for name in ("observation_control", "control_rpc"))]),
        "unattributed_in_tick": distribution([row["unattributed_ns"] for row in rows if row["unattributed_ns"] is not None]),
        "inter_tick_gap": distribution([b["start_ns"] - a["end_ns"] for a, b in zip(rows, rows[1:]) if a["end_ns"] is not None and b["start_ns"] >= a["end_ns"]]),
        "tick_duration_above_ms_counts": {str(ms): sum(value > ms * 1e6 for value in totals) for ms in (50, 100, 500, 1000)}}
    phase_latencies = {phase: distribution([row["total_ns"] for row in rows if row["phase"] == phase and row["total_ns"] is not None]) for phase in PHASES}
    return {"status": "PASS" if all(flags.values()) else "FAIL", "flags": flags, "failed_flags": [name for name, passed in flags.items() if not passed],
        "issues": issues, "metrics": metrics, "phase_total_latency": phase_latencies,
        "persistence_durations_independently_reconstructed": False,
        "scope": "All journaled tick/stage/camera intervals and wall throughput are independently reconstructed. Persistence latency is only a recorded aggregate, not journaled per event. GUI display FPS, learned inference, CPU/GPU attribution and physical actuator timing are not measured."}


def verify_sources(root, quality, prospective, ledger):
    """HH_260906 - Bind eleven archived execution files to both recorded and separately reviewed commits, never to the live worktree."""
    read = lambda name: pilot.read_json(root, name, ledger)
    owner, result, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    require(owner.get("source_bytes_archived") is owner.get("bounds_source_bytes_archived") is True
        and owner.get("wall_timing_enabled") is owner.get("wall_timing_source_bytes_archived") is True
        and owner.get("wall_timing_schema") == "carla.expert_wall_timing.v1", "timing/source archives not declared")
    require(owner["source_sha256"] == prospective["source_hashes"] and set(owner["source_sha256"]) == SOURCES, "execution source set differs from plan")
    require(result.get("source_bytes_unchanged_and_archived") is True and set(result["source_checks"]) == SOURCES
        and all(value is True for value in result["source_checks"].values()), "source postchecks failed")
    require(re.fullmatch(r"[a-f0-9]{40}", owner["source_head_commit"]) is not None, "invalid owner commit")
    require({p.relative_to(root / "provenance").as_posix() for p in (root / "provenance").rglob("*")
        if p.is_file() or p.is_symlink()} == SOURCES, "missing or unexpected source archive")
    for name, digest in owner["source_sha256"].items():
        raw = pilot.checked_bytes(root, "provenance/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == digest and raw == recorded_bytes(COMMIT, name)
            and raw == recorded_bytes(owner["source_head_commit"], name), "reviewed/owner/source archive byte mismatch")
    require(owner.get("host") == "127.0.0.1" and owner.get("map") == started.get("map") == "Town07"
        and owner.get("quality") == started.get("quality") == quality and quality in ("Low", "Epic")
        and owner.get("capture_mode") == result.get("capture_mode") == "expert"
        and owner.get("worker_path") == "scripts/e2e/collect_carla_vad_expert.py"
        and owner.get("route_sha256") == prospective["route_sha256"], "owned capture scope differs from timing plan")
    require(all(item.get(key) is False for item in (owner, result) for key in ("learned_model_control", "vehicle_control_approved")), "unexpected control approval")
    require(owner["server_extra_options"] == ["-RenderOffScreen", "-nosound"]
        and owner["collector_wall_timeout_sec"] == 900 and owner["finish_before_utc"] == prospective["finish_before_utc"], "capture timing/render options changed")
    require(owner["collector_argv"] == [str(root.resolve() / "episode"), owner["route_path"], "--host", "127.0.0.1", "--port", str(owner["port"]), *prospective["capture_flags"]], "exact prospective capture argv differs")
    projected = {**owner, "collector_argv": [value for value in owner["collector_argv"] if value != "--wall-timing"]}
    v4.validate_cli(projected, root)
    pid = integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and started["port"] == owner["port"], "owned process identity mismatch")
    log, times = pilot.checked_bytes(root, "server.log", ledger), []
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
            and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid and evidence.get("generation_id") == f"expert_{pid}"
            and evidence.get("host") == owner["host"] and evidence.get("port") == owner["port"] and evidence.get("expected_map") == "Town07"
            and evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "lifecycle scope/status mismatch")
        require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
            else evidence.get("active_map_basename") == "Town07", "owned release/map evidence mismatch")
        prefix = evidence["server_log"]
        require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
            and hashlib.sha256(log[:prefix["size_bytes"]]).hexdigest() == prefix["sha256"], "lifecycle log prefix mismatch")
        times.append(v4.utc(evidence["checked_at"]))
    require(v4.utc(prospective["declared_at_utc"]) <= v4.utc(owner["planned_at_utc"]) < times[0]
        <= v4.utc(started["started_at_utc"]) < times[1] < times[2] <= v4.utc(result["completed_at_utc"])
        <= v4.utc(prospective["finish_before_utc"]), "prospective/owned lifecycle timing mismatch")
    return owner, result, started, times


def validate_recorded_summary(summary, independent, declared, *, journal_sha, recovery_sha=None, capture_succeeded):
    """HH_260906 - Compare journal-derived metrics while explicitly excluding unjournaled persistence latency from independent proof."""
    for key, value in independent["metrics"].items():
        require(key in summary and equal(summary[key], value), "recorded timing summary differs from raw journal: " + key)
    require(summary.get("schema") == "carla.expert_wall_timing.v1" and summary.get("timing_enabled") is True
        and summary.get("capture_succeeded") is capture_succeeded, "timing summary identity/capture outcome mismatch")
    require(summary.get("journal_sha256") == declared.get("journal_sha256") == journal_sha, "original timing journal SHA mismatch")
    exact = summary.get("journal_exactly_matches_memory")
    require(type(exact) is bool and declared.get("journal_exactly_matches_memory") is exact, "journal/memory identity claim differs")
    require(summary.get("memory_records_sha256") == (journal_sha if exact else recovery_sha), "recovery/memory digest mismatch")
    require((recovery_sha is None) if exact else summary.get("recovery_sha256") == recovery_sha and recovery_sha is not None,
        "unexpected, missing or unbound recovery journal")
    require(declared.get("dataset_admission") is False and declared.get("attempt_count") == independent["metrics"]["attempt_count"], "timing admission or denominator mismatch")
    for key in ("native_wall_observation_hz", "camera_bundle_wall_completion_hz", "simulation_seconds_per_wall_second"):
        require(equal(declared.get(key), independent["metrics"][key]), "manifest throughput differs from raw journal")
    persistence = summary.get("persistence")
    require(isinstance(persistence, dict) and set(persistence) == set(distribution([])), "invalid persistence aggregate schema")
    count = integer(persistence["count"])
    require(count <= independent["metrics"]["attempt_count"], "excess recorded persistence samples")
    for key in set(persistence) - {"count"}:
        require((persistence[key] is None) if count == 0 else number(persistence[key]) >= 0, "invalid persistence aggregate value")
    errors = summary.get("persistence_errors")
    require(isinstance(errors, list), "persistence error ledger missing")
    for error in errors:
        require(set(error) == {"sequence", "operation", "error_type"} and 1 <= integer(error["sequence"]) <= independent["metrics"]["attempt_count"]
            and error["operation"] in ("persist", "persist_clock") and isinstance(error["error_type"], str), "invalid persistence error")
    expected_flags = {
        "nonempty_attempts": independent["flags"]["nonempty_attempts"], "all_attempts_complete": independent["flags"]["every_attempt_complete"],
        "state_ledger_exact": independent["flags"]["exact_state_camera_phase_coverage"] and independent["flags"]["unchanged_frame_timestamp_binding"],
        "camera_ledger_exact": independent["flags"]["exact_state_camera_phase_coverage"] and independent["flags"]["unchanged_frame_timestamp_binding"],
        "phase_ledger_exact": independent["flags"]["exact_state_camera_phase_coverage"] and independent["flags"]["unchanged_frame_timestamp_binding"],
        "camera_schedule_exact": independent["flags"]["camera_schedule_exact"],
        "consecutive_attempt_indices": independent["flags"]["consecutive_attempts_and_frames"],
        "ordered_phase_progression": independent["flags"]["consecutive_attempts_and_frames"],
        "no_tick_overlap_or_clock_reversal": independent["flags"]["no_overlap_or_clock_reversal"], "no_persistence_errors": not errors,
        "observed_native_frame_chain": independent["flags"]["consecutive_attempts_and_frames"],
        "native_sim_cadence_matches_declared": independent["flags"]["native_sim_cadence"],
        "camera_sim_cadence_matches_declared": independent["flags"]["camera_sim_cadence"]}
    require(isinstance(summary.get("flags"), dict) and set(summary["flags"]) == set(expected_flags)
        and all(type(value) is bool for value in summary["flags"].values()), "recorded timing flag schema differs")
    # HH_260906 - A failed journal is retained as a failed diagnostic; a recovered memory ledger never upgrades that outcome.
    if independent["status"] == "PASS":
        require(summary["flags"] == expected_flags, "recorded complete timing flags contradict independent checks")
    status = "COMPLETE_DIAGNOSTIC" if capture_succeeded and all(summary["flags"].values()) and exact else "PARTIAL_OR_FAILED_DIAGNOSTIC"
    require(summary.get("status") == declared.get("status") == status, "diagnostic status contradicts retained failure flags")
    scope = {"all_attempts_retained": True, "sensor_timestamps_modified": False, "gui_display_fps_measured": False,
        "learned_inference_measured": False, "hardware_load_measured": False, "dataset_admission": False,
        "bootstrap_and_setup_teardown_in_native_tick_totals": False}
    require(v4.strict_equal(summary.get("scope"), scope), "recorded timing scope overclaims measurement")
    return {"status": "PASS" if independent["status"] == "PASS" and status == "COMPLETE_DIAGNOSTIC" else "FAIL",
        "journal_exactly_matches_memory": exact, "recovery_used": not exact,
        "persistence_recorded_aggregate_only": persistence, "persistence_errors": errors,
        "persistence_durations_independently_reconstructed": False}


def audit_trial(root, quality, prospective):
    root, ledger = Path(root).resolve(), {}
    require((root / "owner_result.json").is_file(), "INCOMPLETE: owned capture result missing")
    owner, outcome, started, times = verify_sources(root, quality, prospective, ledger)
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        pilot.checked_bytes(root, entry["path"], ledger)
    directories = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(directories) == 1, "INCOMPLETE: exactly one retained full/partial episode required")
    prefix = directories[0] + "/"
    read = lambda name, lines=False: pilot.read_json(root, prefix + name, ledger, lines=lines)
    manifest, states, cameras = read("manifest.json"), read("states.jsonl", True), read("camera_frames.jsonl", True)
    require(v4.utc(started["started_at_utc"]) <= v4.utc(manifest["created_at"]) <= times[1], "capture created outside owned interval")
    require(manifest["runtime"].get("town", "").split("/")[-1] == "Town07" and manifest["runtime"].get("weather") == "ClearNoon"
        and manifest["runtime"].get("vehicle_type") == "vehicle.toyota.prius" and type(manifest["capture_contract"].get("seed")) is int
        and manifest["capture_contract"]["seed"] == 0, "capture map/weather/vehicle/seed differs from plan")
    for name in ("route_sha256", "mapping_sha256", "calibration_sha256"):
        require(manifest["provenance"].get(name) == prospective[name], "capture route or rig digest differs from plan")
    for field, source in (("collector_sha256", "scripts/e2e/collect_carla_vad_expert.py"),
        ("goal_stop_helper_sha256", "scripts/e2e/carla_goal_stop_profile.py")):
        require(manifest["provenance"].get(field) == prospective["source_hashes"][source], "manifest executed source differs from archive")
    transport = v4.validate_profile(manifest)
    expected = {"schema": "carla.expert_wall_timing.v1", "enabled": True, "clock": "time.perf_counter_ns", "stages": list(STAGES),
        "per_camera_stages": ["camera_queue_wait", "jpeg_encode_write"], "journal": "wall_timing.jsonl", "all_attempts_retained": True,
        "sensor_timestamps_changed": False, "gui_display_fps_measured": False, "learned_inference_measured": False,
        "bootstrap_setup_teardown_in_native_tick_totals": False, "journal_write_outside_own_tick_total": True, "journal_errors_can_approve_data": False}
    require(v4.strict_equal(manifest["capture_contract"].get("wall_timing"), expected), "timing measurement contract changed")
    require(manifest["provenance"].get("wall_timing_helper_sha256") == prospective["source_hashes"]["scripts/e2e/carla_wall_timing.py"]
        and manifest["files"].get("wall_timing") == "wall_timing.jsonl", "timing helper or journal provenance mismatch")
    receipts = read("control_receipts.jsonl", True)
    measured = ack.analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    camera_alignment = ack.analyze_camera_alignment(states, cameras)
    declared_transport = manifest["result"]["control_transport"]
    require(declared_transport.get("receipt_journal_sha256") == ledger[prefix + "control_receipts.jsonl"]["sha256"]
        and declared_transport.get("receipt_journal_error") is None and declared_transport.get("physical_actuation_proven") is False, "receipt finalization mismatch")
    for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
        ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
        require(type(declared_transport.get(field)) is int and declared_transport[field] == measured[key], "receipt denominator mismatch")
    summary = read("wall_timing_summary.json")
    original = pilot.checked_bytes(root, prefix + "wall_timing.jsonl", ledger)
    declared = manifest["result"]["wall_timing"]
    require(declared.get("summary_sha256") == ledger[prefix + "wall_timing_summary.json"]["sha256"], "timing summary SHA mismatch")
    recovered = (root / prefix / "wall_timing_recovery.jsonl").exists()
    require(manifest["files"].get("wall_timing_summary") == "wall_timing_summary.json"
        and (manifest["files"].get("wall_timing_recovery") == "wall_timing_recovery.jsonl" if recovered else "wall_timing_recovery" not in manifest["files"]), "timing file manifest mismatch")
    rows = read("wall_timing_recovery.jsonl" if recovered else "wall_timing.jsonl", True)
    raw_rows = pilot.checked_bytes(root, prefix + ("wall_timing_recovery.jsonl" if recovered else "wall_timing.jsonl"), ledger)
    canonical = "".join(json.dumps(row, sort_keys=True, allow_nan=False) + "\n" for row in rows).encode()
    require(raw_rows == canonical and hashlib.sha256(canonical).hexdigest() == summary.get("memory_records_sha256"), "timing journal not exact canonical memory ledger")
    timing = analyze_timing(rows, states, cameras)
    persisted = validate_recorded_summary(summary, timing, declared, journal_sha=hashlib.sha256(original).hexdigest(),
        recovery_sha=hashlib.sha256(raw_rows).hexdigest() if recovered else None, capture_succeeded=manifest["status"] == "complete")
    if rows:
        require(v4.utc(manifest["created_at"]) <= v4.utc(rows[0]["start_utc"]) <= v4.utc(rows[-1]["end_utc"]) <= times[1], "journal UTC outside capture lifecycle")
    protocol = v4.analyze_protocol(states, timeline["native_states"])
    images = []
    for camera in cameras:
        for name in CAMERAS:
            relative = prefix + camera["images"][name]
            raw = pilot.checked_bytes(root, relative, {})
            images.append({"path": relative, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    require(len({item["path"] for item in images}) == len(images), "duplicate camera image path")
    images.sort(key=lambda item: item["path"])
    result.update({"quality": quality, "status": "AUDITED_NOT_ADMITTED", "timing_protocol": timing, "timing_persistence": persisted,
        "transport_protocol": measured, "camera_alignment": camera_alignment, "pilot_protocol": protocol,
        "source_head_commit": owner["source_head_commit"], "reviewed_execution_commit": COMMIT,
        "all_eleven_archives_match_owner_and_reviewed_commits": True, "archived_source_sha256": owner["source_sha256"],
        "comparison_configuration": ack.comparison_configuration(manifest), "goal_stop_profile": manifest["capture_contract"]["goal_stop_profile"],
        "owner_planned_at_utc": owner["planned_at_utc"], "owner_completed_at_utc": outcome["completed_at_utc"],
        "timing_summary_sha256": ledger[prefix + "wall_timing_summary.json"]["sha256"], "timing_journal_sha256": hashlib.sha256(original).hexdigest(),
        "image_byte_integrity": {"file_count": len(images), "size_bytes": sum(item["size_bytes"] for item in images)},
        "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())],
        "dataset_admission": False, "learned_inference_measured": False, "gui_display_fps_measured": False})
    recheck(root, result["source_manifest"] + images)
    return result, images, rows


def recheck(root, entries):
    for entry in entries:
        raw = pilot.checked_bytes(root, entry["path"], {})
        require(hashlib.sha256(raw).hexdigest() == entry["sha256"] and len(raw) == entry["size_bytes"], "audit input changed during analysis")


def audit_campaign(campaign):
    """HH_260906 - Retain the full two-arm denominator; a missing or unfinished arm is INCOMPLETE, never PASS."""
    campaign, ledger = Path(campaign).resolve(), {}
    prospective = pilot.read_json(campaign, "pilot_plan.json", ledger)
    validate_plan(prospective)
    results, audits = [], []
    for arm in prospective["ordered_arms"]:
        parent = campaign / arm["output"].split("/")[0]
        names = sorted(p.name for p in parent.iterdir() if p.name.startswith("run_")) if parent.exists() else []
        require(names in ([], ["run_001"]), "excess or noncontiguous timing attempts")
        root = campaign / arm["output"]
        if not (root / "owner_result.json").is_file():
            results.append({"quality": arm["quality"], "status": "INCOMPLETE", "dataset_admission": False})
            continue
        audited = audit_trial(root, arm["quality"], prospective)
        results.append(audited[0]); audits.append((root, audited))
    complete = len(audits) == 2
    review = None
    if complete:
        low, epic = results
        require(v4.strict_equal(low["comparison_configuration"], epic["comparison_configuration"])
            and v4.strict_equal(low["goal_stop_profile"], epic["goal_stop_profile"]), "between-arm control/world/rig configuration changed")
        review = pilot.read_json(campaign, "epic_preflight_review.json", ledger)
        require(v4.utc(low["owner_completed_at_utc"]) < v4.utc(review["reviewed_at_utc"]) < v4.utc(epic["owner_planned_at_utc"]), "Epic lacks an intervening Low review")
        require(review.get("source_commit") == COMMIT and review.get("all_eleven_sources_match_reviewed_and_owner_commits") is True
            and review.get("completed_low_at_utc") == low["owner_completed_at_utc"] and review.get("training_data_approved") is False
            and review.get("full_independent_timing_audit_complete") is False and review.get("next_arm") == "epic/run_001"
            and type(review.get("next_attempt_count")) is int and review["next_attempt_count"] == 1
            and review.get("journal_sha256") == low["timing_journal_sha256"] and review.get("timing_summary_sha256") == low["timing_summary_sha256"], "Low review source/outcome binding differs")
        for field, value in (("state_count", low["timing_protocol"]["metrics"]["attempt_count"]),
            ("camera_anchor_count", low["image_byte_integrity"]["file_count"] // 6), ("ack_receipt_count", low["transport_protocol"]["receipt_count"]),
            ("scalar_flags", {key: value for key, value in low["independent_qa"]["flags"].items() if key not in
                {"manifest_state_and_camera_counts_match", "manifest_events_match", "capture_cleanup_completed", "collector_goal_claim_matches_measured_goal"}}),
            ("protocol_flags", low["pilot_protocol"]["flags"]),
            ("ack_flags", low["transport_protocol"]["flags"])):
            require(v4.strict_equal(review.get(field), value), "Low review differs from independent reconstruction: " + field)
    report = {"schema": "portable_e2e.wall_timing_quality_audit.v1", "status": "AUDITED_NOT_ADMITTED" if complete else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "reviewed_execution_commit": COMMIT,
        "expected_attempts": 2, "finalized_attempts": len(audits), "all_attempts_retained": True,
        "prospective_plan": prospective, "epic_preflight_review": review, "trials": results,
        "same_configuration_except_quality": complete, "prospective_historical_file_creation_proven": False,
        "training_data_approved": False, "gui_display_fps_measured": False, "learned_inference_measured": False,
        "persistence_durations_independently_reconstructed": False,
        "interpretation": "One ordered Low/Epic pair on one fixed route, not a repeated randomized performance study. Native and camera simulation cadence are distinct from wall throughput. Camera waits are sequential residual queue waits, not sensor generation latency. Tick totals exclude their own journal writes; wall observation throughput includes intervening work. Only per-record next-command RPC is staged, not every bootstrap/boundary/cleanup RPC. Neither GUI display FPS, hardware load attribution nor learned inference was measured. Scalar/ACK/timing PASS is not visual, full future XY, dataset or vehicle admission.",
        "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())],
        "audit_source_sha256": {Path(path).resolve().relative_to(ROOT).as_posix(): base.sha(Path(path))
            for path in (__file__, ack.__file__, v4.__file__, pilot.__file__, base.__file__)}}
    recheck(campaign, report["source_manifest"])
    for root, (result, images, _) in audits:
        recheck(root, result["source_manifest"] + images)
    return report, audits


def render_timing(report, audits, output):
    """HH_260906 - Plot every raw native attempt and independently reconstructed stage aggregates; never smooth away stalls."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = ("#246c99", "#d87527")
    figure, axes = plt.subplots(2, 2, figsize=(13, 7.6))
    for index, (_, (result, _, rows)) in enumerate(audits):
        axis = axes[0, index]
        time = [(row["start_ns"] - rows[0]["start_ns"]) / 1e9 for row in rows]
        axis.plot(time, [row["total_ns"] / 1e6 for row in rows], color=colors[index], lw=.7)
        axis.axhline(50, color="#555555", ls="--", lw=.8, label="50 ms (20 Hz real-time budget)")
        axis.axhline(100, color="#888888", ls=":", lw=.8, label="100 ms")
        axis.set(xlabel="Wall seconds from first attempted tick", ylabel="Native tick duration (ms)",
            title=result["quality"] + " | warmup, driving and tail ticks", ylim=(0, max(row["total_ns"] for _, (_, _, records) in audits for row in records) / 1e6 * 1.08))
        axis.grid(alpha=.2); axis.legend(fontsize=8)
        metrics = result["timing_protocol"]["metrics"]
        values = [metrics["stages"][name]["mean_ms"] for name in STAGES]
        axes[1, 0].bar([i + (index - .5) * .36 for i in range(len(STAGES))], values, width=.36, color=colors[index], label=result["quality"])
        axes[1, 1].bar([index * 3, index * 3 + 1], [metrics["native_wall_observation_hz"], metrics["camera_bundle_wall_completion_hz"]], color=colors[index])
        for x, value in zip([index * 3, index * 3 + 1], [metrics["native_wall_observation_hz"], metrics["camera_bundle_wall_completion_hz"]]):
            axes[1, 1].text(x, value + 1.2, f"{value:.2f}", ha="center", fontsize=9)
    axes[1, 0].set_xticks(range(len(STAGES)), ["Tick +\nsnapshot", "Observation\n+ control", "6-camera\nqueue wait", "6-camera\nJPEG + disk", "Next-command\nACK RPC"])
    axes[1, 0].set(ylabel="Mean milliseconds per scheduled stage", title="Stage means: camera stages occur every second tick")
    axes[1, 0].legend(); axes[1, 0].grid(axis="y", alpha=.2)
    axes[1, 1].set_xticks([0, 1, 3, 4], ["Low\nnative", "Low\n6-camera bundle", "Epic\nnative", "Epic\n6-camera bundle"])
    axes[1, 1].set(ylabel="Completions per wall second", title="Wall throughput, NOT simulation cadence or GUI FPS", ylim=(0, 95))
    axes[1, 1].grid(axis="y", alpha=.2)
    figure.suptitle("Town07 / V4 / one Low then one Epic trial: full wall timing ledger", fontsize=14)
    figure.text(.5, .014, "Simulation cadence remains 20 Hz native / 10 Hz cameras. One ordered pair; no randomized quality-effect inference. Persistence is not in raw tick totals.", ha="center", fontsize=8.5)
    figure.tight_layout(rect=(0, .035, 1, .95))
    figure.savefig(output, dpi=150)
    plt.close(figure)


def publish(campaign, visual_root, output):
    """HH_260906 - Publish new redacted evidence, exact actual-camera graphics and complete input hashes without modifying originals."""
    campaign, visual_root = Path(campaign).resolve(), Path(visual_root).resolve()
    output = v4.new_output(output, [campaign, visual_root])
    report, audits = audit_campaign(campaign)
    require(report["status"] != "INCOMPLETE", "INCOMPLETE: publication requires the full declared Low/Epic denominator")
    visuals = {root.parent.name: v4.checked_visual_payloads(visual_root / root.parent.name, root) for root, _ in audits}
    recheck(campaign, report["source_manifest"])
    for root, (result, images, _) in audits:
        recheck(root, result["source_manifest"] + images)
        recheck(visual_root / root.parent.name, visuals[root.parent.name][1]["source_manifest"])
    output.mkdir(parents=True, exist_ok=False)
    def write(name, value):
        with (output / name).open("x", encoding="utf-8") as stream:
            stream.write(value)
    write("summary.json", json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for root, (_, images, _) in audits:
        write(root.parent.name + "_image_hashes.json", json.dumps(images, indent=2, allow_nan=False) + "\n")
    render_timing(report, audits, output / "01_wall_timing_all_attempts.png")
    visual_proof = {}
    for name, (payloads, proof) in visuals.items():
        (output / name).mkdir()
        for relative, raw in payloads.items():
            with (output / name / relative).open("xb") as stream:
                stream.write(raw)
        visual_proof[name] = proof
    rows = []
    for result in report["trials"]:
        metrics, qa = result["timing_protocol"]["metrics"], result["independent_qa"]
        rows.append(f"| {result['quality']} | {metrics['attempt_count']} / {result['image_byte_integrity']['file_count']//6} | "
            f"{metrics['native_wall_observation_hz']:.3f} / {metrics['camera_bundle_wall_completion_hz']:.3f} | "
            f"{metrics['simulation_seconds_per_wall_second']:.3f}× | {metrics['total_tick']['p99_ms']:.3f} / {metrics['total_tick']['maximum_ms']:.3f} | "
            f"{metrics['tick_duration_above_ms_counts']['50']} / {metrics['tick_duration_above_ms_counts']['100']} | "
            f"{qa['maximum_measured_speed_kmh']:.3f} | {qa['final_driving']['goal_error_m']:.3f} |")
    write("README.md", "<!-- HH_260906 - Separate actual simulation cadence, wall throughput, visual observations and unapproved training data. -->\n"
        "# Low/Epic 화면 품질과 실제 처리 시간: Town07 직진 각 1회\n\n"
        "동일 V4 정상 브레이크 0·ACK 제어·6카메라·Town07 직진에서 렌더링 품질만 Low→Epic으로 변경했습니다. "
        "사전에 각 1회만 선언했고 Low 검토 후 Epic을 실행했습니다. **두 실행 모두 종료·전체 속도변화/정지 QA·제어 응답·시간 기록 검증을 통과했지만, 학습 데이터나 실제 차량 승인은 아닙니다.**\n\n"
        "| 품질 | native / 카메라 묶음 수 | 벽시계 native / 카메라 묶음 Hz | 시뮬레이션 진행 배율 | tick p99 / 최대 ms | >50 / >100 ms 횟수 | 최고 km/h | 마지막 주행 목표 오차 m |\n"
        "|---|---:|---:|---:|---:|---:|---:|---:|\n" + "\n".join(rows) + "\n\n"
        "![모든 native tick과 단계별 실제 지연](01_wall_timing_all_attempts.png)\n\n"
        "## 끊김에 대해 이번 시험에서 확인한 범위\n\n"
        "시뮬레이션 시간 기준 native 20 Hz·카메라 10 Hz는 둘 다 누락 없이 유지되었습니다. 벽시계 처리량은 별개이며, 이번에는 Epic이 더 빨랐습니다. "
        "각각 한 번이고 실행 순서가 고정되어 있어 캐시·시스템 변동·순서 효과를 분리하지 못합니다. Epic이 항상 빠르다는 결론은 아닙니다. "
        "각 실행에 100 ms를 넘는 tick 1개가 있었고 모두 그래프에 남겼습니다. GUI 화면 재생 FPS, Autoware, 학습 모델 추론 또는 CPU/GPU 부하는 측정하지 않았으므로 기존 화면 끊김 전체의 원인이 해결되었다고 주장하지 않습니다.\n\n"
        "카메라 대기 시간은 순서대로 읽을 때 남아 있던 큐 대기이며 센서 생성 지연이 아닙니다. RPC 단계는 각 기록 뒤의 다음 제어 명령만 포함합니다. "
        "부트스트랩·구간 경계·종료 정리 ACK는 별도 단계로 계측하지 않았습니다. tick 자체는 자기 journal 저장 시간을 제외하며, tick 사이 간격과 전체 관측 처리량은 중간 작업을 포함합니다. "
        "**journal 저장 지연은 원 실행 요약의 집계만 보존했고 개별 저장 구간이 없으므로 독립 재계산한 값이 아닙니다.**\n\n"
        "## 실제 6카메라·차량 중심 경로\n\n"
        "Low와 Epic 원본을 같은 전체 화각·1600×900 배치로 표시합니다. 확인한 순항 PNG에서 Low 노면은 체크무늬이고 Epic은 노면 텍스처가 보입니다. "
        "이 관찰만으로 전체 영상 품질을 승인하지는 않습니다. 경로 그림의 미래 궤적은 실제 나중에 기록된 차량 위치이며 모델 예측이 아닙니다.\n\n"
        "### Low\n\n![Low 실제 카메라와 경로](low/whole_recording_accelerated.gif)\n\n[순항 PNG](low/02_measured_cruise.png) · [정지 PNG](low/04_goal_dwell.png) · [영상 출처](low/visual_provenance.json)\n\n"
        "### Epic\n\n![Epic 실제 카메라와 경로](epic/whole_recording_accelerated.gif)\n\n[순항 PNG](epic/02_measured_cruise.png) · [정지 PNG](epic/04_goal_dwell.png) · [영상 출처](epic/visual_provenance.json)\n\n"
        "각 GIF는 전체 구간에서 카메라 5개 간격으로 147장 선택한 10 fps 가속 미리보기입니다. 위 벽시계 Hz 측정이나 실제 라이브 화면의 FPS와 같지 않습니다. "
        "차량은 경로 패널 중앙에 고정되며 전체 경로 미니맵도 함께 보입니다. Autoware/RViz 라이브 캡처·학습 모델 주행 영상이 아닙니다.\n\n"
        "## 근거와 재현\n\n"
        f"실행 소스 11개는 보관본·실행 HEAD·별도 검토 커밋 `{COMMIT}` 모두 바이트 일치를 확인했습니다. 현재 작업 파일이 바뀌어도 역사적 보관본으로 검증합니다. "
        "계획·Low 종료·중간 검토·Epic 시작의 기록된 시간을 검증했으며 불변 파일 생성시각의 증명으로 확대하지 않습니다. "
        "각 단계·6개 카메라·프레임·시뮬레이션 시각·위상·ACK·준비/주행/정지 꼬리 전부를 검사했습니다. 물리 ±2.9 m/s², runtime +3/−6, 실제 30 km/h 상한은 바꾸지 않았습니다. "
        "전체 future XY, 센서 의미, 영상 품질, 다중 경로 일반화와 학습 데이터 승인은 별도입니다.\n\n"
        "[독립 검증 결과](summary.json) · [원본/공개 출처](provenance.json) · [Low 모든 카메라 해시](low_image_hashes.json) · [Epic 모든 카메라 해시](epic_image_hashes.json) · [공개 SHA256](SHA256SUMS)\n\n"
        "재현: `python3 scripts/e2e/audit_carla_wall_timing_quality.py <wall_timing_quality_v1 원본> --visual-root <wall_timing_visual_v1> --output-dir <새 폴더>`\n"
        "원본 자료와 이미 존재하는 출력은 덮어쓰지 않습니다. 공개 JSON의 개인 경로는 가렸으며 원본 SHA는 유지했습니다.\n")
    proof = {"schema": "portable_e2e.wall_timing_quality_publication.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "raw_campaign_reference": "artifacts/training/2026-09-08/wall_timing_quality_v1",
        "reviewed_execution_commit": COMMIT, "audit_source_sha256": report["audit_source_sha256"],
        "renderer_source_sha256": {name: base.sha(ROOT / name) for name in ("scripts/e2e/render_carla_raw_trial.py", "scripts/e2e/render_carla_vad_expert.py")},
        "visuals": visual_proof, "source_metadata_redacted": True, "actual_rendered_pixels_copied_exactly": True,
        "original_inputs_modified": False, "new_fit_performed": False, "dataset_admission": False,
        "persistence_latency_independently_reconstructed": False, "binary_matches_source_proven": False}
    write("provenance.json", json.dumps(v4.sanitize(proof), indent=2, allow_nan=False) + "\n")
    recheck(campaign, report["source_manifest"])
    for root, (result, images, _) in audits:
        recheck(root, result["source_manifest"] + images)
        recheck(visual_root / root.parent.name, visuals[root.parent.name][1]["source_manifest"])
    files = sorted(path for path in output.rglob("*") if path.is_file())
    write("SHA256SUMS", "".join(base.sha(path) + "  " + path.relative_to(output).as_posix() + "\n" for path in files))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--visual-root", type=Path)
    args = parser.parse_args(argv)
    if args.visual_root is not None:
        report = publish(args.campaign, args.visual_root, args.output_dir)
        print(json.dumps({"status": report["status"], "finalized_attempts": report["finalized_attempts"], "dataset_admission": False}))
        return 0
    output = v4.new_output(args.output_dir, [args.campaign])
    report, audits = audit_campaign(args.campaign)
    output.mkdir(parents=True, exist_ok=False)
    (output / "audit.json").write_text(json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for root, (_, images, _) in audits:
        (output / (root.parent.name + "_image_hashes.json")).write_text(json.dumps(images, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "finalized_attempts": report["finalized_attempts"], "dataset_admission": False}))
    return 0 if report["status"] != "INCOMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
