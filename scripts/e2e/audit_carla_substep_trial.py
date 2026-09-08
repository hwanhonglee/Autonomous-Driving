#!/usr/bin/env python3
"""HH_260906 - Audit a new numerical-substep experiment without changing historical pins, labels or admission."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import re
import subprocess

from scripts.e2e import audit_carla_turn_launch_matrix as matrix
from scripts.e2e import audit_carla_turn_launch_raw_geometry as geometry
from scripts.e2e import audit_carla_raw_pre_admission as raw

ROOT = Path(__file__).resolve().parents[2]
base, pilot, ack, init, v4 = matrix.base, matrix.pilot, matrix.ack, matrix.initialization_audit, matrix.v4
require, equal, utc = base.require, matrix.strict_equal, matrix.utc
HELPER = "scripts/e2e/carla_physics_substeps.py"
SOURCES = matrix.SOURCES | {HELPER}
PROFILE = "turn_launch_013_v1"
SUBSTEPS = {"reference_10ms": .01, "fine_5ms": .005}
ORDER = (("A_reference_10ms/run_001", "reference_10ms"), ("B_fine_5ms/run_001", "fine_5ms"),
         ("A_reference_10ms/run_002", "reference_10ms"), ("B_fine_5ms/run_002", "fine_5ms"))
SCHEMA = "carla_expert.substep_ab_audit.v1"
PLAN_FIXED = {"schema": "carla_expert.substep_ab_campaign.v1", "campaign_id": "hh260909-c-track-substeps-ab-v1",
    "route_path": matrix.turn.PLAN_FIXED["route_path"], "route_sha256": matrix.ROUTE_SHA,
    "finish_before_utc": "2026-09-09T01:00:00Z", "port": 2100, "quality": "Epic", "wall_timeout_sec": 900,
    "training_data_approved": False, "development_only": True, "automatic_retry": False, "maximum_attempts": 4}


def digest(data):
    """HH_260906 - Hash original bytes rather than reformatted metadata."""
    return hashlib.sha256(data).hexdigest()


def read_json(root, relative, ledger, lines=False):
    """HH_260906 - Re-reading an already bound metadata file must not silently replace its earlier hash."""
    current = {}
    payload = pilot.checked_bytes(root, relative, current)
    require(relative not in ledger or ledger[relative] == current[relative], "metadata changed between analytical passes")
    ledger.update(current)
    text = payload.decode("utf-8")
    return [base._loads(line) for line in text.splitlines() if line.strip()] if lines else base._loads(text)


def git_bytes(commit, name):
    """HH_260906 - Resolve only the explicit twelve-source allowlist from existing offline Git objects."""
    require(isinstance(commit, str) and re.fullmatch(r"[a-f0-9]{40}", commit) and name in SOURCES,
            "unsafe substep execution source")
    result = subprocess.run(["git", "-c", "protocol.allow=never", "show", commit + ":" + name], cwd=ROOT,
        env=dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0"),
        capture_output=True, timeout=10, check=False)
    require(result.returncode == 0, "substep source unavailable in offline Git; no fetch attempted")
    return result.stdout


def validate_cli(owner, root, profile):
    """HH_260906 - Remove only the new explicit numerical option before checking the unchanged strict launch013 ABI."""
    require(profile in SUBSTEPS, "unknown numerical profile")
    argv = owner.get("collector_argv")
    require(isinstance(argv, list) and all(isinstance(x, str) for x in argv)
            and argv.count("--physics-substep-profile") == 1, "unique full substep option required")
    index = argv.index("--physics-substep-profile")
    require(index >= 2 and index + 1 < len(argv) and argv[index + 1] == profile, "substep CLI differs")
    unchanged = {**owner, "collector_argv": argv[:index] + argv[index + 2:]}
    require(matrix.validated_profile(unchanged) == PROFILE, "only frozen launch013 is in scope")
    matrix.validate_cli(unchanged, root)


def validate_plan(plan, expected_commit):
    """HH_260906 - Bind four declared trials and unchanged controls; extra descriptive text cannot alter the literal experiment."""
    require(isinstance(plan, dict) and all(equal(plan.get(k), v) for k, v in PLAN_FIXED.items()),
            "prospective substep campaign differs")
    require(plan.get("source_commit") == expected_commit, "unexpected reviewed source commit")
    require(utc(plan["declared_at_utc"]) < utc(plan["finish_before_utc"]), "declaration is after the deadline")
    pins = plan.get("source_sha256")
    require(isinstance(pins, dict) and set(pins) == SOURCES, "exact twelve prospective source pins required")
    for name, value in pins.items():
        require(isinstance(value, str) and re.fullmatch(r"[a-f0-9]{64}", value)
                and digest(git_bytes(expected_commit, name)) == value, "prospective source pin differs from reviewed Git")
    # HH_260906 - Numerical source and gate definitions must remain the existing physical model, not a permissive new decoder.
    for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py", "scripts/e2e/carla_goal_stop_profile.py"):
        require(git_bytes(expected_commit, name) == git_bytes(matrix.COMMIT, name), "substep experiment changed model, gates or governor")
    cases = plan.get("cases")
    require(isinstance(cases, list) and len(cases) == 4, "all four declared cases required")
    common = plan.get("collector_common_argv")
    require(isinstance(common, list) and all(isinstance(x, str) for x in common)
            and "--physics-substep-profile" not in common, "shared arguments contain a numerical override")
    for case, (name, profile) in zip(cases, ORDER):
        require(isinstance(case, dict) and case.get("id") == name and case.get("substep_profile") == profile,
                "substep case order or repeated-case quota differs")
        fake = {"port": 2100, "route_path": "route.json", "collector_argv": [str(ROOT / "episode"), "route.json",
                "--host", "127.0.0.1", "--port", "2100", *common, "--physics-substep-profile", profile]}
        validate_cli(fake, ROOT, profile)
    return plan


def source_identity():
    """HH_260906 - Record actual pure-analysis dependencies before and after the entire read-only pass."""
    expected_bounds = {"PHYSICAL_TIME_STEP_S": .1, "PHYSICAL_MAXIMUM_SPEED_MPS": 30 / 3.6,
        "PHYSICAL_MAXIMUM_ACCELERATION_MPS2": 2.9, "PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M": .2,
        "PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2": 2.8, "PHYSICAL_MAXIMUM_ROUTE_SLIP_RAD": math.radians(7.5),
        "ROUNDING_TOLERANCE": 1e-6, "HEADING_MINIMUM_STEP_M": .0001}
    require(all(equal(getattr(raw.targets, key), value) for key, value in expected_bounds.items()),
            "loaded raw-target diagnostic bounds changed")
    identity = raw.source_identity()
    modules = (matrix, geometry, base, pilot, ack, init, v4, matrix.turn)
    files = {Path(__file__).resolve(), *(Path(module.__file__).resolve() for module in modules)}
    identity["files"].update({p.relative_to(ROOT).as_posix(): base.sha(p) for p in files})
    functions = (matrix.analyze_protocol, matrix.validate_profile, init.analyze_initialization, ack.analyze_transport,
                 raw.camera_audit, raw.measured_timeline, geometry.geometry_futures, geometry.snapshot_diagnostic)
    identity["functions"].update({f"{f.__module__}.{f.__name__}": digest(inspect.getsource(f).encode()) for f in functions})
    return identity


def verify_owner(root, plan, case, ledger):
    """HH_260906 - Authenticate new source archives and the actual owned lifecycle without invoking old revision auditors."""
    read = lambda path: read_json(root, path, ledger)
    owner_plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    flags = ("source_bytes_archived", "bounds_source_bytes_archived", "wall_timing_enabled",
             "wall_timing_source_bytes_archived", "physics_substeps_source_bytes_archived")
    require(all(owner_plan.get(k) is True for k in flags), "twelve-source archive contract missing")
    require(owner_plan.get("source_sha256") == plan["source_sha256"] and set(owner.get("source_checks", {})) == SOURCES
            and all(x is True for x in owner["source_checks"].values())
            and owner.get("source_bytes_unchanged_and_archived") is True, "source postcheck or plan binding failed")
    require({p.relative_to(root / "provenance").as_posix() for p in (root / "provenance").rglob("*")
             if p.is_file() or p.is_symlink()} == SOURCES, "unexpected or missing archived source")
    for name, pin in plan["source_sha256"].items():
        payload = pilot.checked_bytes(root, "provenance/" + name, ledger)
        require(digest(payload) == pin and payload == git_bytes(plan["source_commit"], name)
                and payload == git_bytes(owner_plan["source_head_commit"], name), "archive/reviewed/owner source mismatch")
    require(owner_plan.get("physics_substep_profile") == case["substep_profile"]
            and owner_plan.get("physics_substeps_schema") == "carla.physics_substep_experiment.v1",
            "owner numerical option or schema differs")
    require(owner_plan.get("wall_timing_schema") == "carla.expert_wall_timing.v1"
            and owner_plan.get("host") == "127.0.0.1" and owner_plan.get("port") == plan["port"]
            and owner_plan.get("map") == started.get("map") == matrix.MAP
            and owner_plan.get("quality") == started.get("quality") == "Epic"
            and owner_plan.get("capture_mode") == owner.get("capture_mode") == "expert"
            and owner_plan.get("worker_path") == "scripts/e2e/collect_carla_vad_expert.py"
            and owner_plan.get("route_sha256") == matrix.ROUTE_SHA, "owned endpoint/map/input contract differs")
    require(all(item.get(k) is False for item in (owner_plan, owner) for k in ("learned_model_control", "vehicle_control_approved")),
            "experiment is no longer expert-only and denied")
    require(owner_plan.get("server_extra_options") == ["-RenderOffScreen", "-nosound"]
            and owner_plan.get("collector_wall_timeout_sec") == plan["wall_timeout_sec"]
            and owner_plan.get("finish_before_utc") == plan["finish_before_utc"], "render/deadline contract differs")
    require(Path(owner_plan["route_path"]).resolve() == (ROOT / plan["route_path"]).resolve(), "route path differs")
    expected = [str(root / "episode"), owner_plan["route_path"], "--host", "127.0.0.1", "--port", str(plan["port"]),
                *plan["collector_common_argv"], "--physics-substep-profile", case["substep_profile"]]
    require(owner_plan["collector_argv"] == expected, "actual full CLI differs from declaration")
    validate_cli(owner_plan, root, case["substep_profile"])
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and started["port"] == plan["port"], "owned process identity differs")
    log, times = pilot.checked_bytes(root, "server.log", ledger), []
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
                and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid
                and evidence.get("generation_id") == f"expert_{pid}" and evidence.get("host") == "127.0.0.1"
                and evidence.get("port") == plan["port"] and evidence.get("expected_map") == matrix.MAP
                and evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "lifecycle proof differs")
        require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                else evidence.get("active_map_basename") == matrix.MAP, "running map or stopped proof differs")
        prefix = evidence["server_log"]
        require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
                and digest(log[:prefix["size_bytes"]]) == prefix["sha256"], "lifecycle log prefix differs")
        times.append(utc(evidence["checked_at"]))
    require(utc(plan["declared_at_utc"]) <= utc(owner_plan["planned_at_utc"]) < times[0]
            <= utc(started["started_at_utc"]) < times[1] < times[2] <= utc(owner["completed_at_utc"])
            <= utc(plan["finish_before_utc"]), "prospective/owner/lifecycle time ordering differs")
    return owner_plan, owner, started, times


def audit_trial(root, plan, case):
    """HH_260906 - Keep native failures separate from transport, numerical-settings and every full-image/future diagnostic."""
    root, ledger = Path(root).resolve(), {}
    owner_plan, owner, started, times = verify_owner(root, plan, case, ledger)
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        payload = pilot.checked_bytes(root, entry["path"], {})
        require(digest(payload) == entry["sha256"] and len(payload) == entry["size_bytes"], "scalar source changed before second pass")
        require(entry["path"] not in ledger or ledger[entry["path"]] == {k: entry[k] for k in ("sha256", "size_bytes")},
                "owner and scalar source binding differs")
        ledger[entry["path"]] = {k: entry[k] for k in ("sha256", "size_bytes")}
    directories = [p for p in (root / "episode", root / "episode.partial") if p.exists()]
    require(len(directories) <= 1, "ambiguous original episode/partial payload")
    manifest, states, cameras, receipts, transport = None, [], [], [], {}
    images, futures, snapshots, pixels, future_report, snapshot_report = [], [], [], {}, {}, {}
    settings_report = {"status": "NOT_REACHED", "all_checks_pass": False}
    if directories:
        episode = directories[0]
        prefix = episode.name + "/"
        read = lambda name, lines=False: read_json(root, prefix + name, ledger, lines=lines)
        manifest, route, states, cameras = read("manifest.json"), read("route.json"), read("states.jsonl", True), read("camera_frames.jsonl", True)
        transport = matrix.validate_profile(manifest, route)
        require(manifest["capture_contract"]["goal_stop_profile"]["profile_id"] == PROFILE, "raw control profile differs")
        require(utc(started["started_at_utc"]) <= utc(manifest["created_at"]) <= times[1], "manifest timestamp outside owned execution")
        for key, value in (("route_sha256", matrix.ROUTE_SHA), ("mapping_sha256", matrix.turn.PLAN_FIXED["mapping_sha256"]),
                           ("calibration_sha256", matrix.turn.PLAN_FIXED["calibration_sha256"]),
                           ("physics_substeps_helper_sha256", plan["source_sha256"][HELPER])):
            require(manifest["provenance"].get(key) == value, "raw route/rig/helper provenance differs")
        settings_report = analyze_substeps(manifest["capture_contract"].get("physics_substeps"),
                                           manifest.get("runtime"), case["substep_profile"])
        if (episode / "control_receipts.jsonl").exists():
            receipts = read("control_receipts.jsonl", True)
        else:
            require(not states and owner["exit_code"] != 0, "measured capture lacks receipt journal")
        for name in ("wall_timing.jsonl", "wall_timing_summary.json", "wall_timing_recovery.jsonl"):
            if (episode / name).exists(): pilot.checked_bytes(root, prefix + name, ledger)
        if states:
            require(settings_report["all_checks_pass"], "native recording exists after failed numerical preflight")
            measured = raw.measured_timeline(states)
            pixels, image_rows, anchor_rows = raw.camera_audit(episode, manifest, cameras, measured)
            images = [{**row, "path": prefix + row["path"]} for row in image_rows]
            future_report, futures = geometry.geometry_futures(measured, cameras, route, anchor_rows, case["id"])
            snapshot_report = geometry.snapshot_diagnostic(states, manifest["coordinate_contract"]["wheelbase_m"])
            snapshots = snapshot_report.pop("observations")
        else:
            require(not cameras and owner["exit_code"] != 0, "empty native payload contradicts camera/success evidence")
            require(not any(p.is_file() or p.is_symlink() for p in (episode / "images").rglob("*")),
                    "empty native payload contains hidden image files")
    measured_ack = ack.analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    measured_ack["camera_alignment"] = ack.analyze_camera_alignment(states, cameras)
    measured_ack["flags"]["complete_camera_frame_alignment"] = measured_ack["camera_alignment"]["status"] == "PASS"
    measured_ack["failed_flags"] = [k for k, v in measured_ack["flags"].items() if not v]
    measured_ack["status"] = "FAIL" if measured_ack["failed_flags"] else "PASS"
    declared = manifest["result"].get("control_transport", {}) if manifest else {}
    if receipts or states or declared:
        require(declared.get("receipt_journal_sha256") == ledger[prefix + "control_receipts.jsonl"]["sha256"]
                and declared.get("receipt_journal_error") is None and declared.get("physical_actuation_proven") is False,
                "receipt finalization differs")
        for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
                           ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
            require(type(declared.get(field)) is int and declared[field] == measured_ack[key], "receipt denominator differs")
    protocol = matrix.analyze_protocol(states, timeline["native_states"] if timeline else [], PROFILE)
    initialization = init.analyze_initialization(manifest["capture_contract"].get("agent_initialization") if manifest else None,
                                               states, receipts, transport.get("bootstrap_observation"))
    derived = {str(offset): base.intervals(timeline["native_states"][offset::2], .1,
                manifest["capture_contract"]["goal_stop_profile"]["bounds"])[0] for offset in (0, 1)} if timeline else {}
    protocol_clear = (settings_report["all_checks_pass"] and measured_ack["status"] == "PASS"
                      and initialization["status"] == "PASS" and protocol["all_checks_pass"])
    result.update(status="AUDITED_NOT_ADMITTED", case_id=case["id"], substep_profile=case["substep_profile"],
        reviewed_execution_commit=plan["source_commit"], source_head_commit=owner_plan["source_head_commit"],
        all_twelve_archives_match_owner_and_reviewed_commits=True, archived_source_sha256=plan["source_sha256"],
        substep_settings=settings_report, transport_protocol=measured_ack, initialization_protocol=initialization,
        pilot_protocol=protocol, independent_protocol_clear=protocol_clear,
        alternate_10_hz_offsets_diagnostic_only=derived, camera_pixels=pixels, measured_future=future_report,
        snapshot_kinematics=snapshot_report, native_state_count=len(states), camera_anchor_count=len(cameras),
        training_data_approved=False, development_only=True, dataset_admission=False, full_future_xy_admission=False,
        labels_modified=False, common10_dataset_written=False, qualification_30_kph="NOT_CLAIMED",
        physical_actuation_proven=False, wall_timing_independently_reconstructed=False,
        source_manifest=[{"path": path, **info} for path, info in sorted(ledger.items())])
    result["continuation_observations"] = matrix.continuation_observations(result, states)
    streams = {"future_geometry_audit": futures, "native_snapshot_audit": snapshots, "jpeg_audit": images}
    result["diagnostic_stream_counts"] = {k: len(v) for k, v in streams.items()}
    recheck_trial(root, result, images)
    return result, images, {"numerical_timeline": timeline, "streams": streams}


def recheck_trial(root, report, images):
    """HH_260906 - Rehash exact source/image bytes after computation and reject newly hidden payloads."""
    for entry in report["source_manifest"] + images:
        require(base.sha(raw.contract._safe_file(root, entry["path"], "substep final hash")) == entry["sha256"],
                "source or image changed during numerical audit")
    if "bounds_source_proof" in report: base.recheck_bounds_source_archive(root, report["bounds_source_proof"])
    folders = [p for p in (root / "episode", root / "episode.partial") if p.exists()]
    if folders:
        require(len(folders) == 1, "episode directory changed during audit")
        found = {p.relative_to(root).as_posix() for p in (folders[0] / "images").rglob("*")
                 if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg")}
        require(found == {x["path"] for x in images}, "image inventory changed during audit")


def analyze_substeps(metadata, runtime, profile):
    """HH_260906 - Independently reconstruct all eleven settings; readback does not prove internal integrator frequency."""
    require(profile in SUBSTEPS, "unknown substep profile")
    fixed = {"schema": "carla.physics_substep_experiment.v1", "profile": profile, "explicit_opt_in": True,
        "numerical_only": True, "training_data_approved": False, "development_only": True,
        "comparison_absolute_tolerance": 1e-12,
        "required_original": {"substepping": True, "max_substep_delta_time": .01, "max_substeps": 10},
        "requested_max_substep_delta_time": SUBSTEPS[profile],
        "notice": "Maximum numerical substep setting, not measured internal substeps or improved physics accuracy; no dataset admission."}
    require(isinstance(metadata, dict) and all(equal(metadata.get(k), v) for k, v in fixed.items()),
            "missing or altered explicit substep contract")
    status = metadata.get("status")
    require(status in ("NOT_REACHED", "PASS", "FAIL") and set(metadata) == set(fixed) | {"status", "before", "requested", "after"}
            | ({"error"} if status == "FAIL" else set()), "unexpected substep stage fields")
    bools = {"synchronous_mode", "no_rendering_mode", "substepping", "deterministic_ragdolls", "spectator_as_ego"}
    floats = {"fixed_delta_seconds", "max_substep_delta_time", "max_culling_distance", "tile_stream_distance", "actor_active_distance"}
    snapshots = [metadata[k] for k in ("before", "requested", "after")]
    for value in snapshots:
        if value is None: continue
        require(isinstance(value, dict) and set(value) == bools | floats | {"max_substeps"}, "eleven settings required")
        require(all(type(value[k]) is bool for k in bools) and type(value["max_substeps"]) is int
                and 1 <= value["max_substeps"] <= 16, "invalid boolean or substep count")
        for key in floats:
            require((key == "fixed_delta_seconds" and value[key] is None)
                    or (type(value[key]) in (int, float) and math.isfinite(value[key]) and value[key] >= 0),
                    "nonfinite or invalid setting")
    flags = {"settings_stage_complete": status == "PASS", "reviewed_original": False,
             "requested_only_declared_changes": False, "all_eleven_readback_match": False,
             "runtime_summary_matches": False, "substep_capacity_20hz": False}
    before, requested, after = snapshots
    def settings_equal(a, b):
        return (isinstance(a, dict) and set(a) == set(b) and all(
            equal(a[k], v) if k in bools or k == "max_substeps" or v is None
            else type(a[k]) in (int, float) and abs(a[k] - v) <= 1e-12 for k, v in b.items()))
    if before is not None:
        flags["reviewed_original"] = settings_equal({k: before[k] for k in fixed["required_original"]}, fixed["required_original"])
        expected = dict(before, synchronous_mode=True, fixed_delta_seconds=.05, max_substep_delta_time=SUBSTEPS[profile])
        flags["requested_only_declared_changes"] = settings_equal(requested, expected)
        flags["all_eleven_readback_match"] = settings_equal(after, expected)
    if after is not None:
        flags["substep_capacity_20hz"] = (after["substepping"] is True and after["max_substeps"] == 10
            and after["fixed_delta_seconds"] == .05 and .05 <= after["max_substep_delta_time"] * after["max_substeps"] + 1e-12)
    if runtime is not None:
        require(runtime.get("town", "").rsplit("/", 1)[-1] == matrix.MAP and runtime.get("weather") == "ClearNoon"
                and runtime.get("vehicle_type") == "vehicle.toyota.prius", "runtime identity differs")
        small = {"synchronous_mode", "fixed_delta_seconds", "no_rendering_mode", "substepping", "max_substep_delta_time", "max_substeps"}
        flags["runtime_summary_matches"] = (before is not None and after is not None
            and settings_equal(runtime.get("original_world_settings"), {k: before[k] for k in small})
            and settings_equal(runtime.get("capture_world_settings"), {k: after[k] for k in small}))
    if status == "PASS": require(all(v for k, v in flags.items() if k != "runtime_summary_matches"), "falsely claimed substep PASS")
    elif status == "NOT_REACHED": require(all(v is None for v in snapshots), "unreached stage has unexplained snapshots")
    else: require(isinstance(metadata["error"], str) and bool(metadata["error"].strip()), "failed stage lacks original error")
    return {"status": status, "all_checks_pass": all(flags.values()), "flags": flags,
        "failed_flags": [k for k, v in flags.items() if not v], "original_metadata": metadata,
        "requested_maximum_delta_seconds": SUBSTEPS[profile], "internal_substep_count_measured": False,
        "vehicle_physics_parameters_independently_measured": False, "training_data_approved": False}


def validate_first_pair_review(campaign, plan, plan_sha, first_pair, next_owner_plan, ledger):
    """HH_260906 - A repeat pair needs an earlier explicit review of both original outcomes, never a best-case selection."""
    review = read_json(campaign, "reviews/first_pair.json", ledger)
    require(review.get("schema") == "carla_expert.substep_first_pair_review.v1"
            and review.get("plan_sha256") == plan_sha and review.get("proceed") is True
            and review.get("training_data_approved") is False and isinstance(review.get("rationale"), str)
            and bool(review["rationale"].strip()), "first-pair review does not authorize the unchanged repeat")
    require(set(review.get("first_pair_owner_sha256", {})) == {c[0] for c in ORDER[:2]}, "review must bind both first outcomes")
    for case_id, trial in first_pair:
        path = case_id + "/owner_result.json"
        owner = read_json(campaign, path, ledger)
        require(ledger[path]["sha256"] == review["first_pair_owner_sha256"][case_id]
                and utc(owner["completed_at_utc"]) < utc(review["reviewed_at_utc"]), "review predates or changes a first outcome")
        require(trial["independent_protocol_clear"]
                and not trial["continuation_observations"]["established_continuation_stop_condition"],
                "repeat may not bypass a first-pair safety/protocol failure")
    require(utc(review["reviewed_at_utc"]) < utc(next_owner_plan["planned_at_utc"]), "repeat started before required review")
    previous = review["interim_audit"]
    payload = pilot.checked_bytes(ROOT, previous["path"], {})
    require(digest(payload) == previous["sha256"], "reviewed interim audit changed")
    interim = base._loads(payload.decode("utf-8"))
    require(interim.get("schema") == SCHEMA and interim.get("prospective_plan_sha256") == plan_sha
            and interim.get("finalized_case_count") == 2 and len(interim.get("cases", [])) == 4
            and [c["status"] for c in interim["cases"]] == ["FINALIZED", "FINALIZED", "NOT_RUN", "NOT_RUN"],
            "review did not bind exactly the complete first pair and not-yet-run repeat pair")
    require(utc(interim["created_at_utc"]) < utc(review["reviewed_at_utc"])
            and interim.get("reviewed_execution_commit") == plan["source_commit"], "interim audit time/source differs")
    for old, (case_id, trial) in zip(interim["cases"][:2], first_pair):
        require(old["id"] == case_id and old.get("audit", {}).get("independent_protocol_clear") is True
                and old["audit"].get("archived_source_sha256") == trial["archived_source_sha256"],
                "interim does not bind the exact reviewed first-pair source")
        old_ledger = {r["path"]: r["sha256"] for r in old["audit"].get("source_manifest", [])}
        require(old_ledger == {r["path"]: r["sha256"] for r in trial["source_manifest"]}, "interim raw source ledger differs")
    return review


def recheck_campaign(campaign, report):
    """HH_260906 - Freeze the declaration, review and performed/unperformed inventory through final output persistence."""
    for entry in report["source_manifest"]:
        require(base.sha(raw.contract._safe_file(campaign, entry["path"], "campaign recheck")) == entry["sha256"],
                "campaign/review bytes changed")
    review = report.get("first_pair_review")
    if review:
        source = raw.contract._safe_file(ROOT, review["interim_audit"]["path"], "interim audit recheck")
        require(base.sha(source) == review["interim_audit"]["sha256"], "interim audit changed during persistence")
    expected = {case["id"] for case in report["cases"] if case["status"] != "NOT_RUN"}
    found = {p.relative_to(campaign).as_posix() for p in campaign.glob("*/run_*")}
    require(found == expected, "performed/unperformed case inventory changed")
    for case in report["cases"]:
        root = campaign / case["id"]
        require(not root.is_symlink(), "case became a symlink")
        if case["status"] != "NOT_RUN":
            require((root / "owner_result.json").is_file() == (case["status"] != "INCOMPLETE"),
                    "case finalization changed during analysis or persistence")


def audit_campaign(campaign, *, expected_plan_sha256, expected_source_commit):
    """HH_260906 - Preserve all four denominators and failed/incomplete stages without overriding historical source validators."""
    campaign = Path(campaign).resolve()
    sources, ledger = source_identity(), {}
    plan = read_json(campaign, "plan.json", ledger)
    require(ledger["plan.json"]["sha256"] == expected_plan_sha256, "prospective plan bytes differ")
    validate_plan(plan, expected_source_commit)
    require(all(sources["files"].get(name) == plan["source_sha256"][name]
                for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py")),
            "live numerical-analysis bounds source differs from executed archives")
    expected_paths = {item[0] for item in ORDER}
    found = {p.relative_to(campaign).as_posix() for arm in ("A_reference_10ms", "B_fine_5ms")
             for p in (campaign / arm).glob("run_*")}
    require(found <= expected_paths, "undeclared additional trial")
    cases, audits, first_pair, review, seen_unfinished, ordering_errors = [], [], [], None, False, []
    for index, case in enumerate(plan["cases"]):
        root = campaign / case["id"]
        require(not root.is_symlink(), "symlinked trial directory")
        if not root.exists():
            cases.append({**case, "status": "NOT_RUN", "training_data_approved": False})
            seen_unfinished = True
            continue
        require(root.is_dir(), "invalid trial directory")
        if seen_unfinished:
            ordering_errors.append({"case_id": case["id"], "error": "case exists after unrun/incomplete/unverified predecessor"})
        if not (root / "owner_result.json").exists():
            cases.append({**case, "status": "INCOMPLETE", "training_data_approved": False})
            seen_unfinished = True
            continue
        try:
            result = audit_trial(root, plan, case)
        except (base.EvidenceError, ValueError, TypeError, KeyError) as error:
            # HH_260906 - Do not omit a failed validator or pretend its uninterpreted native payload passed measurement checks.
            cases.append({**case, "status": "AUDIT_FAILED", "error_type": type(error).__name__, "error": str(error),
                "raw_payload_not_modified": True, "training_data_approved": False, "independent_protocol_clear": False})
            seen_unfinished = True
            continue
        trial = result[0]
        owner_plan = read_json(campaign, case["id"] + "/owner_plan.json", ledger)
        owner = read_json(campaign, case["id"] + "/owner_result.json", ledger)
        if index and (campaign / cases[index - 1]["id"] / "owner_result.json").is_file():
            previous_owner = read_json(campaign, cases[index - 1]["id"] + "/owner_result.json", ledger)
            require(utc(previous_owner["completed_at_utc"]) < utc(owner_plan["planned_at_utc"]), "trials overlap or reorder")
        if index == 2 and len(first_pair) == 2:
            review = validate_first_pair_review(campaign, plan, expected_plan_sha256, first_pair, owner_plan, ledger)
        if index < 2: first_pair.append((case["id"], trial))
        cases.append({**case, "status": "FINALIZED", "audit": trial, "training_data_approved": False})
        audits.append((root, result))
    for root, (trial, images, _) in audits: recheck_trial(root, trial, images)
    for path, info in ledger.items(): require(base.sha(campaign / path) == info["sha256"], "campaign/review bytes changed")
    if review:
        require(base.sha(ROOT / review["interim_audit"]["path"]) == review["interim_audit"]["sha256"], "interim audit changed during review")
    require(source_identity() == sources, "auditor dependency changed during analysis")
    finalized = sum(c["status"] == "FINALIZED" for c in cases)
    report = {"schema": SCHEMA, "status": "AUDITED_NOT_ADMITTED" if finalized == 4 and not ordering_errors else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "prospective_plan_sha256": expected_plan_sha256,
        "reviewed_execution_commit": expected_source_commit, "prospective_plan": plan, "planned_case_count": 4,
        "finalized_case_count": finalized, "all_planned_cases_retained": True, "cases": cases, "first_pair_review": review,
        "ordering_errors": ordering_errors,
        "source_identity": sources, "source_manifest": [{"path": path, **info} for path, info in sorted(ledger.items())],
        "training_data_approved": False, "development_only": True, "dataset_admission": False,
        "automatic_winner_selection": False, "same_initial_repetitions_are_independent_routes": False,
        "limits": "Settings readback is not a measured internal substep count. Scalar QA, discrete raw XY and image integrity remain separate. No filtering, smoothing, gate changes, labels, model inference, training or Common10 conversion."}
    recheck_campaign(campaign, report)
    return report, audits


def run(campaign, output, *, expected_plan_sha256, expected_source_commit):
    """HH_260906 - Create one new diagnostic folder outside raw inputs and physical dataset roots."""
    campaign, output = Path(campaign).resolve(), Path(output)
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(campaign)
            and not output.resolve().is_relative_to((ROOT / "datasets").resolve()), "fresh output must be outside inputs/datasets")
    report, audits = audit_campaign(campaign, expected_plan_sha256=expected_plan_sha256, expected_source_commit=expected_source_commit)
    output.mkdir(parents=True, exist_ok=False)
    with (output / "audit.json").open("x") as stream:
        stream.write(json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for kind in ("future_geometry_audit", "native_snapshot_audit", "jpeg_audit"):
        with (output / (kind + ".jsonl")).open("x") as stream:
            for _, (trial, _, data) in audits:
                for row in data["streams"][kind]:
                    stream.write(json.dumps({"case_id": trial["case_id"], **row}, separators=(",", ":"), allow_nan=False) + "\n")
    for root, (trial, images, _) in audits: recheck_trial(root, trial, images)
    recheck_campaign(campaign, report)
    require(source_identity() == report["source_identity"], "auditor source changed during persistence")
    # HH_260906 - Freeze payload paths before creating the checksum manifest; it cannot hash its own still-empty bytes.
    payload_paths = sorted(output.iterdir())
    with (output / "SHA256SUMS").open("x") as stream:
        stream.write("".join(f"{base.sha(path)}  {path.name}\n" for path in payload_paths))
    return report


def main(argv=None):
    """HH_260906 - Explicit immutable-plan inputs cannot act as source, physics or admission override switches."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--expected-plan-sha256", required=True)
    parser.add_argument("--expected-source-commit", required=True)
    args = parser.parse_args(argv)
    report = run(args.campaign, args.output_dir, expected_plan_sha256=args.expected_plan_sha256,
                 expected_source_commit=args.expected_source_commit)
    print(json.dumps({"status": report["status"], "finalized": report["finalized_case_count"], "training_data_approved": False}))
    return 0 if report["status"] == "AUDITED_NOT_ADMITTED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
