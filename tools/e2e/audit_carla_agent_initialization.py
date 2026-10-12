#!/usr/bin/env python3
"""HH_260906 - Audit a distinct after-bootstrap initialization revision without changing historical turn pins or admitting data."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

if __package__:
    from . import audit_carla_turn_low_goal_stop as turn
    from . import audit_carla_acknowledged_control as ack
    from . import audit_carla_brake_free_goal_stop as v4
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_turn_low_goal_stop as turn
    import audit_carla_acknowledged_control as ack
    import audit_carla_brake_free_goal_stop as v4
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "dfccaa8ed9f00295ebd2014f4191671b7fa98ddb"
SOURCES = turn.SOURCES
MAP, ROUTE_SHA = turn.MAP, turn.ROUTE_SHA
require, strict_equal, utc = base.require, v4.strict_equal, v4.utc
recorded_bytes = turn.recorded_bytes
REVIEWED_SOURCES = {
    "scripts/e2e/collect_carla_vad_expert.py": "29af74e7ec397e288f41339753e334a710dac7d38241d7efc0f46e821e82e276",
    "scripts/e2e/carla_goal_stop_profile.py": "7ef2846bf8261766e6b41117fb8d901051e2330e4aac771130e5203740c41ca8",
    "scripts/e2e/carla_wall_timing.py": "2037b681ec75bfa85f1e34a7510766d6cd06dee20b0478773732f7fe0fafbf0a",
}
# HH_260906 - Shared unchanged low-speed parameters are explicit; the new revision, CLI and initialization witnesses are independently bound.
PLAN_FIXED = {**turn.PLAN_FIXED, "schema": "portable_e2e.agent_initialization_plan.v1",
              "source_commit": COMMIT, "agent_initialization": "after_bootstrap"}
INIT_CONTRACT = {
    "schema": "carla.basic_agent_initialization.v1", "explicit_opt_in": True,
    "constructor_requires_verified_bootstrap": True, "existing_bootstrap_tick_reused": True,
    "additional_world_ticks": 0, "pid_history_overwritten": False, "observed_zero_steering_required": True,
    "first_proposed_steering_delta_limit": .1, "comparison_absolute_tolerance": 1e-6,
    "physical_cause_proven": False, "training_data_approved": False,
}
HISTORICAL_ROOT = "artifacts/training/2026-09-08/turn_low_goal_stop_v1"
HISTORICAL_SHA = {
    "pilot_plan.json": "b7fe0b238d018318a404f35a72feac5c2df7431706153926a3441855da936f79",
    "c_track_left/run_001/episode.partial/states.jsonl": "62b2d3d0a967a38810424071492d8643a5b66f3b38578531bdb446572ad3502f",
    "c_track_left/run_001/episode.partial/control_receipts.jsonl": "4a7c7c89f79bd436e0d593a24a453669a1b37be5f83d9e950c5ed883f3a52e00",
    "c_track_left/run_001/episode.partial/manifest.json": "8b990559cbf7777ca67aa0fa5680ec7992f0f2f301e5e7afab7f1de006d36d3d",
}


def validate_cli(owner, root):
    """HH_260906 - Remove only the explicit new option from a copy, then apply the unchanged strict turn CLI ABI."""
    argv = owner["collector_argv"]
    require(isinstance(argv, list) and argv.count("--agent-initialization") == 1,
            "explicit unique agent initialization option required")
    index = argv.index("--agent-initialization")
    require(index >= 2 and index + 1 < len(argv) and argv[index + 1] == "after_bootstrap",
            "after_bootstrap initialization required")
    turn.validate_cli({**owner, "collector_argv": argv[:index] + argv[index + 2:]}, root)


def analyze_initialization(metadata, states, receipts, bootstrap):
    """HH_260906 - Link frozen construction and engagement witnesses to bootstrap, raw states and the subsequent ACK observation."""
    flags = {name: False for name in ("explicit_contract", "construction_completed", "construction_frame_and_snapshot",
        "zero_pid_history", "engagement_completed", "exact_warmup_without_extra_ticks",
        "engagement_snapshot", "first_step_frame_and_delta", "proposed_to_sent_guard", "first_sent_receipt", "first_subsequent_observation")}
    failures, measurements = [], {}
    def check(name, valid, detail):
        flags[name] = bool(valid)
        if not valid:
            failures.append({"check": name, "detail": detail})
    if metadata is None:
        return {"status": "FAIL", "flags": flags, "failed_flags": list(flags), "failures": [{"detail": "No initialization witness"}],
                "measurements": {}, "training_data_approved": False, "physical_cause_proven": False}
    fixed = {**INIT_CONTRACT, "mode": "after_bootstrap", "required_control_transport": "acknowledged_batch"}
    require(isinstance(metadata, dict) and set(metadata) == set(fixed) | {"construction", "engagement"}
            and all(strict_equal(metadata[k], v) for k, v in fixed.items()), "initialization contract differs")
    flags["explicit_contract"] = True
    construction, engagement = metadata["construction"], metadata["engagement"]
    require(isinstance(construction, dict) and isinstance(engagement, dict), "initialization stages malformed")
    check("construction_completed", construction.get("status") == "PASS", "Construction did not complete")
    check("engagement_completed", engagement.get("status") == "PASS"
          and engagement.get("first_command_acknowledged") is True, "First command was not acknowledged")
    measurements.update(construction_status=construction.get("status"), engagement_status=engagement.get("status"),
        first_command_acknowledged=engagement.get("first_command_acknowledged", False))
    # HH_260906 - A failed stage remains an auditable failure, including failed/partial snapshots; it is never filled with invented success.
    initial = [r for r in receipts if r.get("reason") == "initial_drive_control"]
    bootstrap_receipts = [r for r in receipts if r.get("reason") == "pre_capture_bootstrap"]
    def witness(w, reference, actor_id, pid=False):
        require(isinstance(w, dict) and isinstance(reference, dict), "missing frame witness/reference")
        require(w.get("status") == "PASS", "frame witness was not accepted")
        frame = base.integer(reference["frame"])
        require(all(type(w.get(k)) is int and w[k] == frame for k in ("expected_frame", "frame_before", "frame_after")),
                "initialization witness frame differs")
        require(base.number(w["timestamp"]) == base.number(reference["timestamp"])
                and type(w.get("actor_id")) is int and w["actor_id"] == actor_id, "witness timestamp or actor differs")
        pose = w["actor_snapshot_transform_carla"]
        require(set(pose) == {"x", "y", "z", "roll", "pitch", "yaw"}
                and all(base.number(v) == base.number(reference["actor_snapshot_transform_carla"][k]) for k, v in pose.items()),
                "witness pose differs from immutable snapshot")
        control = ack.full_control(w["current_control"])
        require(strict_equal(control, ack.full_control(reference["current_control"])), "witness cached control differs")
        require(control["throttle"] == control["steer"] == 0 and control["brake"] == 1
                and all(control[k] is False for k in ("hand_brake", "reverse", "manual_gear_shift")),
                "constructor/engagement was not at verified zero-steering full brake")
        require(w.get("raw_state_reference") == "CARLA actor snapshot API reference point; physical COM identity is unverified.",
                "initialization reference-point caveat differs")
        if pid:
            require(base.number(w["pid_past_steering"]) == 0, "PID history is not observed zero")
    if flags["construction_completed"]:
        try:
            require(len(bootstrap_receipts) == 1 and isinstance(bootstrap, dict), "unique measured bootstrap missing")
            br = bootstrap_receipts[0]
            actor_id = base.integer(br["actor_id"])
            require(br["status"] == "ACKNOWLEDGED" and br["server_accepted"] is True
                    and br["response_actor_id"] == actor_id and br["do_tick"] is False
                    and br["before_frame"] == br["after_ack_frame"] == bootstrap["frame"] - 1,
                    "bootstrap receipt linkage differs")
            require(bootstrap["alignment"]["status"] == "PASS"
                    and bootstrap["alignment"]["expected_receipt_sequence"] == br["sequence"], "bootstrap observation was not verified")
            before, after = construction["before"], construction["after"]
            witness(before, bootstrap, actor_id)
            witness(after, bootstrap, actor_id, True)
            check("construction_frame_and_snapshot", True, "")
            check("zero_pid_history", True, "")
            measurements.update(construction_frame=bootstrap["frame"], constructor_pid_past_steering=after["pid_past_steering"])
        except (KeyError, TypeError, ValueError, base.EvidenceError) as error:
            failures.append({"check": "construction_frame_and_snapshot", "detail": str(error)})
    if flags["engagement_completed"]:
        try:
            warmup = [s for s in states if s.get("capture_phase") == "stationary_warmup"]
            driving = [s for s in states if s.get("capture_phase") == "driving"]
            require(len(warmup) == 70 and [s["frame"] for s in warmup] == list(range(bootstrap["frame"] + 1, bootstrap["frame"] + 71))
                    and states[:70] == warmup, "existing exact seventy warmup ticks not retained")
            check("exact_warmup_without_extra_ticks", True, "")
            last = warmup[-1]
            witness(engagement["before"], last, bootstrap_receipts[0]["actor_id"], True)
            check("engagement_snapshot", True, "")
            proposed = ack.full_control(engagement["first_proposed_control"])
            delta = base.number(proposed["steer"]) - base.number(engagement["before"]["pid_past_steering"])
            require(type(engagement["frame_after_agent_step"]) is int and engagement["frame_after_agent_step"] == last["frame"]
                    and base.number(engagement["first_proposed_steering_delta"]) == delta and abs(delta) <= .1 + 1e-6,
                    "first step frame or steering delta differs")
            check("first_step_frame_and_delta", True, "")
            sent = ack.full_control(engagement["first_sent_control"])
            expected_sent = dict(proposed)
            emergency = last["goal_stop"].get("next_control_is_unchanged_emergency_override") is True
            speed = math.sqrt(sum(base.number(v) ** 2 for v in last["world_velocity_carla"]))
            if not emergency and speed < .3 and (proposed["brake"] > .05 or proposed["hand_brake"]):
                expected_sent["steer"] = 0.
            require(strict_equal(sent, expected_sent), "proposed-to-sent change is not the unchanged stopped-steering guard")
            check("proposed_to_sent_guard", True, "")
            require(len(initial) == 1, "first command receipt missing or duplicated")
            receipt = initial[0]
            require(type(engagement["first_command_receipt_sequence"]) is int
                    and receipt["sequence"] == engagement["first_command_receipt_sequence"]
                    and receipt["status"] == "ACKNOWLEDGED" and receipt["server_accepted"] is True
                    and receipt["actor_id"] == receipt["response_actor_id"] == bootstrap_receipts[0]["actor_id"]
                    and receipt["expected_before_frame"] == receipt["before_frame"] == receipt["after_ack_frame"] == last["frame"]
                    and receipt["do_tick"] is False and receipt["physical_actuation_proven"] is False
                    and strict_equal(sent, ack.full_control(receipt["requested_control"]))
                    and strict_equal(sent, ack.full_control(last["next_control"]))
                    and last["control_transport"]["next_command_receipt_sequence"] == receipt["sequence"]
                    and last["goal_stop"]["next_control_starts_driving"] is True, "first sent command/receipt/native engagement differs")
            check("first_sent_receipt", True, "")
            require(driving and states[70] == driving[0] and driving[0]["frame"] == last["frame"] + 1
                    and driving[0]["control_transport"]["expected_receipt_sequence"] == receipt["sequence"]
                    and driving[0]["control_transport"]["status"] == "PASS"
                    and not ack.mismatched_controls(driving[0]["current_control"], sent), "first subsequent observed control is not receipt-bound")
            check("first_subsequent_observation", True, "")
            measurements.update(engagement_frame=last["frame"], engagement_pid_past_steering=engagement["before"]["pid_past_steering"],
                first_proposed_control=proposed, first_sent_control=sent, first_command_receipt_sequence=receipt["sequence"],
                first_observed_frame=driving[0]["frame"], first_observed_control=driving[0]["current_control"],
                first_proposed_steering_delta=delta)
        except (KeyError, TypeError, ValueError, base.EvidenceError) as error:
            failures.append({"check": "engagement_chain", "detail": str(error)})
    failed = [k for k, value in flags.items() if not value]
    return {"schema": "carla.basic_agent_initialization_independent_audit.v1", "status": "FAIL" if failed else "PASS",
        "flags": flags, "failed_flags": failed, "failures": failures, "measurements": measurements,
        "original_construction_witness": construction, "original_engagement_witness": engagement,
        "raw_state_count": len(states), "receipt_count": len(receipts), "all_observations_retained": True,
        "training_data_approved": False, "physical_cause_proven": False,
        "notice": "Frame-bound API observations and server acceptance, not physical pedal-application timing. Whole constructor timing context changes, including location reads; not a steering-history-only causal proof."}


def verify_owner_sources(root, ledger):
    """HH_260906 - Bind the exact new source revision and Epic C-track lifecycle, never the current worktree or V4 pins."""
    read = lambda name: pilot.read_json(root, name, ledger)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    require(plan.get("source_bytes_archived") is plan.get("bounds_source_bytes_archived") is True
            and plan.get("wall_timing_enabled") is plan.get("wall_timing_source_bytes_archived") is True
            and plan.get("wall_timing_schema") == "carla.expert_wall_timing.v1", "turn eleven-source archive contract missing")
    require(set(plan["source_sha256"]) == SOURCES and set(owner["source_checks"]) == SOURCES
            and owner.get("source_bytes_unchanged_and_archived") is True
            and all(v is True for v in owner["source_checks"].values()), "turn source postcheck failed")
    require({p.relative_to(root / "provenance").as_posix() for p in (root / "provenance").rglob("*")
             if p.is_file() or p.is_symlink()} == SOURCES, "missing or unexpected turn source archive")
    for name, expected in plan["source_sha256"].items():
        raw = pilot.checked_bytes(root, "provenance/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == expected and raw == recorded_bytes(COMMIT, name)
                and raw == recorded_bytes(plan["source_head_commit"], name), "turn reviewed/owner/archive source mismatch")
        if name in REVIEWED_SOURCES:
            require(expected == REVIEWED_SOURCES[name], "unreviewed turn source SHA")
    require(plan.get("host") == "127.0.0.1" and plan.get("map") == started.get("map") == MAP
            and plan.get("quality") == started.get("quality") == "Epic"
            and plan.get("capture_mode") == owner.get("capture_mode") == "expert"
            and plan.get("worker_path") == "scripts/e2e/collect_carla_vad_expert.py"
            and plan.get("route_sha256") == ROUTE_SHA, "turn endpoint/map/quality/input contract differs")
    require(all(item.get(k) is False for item in (plan, owner) for k in ("learned_model_control", "vehicle_control_approved")),
            "turn remains expert-only and not approved")
    require(plan.get("server_extra_options") == ["-RenderOffScreen", "-nosound"]
            and plan.get("collector_wall_timeout_sec") == 900 and plan.get("finish_before_utc") == "2026-09-08T01:00:00Z",
            "turn timing/render contract differs")
    validate_cli(plan, root)
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and started["port"] == plan["port"], "turn owned process identity mismatch")
    log, times = pilot.checked_bytes(root, "server.log", ledger), []
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
                and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid
                and evidence.get("generation_id") == f"expert_{pid}" and evidence.get("host") == plan["host"]
                and evidence.get("port") == plan["port"] and evidence.get("expected_map") == MAP
                and evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "turn lifecycle contract differs")
        require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                else evidence.get("active_map_basename") == MAP, "turn running map or stopped proof differs")
        prefix = evidence["server_log"]
        require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
                and hashlib.sha256(log[:prefix["size_bytes"]]).hexdigest() == prefix["sha256"], "turn lifecycle log prefix mismatch")
        times.append(utc(evidence["checked_at"]))
    require(utc(plan["planned_at_utc"]) < times[0] <= utc(started["started_at_utc"]) < times[1] < times[2]
            <= utc(owner["completed_at_utc"]) <= utc(plan["finish_before_utc"]), "turn lifecycle times invalid")
    return plan, owner, started, times


def audit_trial(root):
    """HH_260906 - Preserve failed, aborted and no-tail trials; protocol success does not waive raw scalar or future XY quality."""
    root, ledger = Path(root).resolve(), {}
    require((root / "owner_result.json").is_file(), "INCOMPLETE: turn owner result missing")
    plan, owner, started, times = verify_owner_sources(root, ledger)
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        pilot.checked_bytes(root, entry["path"], ledger)
    directories = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(directories) <= 1, "ambiguous turn final/partial evidence")
    states, cameras, receipts, images, transport, manifest = [], [], [], [], {}, None
    if directories:
        prefix = directories[0] + "/"
        read = lambda name, lines=False: pilot.read_json(root, prefix + name, ledger, lines=lines)
        manifest, route, states, cameras = read("manifest.json"), read("route.json"), read("states.jsonl", True), read("camera_frames.jsonl", True)
        transport = turn.validate_profile(manifest, route)
        require(utc(started["started_at_utc"]) <= utc(manifest["created_at"]) <= times[1], "turn manifest created outside owned interval")
        if "runtime" in manifest:
            require(manifest["runtime"]["town"].split("/")[-1] == MAP and manifest["runtime"]["weather"] == "ClearNoon"
                    and manifest["runtime"]["vehicle_type"] == "vehicle.toyota.prius", "turn measured runtime identity changed")
        for key, digest in {"route_sha256": ROUTE_SHA,
            "mapping_sha256": "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
            "calibration_sha256": "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"}.items():
            require(manifest["provenance"].get(key) == digest, "turn rig/route provenance differs")
        if (root / prefix / "control_receipts.jsonl").exists():
            receipts = read("control_receipts.jsonl", True)
        else:
            require(not states and owner["exit_code"] != 0, "observed turn capture has no receipt journal")
        # HH_260906 - Preserve timing evidence hashes here; its independent interval audit is a separate diagnostic.
        for name in ("wall_timing.jsonl", "wall_timing_summary.json", "wall_timing_recovery.jsonl"):
            if (root / prefix / name).exists():
                pilot.checked_bytes(root, prefix + name, ledger)
    measured = ack.analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    measured["camera_alignment"] = ack.analyze_camera_alignment(states, cameras)
    measured["flags"]["complete_camera_frame_alignment"] = measured["camera_alignment"]["status"] == "PASS"
    measured["failed_flags"] = [k for k, v in measured["flags"].items() if not v]
    measured["status"] = "FAIL" if measured["failed_flags"] else "PASS"
    declared = manifest["result"].get("control_transport", {}) if manifest else {}
    if receipts or states or declared:
        require(declared.get("receipt_journal_sha256") == ledger[prefix + "control_receipts.jsonl"]["sha256"]
                and declared.get("receipt_journal_error") is None and declared.get("physical_actuation_proven") is False,
                "turn receipt finalization differs")
        for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
            ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
            require(type(declared.get(field)) is int and declared[field] == measured[key], "turn receipt denominator differs")
    protocol = turn.analyze_protocol(states, timeline["native_states"]) if timeline else turn.analyze_protocol([], [])
    initialization = analyze_initialization(manifest["capture_contract"].get("agent_initialization") if manifest else None,
        states, receipts, transport.get("bootstrap_observation"))
    derived = {}
    if timeline:
        bounds = manifest["capture_contract"]["goal_stop_profile"]["bounds"]
        for offset in (0, 1):
            derived[str(offset)] = base.intervals(timeline["native_states"][offset::2], .1, bounds)[0]
    for camera in cameras:
        for name in base.CAMERAS:
            path = prefix + camera["images"][name]
            raw = pilot.checked_bytes(root, path, {})
            images.append({"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    require(len({x["path"] for x in images}) == len(images), "duplicate turn image references")
    images.sort(key=lambda x: x["path"])
    normal = [s for s in states if s["capture_phase"] == "driving" and not s["goal_stop"].get("termination_reason")
              and not s["goal_stop"].get("emergency_failure_pending_next_tick")]
    result.update({"status": "AUDITED_NOT_ADMITTED", "transport_protocol": measured, "pilot_protocol": protocol,
        "initialization_protocol": initialization, "alternate_10_hz_offsets_diagnostic_only": derived,
        "normal_driving_nonzero_brake_count": sum(s["next_control"]["brake"] != 0 for s in normal),
        "normal_driving_command_count": len(normal), "source_head_commit": plan["source_head_commit"],
        "reviewed_execution_commit": COMMIT, "all_eleven_archives_match_owner_and_reviewed_commits": True,
        "archived_source_sha256": plan["source_sha256"], "comparison_configuration": ack.comparison_configuration(manifest) if manifest and "runtime" in manifest else None,
        "image_byte_integrity": {"file_count": len(images), "total_size_bytes": sum(x["size_bytes"] for x in images),
            "canonical_sorted_file_ledger_sha256": hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "pixels_decoded_or_visually_approved": False}, "source_manifest": [{"path": n, **v} for n, v in sorted(ledger.items())],
        "qualification_30_kph": "NOT_CLAIMED", "low_speed_stability_band_mps": [3.8, 4.2],
        "training_data_approved": False, "dataset_admission": False, "full_future_xy_admission": False,
        "physical_actuation_proven": False, "failed_before_first_observation": not states,
        "wall_timing_independently_reconstructed": False, "campaign_preregistration_checked_by_trial_only": False})
    for entry in result["source_manifest"] + images:
        require(base.sha(root / entry["path"]) == entry["sha256"], "turn raw input changed during audit")
    if "bounds_source_proof" in result:
        base.recheck_bounds_source_archive(root, result["bounds_source_proof"])
    return result, images, timeline


def validate_plan_document(plan):
    """HH_260906 - Bind all declared settings, source files and historical lateral provenance before interpreting any result."""
    extras = {"comment", "declared_at_utc", "source_hashes", "capture_flags", "historical_lateral_configuration", "comparison_notice", "expected_risks",
        "historical_before_bootstrap_reference", "initialization_contract", "review_measurements", "after_first_trial"}
    require(isinstance(plan, dict) and set(plan) == set(PLAN_FIXED) | extras, "unknown or missing low-turn prospective field")
    require(all(strict_equal(plan[k], value) for k, value in PLAN_FIXED.items()), "low-turn prospective contract changed")
    require(all(isinstance(plan[k], str) and plan[k] for k in ("comment", "comparison_notice")), "prospective explanation missing")
    require(isinstance(plan["expected_risks"], list) and len(plan["expected_risks"]) == 3
            and all(isinstance(v, str) and v for v in plan["expected_risks"]), "prospective risk disclosure missing")
    require(utc(plan["declared_at_utc"]) < utc(plan["finish_before_utc"]), "prospective declaration after deadline")
    require(isinstance(plan["source_hashes"], dict) and set(plan["source_hashes"]) == SOURCES, "prospective exact eleven sources missing")
    for name, digest in plan["source_hashes"].items():
        require(hashlib.sha256(recorded_bytes(COMMIT, name)).hexdigest() == digest, "prospective source differs from frozen commit")
    require(isinstance(plan["capture_flags"], list), "prospective CLI flags missing")
    validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": [str(ROOT / "episode"), "route.json",
        "--host", "127.0.0.1", "--port", "2100", *plan["capture_flags"]]}, ROOT)
    # HH_260906 - The old train manifest is read only for lateral settings; no test payloads or old dataset bytes are changed.
    ledger = {}
    original = pilot.read_json(ROOT, plan["historical_lateral_baseline_path"], ledger)
    require(ledger[plan["historical_lateral_baseline_path"]]["sha256"] == plan["historical_lateral_baseline_sha256"], "historical lateral source SHA differs")
    control = original["capture_contract"]["basic_agent_control"]
    require(strict_equal(control, plan["historical_lateral_configuration"]), "historical lateral configuration differs from source")
    require(control["effective_opt_dict"]["base_min_distance"] == control["waypoint_purge_lookahead"]["base_min_distance_m"] == 2.
            and control["effective_opt_dict"]["distance_ratio"] == control["waypoint_purge_lookahead"]["distance_ratio_s"] == .2,
            "historical lateral configuration is not the declared compact C-track baseline")
    require(strict_equal(plan["initialization_contract"], INIT_CONTRACT), "prospective initialization contract changed")
    require(isinstance(plan["review_measurements"], list) and len(plan["review_measurements"]) == 5
            and all(isinstance(v, str) and v for v in plan["review_measurements"])
            and isinstance(plan["after_first_trial"], str) and plan["after_first_trial"], "initialization review obligations missing")
    reference = plan["historical_before_bootstrap_reference"]
    require(set(reference) == {"campaign", "source_commit", "source_sha256", "native_launch_maximum_acceleration_mps2", "initial_next_steering", "notice"}
            and reference["campaign"] == HISTORICAL_ROOT and reference["source_commit"] == turn.COMMIT
            and reference["source_sha256"] == HISTORICAL_SHA
            and strict_equal(reference["native_launch_maximum_acceleration_mps2"], 3.654932706817288)
            and strict_equal(reference["initial_next_steering"], -.8)
            and isinstance(reference["notice"], str) and "nonrandomized" in reference["notice"], "historical initialization reference changed")
    for name, expected in HISTORICAL_SHA.items():
        raw = pilot.checked_bytes(ROOT, HISTORICAL_ROOT + "/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == expected, "historical initialization evidence SHA differs")
    return ledger


def validate_second_review(review, first, first_root, second_root, first_owner, second_plan):
    """HH_260906 - Any optional repetition needs an intervening source-bound review, never a selective automatic retry."""
    state_entries = [e for e in first["source_manifest"] if e["path"].endswith("/states.jsonl")]
    require(len(state_entries) == 1, "second review has no unique preceding native source")
    fixed = {"schema": "portable_e2e.agent_initialization_second_review.v1", "source_commit": COMMIT,
        "previous_output": "c_track_left/run_001", "next_output": "c_track_left/run_002",
        "previous_states_sha256": state_entries[0]["sha256"], "previous_owner_exit_code": first_owner["exit_code"],
        "previous_transport_status": first["transport_protocol"]["status"],
        "previous_native_scalar_quality_clear": first["independent_qa"]["raw_scalar_quality_clear"],
        "previous_pilot_protocol_all_checks_pass": first["pilot_protocol"]["all_checks_pass"],
        "previous_initialization_status": first["initialization_protocol"]["status"],
        "source_and_parameters_changed": False, "automatic_retry": False, "training_data_approved": False,
        "qualification_30_kph": "NOT_CLAIMED"}
    require(isinstance(review, dict) and set(review) == set(fixed) | {"reviewed_at_utc", "comment", "reason"}
            and all(strict_equal(review[k], value) for k, value in fixed.items()), "optional second review contradicts independent first evidence")
    require(all(isinstance(review[k], str) and review[k] for k in ("comment", "reason")), "second review rationale missing")
    require(utc(first_owner["completed_at_utc"]) <= utc(review["reviewed_at_utc"]) < utc(second_plan["planned_at_utc"]),
            "second review does not intervene before second planning")


def audit_campaign(campaign):
    """HH_260906 - Retain every discovered attempt and distinguish maximum two from a requirement to repeat failed data."""
    campaign, ledger = Path(campaign).resolve(), {}
    plan = pilot.read_json(campaign, "pilot_plan.json", ledger)
    historical_sources = validate_plan_document(plan)
    historical_audit, historical_images, historical_timeline = turn.audit_trial(ROOT / HISTORICAL_ROOT / "c_track_left/run_001")
    require(historical_audit["reviewed_execution_commit"] == turn.COMMIT, "historical audit revision changed")
    parent = campaign / "c_track_left"
    roots = sorted(p for p in parent.iterdir() if p.name.startswith("run_")) if parent.exists() else []
    require([p.name for p in roots] in ([], ["run_001"], ["run_001", "run_002"]), "excess, omitted or noncontiguous low-turn attempts")
    results, audits, owners, plans = [], [], [], []
    for root in roots:
        require(root.is_dir() and not root.is_symlink(), "unsafe low-turn attempt directory")
        if not (root / "owner_result.json").exists():
            results.append({"trial_id": root.name, "status": "INCOMPLETE", "dataset_admission": False})
            continue
        result = audit_trial(root)
        audited = result[0]
        relative = root.relative_to(campaign).as_posix()
        owner = pilot.read_json(campaign, relative + "/owner_result.json", ledger)
        owner_plan = pilot.read_json(campaign, relative + "/owner_plan.json", ledger)
        require(utc(plan["declared_at_utc"]) <= utc(owner_plan["planned_at_utc"]), "turn was planned before prospective declaration")
        require(owner_plan["source_sha256"] == plan["source_hashes"]
                and Path(owner_plan["route_path"]).resolve() == (ROOT / plan["route_path"]).resolve(), "turn source or route differs from declaration")
        require(owner_plan["collector_argv"] == [str(root / "episode"), owner_plan["route_path"], "--host", "127.0.0.1",
                "--port", str(owner_plan["port"]), *plan["capture_flags"]], "turn exact argv differs from prospective plan")
        results.append(audited); audits.append((root, result)); owners.append(owner); plans.append(owner_plan)
    review = None
    if len(roots) == 2:
        require(len(audits) >= 1 and results[0]["status"] != "INCOMPLETE", "second attempt started before first was finalized")
        review = pilot.read_json(campaign, "run_002_preflight_review.json", ledger)
        second_plan = pilot.read_json(campaign, "c_track_left/run_002/owner_plan.json", ledger)
        validate_second_review(review, results[0], roots[0], roots[1], owners[0], second_plan)
    complete = bool(roots) and len(audits) == len(roots)
    report = {"schema": "portable_e2e.agent_initialization_campaign_audit.v1", "status": "AUDITED_NOT_ADMITTED" if complete else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "reviewed_execution_commit": COMMIT,
        "prospective_plan": plan, "prospective_plan_sha256": ledger["pilot_plan.json"]["sha256"],
        "discovered_attempts": len(roots), "finalized_attempts": len(audits), "maximum_attempts": 2,
        "all_discovered_attempts_retained": True, "optional_second_review": review, "trials": results,
        "historical_before_bootstrap_trial": historical_audit,
        "historical_reference_is_new_paired_trial": False, "initialization_alone_physical_spike_cause_proven": False,
        "historical_file_creation_time_proven": False, "qualification_30_kph": "NOT_CLAIMED", "training_data_approved": False,
        "dataset_admission": False, "full_future_xy_admission": False, "automatic_promotion": False,
        "same_initial_condition_repetitions_not_independent_routes": True,
        "identical_control_between_maps_comparison": False, "wall_timing_independently_reconstructed": False,
        "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())],
        "historical_lateral_source_manifest": [{"path": name, **value} for name, value in sorted(historical_sources.items())],
        "audit_source_sha256": {Path(p).resolve().relative_to(ROOT).as_posix(): base.sha(Path(p)) for p in (__file__, turn.__file__, ack.__file__, v4.__file__, pilot.__file__, base.__file__)},
        "interpretation": "A separate 14.4 km/h C-track left development diagnostic with historical C-track compact lookahead, not an identical-control Town07 comparison. All raw failures remain. Low-speed stability is measured 3.8..4.2 m/s for 5 seconds; the inherited base 30 km/h ceiling flag is only a loose upper bound, never 30 km/h cruise qualification. Exact native 20 Hz acceleration, stop/goal geometry and ACK evidence are independently recomputed. Camera byte hashes are not pixel approval. Timing files are preserved but not independently reconstructed here. No model, training, test payload, native coordinate rewrite or data admission."}
    for entry in report["source_manifest"]:
        require(base.sha(campaign / entry["path"]) == entry["sha256"], "prospective source changed during audit")
    for entry in report["historical_lateral_source_manifest"]:
        require(base.sha(ROOT / entry["path"]) == entry["sha256"], "historical lateral source changed during audit")
    for root, (result, images, _) in audits:
        for entry in result["source_manifest"] + images:
            require(base.sha(root / entry["path"]) == entry["sha256"], "turn source changed during campaign audit")
    return report, audits




def main(argv=None):
    """HH_260906 - Create only a new audit folder; all original failed trials and timestamps remain untouched."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--visual-root", type=Path, help="Publish the first actual comparison after independent audit")
    args = parser.parse_args(argv)
    if args.visual_root is not None:
        report = publish(args.campaign, args.output_dir, args.visual_root)
        print(json.dumps({"status": report["status"], "attempts": report["discovered_attempts"], "dataset_admission": False}))
        return 0
    output = v4.new_output(args.output_dir, [args.campaign])
    report, audits = audit_campaign(args.campaign)
    output.mkdir(parents=True, exist_ok=False)
    (output / "audit.json").write_text(json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for root, (_, images, _) in audits:
        (output / (root.name + "_image_hashes.json")).write_text(json.dumps(images, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "attempts": report["discovered_attempts"], "dataset_admission": False}))
    return 0 if report["status"] != "INCOMPLETE" else 2


def render_comparison(new_timeline, output):
    """HH_260906 - Plot whole native recordings and explicitly labelled startup insets, never delete a violating interval."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    historical = ROOT / HISTORICAL_ROOT / "c_track_left/run_001"
    _, old_timeline = base.summarize_trial(historical, None)
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 7.2))
    metadata = []
    for timeline, label, color in ((old_timeline, "Before bootstrap (historical)", "#ab5b17"),
                                   (new_timeline, "After verified bootstrap", "#146995")):
        rows, rates = timeline["native_states"], timeline["native_intervals"]
        first_driving = next(r for r in rows if r["phase"] == "driving")
        origin = first_driving["elapsed_s"] - .05
        axes[0, 0].plot([r["elapsed_s"] - origin for r in rows], [r["speed_mps"] * 3.6 for r in rows], label=label, color=color)
        axes[0, 1].plot([r["time_from_capture_start_s"] - origin for r in rates], [r["speed_rate_mps2"] for r in rates], label=label, color=color, lw=.8)
        axes[1, 0].plot([r["elapsed_s"] - origin for r in rows], [r["current_control"]["steer"] for r in rows], label=label, color=color)
        axes[1, 1].plot([r["time_from_capture_start_s"] - origin for r in rates], [r["speed_rate_mps2"] for r in rates], label=label, color=color)
        metadata.append({"label": label, "native_states": len(rows), "native_intervals": len(rates),
            "all_native_intervals_plotted": True, "engagement_elapsed_seconds": origin,
            "minimum_mps2": min(r["speed_rate_mps2"] for r in rates), "maximum_mps2": max(r["speed_rate_mps2"] for r in rates)})
    axes[0, 0].set(title="Entire speed records: warmup, driving, goal and tail", ylabel="Measured speed (km/h)")
    axes[0, 1].set(title="All native 20 Hz intervals: both failures retained", ylabel="Scalar speed rate (m/s²)")
    axes[1, 0].set(title="API-reported steering after engagement (inset)", ylabel="Normalized steering", xlim=(-.2, 2.5))
    axes[1, 1].set(title="Launch acceleration (inset): neither trial passes", ylabel="Scalar speed rate (m/s²)", xlim=(0, 4))
    for axis in (axes[0, 1], axes[1, 1]):
        axis.axhline(2.9, color="black", linestyle="--", linewidth=.8, label="Decoder +2.9")
        axis.axhline(-2.9, color="black", linestyle="--", linewidth=.8)
        axis.axhline(3.0, color="#900000", linestyle=":", linewidth=.8, label="Runtime acceleration +3.0")
    for axis in axes.flat:
        axis.set_xlabel("Seconds from engagement; warmup negative")
        axis.legend(fontsize=7); axis.grid(alpha=.2)
    fig.suptitle("C-track initialization-order comparison | first steering corrected | native acceleration still FAIL", fontsize=12)
    fig.text(.5, .009, "Historical nonrandomized reference. Whole constructor context changes; not physical causality, model driving or training admission.", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, .025, 1, .95)); fig.savefig(output, dpi=150); plt.close(fig)
    return {"schema": "carla.agent_initialization_comparison_plot.v1", "series": metadata,
            "raw_values_smoothed_or_filtered": False, "api_control_is_physical_timing_proof": False}


def publish(campaign, output, visual_root):
    """HH_260906 - Publish the complete first failed initialization comparison create-only, without relabelling the old failure."""
    campaign, visual_root = Path(campaign).resolve(), Path(visual_root).resolve()
    output = v4.new_output(output, [campaign, visual_root])
    report, audits = audit_campaign(campaign)
    require(report["status"] == "AUDITED_NOT_ADMITTED" and report["discovered_attempts"] == 1,
            "first initialization publication requires exactly one finalized actual attempt")
    root, (trial, images, timeline) = audits[0]
    require(trial["owner_exit_code"] == 1 and trial["initialization_protocol"]["status"] == "PASS"
            and not trial["independent_qa"]["raw_scalar_quality_clear"], "first initialization failure outcome differs")
    journals = {"episode.partial/states.jsonl": "7e763dc564292afb0e35500cd78fbd7f236a213e6240bf763c92fbaedec83e9a",
        "episode.partial/control_receipts.jsonl": "573e6f72e83a1dd74d129b051792d65568a34ed6e081dc84d929ff8b8a5db88d"}
    require(all(base.sha(root / name) == digest for name, digest in journals.items()), "first initialization journal identity changed")
    payloads, visual_proof = turn.checked_turn_visuals(visual_root / "run_001", root)
    output.mkdir(parents=True, exist_ok=False)
    def write(name, value):
        with (output / name).open("x", encoding="utf-8") as stream: stream.write(value)
    report["optional_second_attempt"] = {"status": "NOT_RUN", "maximum_two_attempt_quota": "NOT_EXHAUSTED", "mandatory_repeat": False}
    report["publication_notice"] = "Redacted metadata view, not exact JSON bytes. Raw source hashes and all failed records remain private and unchanged; no full camera corpus copied to Git."
    write("audit.json", json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    write("run_001_image_hashes.json", json.dumps(images, indent=2) + "\n")
    camera_dir = output / "actual_camera_route_evidence"
    camera_dir.mkdir()
    for name, raw in payloads.items():
        if name.endswith(".json"):
            write("actual_camera_route_evidence/" + name, json.dumps(v4.sanitize(json.loads(raw)), indent=2) + "\n")
        else:
            with (camera_dir / name).open("xb") as stream: stream.write(raw)
    midpoint = turn.render_actual_left_midpoint(root, camera_dir / "07_catalog_left_midpoint.png")
    write("actual_camera_route_evidence/07_catalog_left_midpoint_provenance.json", json.dumps(v4.sanitize(midpoint), indent=2) + "\n")
    plotted = render_comparison(timeline, output / "01_initialization_and_native_acceleration_comparison.png")
    turn.render_numerical_failure(trial, timeline, root, output / "02_new_full_route_and_native_launch_failure.png")
    proof = {"schema": "carla.agent_initialization_publication.v1", "audit_sources": report["audit_source_sha256"],
        "raw_journal_sha256": journals, "original_png_gif_exact_bytes": True,
        "metadata_json_policy": "Redacted/reformatted views; exact original hashes retained separately.",
        "actual_visual_proof": visual_proof, "additional_left_midpoint_proof": midpoint, "comparison_plot": plotted,
        "source_raw_trial": root.relative_to(ROOT).as_posix(), "source_visual_root": visual_root.relative_to(ROOT).as_posix(),
        "historical_reference_raw_root": HISTORICAL_ROOT, "training_data_approved": False,
        "first_initialization_trial_count": 1, "historical_reference_count": 1, "physical_cause_proven": False}
    write("publication_manifest.json", json.dumps(v4.sanitize(proof), indent=2) + "\n")
    write("README.md", """<!-- HH_260906 - Preserve both initialization-order outcomes and the native launch failures without causal or admission claims. -->
# C-track 초기화 순서 비교: 초기 조향 정상화, 출발 가속도는 여전히 실패

실제 CARLA BasicAgent 전문가 주행입니다. 학습 모델이나 Autoware 자율주행 화면이 아닙니다. 목표 14.4 km/h, Epic, 동일 원본 좌회전 경로·센서·제어 설정이며 학습 데이터 승인은 하지 않았습니다.

| 구분 | 이전: bootstrap 이전 생성 | 새 시험: 검증된 bootstrap 후 생성 |
|---|---:|---:|
| 첫 API 관측 조향 | −0.800000 | −0.000218737 |
| native 20Hz 최대 가속도 | +3.654933 m/s² | +3.467811 m/s² |
| 카메라 10Hz 최대 가속도 | +2.450543 m/s² | +2.715636 m/s² |
| native 상태 / 카메라 묶음 | 1,907 / 954 | 1,939 / 970 |
| 종점 오차 | 0.938601 m | 0.952436 m |
| 저속 안정 구간 | 39.45초 | 39.50초 |
| native 품질 결과 | FAIL | FAIL |

이전 결과는 별도 시점의 **비무작위 역사 비교**입니다. 새 시험은 **1회 / 최대 2회** 실행했으며 두 번째는 미실행·한도 미소진입니다. 유리한 반복만 고른 결과가 아니며 동일 조건의 독립 경로 표본도 아닙니다.

새 시험은 bootstrap frame 2863에서 생성 전후 pose·시간·control이 일치하고 PID 조향 이력이 0임을 확인했습니다. 기존 warmup 70틱 후 frame 2933의 첫 제안·송신 명령이 receipt 73으로 승인됐으며 다음 frame 2934의 관측 control까지 일치합니다. ACK 1,944개 모두 확인, 불일치·충돌·차선 침범 0개입니다. 초기 조향이 거의 0으로 바뀌었지만 **출발 가속도 초과는 해결되지 않았습니다**.

BasicAgent 생성자는 현재 위치와 cached control 등을 읽으므로 이것은 전체 생성 시점 변경 실험이지 조향 이력만의 원인 증명이 아닙니다. API 관측과 서버 ACK는 실제 물리 적용 시각의 증명도 아닙니다. 추가 read-only 관측의 시간 비용도 0이라고 주장하지 않습니다.

[이전/신규 전체 속도·20Hz 가속도와 출발 확대](01_initialization_and_native_acceleration_comparison.png) · [새 전체 경로와 위반 구간](02_new_full_route_and_native_launch_failure.png) · [시작 실제 6카메라](actual_camera_route_evidence/01_start.png) · [좌회전 중앙](actual_camera_route_evidence/07_catalog_left_midpoint.png) · [종점 정지](actual_camera_route_evidence/04_goal_dwell.png) · [전체 구간 GIF](actual_camera_route_evidence/whole_recording_accelerated.gif)

10Hz 결과나 두 가지 대체 10Hz offset은 진단용이며 **20Hz +2.9 디코더 / +3.0 runtime 가속도 실패를 대체하지 않습니다**. 전체 warmup·driving·정차 tail과 위반 구간을 제거·평활화하지 않았습니다. 2초 정지 유지 및 130개 tail 상태를 보존했습니다. 전체 XY·픽셀 품질·30 km/h 검증·실차·학습 데이터 승인과는 별개입니다.

GIF는 10Hz 묶음을 5개 간격으로 고른 10fps 가속 재생으로 실제 FPS 계측이 아닙니다. 카메라 합성은 1600×900, 전체 FOV와 차량 중심 경로를 유지합니다. 07 장면은 원본 LEFT 구간 중간 진행거리에 가장 가까운 카메라로 선정했습니다. 기존 renderer의 02는 7.8m/s 조건이므로 이 저속 기록에는 없습니다.

[독립 감사·사전 계획·원본 해시](audit.json) · [전체 5,820개 이미지 해시](run_001_image_hashes.json) · [발행 출처](publication_manifest.json) · [파일 검증값](SHA256SUMS) · [이전 실패 원본 결과](../18_c_track_low_speed_turn/README.md)

원본 실패 기록·카메라 데이터는 private artifacts에 그대로 있습니다. 기존 렌더 PNG/GIF는 정확한 원본 바이트로 복사했고 JSON은 계정 경로를 가린 메타데이터 뷰입니다. 원본 JSON 해시를 보존합니다. 새 검증기는 dfccaa8 소스를 따로 고정하여 이전 c4f40fc 감사기의 소스 제한을 변경하지 않습니다.
""")
    for path, digest in report["audit_source_sha256"].items():
        require(base.sha(ROOT / path) == digest, "initialization auditor changed during publication")
    for entry in trial["source_manifest"] + images:
        require(base.sha(root / entry["path"]) == entry["sha256"], "raw initialization evidence changed during publication")
    for entry in visual_proof["source_manifest"]:
        require(base.sha(visual_root / "run_001" / entry["path"]) == entry["sha256"], "actual visual changed during publication")
    files = sorted(p for p in output.rglob("*") if p.is_file())
    write("SHA256SUMS", "".join(base.sha(path) + "  " + path.relative_to(output).as_posix() + "\n" for path in files))
    return report


if __name__ == "__main__":
    raise SystemExit(main())

