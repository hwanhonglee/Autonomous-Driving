"""HH_261001 - Bind the failed V2 archive and reuse frozen pure body geometry without inventing legacy candidates."""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_future_actor_lane_containment as body
from scripts.e2e import audit_simulation_research_body_lane_20261001 as legacy


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-01/temporal_v2_owned_camera_town07_screen_v1"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
DECLARATION_SHA = "c6f25e089366ce5f71c24ecffe705026843e9014da11704ee80a41a7b3dd013a"
CHECKPOINT_SHA = "7902216449dd8e7a601d63128a38773a5f3bf59c1660a7684b7f2d79791659b6"
SCHEMA = "portable_e2e.temporal_v2_archive_native_body_and_failure.20261001.v1"
FROZEN = {
    "tools/e2e/audit_simulation_research_body_lane_20261001.py": "6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac",
    "tools/e2e/audit_future_actor_lane_containment.py": "75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d",
    "tools/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
}
RAW_NAMES = ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl",
             "control_receipts.jsonl", "learned_control.jsonl", "learned_summary.json",
             "wall_timing.jsonl", "active_map_opendrive.xodr")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular non-symlink input required: " + str(path))
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    require(Path(path).stat().st_size <= 4*1024**2, "bounded JSON required")
    return json.loads(Path(path).read_text())


def read_rows(path, maximum=1000):
    require(Path(path).stat().st_size <= 128*1024**2, "bounded native journal required")
    result = []
    with Path(path).open() as stream:
        for line in stream:
            result.append(json.loads(line))
            require(len(result) <= maximum, "native journal exceeds fixed row bound")
    return result


def state_annotation_counts(states):
    """HH_261001 - A missing outgoing annotation is explicitly unrecorded, never guessed from future command rows."""
    return dict(Counter(row.get("research_control_source") or "unrecorded" for row in states))


def first_failure_evidence(commands, states):
    """HH_261001 - Report the first original rejection and retain the entire later brake latch, not a cropped success."""
    by_frame = {row["frame"]: row for row in states}
    first = next((i for i, row in enumerate(commands)
                  if row.get("emergency_reason") or row.get("control_source") == "emergency_brake"), None)
    require(first is not None, "this fixed failed V2 archive must contain its first latched rejection")
    failed = commands[first]
    source = by_frame[failed["inference_frame"]]
    previous_fresh = next((row for row in reversed(commands[:first])
        if row.get("held_between_camera_frames") is False
        and row.get("application", {}).get("learned_control_applied") is True), None)
    handoff = next(row for row in commands
                   if row.get("application", {}).get("learned_control_applied") is True)
    tail = commands[first:]
    require(all(row.get("control_source") == "emergency_brake"
        and row.get("learned_control_proposed") is False
        and row.get("application", {}).get("learned_control_applied") is False
        and row["control"]["throttle"] == 0. and row["control"]["brake"] == 1.
        and row["control"]["steer"] == 0. for row in tail),
        "later latch must not resume learned/expert controls or silently discard a command")
    speed = math.hypot(source["vx"], source["vy"])
    actor_speed = math.hypot(*source["world_velocity_carla"][:2])
    require(speed > 2.5 and failed["emergency_reason"] ==
            "actual current speed exceeds unchanged low-speed envelope; no clipping",
            "original first failure changed; do not relabel it as a deadline failure")
    require(failed["total_seconds"] <= .1 and isinstance(failed.get("raw_output"), dict)
            and failed.get("decoded_output") is None and failed.get("selected") is None,
            "original rejected raw neural result/control authority must be retained")
    return dict(first_rejection_ACK_sequence=failed["ack_receipt_sequence"],
        first_rejection_frame=failed["inference_frame"], first_rejection_timestamp_s=source["timestamp"],
        elapsed_since_first_learned_command_s=source["timestamp"]-by_frame[handoff["command_frame"]]["timestamp"],
        original_emergency_reason=failed["emergency_reason"],
        actual_base_vx_mps=source["vx"], actual_base_vy_mps=source["vy"],
        actual_unclipped_base_planar_speed_mps=speed, actor_world_planar_speed_mps=actor_speed,
        unchanged_base_speed_envelope_mps=2.5, base_speed_excess_mps=speed-2.5,
        original_policy_seconds=failed["total_seconds"], original_raw_NN_seconds=failed["raw_nn_seconds"],
        original_rejection_was_policy_deadline=False, original_rejected_raw_output=failed["raw_output"],
        previous_accepted_fresh_command=previous_fresh,
        whole_latched_command_tail_count=len(tail),
        subsequent_latched_reasons=dict(Counter(row.get("emergency_reason") for row in tail)),
        no_post_rejection_resumption=True, failed_tail_cropped=False)


def bind_archive(run=RUN):
    """HH_261001 - Pin before parsing, verify archive/live closure, and never mutate failed manifests or candidate ABI."""
    require(run.resolve() == RUN and not run.is_symlink(), "fixed actual V2 v1 archive required")
    episode = run / "episode.partial"
    require(episode.is_dir() and not (run / "episode").exists(), "one original failed partial episode required")
    hashes = {}
    def pin(path):
        value = digest(path)
        hashes[str(path)] = value
        return value
    require(pin(run / "declaration.json") == DECLARATION_SHA, "original prospective declaration differs")
    for name in ("owner_result.json", "owner_started.json", "diagnostic_report.json"):
        pin(run / name)
    for name in RAW_NAMES:
        pin(episode / name)
    for name, expected in FROZEN.items():
        require(pin(ROOT/name) == expected, "frozen pure geometry/auditor source changed")
    pin(Path(__file__))
    declaration, owner, manifest = [read_json(path) for path in
        (run / "declaration.json", run / "owner_result.json", episode / "manifest.json")]
    require(owner["declaration_sha256"] == DECLARATION_SHA
        and owner["source_checkpoint_sha256"] == CHECKPOINT_SHA
        and owner["source_hashes_match"] is True and owner["external_source_hashes_match"] is True
        and owner["cleanup_errors"] == [] and owner["port_release"]["original_TCP_UDP_bind_checks_passed"] is True
        and owner["port_release"]["SO_REUSE_used"] is False, "source and original cleanup/ownership proof required")
    require(declaration["source_checkpoint_sha256"] == CHECKPOINT_SHA
        and declaration["candidate_count"] == 1 and declaration["optimizer_updates"] == 3000,
        "fixed final3000 one-candidate identity differs")
    for name, expected in declaration["source_sha256"].items():
        require(pin(ROOT/name) == expected == pin(run/"source"/name), "live/archive runtime source changed: " + name)
    for name, expected in declaration["external_source_sha256"].items():
        source = Path(name)
        require(source.is_relative_to(CARLA_ROOT), "external source escaped pinned CARLA root")
        require(pin(source) == expected == pin(run/"source_external"/source.relative_to(CARLA_ROOT)),
                "live/archive external source changed")
    for name, expected in declaration["asset_sha256"].items():
        source = Path(name)
        if source.is_relative_to(ROOT):
            archive = run/"source"/source.relative_to(ROOT)
        else:
            require(source.is_relative_to(CARLA_ROOT), "fixed asset escaped source roots")
            archive = run/"source_external"/source.relative_to(CARLA_ROOT)
        require(pin(source) == expected == pin(archive), "fixed live/archive asset changed")
    capture = manifest["capture_contract"]
    require(manifest["status"] == "failed" and manifest["training_data_approved"] is False
        and manifest["vehicle_control_approved"] is False and manifest["cleanup"]["completed"] is True
        and manifest["cleanup"]["errors"] == [] and capture["physics_hz"] == 20.
        and capture["camera_hz"] == 10. and capture["control_transport"]["mode"] == "acknowledged_batch"
        and capture["learned_research"]["candidate_count"] == 1
        and capture["learned_research"]["checkpoint_sha256"] == CHECKPOINT_SHA
        and capture["learned_research"]["stream_identity"]["source_sha256"] == DECLARATION_SHA,
        "original failed unapproved V2 capture contract differs")
    require(pin(run/"source/tools/e2e/collect_carla_vad_expert.py") == manifest["provenance"]["collector_sha256"]
        and hashes[str(episode/"route.json")] == manifest["provenance"]["route_sha256"],
        "original collector/route provenance differs")
    states, cameras, receipts, commands = [read_rows(episode/name) for name in
        ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl", "learned_control.jsonl")]
    require(len(states) == 400 and len(cameras) == 200, "all400 native states/all200 six-camera bundles required")
    require(manifest["result"]["control_transport"]["receipt_journal_sha256"] ==
            hashes[str(episode/"control_receipts.jsonl")], "native receipt journal hash differs")
    for camera in cameras:
        require(len(camera["images"]) == 6, "six actual stored JPEGs required for every bundle")
        for relative in camera["images"].values():
            path = episode/relative
            require(path.resolve().is_relative_to((episode/"images").resolve()), "stored camera path escaped episode")
            pin(path)
    return episode, declaration, owner, manifest, states, cameras, receipts, commands, hashes


def audit(output):
    output = Path(output).absolute()
    require(output.resolve().is_relative_to(RUN/"audit_v1") and not output.exists() and not output.is_symlink(),
            "fresh explicit derived output under the fixed run audit_v1 required")
    episode, declaration, owner, manifest, states, cameras, receipts, commands, hashes = bind_archive()
    require(sys.version_info[:2] == (3, 10), "pinned offline CARLA map API requires Python3.10")
    sys.path.insert(0, str(EGG))
    import carla
    require(str(carla.__file__).startswith(str(EGG)+"/"), "foreign offline CARLA API")
    actor_ids = {row["actor_id"] for row in receipts}
    require(len(actor_ids) == 1, "one acknowledged actor body required")
    transport = ack.analyze_transport(states, receipts,
        manifest["capture_contract"]["control_transport"]["bootstrap_observation"],
        wheelbase=manifest["coordinate_contract"]["wheelbase_m"])
    camera_alignment = ack.analyze_camera_alignment(states, cameras)
    require(transport["status"] == camera_alignment["status"] == "PASS", "independent native/camera ACK reconstruction failed")
    route = read_json(episode/"route.json")
    geometry, census = legacy.body_census(manifest, route, states,
        (episode/"active_map_opendrive.xodr").read_bytes(), next(iter(actor_ids)))
    require(len(geometry) == 400, "frozen pure body census must retain the failed native tail")
    failure = first_failure_evidence(commands, states)
    report = dict(schema=SCHEMA,
        status="PASS_GEOMETRY_ONLY_WITH_DRIVING_FAILURE_RETAINED" if all(value["status"] == "PASS" for value in census.values()) else "FAIL_BODY_GEOMETRY",
        status_scope="all400 actual actor boxes and399 bounded snapshot-linear sweeps; NOT learned driving success",
        original_manifest_status=manifest["status"], original_capture_error=manifest["error"],
        original_owner_result=owner, original_declaration_sha256=DECLARATION_SHA,
        source_checkpoint_sha256=CHECKPOINT_SHA, source_sha256=hashes,
        native_state_count=len(states), camera_bundle_count=len(cameras),
        acknowledged_receipt_count=len(receipts), whole_proposed_command_count=len(commands),
        actual_state_outgoing_source_annotation_counts=state_annotation_counts(states),
        outgoing_source_annotations_are_not_current_actuator_proof=True,
        observed_collision_event_count=sum(len(row["collision"]) for row in states),
        observed_lane_invasion_event_count=sum(len(row["lane_invasion"]) for row in states),
        independent_transport=transport, independent_camera_alignment=camera_alignment,
        body_geometry=census, first_latched_failure=failure,
        legacy_twelve_candidate_bound_audit=dict(status="NOT_APPLICABLE_UNCHANGED_V2_RAW_NEURAL_ABI",
            original_raw_output_contains="neural curvature/preferred-speed knots, acceleration and untrained intent/stop outputs",
            decoded_trajectory_location="separate original decoded_output, one candidate only; rejected decision has none",
            legacy_candidate_arrays_fabricated=False, original_outputs_rewritten=False),
        frozen_geometry_policy=dict(lane_sample_m=body.LANE_SAMPLE_M, lane_inset_m=body.LANE_INSET_M,
            seam_close_m=body.SEAM_GAP_CLOSE_M, sweep_position_guard_m=body.SWEEP_POSITION_GUARD_M,
            actor_box="all8 vertices with original full roll/pitch/yaw; receipt actor/bbox ID matched",
            scope="bounded snapshot-linear envelope, not continuous physical collision proof"),
        threshold_relaxed=False, original_failed_tail_cropped=False,
        original_manifest_and_goal_flags_rewritten=False, physical_continuous_time_proven=False,
        learned_driving_success_claimed=False, full_RTE01_approved=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    require(all(digest(Path(name)) == value for name, value in hashes.items()), "source/raw archive changed during derived audit")
    # HH_261001 - Complete strict JSON encoding before creating outputs, so encoding failure leaves no partial report.
    serialized = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+"\n"
    output.mkdir(parents=True, exist_ok=False)
    with gzip.open(output/"all400_native_body_and399_sweeps.jsonl.gz", "xt") as stream:
        for row in geometry:
            stream.write(json.dumps(row, allow_nan=False)+"\n")
    with (output/"summary.json").open("x") as stream:
        stream.write(serialized)
    print(json.dumps({key: report[key] for key in ("status", "native_state_count", "camera_bundle_count", "body_geometry")}, indent=2))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    audit(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
