"""HH_261004 - Own a fresh Town03 straight rerun with actual post-ACK recording and authenticated start corridor."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

from . import audit_town03_straight_start_corridor_20261004 as start_geometry
from . import collect_temporal_route_joint_town03_straight_v2_20261004 as live
from . import research_temporal_route_joint_town03_straight_20261004 as previous


base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
require, digest, read_json = previous.require, previous.digest, previous.read_json
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_v2_20261004"
SCHEMA = live.OWNER_SCHEMA
ROUTE, XODR, UMAP = previous.ROUTE, previous.XODR, previous.UMAP
XODR_SHA, UMAP_SHA = previous.XODR_SHA, previous.UMAP_SHA
PARENT_OWNER_SHA = "6dbbfde5798200ed686a955848dd1f929c47976a406e2e5d3c0c603f96ceba95"
START_GEOMETRY_SHA = "3ff6e4cd1d231cf3e839aa744f4ff4d6dda61400796cfc3d0249894e1c46cd05"
V1_OUTPUT = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v1"
V1_EVIDENCE_SHA256 = {
    "declaration.json": "9f26d7bf38459e7db6734db08c43c8c68fe84799aa7c36a0d2157b49f1bf2f0e",
    "diagnostic_report.json": "05f532b8465c1cde5bc854bf1c85078ea7c3453f0bc643b32261f53bf723db7a",
    "owner_result.json": "4db77d9bd4f10a520866fe60570899aae3ab7005ffd302665cff80b429da0a44",
    "episode.partial/manifest.json": "0e7eda89013865c8ce44149b93306dc748f7e13044f391dd12e8ffbb7ce7f89c",
}
_parent_capture = previous.capture_argv
_parent_fixed = previous.fixed_contract
_parent_sources = previous.runtime_sources
_parent_diagnostic = previous.diagnostic_report
_parent_body_geometry = previous.previous.geometry
_parent_body_report = previous.previous.body_report
_active = False


def paths(run_id):
    require(run_id == "v2", "fresh post-ACK Town03 straight rerun requires exact v2")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    """HH_261004 - Reuse the exact v1 CLI after scoped paths/module replacement; no drive setting changes."""
    require(run_id == "v2" and _active, "v2 capture CLI requires its exclusive owner scope")
    return _parent_capture(run_id, longitudinal_pi, declaration_sha)


def _verify_v1_failure():
    """HH_261004 - Preserve and authenticate the entire failed predecessor instead of overwriting or relabeling it."""
    require(V1_OUTPUT.is_dir() and not V1_OUTPUT.is_symlink(), "archived v1 failure directory required")
    require(all(digest(V1_OUTPUT / name) == sha for name, sha in V1_EVIDENCE_SHA256.items()),
            "archived v1 failure evidence changed")
    diagnostic = read_json(V1_OUTPUT / "diagnostic_report.json")
    owner = read_json(V1_OUTPUT / "owner_result.json")
    require(diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and diagnostic.get("diagnostic_error") == "RuntimeError: complete unique camera/ACK receipts required"
        and diagnostic.get("bounded_capture_complete") is True
        and diagnostic.get("actual_native_state_count") == 900
        and diagnostic.get("actual_camera_bundle_count") == 450
        and diagnostic.get("actual_ACK_receipt_count") == 904
        and owner.get("status") == "CAPTURE_RETURNED_NONZERO_RETAINED"
        and owner.get("declaration_sha256") == V1_EVIDENCE_SHA256["declaration.json"],
        "v1 failure identity or retained denominator differs")
    return diagnostic, owner


def _verify_start_geometry():
    """HH_261004 - Freeze the independently reviewed prefix kernel and its exact map/route/lane scope."""
    require(digest(Path(start_geometry.__file__)) == START_GEOMETRY_SHA
        and start_geometry.EXPECTED_ROUTE_SHA256 == live.ROUTE_SHA
        and start_geometry.EXPECTED_XODR_SHA256 == XODR_SHA
        and start_geometry.INITIAL_ROUTE_LANE == (1426, 0, 2)
        and start_geometry.PREFIX_LANE == (67, 0, 2),
        "reviewed Town03 start-corridor geometry changed")


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Change only recording evidence and authenticated initial corridor measurement in a fresh run."""
    diagnostic, _ = _verify_v1_failure()
    _verify_start_geometry()
    result = _parent_fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, output=str(paths(run_id)[0]), declaration_path=str(paths(run_id)[1]),
        experiment="Fresh fixed45 Town03 straight rerun with actual post-ACK JPEG receipts and start-corridor audit",
        failed_v1_archived_reference=dict(path=str(V1_OUTPUT), sha256=dict(V1_EVIDENCE_SHA256),
            status=diagnostic["status"], diagnostic_error=diagnostic["diagnostic_error"],
            native_state_count=900, camera_bundle_count=450, ACK_receipt_count=904,
            overwritten=False, promoted=False),
        intervention=dict(
            post_ACK_recording="late-bind copied `_save_jpeg` to the same bounded recorder as the actual ACK transport",
            physical_start_corridor="authenticate only the connected upstream lane needed by the recorded body radius",
            model_weights_changed=False, controller_changed=False, policy_changed=False,
            speed_or_acceleration_gate_changed=False, latency_gate_changed=False,
            route_projection_changed=False, body_area_or_inset_gate_changed=False),
        actual_post_ACK_recording=dict(required_camera_bundles=450, required_JPEG_count=2700,
            required_complete_ACK_receipts=904, six_images_per_bundle=True,
            synchronous_complete_write_before_next_tick=True, failure_rows_retained=True,
            recording_wall_time_excluded_from_existing_bundle_delivery_to_ACK_metric=True,
            full_sensor_exposure_to_ACK_still_unmeasured=True),
        actual_deferred_JPEG_after_ACK_required=True,
        authenticated_start_corridor=dict(schema=start_geometry.SCHEMA,
            source=str(Path(start_geometry.__file__).resolve().relative_to(ROOT)),
            source_sha256=digest(Path(start_geometry.__file__)),
            route_sha256=live.ROUTE_SHA, active_map_opendrive_sha256=XODR_SHA,
            prefix_lane=list(start_geometry.PREFIX_LANE), initial_route_lane=list(start_geometry.INITIAL_ROUTE_LANE),
            upstream_lane=list(start_geometry.PREFIX_LANE),
            exact_prefix_predicate="route arc minus recorded actor bounding-box radius is below 0 m",
            recorded_actor_bounding_box_radius_m=2.884351637197339,
            lane_inset_m=0.1, topology_seam_maximum_m=0.005, geometry_sampling_guard_m=0.02,
            topology="OpenDRIVE road67 lane2 to road1426 lane2 through junction1352 connection7",
            applied_only_to_negative_start_coverage=True,
            enabled_only_when_body_arc_lower_bound_negative=True,
            original_navigation_bytes_unchanged=True,
            all_actual_900_boxes_and_899_sweeps_required=True,
            physical_continuous_time_proven=False, gate_relaxed=False, failed_prefix_cropped=False),
        body_report_extension_hook="right-owner geometry namespace scoped to the authenticated fresh module only",
        fixed45_full_failed_budget_tail_required=True, attempts=1, automatic_retry=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    require(result["total_capture_seconds"] == 45 and result["maximum_native_state_count"] == 900
        and result["maximum_camera_bundle_count"] == 450 and result["maximum_driving_native_ticks"] == 876
        and result["model_speed_cap_mps"] == 2.5 and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["policy_deadline_seconds"] == 0.1 and result["controller_target_cap_mps"] == 2.4
        and result["speed_preview_s"] == 0.5
        and result["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.0
        and result["precursor_motion_screen"]["cte_p95_maximum_m"] == 0.35
        and result["precursor_motion_screen"]["cte_absolute_maximum_m"] == 0.75
        and result["body_clearance_screen"]["lane_inset_m"] == 0.1
        and result["additive_native_speed_rate_guard_mps2"] ==
            {"minimum": -6.0, "maximum": 3.0, "warmup_to_first_driving_interval_included": True}
        and result["cpu_thread_numerical_preset"]["intra_op_threads"] == 4
        and result["cpu_thread_numerical_preset"]["inter_op_threads"] == 1
        and result["worker_command"][result["worker_command"].index("-m") + 1] == MODULE,
        "v1 model/controller/speed/motion/body/latency/thread contract changed")
    return result


def runtime_sources():
    """HH_261004 - Archive the new owner, recording adapter, and authenticated geometry implementation."""
    require(digest(Path(previous.__file__)) == PARENT_OWNER_SHA, "archived Town03 straight v1 owner changed")
    _verify_v1_failure()
    _verify_start_geometry()
    result = _parent_sources()
    live.verify_frozen()
    imported = base.imported_source_closure((Path(__file__).resolve(), Path(live.__file__).resolve(),
                                             Path(start_geometry.__file__).resolve()))
    for path in imported:
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "fresh v2 source closure conflict")
        result[name] = sha
    require(result[str(Path(start_geometry.__file__).resolve().relative_to(ROOT))] ==
            digest(Path(start_geometry.__file__)), "authenticated start-corridor source missing")
    return result


def body_report(output, plan, episode, states, receipts, first_effect_frame):
    """HH_261004 - Run the unchanged full denominator and retain its authenticated prefix topology separately."""
    require(previous.previous.geometry is start_geometry,
            "fresh start-corridor geometry must be scoped before the full body census")
    result = _parent_body_report(output, plan, episode, states, receipts, first_effect_frame)
    actors = {row["actor_id"] for row in receipts}
    require(len(actors) == 1, "one actual ACK actor required for start-corridor topology")
    route = read_json(episode / "route.json")
    proof = start_geometry.topology_provenance(
        read_json(episode / "manifest.json"), route,
        (episode / "active_map_opendrive.xodr").read_bytes(), next(iter(actors)))
    require(proof["schema"] == start_geometry.SCHEMA
        and proof["authorized_start_prefix_lane"] == list(start_geometry.PREFIX_LANE)
        and proof["initial_route_lane"] == list(start_geometry.INITIAL_ROUTE_LANE)
        and proof["route_input_modified"] is False
        and proof["global_driving_lane_union_used"] is False
        and proof["lane_inset_m"] == 0.1
        and proof["body_radius_m"] == 2.884351637197339,
        "authenticated start-corridor topology differs")
    previous.previous.write_new(output / "derived_body/start_corridor_topology.json", proof)
    return result


def diagnostic_report(output, plan, owner):
    """HH_261004 - Require the new same-recorder proof in addition to every unchanged v1 diagnostic gate."""
    result = _parent_diagnostic(output, plan, owner)
    result.update(schema=SCHEMA + ".diagnostic_report", actual_after_ACK_recording_binding_passed=False,
                  archived_v1_failure_preserved=False)
    try:
        _verify_v1_failure()
        result["archived_v1_failure_preserved"] = True
        episode = output / "episode" if (output / "episode").is_dir() else output / "episode.partial"
        proof = read_json(episode / "actual_recording_binding.json")
        recordings = previous.previous.jsonl(episode / "recording_after_ack.jsonl")
        cameras = previous.previous.jsonl(episode / "camera_frames.jsonl")
        receipts = previous.previous.jsonl(episode / "control_receipts.jsonl")
        timing = previous.previous.jsonl(episode / "wall_timing.jsonl")
        topology = read_json(output / "derived_body/start_corridor_topology.json")
        require(proof.get("schema") == live.SCHEMA + ".actual_recording_binding"
            and proof.get("copied_provider_names") == ["_blank_brake_control",
                "checked_agent_initialization_observation", "apply_spawn_z_offset",
                "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"]
            and proof.get("all_copied_mutable_providers_current") is True
            and proof.get("enqueue_and_ACK_share_exact_recorder") is True
            and proof.get("controller_or_policy_math_changed") is False
            and proof.get("actual_recording_bundle_count") == 450
            and proof.get("actual_after_ACK_receipt_count") == 450
            and proof.get("pending_source_camera_count") == 0
            and len(recordings) == len({row["frame"] for row in recordings}) == len(cameras) == 450
            and len(receipts) == len({row["sequence"] for row in receipts}) == 904
            and len(timing) == len({row["ACK_sequence"] for row in timing}) == 904
            and result.get("recording", {}).get("passed") is True
            and result["recording"].get("JPEG_count") == 2700
            and topology.get("schema") == start_geometry.SCHEMA
            and topology.get("authorized_start_prefix_lane") == list(start_geometry.PREFIX_LANE)
            and topology.get("initial_route_lane") == list(start_geometry.INITIAL_ROUTE_LANE)
            and topology.get("route_input_modified") is False
            and topology.get("global_driving_lane_union_used") is False,
            "complete same-recorder 450-camera/904-ACK evidence required")
        previous.previous.verify_closure(output, plan)
        require(all(digest(path) == sha for path, sha in result.get("raw_source_sha256", {}).items()),
                "raw evidence changed during fresh recording audit")
        result["actual_after_ACK_recording_binding_passed"] = True
        result["authenticated_start_corridor"] = topology
    except Exception as error:
        result["recording_binding_diagnostic_error"] = type(error).__name__ + ": " + str(error)
    for key in ("learned_driving_screen_passed", "straight_motion_body_policy_wall100ms_passed"):
        result[key] = bool(result.get(key)) and result["actual_after_ACK_recording_binding_passed"] \
            and result["archived_v1_failure_preserved"]
    result.update(full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    result["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if result["learned_driving_screen_passed"] \
        else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Scope v2 aliases and the fresh geometry implementation, restoring every frozen provider."""
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_OWNER_SHA,
            "fresh Town03 straight v2 owner cannot nest or use a changed v1 owner")
    _verify_start_geometry()
    values = dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, diagnostic_report=diagnostic_report,
        live=live, MODULE=MODULE, SCHEMA=SCHEMA)
    saved = {name: getattr(previous, name) for name in values}
    original_geometry, original_body_report = previous.previous.geometry, previous.previous.body_report
    _active = True
    try:
        for name, value in values.items():
            setattr(previous, name, value)
        previous.previous.geometry = start_geometry
        previous.previous.body_report = body_report
        with previous.activated(pins):
            yield
    finally:
        previous.previous.geometry, previous.previous.body_report = original_geometry, original_body_report
        for name, value in reversed(tuple(saved.items())):
            setattr(previous, name, value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
            for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
