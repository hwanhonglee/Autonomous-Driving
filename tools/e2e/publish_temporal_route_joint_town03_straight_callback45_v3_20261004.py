"""HH_261004 - Publish one measured callback-to-plan fixed45 attempt with immutable whole-interval evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import shutil
import sys

from PIL import Image, ImageDraw

from . import publish_temporal_route_joint_town03_right_20261002 as previous


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v3"
OUTPUT = ROOT / "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/09_town03_callback_to_plan_v3"
POLICY_PATH = ROOT / ("storage/experiments/plans/2026-10-04/town03_known_dev_straight_callback45_v3/"
    "ROOT_prospective_publication_policy.json")
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SCHEMA = "portable_e2e.actual_town03_known_dev_straight_fixed45_publication.20261004.callback45_v3"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
POLICY_SCHEMA = "portable_e2e.town03_known_dev_straight_prospective_publication_policy.20261004.callback45_v3"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.callback45_v3"
CALLBACK_SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.callback45_v3.callback_receipts"
V2_RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v2"
V2_EVIDENCE_SHA = {
    "declaration.json": "5794c055f01fc342fd049b1e20c4b8026d2c3d0d6e1f88a521fa30196a4043fb",
    "diagnostic_report.json": "3f8561aca288960113a64f3429bc3381aace87a7bef68fdc850fabb5848c384b",
    "owner_result.json": "798612158de7f90b4b111a3d933a237cd6020f4d030df150cd153c8e1fff151f",
    "episode.partial/manifest.json": "0be981c4a40964fac04020e2b2972d5a6ba4fedc58c1541dea92f52fa25e2ca1",
    "episode.partial/callback_timing_receipts.json": "ea38029aef488350a67f9cbc5487af9a4d772cf126a83147be00403a6ee16fa2",
}
V2_SOURCE_SHA = {
    "tools/e2e/collect_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "5c645accae929efd2e004a5e2cc417e9eb5d8be59b8f6743ab4d9c8b92b9df07",
    "tools/e2e/research_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "5981f7f2a65523887dae1a708d0562cfbe2d464ba8afc2c92f9b601cd767d88c",
    "learning/tests/test_temporal_route_joint_town03_straight_callback45_v2_20261004.py": "3d358ef3b40a4f3c4cd887f6c237bcd8984a06c0461afd763749222d655c44c6",
}
NATIVE_STAGES = ("world_tick_started", "world_tick_returned", "snapshot_bound", "actor_state_ready",
    "agent_branch_started", "agent_branch_finished", "native_state_finished", "exact_bundle_started", "exact_bundle_finished")
POLICY_STAGES = ("input_ready", "raw_nn_finished", "decoded_plan_ready")
ROUTE_SHA = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
PREVIOUS_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
PNG_INDEX_POLICY = "predeclared_full450_[0,112,224,337,449]_start_25_50_75_100"
PNG_PERCENT_NUMERATORS = {
    "01_start.png": 0,
    "02_quarter.png": 1,
    "03_half.png": 2,
    "04_three_quarters.png": 3,
    "05_last.png": 4,
}
FULL450_PNG_CAMERA_INDICES = {
    "01_start.png": 0,
    "02_quarter.png": 112,
    "03_half.png": 224,
    "04_three_quarters.png": 337,
    "05_last.png": 449,
}
EXPECTED_BUDGET_ERROR = (
    "CollectionError: BasicAgent did not finish within the driving budget reserved by "
    "the maximum total duration and stationary tail"
)
MODEL_CAMERA_ORDER = previous.MODEL_CAMERA_ORDER
require, prior, visual = previous.require, previous.prior, previous.visual


def publication_sources():
    """HH_261004 - Seal this publisher and its complete imported diagnostic renderer closure."""
    require(prior.digest(Path(previous.__file__)) == PREVIOUS_SHA, "reviewed Town03 renderer changed")
    return previous.core.audit.source_closure(
        Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")


def output_for(run):
    run = previous.canonical(run)
    require(run == RUN, "only the prospectively declared Oct4 straight run is publishable")
    return previous.canonical(OUTPUT)


def camera_indices(count):
    """HH_261004 - Keep fixed half-second bins and the exact final real camera, including failed tails."""
    require(type(count) is int and 1 <= count <= 450, "actual fixed45 camera denominator required")
    return sorted(set(range(0, count, 5)) | {count - 1})


def png_indices(count):
    """HH_261004 - Resolve only the prospectively fixed 0/25/50/75/100 percent sampling rule."""
    require(type(count) is int and 1 <= count <= 450, "actual PNG camera denominator required")
    if count == 450:
        return dict(FULL450_PNG_CAMERA_INDICES)
    return {name: (count - 1) * numerator // 4 for name, numerator in PNG_PERCENT_NUMERATORS.items()}


def playback(bundle, indices):
    return previous.playback(bundle, indices)


def verify_records(states, cameras, commands, receipts, geometry, *, unlogged_preNN=False):
    require(1 < len(states) <= 900 and 1 <= len(cameras) <= 450,
            "only the complete or retained-short fixed45 denominator is publishable")
    return previous.verify_records(
        states, cameras, commands, receipts, geometry, unlogged_preNN=unlogged_preNN)


def _pin_declared_sources(pins, run, plan):
    # HH_261004 - Verify live and archived bytes for every owner-declared source, external, and asset pin.
    groups = ((plan["source_sha256"], ROOT), (plan["external_source_sha256"], None),
              (plan["asset_sha256"], None))
    for table, relative_root in groups:
        for name, expected in table.items():
            source = previous.canonical(relative_root / name if relative_root is not None else name)
            if source.is_relative_to(ROOT):
                archived = run / "source" / source.relative_to(ROOT)
            else:
                require(source.is_relative_to(CARLA_ROOT), "declared source outside owned roots")
                archived = run / "source_external" / source.relative_to(CARLA_ROOT)
            pins.file(source, expected)
            pins.file(archived, expected)


def _verify_prospective_policy(caller, pins, sources):
    """HH_261004 - Make the post-episode caller bind the pre-episode media and source decision."""
    reference = caller.get("prospective_publication_policy", {})
    require(set(reference) == {"path", "sha256"}, "exact prospective publication policy reference required")
    path = previous.canonical(reference["path"])
    require(path == POLICY_PATH, "unexpected prospective publication policy path")
    pins.file(path, reference["sha256"])
    policy = pins.json(path)
    expected_sources = {str(source.relative_to(ROOT)): prior.digest(source) for source in sources}
    require(policy.get("schema") == POLICY_SCHEMA and policy.get("created_before_actual_episode") is True
        and policy.get("actual_episode_observed") is False
        and policy.get("run_root") == str(RUN) and policy.get("output") == str(OUTPUT)
        and policy.get("attempts") == 1 and policy.get("automatic_retry") is False
        and policy.get("PNG_selection", {}).get("policy") == PNG_INDEX_POLICY
        and policy.get("PNG_selection", {}).get("camera_indices_if_full450") == FULL450_PNG_CAMERA_INDICES
        and policy.get("PNG_selection", {}).get("selection_depends_on_outcome_or_failure_location") is False,
        "pre-episode run/output/PNG policy differs")
    gif = policy.get("GIF_selection", {})
    require(gif.get("regular_bin_seconds") == .5
        and gif.get("regular_camera_indices") == list(range(0, 450, 5))
        and gif.get("exact_last_camera_index") == 449
        and gif.get("resolved_full450_camera_indices_count") == 91
        and gif.get("resolved_full450_playback_duration_ms") == 45_000
        and gif.get("failed_tail_cropped") is False
        and gif.get("selection_depends_on_outcome_or_failure_location") is False,
        "pre-episode whole-GIF policy differs")
    require(policy.get("publication_source_sha256") == expected_sources
        and caller.get("publication_source_sha256") == expected_sources,
        "caller and pre-episode publication source closure must be byte identical")
    publisher = policy.get("publisher", {})
    require(publisher.get("path") == str(Path(__file__).resolve())
        and publisher.get("sha256") == prior.digest(Path(__file__).resolve()),
        "frozen final publisher differs from prospective policy")
    require(policy.get("font_sha256") == caller.get("font_sha256"),
            "caller fonts differ from prospective policy")
    declaration = policy.get("run_declaration", {})
    declaration_path = previous.canonical(declaration.get("path", ""))
    require(declaration_path == RUN.with_name(RUN.name + "_plan.json"),
            "unexpected prospectively declared owner plan path")
    pins.file(declaration_path, declaration.get("sha256"))
    require(caller["run_files_sha256"].get("declaration.json") == declaration.get("sha256"),
            "executed declaration differs from the pre-episode plan")
    for key in ("known_DEV_site_reused",):
        require(policy.get(key) is True, "prospective known-DEV disclosure differs")
    for key in ("independent_validation", "full_RTE01_approved", "full_MAIN9_completion_claimed",
                "training_data_approved", "vehicle_control_approved", "model_promoted"):
        require(policy.get(key) is False, "prospective denial differs: " + key)
    return policy


def verify_callback_contract(plan):
    """HH_261004 - Refuse a gallery whose prospective measured-clock contract became an unmeasured V3 claim."""
    timing = plan.get("actual_callback_timing", {})
    require(timing.get("schema") == CALLBACK_SCHEMA and timing.get("attempts") == 1
        and timing.get("callback_receipt_JSON_maximum_bytes") == 16 * 1024**2
        and timing.get("callback_receipt_JSON_SHA_and_schema_and_array_bounds_required") is True
        and timing.get("generic_JSON_reader_limit_changed") is False
        and timing.get("original_same_recorder_bundle_provider_replaced") is False
        and timing.get("timed_policy_globals_bound_after_complete_preview_PI_activation") is True
        and timing.get("latest_camera_bundles_required") == 450
        and timing.get("latest_camera_callback_receipts_required") == 2700
        and timing.get("first_callback_to_plan_maximum_seconds") == .1
        and timing.get("first_callback_to_ACK_maximum_seconds") == .1
        and timing.get("actual_base_policy_observation_statement_count") == 3
        and timing.get("selected_source_join_fields") == ["role", "native_frame", "image_object_id", "source_image_timestamp_simulation_ns"]
        and all(timing.get(key) is True for key in (
            "repeated_role_frame_or_simstamp_callbacks_preserved", "selected_source_exactly_one_original_callback_required",
            "ambiguous_same_role_native_frame_object_receipts_denied", "source_actor_and_BGRA_SHA_required",
            "causal_history_joins_consumed_bundle_identity_and_SHA_only",
            "repeated_callbacks_with_different_pixels_permitted_only_as_distinct_unmodified_sources",
            "all_extra_bootstrap_or_unconsumed_callbacks_retained", "source_math_inverse_AST_required",
            "original_queue_put_argument_and_return_and_exception_preserved",
            "callback_to_policy_source_SHA_required", "every_causal_history_SHA_required",
            "fresh_and_held_and_teacher_and_prefill_separate"))
        and all(timing.get(key) is False for key in (
            "repeated_native_frame_or_simstamp_alone_is_image_identity", "earliest_or_latest_duplicate_receipt_selected_by_latency",
            "additional_queue_or_source_image_buffer", "input_control_or_inference_order_changed",
            "hardware_exposure_wall_clock_observed", "ten_hz_wall_rate_proven",
            "callback_receipt_file_IO_during_capture", "model_structure_changed", "model_weights_changed",
            "controller_math_changed", "decoder_gate_or_tracker_math_changed", "original_deadline_relaxed")),
        "exact prospective callback-to-plan contract required")
    native = timing.get("native_dispatch_timing", {})
    require(native.get("stages") == list(NATIVE_STAGES) and native.get("native_ticks_required") == 900
        and native.get("native_camera_ticks_required") == 450 and native.get("native_scalar_marks_required") == 7200
        and native.get("first_tick_frame_unknown_until_actual_world_tick_returns") is True
        and native.get("scalar_clocks_only_no_waits") is True
        and native.get("physics_control_RPC_or_source_order_changed") is False
        and native.get("external_CPU_load_or_hardware_exposure_causation_claimed") is False,
        "prospective original-order7200 native scalar stage contract required")


def verify_v2_failure_reference(plan, pins):
    """HH_261004 - Bind the unchanged genuine v2 FAIL and all three frozen source bytes, not a success substitute."""
    ref = plan.get("failed_callback45_v2_reference", {})
    require(ref.get("path") == str(V2_RUN) and ref.get("sha256") == V2_EVIDENCE_SHA
        and ref.get("source_sha256") == V2_SOURCE_SHA and ref.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and ref.get("overwritten") is False, "frozen callback45_v2 failure reference required")
    for name, sha in V2_EVIDENCE_SHA.items():
        pins.file(V2_RUN / name, sha)
    # HH_261004 - These source paths are externally fixed rather than resolved through a fixture workspace root.
    source_root = Path(__file__).resolve().parents[2]
    for name, sha in V2_SOURCE_SHA.items():
        pins.file(source_root / name, sha)
    old = pins.json(V2_RUN / "diagnostic_report.json")
    require(old.get("status") == ref["status"] and old.get("full_RTE01_approved") is False,
            "unchanged actual v2 failed diagnostic required")


def verify_exact_consumed_callback_objects(actual, audit, cameras, commands, wall):
    """HH_261004 - Independently join exact consumed source objects; never choose a favorable repeated frame clock."""
    roles = tuple(MODEL_CAMERA_ORDER)
    actors = {row["role"]: row["actor_id"] for row in actual["camera_bindings"]}
    require(tuple(sorted(actors)) == tuple(sorted(roles)) and len(set(actors.values())) == 6,
            "six distinct callback actor bindings required")
    identities = {}
    for index, row in enumerate(actual["callbacks"], 1):
        key = row["role"], row["native_frame"], row["image_object_id"]
        require(key not in identities and row["receipt_sequence"] == index and row["role"] in actors
            and row["actor_id"] == actors[row["role"]] and type(row["image_object_id"]) is int
            and type(row["native_frame"]) is int and type(row["source_image_timestamp_simulation_ns"]) is int
            and row["source_byte_count"] == 360 * 640 * 4
            and re.fullmatch(r"[0-9a-f]{64}", row["source_BGRA_encoding_prefixed_sha256"])
            and row["original_queue_put_calls"] == 1 and row["original_queue_put_returned"] is True
            and row["original_queue_put_error"] is None and row["receipt_error"] is None
            and 0 < row["callback_enter_monotonic_ns"] <= row["queue_put_started_monotonic_ns"]
                <= row["queue_put_finished_monotonic_ns"] <= row["source_SHA_finished_monotonic_ns"],
            "ambiguous or malformed original callback object receipt")
        identities[key] = row
    by_frame, by_stamp = {}, {}
    for camera, bound in zip(cameras, actual["bundles"]):
        frame, stamp = camera["frame"], int(round(camera["timestamp"] * 1.e9))
        require(frame == bound["native_frame"] and frame not in by_frame and stamp not in by_stamp
            and camera["camera_order"] == list(roles) and set(camera["source_timestamps"]) == set(roles)
            and tuple(source["role"] for source in bound["cameras"]) == roles,
            "complete original unique consumed six-camera bundle required")
        selected = []
        for source in bound["cameras"]:
            row = identities[(source["role"], frame, source["image_object_id"])]
            require(source["native_frame"] == frame and source["source_image_timestamp_simulation_ns"]
                == row["source_image_timestamp_simulation_ns"] == stamp
                == int(round(camera["source_timestamps"][source["role"]] * 1.e9))
                and row["callback_enter_monotonic_ns"] <= bound["bundle_binding_finished_monotonic_ns"],
                "exact selected callback frame/object/stamp differs")
            selected.append(row)
        by_frame[frame] = selected
        by_stamp[stamp] = selected
    groups = {}
    for marker in actual["policy_stage_marks"]:
        group = groups.setdefault(marker["frame_timestamp_simulation_ns"], {})
        require(marker["stage"] in POLICY_STAGES and marker["stage"] not in group
            and marker["original_policy_math_changed"] is False, "original unique policy-stage receipt required")
        group[marker["stage"]] = marker
    by_ack = {row["ACK_sequence"]: row for row in wall}
    require(len(by_ack) == len(wall) == 904 and len(groups) == len(commands), "all original904 ACK and fresh stage groups required")
    for measured, command in zip(audit["rows"], commands):
        selected, ack = by_frame[measured["inference_frame"]], by_ack[measured["ACK_sequence"]]
        stamp = selected[0]["source_image_timestamp_simulation_ns"]
        marks = groups[stamp]
        require(set(marks) == set(POLICY_STAGES) and len({row["policy_object_id"] for row in marks.values()}) == 1,
                "three marks of the same actual original policy required")
        inputs, nn, plan = (marks[name]["stage_finished_monotonic_ns"] for name in POLICY_STAGES)
        require(measured["selected_source_callback_receipt_sequences"] == [row["receipt_sequence"] for row in selected]
            and measured["selected_source_image_object_ids"] == [row["image_object_id"] for row in selected]
            and measured["first_callback_monotonic_ns"] == min(row["callback_enter_monotonic_ns"] for row in selected)
            and measured["last_callback_monotonic_ns"] == max(row["callback_enter_monotonic_ns"] for row in selected)
            and measured["source_image_timestamp_simulation_ns"] == stamp
            and measured["input_ready_monotonic_ns"] == inputs and measured["raw_nn_finished_monotonic_ns"] == nn
            and measured["decoded_plan_ready_monotonic_ns"] == plan
            and measured["ACK_finished_monotonic_ns"] == ack["ACK_finished_monotonic_ns"]
            and ack["inference_frame"] == ack["command_frame"] == command["inference_frame"]
            and ack["ACK_server_accepted"] is True and ack["control_source"] == "learned_trajectory_tracker"
            and ack["proposal_started_monotonic_ns"] <= inputs <= nn <= plan
                <= ack["proposal_finished_monotonic_ns"] <= ack["ACK_started_monotonic_ns"] <= ack["ACK_finished_monotonic_ns"],
            "displayed clocks are not the exact original selected source/policy/ACK receipts")
        history = command["owned_camera_history"]["source_camera_frames"]
        require(len(history) == 5 and history[-1]["timestamp_ns"] == stamp
            and all(a["timestamp_ns"] < b["timestamp_ns"] for a, b in zip(history, history[1:])),
            "all five original causal consumed history bundles required")
        for past in history:
            require(len(past["camera_sha256"]) == 6 and past["timestamp_ns"] in by_stamp
                and past["camera_sha256"] == [row["source_BGRA_encoding_prefixed_sha256"] for row in by_stamp[past["timestamp_ns"]]],
                "consumed historical source BGRA SHA differs from actual input")


def native_interval_row(index, state, group, camera):
    """HH_261004 - Derive every displayed native interval only from the original scalar boundary clocks."""
    get = lambda name: group[name]["stage_finished_monotonic_ns"]
    result = dict(tick_index=index, native_frame=state["frame"], capture_phase=state["capture_phase"],
        world_tick_seconds=(get("world_tick_returned") - get("world_tick_started")) / 1.e9,
        snapshot_binding_seconds=(get("snapshot_bound") - get("world_tick_returned")) / 1.e9,
        snapshot_to_actor_state_seconds=(get("actor_state_ready") - get("snapshot_bound")) / 1.e9,
        actor_state_to_agent_branch_seconds=(get("agent_branch_started") - get("actor_state_ready")) / 1.e9,
        actual_agent_or_warmup_brake_branch_seconds=(get("agent_branch_finished") - get("agent_branch_started")) / 1.e9,
        agent_branch_to_native_state_finished_seconds=(get("native_state_finished") - get("agent_branch_finished")) / 1.e9)
    if camera:
        result.update(native_state_to_bundle_call_seconds=(get("exact_bundle_started") - get("native_state_finished")) / 1.e9,
            actual_exact_bundle_provider_seconds=(get("exact_bundle_finished") - get("exact_bundle_started")) / 1.e9)
    return result


def verify_native_stage_closure(actual, diagnostic, cameras, states, wall):
    """HH_261004 - A native7200 PASS requires real frame-bound scalar rows; absent or failed native proof cannot be promoted."""
    native = diagnostic.get("native_pre_delivery_timing", {})
    if diagnostic.get("native_pre_delivery_stage_closure_passed") is not True:
        require(diagnostic.get("callback_straight_motion_body_policy_wall100ms_passed") is not True,
                "missing native stage closure cannot inherit callback PASS")
        return
    marks = actual.get("native_stage_marks", [])
    require(len(marks) == 7200 and states is not None and len(states) == 900
        and native.get("native_tick_count") == 900 and native.get("native_camera_tick_count") == 450
        and native.get("native_scalar_mark_count") == 7200
        and native.get("native_pre_delivery_stage_closure_passed") is True
        and native.get("all_native_stage_receipts_retained") is True
        and native.get("native_frame_invented_before_tick_return") is False
        and native.get("native_mark_clock") == "time.perf_counter_ns_monotonic"
        and native.get("physics_control_RPC_order_changed") is False
        and native.get("hardware_exposure_to_ACK_proven") is False
        and native.get("CPU_scheduling_or_external_load_cause_proven") is False,
        "complete original900 tick7200 scalar closure required")
    groups, camera_frames = {}, {row["frame"] for row in cameras}
    for row in marks:
        group = groups.setdefault(row["tick_index"], {})
        require(row["stage"] in NATIVE_STAGES and row["stage"] not in group
            and row["original_native_order_or_math_changed"] is False
            and type(row["stage_finished_monotonic_ns"]) is int and row["stage_finished_monotonic_ns"] > 0,
            "original unique native scalar mark required")
        group[row["stage"]] = row
    require(set(groups) == set(range(900)), "all900 actual native tick indices required")
    expected_native, by_frame = [], {}
    for index, state in enumerate(states):
        group, frame, phase = groups[index], state["frame"], state["capture_phase"]
        require(set(group) == set(NATIVE_STAGES if frame in camera_frames else NATIVE_STAGES[:7])
            and group["world_tick_started"]["native_frame"] is None
            and all(row["capture_phase"] == phase for row in group.values())
            and all(row["native_frame"] == frame for name, row in group.items() if name != "world_tick_started")
            and all(group[a]["stage_finished_monotonic_ns"] <= group[b]["stage_finished_monotonic_ns"]
                for a, b in zip(NATIVE_STAGES[:7], NATIVE_STAGES[1:7]))
            and all(group[name]["agent_run_step_invoked"] is (phase == "driving")
                for name in ("agent_branch_started", "agent_branch_finished")),
            "native scalar frame/phase/branch/order differs from actual state")
        if frame in camera_frames:
            require(group["native_state_finished"]["stage_finished_monotonic_ns"]
                <= group["exact_bundle_started"]["stage_finished_monotonic_ns"]
                <= group["exact_bundle_finished"]["stage_finished_monotonic_ns"], "original native-to-bundle order differs")
        require(frame not in by_frame, "actual native frame cannot be reused")
        by_frame[frame] = group
        expected_native.append(native_interval_row(index, state, group, frame in camera_frames))
    by_ack, expected_fresh = {row["ACK_sequence"]: row for row in wall}, []
    for receipt in diagnostic["actual_callback_timing"]["rows"]:
        frame = receipt["inference_frame"]
        group, ack = by_frame[frame], by_ack[receipt["ACK_sequence"]]
        get = lambda name: group[name]["stage_finished_monotonic_ns"]
        delivery = ack["observed_bundle"]["exact_bundle_delivery_monotonic_ns"]
        require(get("exact_bundle_started") <= delivery <= get("exact_bundle_finished") <= ack["proposal_started_monotonic_ns"],
                "actual native provider/delivery/proposal boundary differs")
        first, last = receipt["first_callback_monotonic_ns"], receipt["last_callback_monotonic_ns"]
        expected_fresh.append(dict(native_frame=frame, ACK_sequence=receipt["ACK_sequence"],
            first_callback_before_world_tick_started=first < get("world_tick_started"),
            last_callback_before_world_tick_returned=last < get("world_tick_returned"),
            world_tick_return_minus_last_callback_seconds=(get("world_tick_returned") - last) / 1.e9,
            bundle_call_start_minus_last_callback_seconds=(get("exact_bundle_started") - last) / 1.e9,
            last_callback_to_original_bundle_delivery_seconds=(delivery - last) / 1.e9,
            original_bundle_delivery_to_input_ready_seconds=(receipt["input_ready_monotonic_ns"] - delivery) / 1.e9,
            actual_input_ready_to_NN_finished_seconds=(receipt["raw_nn_finished_monotonic_ns"] - receipt["input_ready_monotonic_ns"]) / 1.e9,
            actual_NN_finished_to_decoded_plan_seconds=(receipt["decoded_plan_ready_monotonic_ns"] - receipt["raw_nn_finished_monotonic_ns"]) / 1.e9,
            first_callback_to_decoded_plan_seconds=receipt["callback_first_to_plan_seconds"],
            first_callback_to_ACK_seconds=receipt["callback_first_to_ACK_seconds"]))
    require(native.get("full_native_rows") == expected_native and native.get("all_fresh_critical_path_rows") == expected_fresh,
            "all900 native intervals and every fresh critical path must match original scalar clocks")


def verify_callback_outcome(diagnostic, pins, episode, cameras, commands, states=None):
    """HH_261004 - Recompute displayed measured maxima while retaining failed or absent callback evidence as FAIL."""
    require(diagnostic.get("hardware_exposure_to_ACK_proven") is False
        and diagnostic.get("ten_hz_wall_rate_proven") is False,
        "simulation callbacks cannot certify hardware exposure or real-wall ten Hz")
    passed = diagnostic.get("callback_straight_motion_body_policy_wall100ms_passed") is True
    if not passed:
        require(diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR",
                "failed callback or driving gates require an explicit failed outcome")
    audit = diagnostic.get("actual_callback_timing", {})
    measured_bound = all(audit.get(key) is True for key in (
        "callback_receipt_identity_and_SHA_passed", "callback_plan_ACK_chain_passed",
        "exact_consumed_image_object_join_passed", "consumed_history_bundle_identity_and_SHA_passed"))
    if not measured_bound:
        require(not passed and not audit.get("rows") and not audit.get("metrics"),
                "unbound source objects cannot display measured callback maxima or PASS")
        return
    if passed:
        require(all(diagnostic.get(key) is True for key in (
            "archived_v3_evidence_preserved", "archived_callback45_v1_failure_preserved",
            "archived_callback45_v2_failure_preserved", "native_pre_delivery_stage_closure_passed",
            "callback_receipt_identity_and_SHA_passed", "callback_plan_ACK_chain_passed",
            "fresh_callback_to_plan_wall100ms_passed", "fresh_callback_to_ACK_wall100ms_passed",
            "actual_six_camera_callback_observed")),
            "measured callback PASS cannot omit original failures, native clocks or plan/ACK proof")
    path = episode / "callback_timing_receipts.json"
    require(str(path) in pins.values, "actual callback receipts require whole-archive pins")
    # HH_261004 - Bound externally pinned callback bytes before parsing the separate receipt ABI.
    payload = pins.payload(path)
    require(0 < len(payload) <= 16 * 1024**2, "callback-only16MiB publication bound required")
    actual = previous.strict_json(payload)
    # HH_261004 - Publication independently binds the new callback-only ABI without importing Torch or the live owner.
    require(type(actual.get("callback_count")) is int and actual["callback_count"] <= 12000
        and len(actual.get("policy_stage_marks", [])) <= 1350
        and len(actual.get("policy_stage_marks", [])) % 3 == 0
        and len(actual.get("native_stage_marks", [])) <= 8100,
        "bounded full callback and stage census required")
    readback = diagnostic.get("callback_receipt_JSON_readback", {})
    # HH_261004 - A later native diagnostic failure may omit this owner field; externally pinned raw bytes still bind valid measured FAIL maxima.
    if readback or passed:
        require(readback.get("path") == str(path) and readback.get("sha256") == pins.values[str(path)]
            and type(readback.get("actual_bytes")) is int
            and readback["actual_bytes"] == len(payload)
            and readback.get("maximum_bytes") == 16 * 1024**2
            and readback.get("strict_schema_and_array_bounds_passed") is True
            and readback.get("generic_JSON_reader_limit_changed") is False,
            "exact callback-only actual readback receipt required")
    require(actual.get("schema") == CALLBACK_SCHEMA and actual.get("errors") == []
        and actual.get("clock") == "time.perf_counter_ns_monotonic"
        and actual.get("hardware_exposure_clock_observed") is False
        and actual.get("all_callbacks_including_bootstrap_or_unconsumed_preserved") is True
        and actual.get("source_math_inverse_AST_verified") is True
        and actual.get("callback_count") == len(actual.get("callbacks", [])) >= 2700
        and len(actual.get("bundles", [])) == len(cameras) == 450
        and len(actual.get("camera_bindings", [])) == 6
        and actual.get("inflight_callbacks") == 0 and actual.get("scalar_receipt_limit_exceeded") is False
        and actual.get("hardware_exposure_to_ACK_proven") is False
        and actual.get("ten_hz_wall_rate_proven") is False
        and audit.get("hardware_exposure_to_ACK_proven") is False
        and audit.get("ten_hz_wall_rate_proven") is False,
        "complete genuine measured callback receipts required for a PASS")
    fresh = [row for row in commands if row.get("learned_control_proposed") is True
             and row.get("held_between_camera_frames") is False]
    measured = audit.get("rows", [])
    require(len(measured) == len(fresh) == audit.get("fresh_NN_plan_count") > 0
        and audit.get("selected_callback_count") == 2700
        and audit.get("callback_count") == actual["callback_count"]
        and audit.get("extra_unselected_callback_count") == actual["callback_count"] - 2700,
        "all actual fresh plans and extra callbacks must remain in the displayed denominator")
    wall = pins.rows(episode / "wall_timing.jsonl")
    verify_exact_consumed_callback_objects(actual, audit, cameras, fresh, wall)
    if states is None and diagnostic.get("native_pre_delivery_stage_closure_passed") is True:
        states = pins.rows(episode / "states.jsonl")
    verify_native_stage_closure(actual, diagnostic, cameras, states, wall)
    for row, proposal in zip(measured, fresh):
        require(row["inference_frame"] == proposal["inference_frame"]
            and row["ACK_sequence"] == proposal["ack_receipt_sequence"],
            "displayed callback plan row differs from its actual original proposal")
        for field, finish in (("callback_first_to_plan_seconds", "decoded_plan_ready_monotonic_ns"),
                              ("callback_first_to_ACK_seconds", "ACK_finished_monotonic_ns")):
            expected = (row[finish] - row["first_callback_monotonic_ns"]) / 1.e9
            require(type(expected) in (int, float) and math.isfinite(expected)
                and expected >= 0 and row[field] == expected and (not passed or expected <= .1),
                "actual first callback to plan or ACK contradicts its unchanged100ms outcome")
    for field in ("callback_first_to_plan_seconds", "callback_first_to_ACK_seconds"):
        statistics = audit.get("metrics", {}).get(field, {})
        require(statistics.get("count") == len(measured)
            and statistics.get("maximum") == max(row[field] for row in measured),
            "displayed measured callback maximum or denominator differs")
    for field, gate in (("callback_first_to_plan_seconds", "fresh_callback_to_plan_wall100ms_passed"),
                        ("callback_first_to_ACK_seconds", "fresh_callback_to_ACK_wall100ms_passed")):
        expected = all(row[field] <= .1 for row in measured)
        require(audit.get(gate) is expected and diagnostic.get(gate) is expected,
                "failed100ms tail cannot be omitted or relabelled PASS")


def load_archive(caller_path, caller_sha, *, decode=False):
    """HH_261004 - Read one immutable actual archive; a short capture remains a visible FAIL denominator."""
    require("torch" not in sys.modules and "carla" not in sys.modules,
            "publication cannot run inference or a simulator")
    require(prior.digest(Path(previous.__file__)) == PREVIOUS_SHA, "reviewed Town03 renderer changed")
    pins = previous.Pins()
    pins.file(caller_path, caller_sha)
    caller = pins.json(caller_path)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("attempts") == 1
        and caller.get("PNG_index_policy") == PNG_INDEX_POLICY
        and caller.get("PNG_percent_numerators") == PNG_PERCENT_NUMERATORS
        and caller.get("expected_full_PNG_camera_indices") == FULL450_PNG_CAMERA_INDICES,
        "prospective single-attempt fixed PNG declaration required")
    run = previous.canonical(caller["run_root"])
    output = output_for(run)
    require(caller.get("output") == str(output), "unknown or crossed publication output")
    require(set(caller["run_files_sha256"]) == previous.inventory(run),
            "entire externally pinned run archive required")
    for name, expected in caller["run_files_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and relative.as_posix() == name,
                "safe relative run member required")
        pins.file(run / relative, expected)
    sources = publication_sources()
    require(set(caller["publication_source_sha256"]) == {str(path.relative_to(ROOT)) for path in sources},
            "complete publication source closure required")
    policy = _verify_prospective_policy(caller, pins, sources)
    for path in sources:
        pins.file(path, caller["publication_source_sha256"][str(path.relative_to(ROOT))])
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"):
        path = Path("/usr/share/fonts/truetype/dejavu") / name
        pins.file(path, caller["font_sha256"][str(path)])

    plan, owner, diagnostic = [pins.json(run / name)
        for name in ("declaration.json", "owner_result.json", "diagnostic_report.json")]
    require(plan.get("schema") == OWNER_SCHEMA and plan.get("output") == str(run)
        and plan.get("town") == "Town03" and plan.get("scenario") == "straight"
        and plan.get("dataset_split") == "val" and plan.get("known_DEV_site_reused") is True
        and plan.get("independent_validation") is False and plan.get("attempts") == 1
        and plan.get("automatic_retry") is False and plan.get("total_capture_seconds") == 45
        and plan.get("maximum_native_state_count") == 900 and plan.get("maximum_camera_bundle_count") == 450
        and plan.get("full_original_navigation_points") == 211
        and plan.get("original_navigation_sha256") == ROUTE_SHA
        and plan.get("optimizer_updates") == 6000 and plan.get("speed_preview_s") == .5
        and plan.get("controller_target_cap_mps") == 2.4
        and plan.get("full_RTE01_approved") is False and plan.get("training_data_approved") is False
        and plan.get("vehicle_control_approved") is False and plan.get("model_promoted") is False,
        "exact fixed GOOD6000 known-DEV straight contract required")
    verify_callback_contract(plan)
    verify_v2_failure_reference(plan, pins)
    green = plan.get("owned_green_signal_fixture", {})
    require(green.get("original_own_world_traffic_actors") == "ALL" and green.get("state") == "Green"
        and green.get("frozen") is True and green.get("ignore_traffic_lights") is False
        and green.get("all_original_hazard_detectors_and_emergency_returns_unchanged") is True
        and green.get("closed_course_basic_straight_only_not_traffic_rule_performance") is True
        and plan.get("actual_scoped_camera_and_ack_transport_late_bound_before_capture") is True,
        "owned signal and actual camera/ACK disclosures required")
    # HH_261004 - Bind this separate gallery to the prospectively declared V3 recording and topology correction.
    start = plan.get("authenticated_start_corridor", {})
    require(plan.get("actual_deferred_JPEG_after_ACK_required") is True
        and start.get("upstream_lane") == [67, 0, 2]
        and start.get("enabled_only_when_body_arc_lower_bound_negative") is True
        and start.get("original_navigation_bytes_unchanged") is True,
        "exact V3 after-ACK and start-corridor declaration required")
    # HH_261004 - Require the narrow owner-schema repair and reject unrelated model or gate interventions.
    admission = plan.get("fixture_owner_schema_admission", {})
    require(admission.get("declared_owner_schema") == OWNER_SCHEMA
        and all(admission.get(key) is True for key in (
            "actual_fixture_module_global_required", "inherited_v2_deferred_recording_unchanged",
            "inherited_authenticated_start_corridor_unchanged"))
        and all(admission.get(key) is False for key in (
            "model_weights_changed", "controller_changed", "policy_changed", "route_or_map_changed",
            "camera_or_ACK_changed", "speed_or_acceleration_gate_changed", "latency_gate_changed",
            "body_area_or_inset_gate_changed")),
        "exact V3 fixture-only owner admission required")
    require(owner.get("declaration_sha256") == caller["run_files_sha256"]["declaration.json"]
        and diagnostic.get("schema") == OWNER_SCHEMA + ".diagnostic_report"
        and diagnostic.get("original_owner") == owner, "owner/diagnostic/declaration identity differs")
    for key in ("full_MAIN9_completion_claimed", "full_RTE01_approved", "training_data_approved",
                "vehicle_control_approved", "model_promoted"):
        require(diagnostic.get(key) is False, "original denial cannot be promoted: " + key)
    require(diagnostic.get("all_available_raw_files_retained_and_hashed") is True
        and diagnostic.get("raw_before_after_unchanged") is True,
        "original raw archive must remain complete and byte unchanged")
    _pin_declared_sources(pins, run, plan)

    episode = run / ("episode" if (run / "episode").is_dir() else "episode.partial")
    require(episode.is_dir(), "retained actual episode required")
    raw_names = {str(episode / name) for name in previous.inventory(episode)}
    require(set(diagnostic.get("raw_source_sha256", {})) == raw_names,
            "all actual and failed-tail raw files must remain in the diagnostic")
    for name, expected in diagnostic["raw_source_sha256"].items():
        require(pins.values.get(name) == expected, "raw snapshot differs from caller archive")
    manifest, route = pins.json(episode / "manifest.json"), pins.json(episode / "route.json")
    require(pins.values[str(episode / "route.json")] == ROUTE_SHA
        and route.get("town") == "Town03" and route.get("scenario") == "straight"
        and len(route.get("route", ())) == 211
        and manifest.get("status") == diagnostic.get("original_capture_status")
        and manifest.get("training_data_approved") is False
        and manifest.get("vehicle_control_approved") is False,
        "exact unchanged Town03 straight route and original outcome required")
    states, cameras, receipts = [pins.rows(episode / name)
        for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl")]
    summary = pins.json(episode / "learned_summary.json")
    unlogged_preNN = not (episode / "learned_control.jsonl").exists()
    if unlogged_preNN:
        counts = summary.get("counts", {})
        require(summary.get("handoff_occurred") is False
            and all(type(counts.get(key)) is int and counts[key] == 0 for key in
                ("inference_count", "prefill_count", "learned_commands_acknowledged", "emergency_commands"))
            and all(state.get("research_control_source") in (None, "expert_startup")
                    and state.get("research_inference_frame") is None for state in states),
            "missing proposal journal contradicts zero-NN evidence")
        commands = []
    else:
        commands = pins.rows(episode / "learned_control.jsonl")
    body = pins.json(run / "derived_body/summary.json")
    geometry = pins.rows(run / "derived_body/all_native_boxes_and_sweeps.jsonl.gz")
    require(body == diagnostic.get("independent_body") and body.get("all_native_boxes") == len(states)
        and body.get("all_snapshot_linear_sweeps") == len(states) - 1 and body.get("lane_inset_m") == .1,
        "all actual body and 10cm inset rows required")
    by_frame = verify_records(states, cameras, commands, receipts, geometry, unlogged_preNN=unlogged_preNN)
    require(diagnostic.get("actual_native_state_count") == len(states)
        and diagnostic.get("actual_camera_bundle_count") == len(cameras)
        and diagnostic.get("actual_ACK_receipt_count") == len(receipts),
        "actual state/camera/ACK denominators differ")
    full = len(states) == 900 and len(cameras) == 450
    phases = [state.get("capture_phase") for state in states]
    first_driving = next((index for index, phase in enumerate(phases) if phase == "driving"), len(phases))
    require(all(phase == "stationary_warmup" for phase in phases[:first_driving])
        and all(phase == "driving" for phase in phases[first_driving:]),
        "actual warmup/driving phase order required")
    if full:
        result = manifest.get("result", {})
        require(manifest.get("status") == "failed" and manifest.get("error") == EXPECTED_BUDGET_ERROR
            and first_driving == 24 and len(phases) - first_driving == 876
            and manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
                .get("stationary_tail", {}).get("scheduled_ticks") == 0
            and "goal_reached" not in result and result.get("state_count") == 900
            and result.get("camera_anchor_count") == 450
            and diagnostic.get("bounded_capture_complete") is True
            and diagnostic.get("fixed45_full_denominator_passed") is True,
            "full fixed45 denominator must retain the exact expected budget tail")
        require(diagnostic.get("all_failed_tail_retained") is True,
                "complete failed-budget tail must remain explicitly retained")
    else:
        require(manifest.get("status") == "failed" and type(manifest.get("error")) is str
            and bool(manifest["error"]) and diagnostic.get("original_capture_error") == manifest["error"]
            and diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
            and diagnostic.get("bounded_capture_complete") is False
            and diagnostic.get("fixed45_full_denominator_passed") is False
            and diagnostic.get("learned_driving_screen_passed") is False
            and diagnostic.get("straight_motion_body_policy_wall100ms_passed") is False,
            "a retained short capture cannot become a straight PASS")
    images = []
    for camera in cameras:
        for name in visual.CAMERAS:
            path = previous.canonical(prior.safe_image(episode, camera["images"][name]))
            require(str(path) in pins.values, "real camera image lacks the caller pin")
            if decode:
                with Image.open(path) as image:
                    require(image.format == "JPEG" and image.size == (640, 360), "real JPEG ABI differs")
                    image.load()
            images.append(path)
    require(len(images) == len(set(images)) == 6 * len(cameras), "six distinct real cameras per bundle required")
    verify_callback_outcome(diagnostic, pins, episode, cameras, commands, states)
    trial = dict(trial_index=0,
        original_driving_screen_passed=diagnostic.get("learned_driving_screen_passed") is True,
        fresh_observed_delivery_to_ACK100ms_passed=diagnostic.get("observed_bundle_to_ACK_wall100ms_passed") is True)
    bundle = dict(episode=episode, states=states, cameras=cameras, commands=commands,
        receipts=receipts, by_frame=by_frame, geometry={row["frame"]: row for row in geometry},
        route=route["route"], trial=trial, diagnostic=diagnostic, output=output,
        unlogged_preNN=unlogged_preNN, original_learned_summary=summary, full_fixed45=full,
        prospective_publication_policy=policy,
        prospective_publication_policy_sha256=caller["prospective_publication_policy"]["sha256"])
    bundle["control_census"] = previous.control_census(bundle)
    require(bundle["control_census"]["observed_prior_learned_native_effect_count"] > 0
        or diagnostic.get("straight_motion_body_policy_wall100ms_passed") is False,
        "zero observed NN effect cannot be a driving PASS")
    pins.recheck()
    require(previous.inventory(run) == set(caller["run_files_sha256"]), "run membership changed during publication")
    return caller, bundle, pins


def outcome_text(bundle):
    census, diagnostic = bundle["control_census"], bundle["diagnostic"]
    if not census["observed_prior_learned_native_effect_count"]:
        return "NO observed NN effect / setup failure", "NN 제어 효과 0개 — 모델 직진 성능 판정 불가"
    if diagnostic.get("callback_straight_motion_body_policy_wall100ms_passed") is True:
        return "PASS known-DEV precursor / NOT RTE01 approval", "known-DEV 예비 기준 PASS(기능 승인 아님)"
    return "FAIL / all available failed tail retained", "FAIL(실패 구간 전부 보존)"


def render(bundle, index):
    """HH_261004 - Reuse six real cameras and the vehicle-centered route/travel/prior-NN-plan panel."""
    canvas = previous.renderer_call(previous.core.render, bundle, index)
    draw = ImageDraw.Draw(canvas)
    verdict, _ = outcome_text(bundle)
    draw.rectangle((0, 0, 1699, 91), fill=visual.BG)
    draw.text((24, 18), "Actual Town03 known-DEV STRAIGHT | " + verdict,
              fill=visual.WHITE, font=visual.font(23, True))
    draw.text((24, 63), "GOOD6000 / CPU4+1 / PI2.40 / preview0.5 / actual callback-to-plan clocks / fixed45",
              fill=visual.MUTED, font=visual.font(18))
    draw.rectangle((0, 801, 1699, 859), fill=visual.BG)
    draw.text((24, 807),
        f"Whole actual interval: {len(bundle['states'])}native / {len(bundle['cameras'])}six-camera / "
        f"{bundle['control_census']['observed_prior_learned_native_effect_count']} observed NN effects | MAIN0/9 SUB0/30",
        fill=visual.WHITE, font=visual.font(16))
    draw.text((24, 835),
        "Actual callback timing is NOT hardware exposure or wall 10Hz proof | recorded CARLA diagnostic, NOT Autoware/RViz",
        fill=visual.MUTED, font=visual.font(16))
    return canvas


def markdown(bundle, pngs):
    diagnostic, census = bundle["diagnostic"], bundle["control_census"]
    _, verdict = outcome_text(bundle)
    span = bundle["states"][-1]["timestamp"] - bundle["states"][0]["timestamp"]
    text = "# 실제 Town03 직진과 카메라 수신 시간 측정\n\n"
    text += verdict + "입니다. 독립 평가나 정식 RTE-01 승인이 아니며 현재 메인 0/9·하위 0/30은 변하지 않습니다.\n\n"
    audit = diagnostic.get("actual_callback_timing", {})
    metrics = audit.get("metrics", {})
    def milliseconds(field):
        value = metrics.get(field, {}).get("maximum")
        return "미확인" if value is None else f"{1000 * value:.3f}ms"
    text += ("이번에는 실제 카메라 callback의 첫 문장부터 계획 생성·명령 ACK까지 같은 단조 시계로 기록했습니다. "
        f"최대 수신→계획 {milliseconds('callback_first_to_plan_seconds')}, "
        f"수신→ACK {milliseconds('callback_first_to_ACK_seconds')}입니다. "
        "센서 하드웨어 노출 시각이나 실제 벽시계 10Hz 보장을 측정한 것은 아닙니다.\n\n")
    if audit:
        text += (f"정확한 영상 객체 연결: 수신 {audit.get('callback_count', '미확인')}개, "
            f"실제 사용 {audit.get('selected_callback_count', '미확인')}개, "
            f"미사용 보존 {audit.get('extra_unselected_callback_count', '미확인')}개입니다. "
            "같은 프레임의 반복 callback을 빠르거나 느린 시각으로 선택하지 않고, "
            "실제 소비한 역할·프레임·객체ID·시뮬레이션 시각·센서·BGRA SHA로 연결했습니다.\n\n")
    if diagnostic.get("native_pre_delivery_stage_closure_passed") is True:
        text += "물리 tick·상태 처리·기존 위험 확인·영상 묶음 경계의 원래 순서 7,200개 시계(900 tick)를 확인했습니다. 다른 프로세스 부하가 원인이라고 입증한 것은 아닙니다.\n\n"
    else:
        text += "native 단계 전체 계측은 아직 완전한 증거로 닫히지 않았습니다. 이 실패가 이미 검증된 callback→계획·ACK 수치를 미측정으로 바꾸지는 않으며, 전체 PASS로 승격하지도 않습니다.\n\n"
    if not census["observed_prior_learned_native_effect_count"]:
        text += "실제 NN 제어 효과가 관측되기 전에 끝났으므로 모델 직진 성공·실패로 세지 않습니다.\n\n"
    text += (f"실제 상태 {len(bundle['states'])}개, 6카메라 묶음 {len(bundle['cameras'])}개를 전부 확인했습니다. "
        f"최초→마지막 상태 간격은 {span:.2f}초입니다. GIF는 고정 0.5초 표본과 정확한 마지막 실제 카메라를 포함하고, "
        "PNG는 실행 전 선언한 최초/25%/50%/75%/마지막 규칙만 사용합니다. 짧게 끝났다면 전체 가용 실패 꼬리를 유지하며 PASS로 바꾸지 않습니다.\n\n")
    errors = {key: diagnostic[key] for key in ("original_capture_error", "diagnostic_error", "callback_timing_diagnostic_error") if diagnostic.get(key)}
    if errors:
        text += "원본 오류(그대로 보존): `" + json.dumps(errors, ensure_ascii=False).replace("`", "'") + "`\n\n"
    text += "![실제 전체 기록](full_actual_six_cameras_2hz.gif)\n\n"
    for name in pngs:
        text += f"![{name}]({name})\n\n"
    text += ("지도 패널은 모든 장면에서 측정 차량을 중심에 두고 파랑 원 경로, 초록 실제 주행 궤적, "
        "보라색 실제 이전 ACK가 적용된 NN 계획만 표시합니다. 여섯 영상은 기록된 CARLA JPEG입니다. "
        "이는 기록 기반 진단 렌더링이며 실제 Autoware 또는 RViz 화면이 아닙니다. "
        "원본 판정·SHA·표본 인덱스는 [publication_provenance.json](publication_provenance.json)에 보존합니다.\n")
    return text


def publish(args):
    caller, bundle, pins = load_archive(
        previous.canonical(args.caller_pins), args.caller_pins_sha256, decode=args.render_all_media)
    output = bundle["output"]
    require(not output.exists() and not output.is_symlink(), "fresh create-only gallery output required")
    gif_indices = camera_indices(len(bundle["cameras"]))
    durations = playback(bundle, gif_indices)
    pngs = png_indices(len(bundle["cameras"]))
    if args.validate_only:
        return dict(validated=True, native_count=len(bundle["states"]), camera_count=len(bundle["cameras"]),
            full_fixed45=bundle["full_fixed45"], GIF_frames=len(gif_indices), PNG_camera_indices=pngs,
            rendered=False, output_created=False)
    ancestor = next(path for path in output.parents if path.exists())
    require(shutil.disk_usage(ancestor).free >= 5 * 1024**3 + 256 * 1024**2,
            "5GiB plus 256MiB publication reserve required")
    pins.recheck()
    output.mkdir(parents=True, exist_ok=False)
    frames = [render(bundle, index) for index in gif_indices]
    gif = output / "full_actual_six_cameras_2hz.gif"
    with gif.open("xb") as stream:
        frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:],
                       duration=durations, loop=0, optimize=False)
    with Image.open(gif) as saved:
        require(saved.n_frames == len(gif_indices), "whole actual GIF interval required")
        actual = []
        for index in range(saved.n_frames):
            saved.seek(index)
            actual.append(saved.info["duration"])
        require(actual == durations, "whole actual GIF timing changed")
    media = [gif]
    for name, index in pngs.items():
        path = output / name
        with path.open("xb") as stream:
            render(bundle, index).save(stream, format="PNG")
        media.append(path)
    displayed_gif = [previous.renderer_call(previous.core.displayed_receipt, bundle, index)
                     for index in gif_indices]
    displayed_png = {name: previous.renderer_call(previous.core.displayed_receipt, bundle, index)
                     for name, index in pngs.items()}
    displayed_plan_count = sum(row["prior_learned_plan_drawn"] for row in displayed_gif)
    provenance = dict(schema=SCHEMA, caller_sha256=args.caller_pins_sha256,
        prospective_publication_policy_path=str(POLICY_PATH),
        prospective_publication_policy_sha256=bundle["prospective_publication_policy_sha256"],
        source_and_input_sha256=dict(pins.values), actual_output=str(output),
        original_diagnostic=bundle["diagnostic"], original_learned_summary=bundle["original_learned_summary"],
        PNG_index_policy=PNG_INDEX_POLICY, PNG_percent_numerators=PNG_PERCENT_NUMERATORS,
        expected_full_PNG_camera_indices=FULL450_PNG_CAMERA_INDICES,
        resolved_actual_PNG_camera_indices=pngs, GIF_camera_indices=gif_indices,
        GIF_duration_ms=durations, GIF_total_duration_ms=sum(durations),
        displayed_GIF_frames=displayed_gif, displayed_PNG_frames=displayed_png,
        full_fixed45=bundle["full_fixed45"], failed_tail_cropped=False,
        exact_last_original_camera_included=True, last_native_frame=bundle["states"][-1]["frame"],
        last_camera_frame=bundle["cameras"][-1]["frame"], native_terminal_frame_without_camera_not_fabricated=True,
        six_real_recorded_cameras=True, measured_vehicle_center_fixed_in_all_scenes=True,
        route_and_traveled_trajectory_displayed=True, prior_applied_NN_plan_display_rule_enabled=True,
        actual_prior_applied_NN_plan_displayed=displayed_plan_count > 0,
        actual_prior_applied_NN_plan_GIF_frame_count=displayed_plan_count, prior_ACK_any_source=True,
        emergency_clears_purple_plan=True, teacher_startup_not_counted_as_model_control=True,
        recorded_episode_model_effect_count=bundle["control_census"]["observed_prior_learned_native_effect_count"],
        publication_executed_model_inference=False, publication_started_simulator=False,
        diagnostic_rendering_not_genuine_Autoware_RViz=True, autoware_UI_captured=False,
        actual_callback_timing= bundle["diagnostic"].get("actual_callback_timing"),
        native_pre_delivery_timing=bundle["diagnostic"].get("native_pre_delivery_timing"),
        native_pre_delivery_stage_closure_passed=bundle["diagnostic"].get("native_pre_delivery_stage_closure_passed") is True,
        archived_callback45_v2_failure_preserved=bundle["diagnostic"].get("archived_callback45_v2_failure_preserved") is True,
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        synthetic_driving_image_generated=False, known_DEV_site_reused=True, independent_validation=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        model_promoted=False, vehicle_control_approved=False, training_data_approved=False,
        media_sha256={path.name: prior.digest(path) for path in media})
    pins.recheck()
    require(previous.inventory(Path(caller["run_root"])) == set(caller["run_files_sha256"]),
            "run membership changed during rendering")
    with (output / "publication_provenance.json").open("x") as stream:
        json.dump(provenance, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (output / "README.md").open("x") as stream:
        stream.write(markdown(bundle, pngs))
    pins.recheck()
    return dict(output=str(output), GIF_count=1, PNG_count=5, GIF_frames=len(gif_indices),
        original_raw_rewritten=False, outcome=bundle["diagnostic"]["status"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True)
    parser.add_argument("--caller-pins-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--render-all-media", action="store_true")
    args = parser.parse_args(argv)
    require(args.validate_only != args.render_all_media,
            "choose validation-only or explicit root-authorized media rendering")
    print(json.dumps(publish(args), ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
