"""HH_261004 - Own a create-only known-DEV full catalog drive with disclosed terminal intervention, not approval."""

from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
import inspect
import math
from pathlib import Path
import textwrap
from types import SimpleNamespace

from . import research_temporal_route_joint_town03_straight_v3_20261004 as previous
from . import research_temporal_route_joint_town03_straight_callback45_v3_20261004 as callback_owner
from . import collect_temporal_route_joint_town03_straight_full209_v1_20261004 as live
from . import town03_fullroute_goal_cleanup_guard_v1_20261004 as goal_guard


base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
require, digest, read_json = previous.require, previous.digest, previous.read_json
straight = previous.previous.previous
right, runtime_owner = straight.previous, straight.runtime_owner
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_full209_v1_20261004"
SCHEMA = live.OWNER_SCHEMA
ROUTE, XODR, UMAP = previous.ROUTE, previous.XODR, previous.UMAP
XODR_SHA, UMAP_SHA = previous.XODR_SHA, previous.UMAP_SHA
PARENT_OWNER_SHA = "26e5740c8e6194a8a1872573d33c1f657954556aec3362c74cf36cc50881bd0c"
CALLBACK_READER_OWNER_SHA = "771eb289627ea8b060d6a4b6f832f79dc7c5f3a7e8c9f53e6623531d4b709169"
MAX_NATIVE, MAX_CAMERAS, MAX_SECONDS = 2200, 1100, 110
MAX_ACK, MAX_NATIVE_MARKERS, MAX_CALLBACKS = 2204, 17600, 24000
CALLBACK_JSON_MAX_BYTES = 48 * 1024 * 1024
_baseline_fixed, _baseline_sources = previous.fixed_contract, previous.runtime_sources
_original_capture, _goal_diagnostic = straight.capture_argv, straight._right_diagnostic
_active = False


def paths(run_id):
    require(run_id == "v1", "fresh full209 owner requires exact v1")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    """HH_261004 - Replace only the frozen straight45 duration; original owned ports/ACK/sensors remain intact."""
    require(_active and run_id == "v1", "full209 CLI requires its exclusive owner scope")
    result = _original_capture(run_id, longitudinal_pi, declaration_sha)
    index = result.index("--max-duration-sec") + 1
    require(result[index] == "45", "original straight duration seam changed")
    result[index] = "110"
    return result


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Declare the maximum prospectively; score all actual rows through the original catalog arrival."""
    result = _baseline_fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, output=str(paths(run_id)[0]), declaration_path=str(paths(run_id)[1]),
        experiment="Known-DEV Town03 full209 navigation-terminal-v2 controller research; original catalog2.5m arrival",
        total_capture_seconds=MAX_SECONDS, maximum_native_state_count=MAX_NATIVE,
        maximum_camera_bundle_count=MAX_CAMERAS, maximum_driving_native_ticks=MAX_NATIVE-24,
        fixed45_full_failed_budget_tail_required=False, event_completion_used_for_success=True,
        all_actual_rows_until_original_catalog_goal_or_original_safety_abort_required=True,
        actual_denominator_is_not_prospective_maximum=True, early_metric_success_forbidden=True,
        navigation_terminal_controller_intervention_declared=True,
        original_model_weights_unchanged=True, original_ordinary_decoder_tracker_PI_unchanged=True,
        original_NN_decoder_selected_trajectory_math_unchanged=False,
        original_model_and_gate_math_unchanged=False, original_NN_weights_unchanged=True,
        original_physical_gate_math_unchanged=True, original_tracker_PI_math_unchanged=True,
        raw_NN_decoder_selected_trajectory_actual_v0_teacher_and_all_gate_thresholds_unchanged=False,
        raw_NN_fields_and_teacher_and_gate_thresholds_unchanged=True,
        additional_navigation_terminal_decoder_intervention=True,
        original_catalog_goal_tolerance_m=2.5, catalog_arrival_is_not_learned_STOP=True,
        RTE04_stop_speed_or_hold_criterion_added=False, learned_STOP_claim=False,
        actual_callback_timing=dict(schema=live.SCHEMA+".callback_receipts", required=True,
            callback_clock="time.perf_counter_ns_monotonic", maximum_native=MAX_NATIVE,
            maximum_camera_bundles=MAX_CAMERAS, maximum_ACK=MAX_ACK,
            maximum_callbacks=MAX_CALLBACKS, maximum_native_stage_markers=MAX_NATIVE_MARKERS,
            maximum_callback_JSON_bytes=CALLBACK_JSON_MAX_BYTES,
            callback_to_plan_maximum_s=0.1, callback_to_ACK_maximum_s=0.1,
            generic_JSON_reader_limit_changed=False, hardware_exposure_to_ACK_proven=False),
        source_manifest_sealed_before_declaration=True, attempts=1, automatic_retry=False,
        complete_tree_quota_counter_intervention=dict(
            implementation="fresh fd-relative complete scandir tree walk; no incremental cache",
            exact_owned_episode_partial_only=True, unrelated_paths_original_delegate=True,
            every_camera_walk_and_failed_attempt_retained=True, maximum_walk_count=MAX_CAMERAS,
            original_quota_and_free_disk_floor_and_reserves_unchanged=True,
            no_JPEG_payload_read=True, every_regular_file_byte_counted=True,
            source="tools/e2e/audit_scandir_episode_quota_parity_v1_20261004.py",
            sensor_latency_gain_claimed=False, disk_check_and_dispatch_timing_remain_unresolved=True),
        prospective_visual_publication=dict(
            gallery="docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/11_town03_full_route209",
            complete_actual_interval_including_failures_required=True,
            GIF_camera_indices="every5th actual camera bundle starting0 plus exact last; unique sorted",
            GIF_playback="all selected timestamp intervals plus original final0.1s camera cadence",
            PNG_camera_indices="floor((actual_camera_count-1)*num/4), num=0,1,2,3,4; unique sorted",
            outcome_or_failure_dependent_frame_selection=False,
            real_six_cameras_required=True, vehicle_centered=True,
            original_full_route_and_actual_travel_and_prior_learned_ACK_trajectory_required=True,
            final_native_state_and_final_camera_timestamp_distinction_retained=True,
            actual_Autoware_or_RViz_screenshot_claim=False, illustrative_or_generated_image_allowed=False),
        known_DEV_site_reused=True, independent_validation=False,
        official_completed_main_domains=0, official_completed_subfunctions=0,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False, training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False)
    result["sensor_and_ACK_screen"].update(native_state_count="all actual until genuine catalog goal",
        camera_bundle_count="all actual native[::2]", maximum_native_state_count=MAX_NATIVE,
        maximum_camera_bundle_count=MAX_CAMERAS)
    result["actual_post_ACK_recording"].update(required_camera_bundles="all actual",
        required_JPEG_count="6 times actual camera count", required_complete_ACK_receipts="actual native count plus4",
        maximum_camera_bundles=MAX_CAMERAS, maximum_JPEG_count=6*MAX_CAMERAS, maximum_ACK_receipts=MAX_ACK)
    result["body_clearance_screen"].update(all_native_boxes="all actual N", all_snapshot_linear_sweeps="all actual N-1")
    result["authenticated_start_corridor"].pop("all_actual_900_boxes_and_899_sweeps_required")
    result["authenticated_start_corridor"]["all_actual_N_boxes_and_N_minus1_sweeps_required"] = True
    fixture = result["owned_green_signal_fixture"]
    fixture.pop("fixture_native_ticks_are_outside_the_45s_capture_and_not_NN_performance", None)
    fixture["fixture_native_ticks_are_outside_the_110s_capture_and_not_NN_performance"] = True
    result["fixture_owner_schema_admission"].update(declared_owner_schema=SCHEMA,
        controller_changed=True, policy_changed=True)
    result["intervention"].update(controller_changed=True, policy_changed=True,
        navigation_terminal_retiming="fresh navigation_terminal_v2; ordinary decoder/tracker/PI unchanged",
        goal_reason_guard="authenticated original catalog goal cleanup, no new learned STOP")
    result["execute_exit_status_interpretation"] = "owned cleanup plus retained actual interval; independent full209 research diagnostic determines PASS"
    require(result["model_speed_cap_mps"] == 2.5 and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["policy_deadline_seconds"] == 0.1 and result["controller_target_cap_mps"] == 2.4
        and result["speed_preview_s"] == 0.5 and result["body_clearance_screen"]["lane_inset_m"] == 0.1
        and result["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.0
        and result["precursor_motion_screen"]["cte_p95_maximum_m"] == 0.35
        and result["precursor_motion_screen"]["cte_absolute_maximum_m"] == 0.75
        and result["additive_native_speed_rate_guard_mps2"] ==
            {"minimum": -6.0, "maximum": 3.0, "warmup_to_first_driving_interval_included": True},
        "full209 changed an inherited motion/body/speed/latency threshold")
    return result


def runtime_sources():
    """HH_261004 - Seal all fresh numerical/policy/helper/collector/owner bytes before declaration without overwriting old pins."""
    require(digest(Path(previous.__file__)) == PARENT_OWNER_SHA, "frozen straight v3 owner changed")
    result = _baseline_sources()
    live.verify_frozen()
    seeds = [Path(__file__).resolve(), Path(live.__file__).resolve(), Path(goal_guard.__file__).resolve(),
        ROOT/"learning/portable_e2e/temporal_navigation_terminal_policy_v2_20261004.py",
        ROOT/"learning/portable_e2e/navigation_terminal_retiming_v2_20261004.py",
        ROOT/"learning/portable_e2e/terminaldecoder_v2_20261004.py",
        ROOT/"learning/tests/test_town03_fullroute_goal_cleanup_guard_v1_20261004.py",
        ROOT/"learning/tests/test_temporal_route_joint_town03_straight_full209_owner_v1_20261004.py",
        ROOT/"learning/tests/test_temporal_route_joint_town03_straight_full209_v1_20261004.py",
        ROOT/"learning/tests/test_temporal_navigation_terminal_policy_v2_20261004.py",
        ROOT/"learning/tests/test_navigation_terminal_retiming_v2_20261004.py",
        ROOT/"tools/e2e/audit_scandir_episode_quota_parity_v1_20261004.py",
        ROOT/"learning/tests/test_scandir_episode_quota_parity_v1_20261004.py"]
    require(all(path.is_file() and not path.is_symlink() for path in seeds), "fresh full209 source/test seam absent")
    for path in base.imported_source_closure(seeds):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "full209 source closure conflict")
        result[name] = sha
    return result


def catalog_statuses(route, states):
    """HH_261004 - Recompute the two unchanged goal predicates from original catalog points and actual rear pose."""
    projector = live.collector.RouteProjector(route)
    goal = route["route"][-1]
    require(all(type(row.get(key)) in (int,float) and math.isfinite(row[key])
        for row in states if row.get("capture_phase") == "driving" for key in ("x","y","route_progress_m")),
        "finite exact actual goal pose/progress required")
    return [live.collector.catalog_goal_status(row["route_progress_m"], projector.length_m,
        row["x"], row["y"], goal["x"], goal["y"], 2.5) for row in states if row.get("capture_phase") == "driving"]


def _terminal_receipt_facts(state):
    receipt = state.get("catalog_goal_cleanup_receipt")
    prior = state.get("control_transport", {})
    return isinstance(receipt, dict) and receipt.get("schema") == goal_guard.SCHEMA \
        and receipt.get("status") == "ACKNOWLEDGED" and receipt.get("command_reason") == goal_guard.GOAL_REASON \
        and receipt.get("command_frame") == state["frame"] \
        and receipt.get("source_timestamp_simulation_seconds") == state["timestamp"] \
        and type(receipt.get("prior_learned_ACK_sequence")) is int \
        and receipt["prior_learned_ACK_sequence"] == prior.get("expected_receipt_sequence") \
        and receipt.get("prior_learned_ACK_after_frame") == state["frame"]-1 \
        and receipt.get("cleanup_ACK_sequence") == receipt["prior_learned_ACK_sequence"]+1 \
        and prior.get("status") == "PASS" and receipt.get("original_goal_tolerance_m") == 2.5 \
        and all(receipt.get(key) is True for key in ("original_catalog_two_predicates_passed",
            "actual_prior_learned_ACK_effect_proven", "owner_source_PID_check_passed", "observed_event_snapshot_empty",
            "existing_emergency_latch_clear", "native_guard_attempted", "native_guard_passed", "cleanup_ACK_received")) \
        and all(receipt.get(key) is False for key in ("learned_control_proposed", "learned_STOP_proven",
            "physical_cleanup_actuation_proven", "vehicle_control_approved", "current_tick_hazard_detector_clear_proven"))


def complete_interval(manifest, states, cameras, event=None):
    """HH_261004 - Require the entire variable-length authentic goal interval; an early prefix or full-budget failure is not PASS."""
    try:
        n = len(states)
        if not (25 <= n <= MAX_NATIVE and len(cameras) == (n+1)//2):
            return False
        native = [row["frame"] for row in states]
        chain = all(type(value) is int for value in native) and native == list(range(native[0], native[0]+n)) \
            and [row["frame"] for row in cameras] == native[::2] \
            and all(type(row["timestamp"]) in (int, float) and math.isfinite(row["timestamp"]) for row in states) \
            and all(abs(b["timestamp"]-a["timestamp"]-0.05) <= 1e-6 for a, b in zip(states, states[1:]))
        phase = all(row.get("capture_phase") == "stationary_warmup" for row in states[:24]) \
            and all(row.get("capture_phase") == "driving" for row in states[24:])
        schedule = manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
        schedule_ok = schedule.get("maximum_total_duration_sec") == 110.0 \
            and schedule.get("maximum_total_ticks") == 2200 \
            and schedule.get("stationary_warmup", {}).get("scheduled_ticks") == 24 \
            and schedule.get("driving", {}).get("maximum_ticks") == 2176 \
            and schedule.get("stationary_tail", {}).get("scheduled_ticks") == 0
        result = manifest.get("result", {})
        route = read_json(ROUTE)
        require(digest(ROUTE) == live.ROUTE_SHA, "original route changed during interval audit")
        status = catalog_statuses(route, states)
        genuine = bool(status) and status[-1].reached is True and all(row.reached is False for row in status[:-1])
        goal = manifest.get("status") == "complete" and result.get("goal_reached") is True \
            and result.get("termination_reason") == "catalog_goal_tolerance" \
            and result.get("state_count") == n and result.get("camera_anchor_count") == len(cameras) \
            and result.get("goal_tolerance_m") == 2.5 \
            and result.get("final_remaining_route_m") == status[-1].remaining_route_m \
            and result.get("final_catalog_goal_distance_m") == status[-1].planar_distance_m
        return bool(chain and phase and schedule_ok and genuine and goal and _terminal_receipt_facts(states[-1])
            and isinstance(event, dict) and event.get("completed") is True)
    except (KeyError, TypeError, ValueError, IndexError, RuntimeError, AttributeError):
        return False


def complete_budget_failure_retained(manifest, states, cameras):
    """HH_261004 - Report an intact failed110s denominator separately; it cannot qualify catalog arrival or capability."""
    try:
        frames = [row["frame"] for row in states]
        schedule = manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
        result = manifest.get("result", {})
        return len(states) == MAX_NATIVE and len(cameras) == MAX_CAMERAS \
            and frames == list(range(frames[0],frames[0]+MAX_NATIVE)) \
            and [row["frame"] for row in cameras] == frames[::2] \
            and all(abs(b["timestamp"]-a["timestamp"]-.05) <= 1e-6 for a,b in zip(states,states[1:])) \
            and all(row.get("capture_phase") == "stationary_warmup" for row in states[:24]) \
            and all(row.get("capture_phase") == "driving" for row in states[24:]) \
            and schedule.get("maximum_total_duration_sec") == 110. \
            and schedule.get("maximum_total_ticks") == MAX_NATIVE \
            and schedule.get("stationary_tail", {}).get("scheduled_ticks") == 0 \
            and manifest.get("status") == "failed" \
            and manifest.get("error") == "CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail" \
            and "goal_reached" not in result and result.get("state_count") == MAX_NATIVE \
            and result.get("camera_anchor_count") == MAX_CAMERAS
    except (KeyError,TypeError,IndexError):
        return False


def recomputed_straight_interval(route, states, commands):
    """HH_261004 - Retain the final prior-NN effect while accepting only its authenticated original cleanup, not a new neural decision."""
    observer = live.StraightIntervalObserver(route)
    learned = {row["ack_receipt_sequence"] for row in commands if row.get("application", {}).get("learned_control_applied") is True}
    by_frame = {row["command_frame"]: row for row in commands if "command_frame" in row}
    require(len(by_frame) == sum("command_frame" in row for row in commands), "unique outgoing normal frame required")
    driving = [row for row in states if row.get("capture_phase") == "driving"]
    statuses = catalog_statuses(route, states)
    genuine = bool(statuses) and statuses[-1].reached is True and all(value.reached is False for value in statuses[:-1])
    receipt = driving[-1].get("catalog_goal_cleanup_receipt") if driving else None
    for index, state in enumerate(driving):
        prior = state.get("control_transport", {})
        learned_effect = prior.get("expected_receipt_sequence") in learned and prior.get("status") == "PASS"
        outgoing = by_frame.get(state["frame"], {})
        cleanup = index == len(driving)-1 and genuine and _terminal_receipt_facts(state) and learned_effect
        observer.observe(state, learned_effect=learned_effect,
            outgoing_safe=cleanup or (bool(outgoing) and outgoing.get("control_source") != "emergency_brake"))
    observer.session = SimpleNamespace(_goal_cleanup_pending=receipt)
    summary = observer.summary()
    summary.update(completed=bool(driving) and genuine and _terminal_receipt_facts(driving[-1])
        and len(observer.records) == len(driving) and observer.records[-1]["current_prior_ACK_is_learned_effect"] is True,
        full_actual_interval_including_final_prior_NN_effect_observed=len(observer.records) == len(driving),
        completion_is_denominator_and_catalog_arrival_only_not_performance=True)
    return summary, observer.records


def read_callback_receipts(path, expected_sha256):
    """HH_261004 - Extend only the frozen callback ABI to prospective110s bounds; inverse AST proves no reader guard removed."""
    require(digest(Path(callback_owner.__file__)) == CALLBACK_READER_OWNER_SHA, "frozen callback reader source changed")
    original = ast.parse(textwrap.dedent(inspect.getsource(callback_owner.read_callback_receipts)))
    tree = copy.deepcopy(original)
    changes = []
    replacements = {12000:24000, 450:1100, 1350:3300, 8100:17600}
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in replacements:
            old = node.value
            node.value = replacements[old]
            changes.append((node, old))
    require(len(changes) == 6, "frozen callback ABI count seams changed")
    inverse = copy.deepcopy(tree)
    reverse = {value:key for key,value in replacements.items()}
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in reverse:
            node.value = reverse[node.value]
    require(ast.dump(inverse) == ast.dump(original), "callback reader changed beyond prospective bounds")
    namespace = dict(vars(callback_owner), live=live, CALLBACK_JSON_MAX_BYTES=CALLBACK_JSON_MAX_BYTES)
    exec(compile(ast.fix_missing_locations(tree), str(Path(__file__).resolve()), "exec"), namespace)
    return namespace["read_callback_receipts"](path, expected_sha256, schema=live.SCHEMA+".callback_receipts")


def audit_goal_receipt(states, commands, receipts, timings, native_guards, wrapper, manifest):
    """HH_261004 - Cross-check stored goal permission against independent actual ACK/state/native-guard evidence."""
    require(states and _terminal_receipt_facts(states[-1]), "authenticated final cleanup facts absent")
    state, goal = states[-1], states[-1]["catalog_goal_cleanup_receipt"]
    require(wrapper.get("schema") == live.SCHEMA+".catalog_goal_cleanup" and wrapper.get("receipt") == goal
        and wrapper.get("failed_or_absent_receipt_retained") is True and wrapper.get("learned_STOP_claim") is False,
        "final state and create-only cleanup wrapper differ")
    n = len(states)
    require(n <= MAX_NATIVE and len(receipts) == len(timings) == n+4 <= MAX_ACK
        and [row["sequence"] for row in receipts] == list(range(1,n+5))
        and len({row["ACK_sequence"] for row in timings}) == len(timings), "dynamic full ACK denominator differs")
    selected = [row for row in receipts if row.get("reason") == goal_guard.GOAL_REASON]
    require(len(selected) == 1, "one actual catalog cleanup ACK required")
    cleanup = selected[0]
    prior = receipts[goal["prior_learned_ACK_sequence"]-1]
    actor, frame = goal["actor_id"], state["frame"]
    require(type(actor) is int and actor > 0 and {row["actor_id"] for row in receipts} == {actor}, "one exact actual ACK actor required")
    for row, expected_frame in ((prior,frame-1),(cleanup,frame)):
        require(row.get("status") == "ACKNOWLEDGED" and row.get("server_accepted") is True
            and row.get("mode") == "acknowledged_batch" and row.get("do_tick") is False
            and row.get("actor_id") == row.get("response_actor_id") == actor
            and row.get("expected_before_frame") == row.get("before_frame") == row.get("after_ack_frame") == expected_frame,
            "prior learned/goal cleanup ACK frame or actor differs")
    require(cleanup["sequence"] == goal["cleanup_ACK_sequence"] == prior["sequence"]+1
        and cleanup.get("requested_control") == goal["requested_original_cleanup_control"], "exact goal cleanup control differs")
    require(cleanup["requested_control"] == dict(throttle=0.,steer=0.,brake=1.,hand_brake=False,
        reverse=False,gear=0,manual_gear_shift=False), "unchanged original full-service blank cleanup brake required")
    require(goal["actual_state_binding_sha256"] == goal_guard._state_binding(state), "stored goal pose/control binding differs")
    ordinary = [row for row in commands if row.get("ack_receipt_sequence") == prior["sequence"]]
    wall = [row for row in timings if row.get("ACK_sequence") == prior["sequence"]]
    require(len(ordinary) == len(wall) == 1 and ordinary[0].get("application", {}).get("learned_control_applied") is True
        and ordinary[0].get("command_frame") == frame-1 and ordinary[0].get("control_source") == "learned_trajectory_tracker"
        and wall[0].get("ACK_server_accepted") is True and wall[0].get("learned_control_proposed") is True
        and wall[0].get("command_frame") == frame-1, "actual final prior learned ACK effect absent")
    research = manifest["capture_contract"]["learned_research"]
    require(ordinary[0].get("source_checkpoint_sha256") == wall[0].get("source_checkpoint_sha256") == research["checkpoint_sha256"]
        and wall[0].get("stream_source_sha256") == research["stream_identity"]["source_sha256"],
        "actual final prior learned checkpoint/source identity differs")
    guard = goal["native_guard"]
    actual = [row for row in native_guards if row.get("frame") == frame]
    speed = math.hypot(*state["world_velocity_carla"][:2])
    require(len(actual) == 1 and actual[0] == guard and guard.get("measured_actor_planar_speed_mps") == speed
        and math.isfinite(speed) and speed <= 2.7 and guard.get("maximum_actor_planar_speed_mps") == 2.7
        and type(guard.get("free_disk_bytes")) is int
        and guard.get("required_free_bytes") == goal_guard.frozen.DISK_FLOOR_BYTES+goal_guard.frozen.CLOSING_RESERVE_BYTES
        and guard["free_disk_bytes"] >= guard["required_free_bytes"]
        and not any(row.get("collision") or row.get("lane_invasion") for row in states), "original final native guard or event census differs")
    require(manifest.get("result", {}).get("goal_reached") is True, "manifest actual catalog goal absent")
    return dict(passed=True, actual_goal_frame=frame, prior_learned_ACK_sequence=prior["sequence"],
        actual_cleanup_ACK_sequence=cleanup["sequence"], learned_STOP_proven=False,
        physical_cleanup_actuation_proven=False, current_tick_teacher_hazard_observation_proven=False)


def diagnostic_report(output, plan, owner):
    """HH_261004 - Keep original runtime/body thresholds and add independent dynamic arrival/recording/callback gates."""
    result = _goal_diagnostic(output, plan, owner)
    interval = result.pop("independent_event", {})
    inherited = bool(result.pop("joint_turn_motion_body_policy_wall100ms_passed", False))
    result.pop("full_RTE03_approved", None)
    result.update(schema=SCHEMA+".diagnostic_report", recomputed_straight_interval=interval,
        inherited_motion_body_policy_wall100ms_passed=inherited, full_catalog_goal_receipt_passed=False,
        actual_after_ACK_recording_binding_passed=False, callback_plan_ACK_chain_passed=False,
        fresh_callback_to_plan_wall100ms_passed=False, fresh_callback_to_ACK_wall100ms_passed=False,
        native_pre_delivery_stage_closure_passed=False, complete_budget_failure_retained=False,
        independent_terminal_navigation_replay_passed=False, full_raw_and_source_closure_passed=False,
        complete_tree_quota_walk_receipts_passed=False)
    try:
        previous._verify_v2_failure()
        previous.previous._verify_v1_failure()
        episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
        states, cameras, commands, receipts, timings, guards = [right.jsonl(episode/name) for name in
            ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl", "wall_timing.jsonl", "native_guard.jsonl")]
        manifest = read_json(episode/"manifest.json")
        result["complete_budget_failure_retained"] = complete_budget_failure_retained(manifest,states,cameras)
        try:
            result["actual_complete_tree_quota_walks"] = live.audit_quota_walks(
                read_json(episode/"quota_walk_timing_receipts.json"),cameras,timings)
            require(result["actual_complete_tree_quota_walks"].get("passed") is True,
                "actual complete every-camera quota walk receipts must pass")
            result["complete_tree_quota_walk_receipts_passed"] = True
        except Exception as error:
            result["complete_tree_quota_walk_error"] = type(error).__name__+": "+str(error)
        try:
            route_source = output/"source"/ROUTE.relative_to(ROOT)
            result["independent_terminal_navigation_replay"] = live.audit_terminal_navigation(
                route_source.read_bytes(),states,commands,manifest=manifest,
                stream_source_sha256=owner["declaration_sha256"],episode_id="temporal_v2_"+output.name)
            require(result["independent_terminal_navigation_replay"].get("passed") is True,
                "independent actual terminal witness/decoder replay must pass")
            result["independent_terminal_navigation_replay_passed"] = True
        except Exception as error:
            result["independent_terminal_navigation_replay_error"] = type(error).__name__+": "+str(error)
        # HH_261004 - Goal failure must not suppress actual recording/callback measurements from failed tails.
        try:
            require(complete_interval(manifest,states,cameras,interval), "genuine full actual catalog interval required")
            result["actual_catalog_goal_cleanup"] = audit_goal_receipt(states, commands, receipts, timings, guards,
                read_json(episode/"catalog_goal_cleanup_receipt.json"), manifest)
            result["full_catalog_goal_receipt_passed"] = True
        except Exception as error:
            result["catalog_goal_qualification_error"] = type(error).__name__+": "+str(error)
        try:
            proof = read_json(episode/"actual_recording_binding.json")
            recordings = right.jsonl(episode/"recording_after_ack.jsonl")
            require(proof.get("schema") == live.SCHEMA+".actual_recording_binding"
                and proof.get("copied_provider_names") == ["_blank_brake_control", "checked_agent_initialization_observation",
                    "apply_spawn_z_offset", "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"]
                and proof.get("all_copied_mutable_providers_current") is True
                and proof.get("enqueue_and_ACK_share_exact_recorder") is True
                and proof.get("source_image_transformed") is False and proof.get("source_timestamp_changed") is False
                and proof.get("controller_or_policy_math_changed") is True
                and proof.get("disclosed_navigation_terminal_controller_intervention") is True
                and proof.get("ordinary_decoder_gate_tracker_PI_unchanged") is True
                and proof.get("actual_recording_bundle_count") == proof.get("actual_after_ACK_receipt_count") == len(cameras)
                and proof.get("pending_source_camera_count") == 0
                and len(recordings) == len({row["frame"] for row in recordings}) == len(cameras)
                and result.get("recording", {}).get("passed") is True
                and result["recording"].get("JPEG_count") == 6*len(cameras), "dynamic same-recorder full camera denominator required")
            result["actual_after_ACK_recording_binding_passed"] = True
        except Exception as error:
            result["actual_recording_binding_error"] = type(error).__name__+": "+str(error)
        try:
            path = episode/"callback_timing_receipts.json"
            sha = result.get("raw_source_sha256", {}).get(str(path))
            value = read_callback_receipts(path,sha)
            counts = read_json(episode/"learned_summary.json")["counts"]
            callback = live.audit_receipts(value,cameras,commands,timings,counts)
            result["actual_callback_timing"] = callback
            for key in ("callback_receipt_identity_and_SHA_passed", "callback_plan_ACK_chain_passed",
                "fresh_callback_to_plan_wall100ms_passed", "fresh_callback_to_ACK_wall100ms_passed", "actual_six_camera_callback_observed"):
                result[key] = callback[key]
            result["callback_receipt_JSON_readback"] = dict(path=str(path),sha256=sha,actual_bytes=path.stat().st_size,
                maximum_bytes=CALLBACK_JSON_MAX_BYTES, strict_schema_and_array_bounds_passed=True,
                generic_JSON_reader_limit_changed=False)
            try:
                native = live.audit_native_markers(value,states,cameras,timings,callback,
                    goal_complete=result["full_catalog_goal_receipt_passed"] is True)
                result["native_pre_delivery_timing"] = native
                result["native_pre_delivery_stage_closure_passed"] = native["native_pre_delivery_stage_closure_passed"]
            except Exception as error:
                result["native_pre_delivery_diagnostic_error"] = type(error).__name__+": "+str(error)
        except Exception as error:
            result["callback_timing_diagnostic_error"] = type(error).__name__+": "+str(error)
        right.verify_closure(output,plan)
        require(all(digest(path) == sha for path,sha in result.get("raw_source_sha256",{}).items()), "full raw inputs changed during audit")
        result["full_raw_and_source_closure_passed"] = True
    except Exception as error:
        result["full_catalog_diagnostic_error"] = type(error).__name__+": "+str(error)
    passed = inherited and all(result.get(key) is True for key in ("full_catalog_goal_receipt_passed",
        "actual_after_ACK_recording_binding_passed", "callback_receipt_identity_and_SHA_passed",
        "callback_plan_ACK_chain_passed", "fresh_callback_to_plan_wall100ms_passed",
        "fresh_callback_to_ACK_wall100ms_passed", "native_pre_delivery_stage_closure_passed",
        "independent_terminal_navigation_replay_passed", "full_raw_and_source_closure_passed",
        "complete_tree_quota_walk_receipts_passed"))
    result.update(learned_driving_screen_passed=passed, full209_research_motion_body_goal_callback100ms_passed=passed,
        status="PASS_KNOWN_DEV_FULL209_RESEARCH" if passed else "FAIL_KNOWN_DEV_FULL209_RESEARCH",
        known_DEV_site_reused=True, independent_validation=False, navigation_terminal_controller_intervention=True,
        learned_STOP_claim=False, RTE04_stop_speed_or_hold_criterion_added=False,
        official_completed_main_domains=0, official_completed_subfunctions=0, full_RTE01_approved=False,
        full_MAIN9_completion_claimed=False, training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Reuse the frozen owned lifecycle and start-corridor body census; scope only fresh full-route aliases."""
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_OWNER_SHA, "full209 owner cannot nest or use changed straight v3")
    values = dict(paths=paths,capture_argv=capture_argv,fixed_contract=fixed_contract,runtime_sources=runtime_sources,
        diagnostic_report=diagnostic_report,live=live,MODULE=MODULE,SCHEMA=SCHEMA)
    saved = {name:getattr(previous,name) for name in values}
    extra = {right:{"independent_event":recomputed_straight_interval,"complete_interval":complete_interval},
        runtime_owner:{"complete_interval":complete_interval}}
    _active = True
    try:
        for name,value in values.items():
            setattr(previous,name,value)
        with previous.activated(pins):
            originals = {module:{name:getattr(module,name) for name in changes} for module,changes in extra.items()}
            try:
                for module,changes in extra.items():
                    for name,value in changes.items():
                        setattr(module,name,value)
                yield
            finally:
                for module,changes in reversed(tuple(originals.items())):
                    for name,value in changes.items():
                        setattr(module,name,value)
    finally:
        for name,value in reversed(tuple(saved.items())):
            setattr(previous,name,value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):
        parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remaining = parser.parse_known_args(argv)
    pins = {name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
