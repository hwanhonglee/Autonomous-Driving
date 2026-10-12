"""HH_261004 - Run the complete known-DEV catalog with explicit navigation-owned terminal retiming.

This fresh collector preserves the ordinary learned policy, physical gates,
native ordering, owned fixture and ACK recorder.  It declares only three new
interventions: a110s maximum, verified short-support terminal decoding, and an
authenticated original-brake catalog-arrival reason.  No learned STOP or
independent capability approval is claimed.
"""

from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
import json
import math
from pathlib import Path
import textwrap
import time

from portable_e2e.temporal_navigation_terminal_policy_v2_20261004 import navigation_terminal_policy_class
from . import collect_temporal_route_joint_town03_straight_callback45_v3_20261004 as callback
from . import town03_fullroute_goal_cleanup_guard_v1_20261004 as goal_guard
from . import audit_scandir_episode_quota_parity_v1_20261004 as quota_counter
from . import research_temporal_v2_owned_camera_town07_20261001 as quota_base


previous = callback
straight, fixture = callback.straight, callback.fixture
v2 = callback.previous.previous
parent = straight.parent
generic = parent.generic
frozen, collector, ROOT = callback.frozen, callback.collector, callback.ROOT
SEED, ROUTE_SHA, ROUTE_LENGTH, ROUTE_POINTS = callback.SEED, callback.ROUTE_SHA, callback.ROUTE_LENGTH, callback.ROUTE_POINTS
ENTRY, EXIT, EXPECTED_TURN_DEG = callback.ENTRY, callback.EXIT, callback.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = callback.PREVIEW_SCHEMA, callback.THREAD_SCHEMA
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_full209.20261004.v1"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v1"
MAX_NATIVE, MAX_CAMERAS, MAX_SECONDS = 2200, 1100, 110
# HH_261004 - Reserve bootstrap/unconsumed/duplicate callback tails beyond nominal two-per-camera delivery.
MAX_CALLBACKS = 24000
PARENT_SHA = "da3a9733a6670152d2dd814bcc4921c91d18a303063ff20b9fb29e89b5d1f163"
QUOTA_COUNTER_SHA = "92126ab2ffeaac19a546d3e87aef415d086f078af6b68a8fbdb29c370b65ec47"
_original_instrument = callback.instrument_callback_AST
_original_validate = straight.validate_capture
_original_session = straight.known_dev_session
_active = False
_goal_context = None


def verify_frozen():
    frozen._require(hashlib.sha256(Path(callback.__file__).read_bytes()).hexdigest() == PARENT_SHA,
                    "frozen callback45_v3 source changed")
    callback.verify_frozen()
    goal_guard.verify_frozen()
    frozen._require(hashlib.sha256(Path(quota_counter.__file__).read_bytes()).hexdigest() == QUOTA_COUNTER_SHA,
                    "fresh approved whole-tree quota helper changed")


def _clone_function(function, change):
    """HH_261004 - Require a reversible AST edit before compiling any private fresh function."""
    original = ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed = copy.deepcopy(original)
    edits = change(changed)
    restored = copy.deepcopy(changed)
    edits(restored)
    frozen._require(ast.dump(restored) == ast.dump(original), "private full-route AST inverse proof failed")
    namespace = dict(function.__globals__, SCHEMA=SCHEMA)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":private_inverse_proven", "exec"), namespace)
    return namespace[function.__name__]


def _duration_change(node):
    count = 0
    for part in ast.walk(node):
        if isinstance(part, ast.Compare) and ast.unparse(part) == "args.max_duration_sec == 45":
            part.comparators[0].value = MAX_SECONDS
            count += 1
    frozen._require(count == 1, "exact one original duration predicate required")
    def undo(restored):
        for part in ast.walk(restored):
            if isinstance(part, ast.Compare) and ast.unparse(part) == "args.max_duration_sec == 110":
                part.comparators[0].value = 45
    return undo


_full_validate = _clone_function(_original_validate, _duration_change)


def validate_capture(args, route):
    _full_validate(args, route)
    frozen._require(args.goal_tolerance_m == 2.5, "unchanged original2.5m catalog-arrival tolerance required")


class FullRouteObserver(straight.StraightIntervalObserver):
    def observe(self, state, *, learned_effect, outgoing_safe):
        super().observe(state, learned_effect=learned_effect, outgoing_safe=outgoing_safe)
        self.records[-1].pop("fixed45_interval_continues")
        self.records[-1]["full_catalog_interval_continues"] = True
        return False

    def summary(self):
        receipt = getattr(self.session, "_goal_cleanup_pending", None) if self.session else None
        genuine = isinstance(receipt, dict) and receipt.get("status") == "ACKNOWLEDGED" \
            and receipt.get("original_catalog_two_predicates_passed") is True
        return dict(schema=SCHEMA + ".full_catalog_interval", completed=genuine,
            observation_count=len(self.records), last_observation=self.records[-1] if self.records else None,
            actual_catalog_goal_cleanup=copy.deepcopy(receipt), maximum_total_seconds=MAX_SECONDS,
            maximum_native_state_count=MAX_NATIVE, all_driving_observations_including_goal_retained=True,
            no_early_metric_success_break=True, original_goal_tolerance_m=2.5,
            navigation_terminal_controller_intervention=True, learned_STOP_claim=False,
            split="val", known_DEV_site_reused=True, independent_validation=False,
            training_data_approved=False, model_promoted=False, vehicle_control_approved=False)


RightTurnObserver = StraightIntervalObserver = FullRouteObserver


def observed_after_tick(states):
    # HH_261004 - The original observer's ACK indexing/projector is unchanged; now include the authentic final state.
    return straight._original_fullroute_observed_delegate(states)


def full_goal_control_reason(alignment, phase, stop_reason, goal_status, event_recorder):
    global _goal_context
    reason = goal_guard.choose_goal_control_reason(alignment, phase, stop_reason, goal_status)
    frozen._require(_goal_context is None, "unconsumed catalog-goal reason context")
    if reason == goal_guard.GOAL_REASON:
        _goal_context = goal_status, event_recorder.snapshot()
    return reason


def instrument_fullroute_AST(original):
    changed = _original_instrument(original)
    before = copy.deepcopy(changed)
    counts = dict(final=0, reason=0)
    reason_text = "'alignment_failure_abort' if alignment is not None and alignment['status'] != 'PASS' else 'next_' + phase + '_control'"
    class Fresh(ast.NodeTransformer):
        def visit_If(self, node):
            if ast.unparse(node) == "if stop_reason is None:\n    observed_after_tick(state_records)":
                counts["final"] += 1
                return node.body[0]
            return self.generic_visit(node)
        def visit_Call(self, node):
            node = self.generic_visit(node)
            if ast.unparse(node.func) == "send_control" and len(node.args) == 3 \
                    and ast.unparse(node.args[1]) == reason_text:
                node.args[1] = ast.parse("full_goal_control_reason(alignment, phase, stop_reason, goal_status, event_recorder)", mode="eval").body
                counts["reason"] += 1
            return node
    changed = Fresh().visit(changed)
    frozen._require(counts == dict(final=1, reason=1), "exact final observation and outgoing reason seams required")
    class Undo(ast.NodeTransformer):
        def visit_Expr(self, node):
            if ast.unparse(node) == "observed_after_tick(state_records)":
                return ast.parse("if stop_reason is None:\n    observed_after_tick(state_records)").body[0]
            return self.generic_visit(node)
        def visit_Call(self, node):
            node = self.generic_visit(node)
            if ast.unparse(node.func) == "full_goal_control_reason":
                return ast.parse(reason_text, mode="eval").body
            return node
    frozen._require(ast.dump(Undo().visit(copy.deepcopy(changed))) == ast.dump(before),
                    "native, teacher, alignment, goal or termination math changed outside declared seams")
    return changed


def full_capture_factory():
    capture = callback._original_fullroute_capture_factory()
    function = inspect.getclosurevars(capture).nonlocals["function"]
    function.__globals__["full_goal_control_reason"] = full_goal_control_reason
    return capture


def actual_source_binding(frame, tensors, window):
    """HH_261004 - Authenticate actual current rear pose and motion, without copying images or predicting future motion."""
    session = parent._observer.session
    state = session.states[-1]
    stamp = int(round(float(state["timestamp"]) * 1.e9))
    yaw = float(state["yaw"])
    position = tuple(float(state[name]) for name in ("x", "y", "z"))
    orientation = (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))
    frozen._require(frame is session.observed_frames[-1] and state["frame"] == session.bundle_frame
        and stamp == frame.timestamp_ns == frame.pose_timestamp_ns == session.history[-1][2]
        and position == frame.position_map_m and orientation == frame.orientation_xyzw
        and tuple(frame.inputs.ego_history[-1]) == tuple(session.history[-1][1])
        and frame.split == "val" and frame.source_sha256 == session.source_sha256,
        "terminal input does not match the actual current owned native source")
    shared = dict(native_frame=state["frame"], timestamp_ns=stamp, episode_id=frame.episode_id,
        split=frame.split, rig_id=frame.rig_id, stream_source_sha256=frame.source_sha256,
        counterfactual_not_observed=False)
    pose = dict(shared, actual_observed_native_pose=True, position_map_m=list(position),
        orientation_xyzw=list(orientation), actor_snapshot_transform_carla=state["actor_snapshot_transform_carla"])
    motion = dict(shared, actual_observed_native_motion=True,
        actual_base_link_motion_row=list(session.history[-1][1]),
        world_velocity_carla=state["world_velocity_carla"], world_acceleration_carla=state["world_acceleration_carla"],
        world_angular_velocity_carla_deg_s=state["world_angular_velocity_carla_deg_s"])
    return pose, motion


def known_dev_session():
    original, thread = _original_session()
    class FullRouteSession(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.goal_cleanup_catalog_route = copy.deepcopy(args[2])
        def propose(self, *args, **kwargs):
            chosen, decision = super().propose(*args, **kwargs)
            if self.policy is not None:
                bridge = getattr(self.policy, "navigation_terminal_bridge", None)
                frozen._require(bridge is not None and bridge.policy is self.policy
                    and self.policy.decoder is bridge and self.policy.identity[1] == "val",
                    "actual instantiated fullroute VAL policy is not bound to its navigation terminal bridge")
            if decision is not None:
                decision["navigation_terminal_controller_intervention_declared"] = True
                decision["navigation_terminal_actual_policy_bridge_bound"] = self.policy is not None
                if (decision.get("navigation_terminal_research") or {}).get("terminal_controller_intervention") is True:
                    decision["decoder_gate_tracker_math_unchanged_except_disclosed_headroom_and_speed_preview"] = False
                    decision["additional_navigation_terminal_decoder_intervention"] = True
            return chosen, decision
    return FullRouteSession, thread


def run_fullroute_capture(function, *args, **kwargs):
    manifest, partial = args[6], args[3]
    manifest.setdefault("capture_contract", {})["known_DEV_straight_full_catalog_interval"] = dict(
        schema=SCHEMA, dataset_split="val", known_DEV_site_reused=True, independent_validation=False,
        original_navigation_sha256=ROUTE_SHA, original_navigation_points=ROUTE_POINTS,
        maximum_total_seconds=MAX_SECONDS, maximum_native_state_count=MAX_NATIVE,
        maximum_camera_bundle_count=MAX_CAMERAS, original_goal_tolerance_m=2.5,
        no_early_metric_success_break=True, actual_prior_learned_ACK_required_at_goal=True,
        terminal_controller_intervention_not_learned_STOP=True, training_data_approved=False)
    try:
        return function(*args, **kwargs)
    finally:
        frozen._write_new(partial / "known_dev_straight_summary.json", parent._observer.summary())
        receipt = getattr(parent._observer.session, "_goal_cleanup_pending", None)
        frozen._write_new(partial / "catalog_goal_cleanup_receipt.json", dict(
            schema=SCHEMA + ".catalog_goal_cleanup", receipt=receipt,
            failed_or_absent_receipt_retained=True, learned_STOP_claim=False))


def checked_fullroute_capture(function, *args, **kwargs):
    recorder = v2._recorder()
    names = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
    frozen._require(all(function.__globals__.get(name) is getattr(collector, name) for name in names),
                    "fresh fullroute retained a stale actual recorder/provider")
    try:
        return run_fullroute_capture(function, *args, **kwargs)
    finally:
        frozen._write_new(args[3] / "actual_recording_binding.json", dict(
            schema=SCHEMA + ".actual_recording_binding", copied_provider_names=list(names),
            all_copied_mutable_providers_current=True, enqueue_and_ACK_share_exact_recorder=True,
            source_image_transformed=False, source_timestamp_changed=False,
            recording_wall_time_added_to_bundle_to_ACK_metric=False, exposure_to_ACK_latency_measured=False,
            controller_or_policy_math_changed=True, disclosed_navigation_terminal_controller_intervention=True,
            ordinary_decoder_gate_tracker_PI_unchanged=True, every_recording_event_preserved=True,
            actual_recording_bundle_count=len(recorder.events),
            actual_after_ACK_receipt_count=sum(row["acknowledged_before_recording"] is True for row in recorder.events),
            pending_source_camera_count=len(recorder.pending), full_catalog_success_not_claimed_by_binding_alone=True))


class FullLedger(callback.CallbackLedger):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.quota_walks = []
    def native_mark(self, tick_index, phase, frame, stage, agent_run_step_invoked=None):
        frozen._require(type(tick_index) is int and 0 <= tick_index < MAX_NATIVE, "fullroute native marker index exceeded")
        return _full_native_mark(self, tick_index, phase, frame, stage, agent_run_step_invoked)
    def mark(self, policy, frame, stage):
        super().mark(policy, frame, stage)
        row = self.plans[-1]
        row.pop("original_policy_math_changed")
        row.update(original_policy_step_statements_changed=False, navigation_terminal_decoder_intervention_declared=True)
    def finish(self):
        result = _full_finish(self)
        if result is None:
            return None
        target = self.output if self.output.is_dir() else self.output.with_name(self.output.name.removesuffix(".partial"))
        frozen._write_new(target / "quota_walk_timing_receipts.json", dict(
            schema=SCHEMA + ".quota_walk_receipts", clock="time.perf_counter_ns_monotonic",
            rows=copy.deepcopy(self.quota_walks), actual_walk_count=len(self.quota_walks),
            maximum_walk_count=MAX_CAMERAS, maximum_count_exceeded=len(self.quota_walks) > MAX_CAMERAS,
            every_attempt_including_failures_retained=True, complete_walk_every_call=True,
            incremental_cache_used=False, original_quota_free_floor_and_reserves_unchanged=True,
            disk_check_dispatch_and_scheduling_interval_separately_unresolved=True,
            actual_sensor_latency_gain_proven_by_walk_timing=False, model_weights_changed=False))
        return result


def _native_limit_change(node):
    count = 0
    for part in ast.walk(node):
        if isinstance(part, ast.Constant) and part.value == 900:
            part.value = MAX_NATIVE
            count += 1
    frozen._require(count == 1, "one native scalar-index bound required")
    def undo(restored):
        for part in ast.walk(restored):
            if isinstance(part, ast.Constant) and part.value == MAX_NATIVE:
                part.value = 900
    return undo


_full_native_mark = _clone_function(callback.CallbackLedger.native_mark, _native_limit_change)


def _callback_limit_change(node):
    count = 0
    for part in ast.walk(node):
        if isinstance(part, ast.Constant) and type(part.value) is int and part.value == 12000:
            part.value = MAX_CALLBACKS
            count += 1
    frozen._require(count == 2, "exact fullroute scalar ledger limit and exceedance predicates required")
    def undo(restored):
        for part in ast.walk(restored):
            if isinstance(part, ast.Constant) and part.value == MAX_CALLBACKS:
                part.value = 12000
    return undo


_full_finish = _clone_function(callback.CallbackLedger.finish, _callback_limit_change)


@contextmanager
def activated(model, args):
    """HH_261004 - Scope fresh aliases only, preserving all inherited owner/fixture/PID/ACK cleanup guards."""
    global _active, _goal_context
    frozen._require(not _active and not callback._active and _goal_context is None, "fullroute activation cannot nest")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    route_payload = Path(args.route_file).read_bytes()
    changes = {callback: dict(SCHEMA=SCHEMA, OWNER_SCHEMA=OWNER_SCHEMA, CallbackLedger=FullLedger,
            instrument_callback_AST=instrument_fullroute_AST, validate_capture=validate_capture,
            callback_capture_factory=full_capture_factory),
        straight: dict(validate_capture=validate_capture, StraightIntervalObserver=FullRouteObserver,
            observed_after_tick=observed_after_tick, known_dev_session=known_dev_session, run_straight_capture=run_fullroute_capture),
        v2: dict(_straight_capture=run_fullroute_capture, checked_straight_capture=checked_fullroute_capture)}
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    callback._original_fullroute_capture_factory = saved[callback]["callback_capture_factory"]
    straight._original_fullroute_observed_delegate = saved[straight]["observed_after_tick"]
    _active = True
    try:
        for module, values in changes.items():
            for name, value in values.items():
                setattr(module, name, value)
        with callback.activated(model, args):
            old_policy, old_transport = generic.TemporalSimulationResearchPolicyV2, collector.AcknowledgedControlTransport
            old_counter = quota_base.directory_bytes
            partial = args.output.with_name(args.output.name + ".partial")
            def counted_owned_tree(directory):
                # HH_261004 - Replace only the exact owned quota tree; unrelated postchecks retain their original delegate.
                if Path(directory) != partial:
                    return old_counter(directory)
                ledger = callback._ledger
                frozen._require(type(ledger) is FullLedger and ledger.capture_started and not ledger.closed,
                                "whole-tree quota receipt requires the active exact fullroute capture")
                started = time.perf_counter_ns()
                row = dict(sequence=len(ledger.quota_walks)+1, owned_partial_path=str(partial),
                    walk_started_monotonic_ns=started,status="RUNNING",source_payload_read=False)
                session = parent._observer.session
                if session is not None and session.states:
                    state = session.states[-1]
                    row.update(native_frame=state["frame"],timestamp_simulation_ns=int(round(state["timestamp"]*1.e9)),
                               capture_phase=state["capture_phase"])
                ledger.quota_walks.append(row)
                try:
                    used = quota_counter.scandir_directory_bytes(directory,receipt=row)
                    row.update(status="PASS",returned_regular_file_bytes=used)
                    return used
                except BaseException as error:
                    row.update(status="FAIL",error=type(error).__name__+": "+str(error))
                    raise
                finally:
                    row["walk_finished_monotonic_ns"] = time.perf_counter_ns()
            generic.TemporalSimulationResearchPolicyV2 = navigation_terminal_policy_class(old_policy, route_payload, actual_source_binding)
            class GoalCleanupTransport(old_transport):
                def send(self, control, reason, expected_before_frame=None):
                    global _goal_context
                    if reason != goal_guard.GOAL_REASON:
                        return super().send(control, reason, expected_before_frame)
                    frozen._require(_goal_context is not None, "actual original catalog goal context missing")
                    status, events = _goal_context
                    _goal_context = None
                    session = parent._observer.session
                    goal_guard.authenticate_goal_cleanup(session, self, control, expected_before_frame,
                        status, events, lambda: fixture.assert_owned_capture(args))
                    goal_guard.run_original_native_guard(session)
                    goal_guard.claim_goal_cleanup_send(session, self, control, expected_before_frame)
                    receipt = super().send(control, reason, expected_before_frame)
                    completed = goal_guard.complete_goal_cleanup_receipt(session, self, receipt)
                    session.states[-1]["catalog_goal_cleanup_receipt"] = completed
                    return receipt
            collector.AcknowledgedControlTransport = GoalCleanupTransport
            quota_base.directory_bytes = counted_owned_tree
            try:
                yield
            finally:
                generic.TemporalSimulationResearchPolicyV2, collector.AcknowledgedControlTransport = old_policy, old_transport
                quota_base.directory_bytes = old_counter
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items():
                setattr(module, name, value)
        del callback._original_fullroute_capture_factory
        del straight._original_fullroute_observed_delegate
        _goal_context, _active = None, False


def _dynamic_audit(function, *, native_count, camera_count, goal_complete=False):
    """HH_261004 - Replace only declared fixed45 denominators and final true-goal branch expectation."""
    original = ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed = copy.deepcopy(original)
    replacements = {900: "native_count", 450: "camera_count", 904: "native_count + 4", 7200: "7 * native_count + 2 * camera_count", 12000: "max_callback_count"}
    class Rewrite(ast.NodeTransformer):
        def visit_Constant(self, node):
            if type(node.value) is int and node.value in replacements:
                return ast.parse(replacements[node.value], mode="eval").body
            if node.value == "original_policy_math_changed":
                return ast.Constant("original_policy_step_statements_changed")
            return node
        def visit_Assign(self, node):
            node = self.generic_visit(node)
            if ast.unparse(node) == "invoked = phase == 'driving'":
                node.value = ast.parse("phase == 'driving' and not (goal_complete and index == native_count - 1)", mode="eval").body
            return node
    changed = Rewrite().visit(changed)
    # HH_261004 - Every replacement has a unique named inverse; no time/physics/safety predicate is altered.
    class Undo(ast.NodeTransformer):
        def visit(self, node):
            text = ast.unparse(node)
            for old, new in replacements.items():
                if text == new:
                    return ast.Constant(old)
            return super().visit(node)
        def visit_Constant(self, node):
            return ast.Constant("original_policy_math_changed") if node.value == "original_policy_step_statements_changed" else node
        def visit_Assign(self, node):
            if ast.unparse(node) == "invoked = phase == 'driving' and (not (goal_complete and index == native_count - 1))":
                node.value = ast.parse("phase == 'driving'", mode="eval").body
            return self.generic_visit(node)
    frozen._require(ast.dump(Undo().visit(copy.deepcopy(changed))) == ast.dump(original), "dynamic callback/native audit changed another gate")
    namespace = dict(vars(callback), SCHEMA=SCHEMA, native_count=native_count, camera_count=camera_count,
                     goal_complete=goal_complete, max_callback_count=MAX_CALLBACKS)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
        str(Path(__file__)) + ":fullroute_denominator", "exec"), namespace)
    return namespace[function.__name__]


def audit_receipts(value, cameras, commands, timings, counts):
    native = len(timings) - 4
    frozen._require(24 < native <= MAX_NATIVE and len(cameras) == (native + 1) // 2, "actual fullroute ACK/camera denominator differs")
    return _dynamic_audit(callback.audit_receipts, native_count=native, camera_count=len(cameras))(value, cameras, commands, timings, counts)


def audit_native_markers(value, states, cameras, timings, callback_audit, *, goal_complete=False):
    frozen._require(24 < len(states) <= MAX_NATIVE, "actual fullroute native denominator outside declared budget")
    return _dynamic_audit(callback.audit_native_markers, native_count=len(states), camera_count=len(cameras), goal_complete=goal_complete)(
        value, states, cameras, timings, callback_audit)


def audit_terminal_navigation(route_payload, states, commands, *, manifest, stream_source_sha256, episode_id):
    """HH_261004 - Independently reconstruct every accepted terminal source/witness/output from retained actual scalar inputs.

    This is a post-capture CPU replay, not a camera exposure measurement or model
    forward.  Original callback/object/source-SHA checks remain separately
    required.  Failed decisions remain in their original files and denominator.
    """
    from dataclasses import asdict
    import torch
    from portable_e2e import temporal_navigation_terminal_policy_v2_20261004 as nav
    from portable_e2e import terminal_witness_v1_20261002 as proof
    from portable_e2e.temporal_owned_camera_history_v2_20261001 import normalized_tensor_hashes
    from portable_e2e.path_time_decoder_v2 import GO
    route, points, total = nav.complete_route_geometry(route_payload)
    frozen._require(type(stream_source_sha256) is str and len(stream_source_sha256) == 64
        and type(episode_id) is str and episode_id, "exact declared actual stream/episode identity required")
    specs = [collector.CameraSpec(**row) for row in manifest["cameras"]]
    calibration = generic.original.calibration_from_specs(specs)
    rig = hashlib.sha256(json.dumps(calibration, allow_nan=False).encode()).hexdigest()
    by_frame = {row["frame"]: row for row in states}
    frozen._require(len(by_frame) == len(states), "unique actual terminal native state census required")
    fresh = [row for row in commands if row.get("learned_control_proposed") is True
        and row.get("held_between_camera_frames") is False]
    decoder, previous_witness, mission_identity, terminal_rows = nav.NavigationTerminalRetimingV2(), None, None, []
    ordinary_count = 0
    for decision in fresh:
        receipt = decision.get("navigation_terminal_research")
        frozen._require(isinstance(receipt, dict) and receipt.get("schema") == nav.SCHEMA
            and receipt.get("original_NN_fields_changed") is False and receipt.get("learned_STOP_claim") is False,
            "each actual fresh NN must retain an honest navigation branch receipt")
        if receipt.get("branch") == "ordinary_exact_delegate":
            frozen._require(receipt.get("terminal_controller_intervention") is False
                and receipt.get("inherited_tensor_support_m") == decision["causal_navigation_support_m"] >= 18.,
                "ordinary decoder branch was changed or had short causal support")
            ordinary_count += 1
            continue
        frozen._require(receipt.get("branch") == "authenticated_navigation_terminal_V2"
            and receipt.get("terminal_controller_intervention") is True
            and receipt.get("actual_support_extrapolated") is False
            and receipt.get("actual_observed_speed_clipped") is False
            and receipt.get("raw_semantic_heads_used") is False
            and receipt.get("witness_and_decoder_inside_original_policy_deadline") is True
            and receipt.get("inherited_tensor_support_m") == decision["causal_navigation_support_m"] < 18.,
            "actual terminal branch intervention/support contract differs")
        frame = decision["inference_frame"]
        state = by_frame[frame]
        stamp = int(round(float(state["timestamp"]) * 1.e9))
        frozen._require(decision["command_frame"] == frame and receipt["frame_timestamp_ns"] == stamp,
                        "terminal decision does not use its actual current native state")
        owned = decision["owned_camera_history"]
        current_camera = owned["source_camera_frames"][-1]
        frozen._require(owned["frame_timestamp_ns"] == current_camera["timestamp_ns"] == stamp
            and owned["every_source_camera_SHA_preserved"] is True and owned["source_pixels_changed"] is False,
            "actual terminal owned current camera binding differs")
        camera_states = [row for row in states[::2] if row["frame"] <= frame][-10:]
        frozen._require(len(camera_states) == 10 and camera_states[-1] is state,
                        "ten actual camera-cadence motion rows required at terminal decoding")
        motion_rows = tuple(generic.original.state_features(row) for row in camera_states)
        ego_tensor = torch.tensor(motion_rows, dtype=torch.float32)[None]
        # HH_261004 - Reconstruct the original all-ten-valid ego ABI, including its mask; never bypass the measured-speed guard.
        ego_tensors = dict(ego_history=ego_tensor, ego_history_mask=torch.ones((1,10),dtype=torch.bool))
        ego_hashes = normalized_tensor_hashes(ego_tensors)
        ego_sha = ego_hashes["ego_history"]
        frozen._require(all(value == owned["normalized_input_tensor_sha256"][name]
            for name,value in ego_hashes.items()),
                        "terminal normalized ego tensor/mask differs from actual retained native camera history")
        speed = callback.policy_runtime.measured_current_speed(ego_tensors)
        frozen._require(float(speed[0]) == decision["measured_current_speed_mps"],
                        "terminal measured current speed was clipped or differs from original float32 math")
        yaw = float(state["yaw"])
        position = tuple(float(state[name]) for name in ("x", "y", "z"))
        quaternion = (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))
        shared = dict(native_frame=frame, timestamp_ns=stamp, episode_id=episode_id,
            split="val", rig_id=rig, stream_source_sha256=stream_source_sha256, counterfactual_not_observed=False)
        pose = dict(shared, actual_observed_native_pose=True, position_map_m=list(position),
            orientation_xyzw=list(quaternion), actor_snapshot_transform_carla=state["actor_snapshot_transform_carla"])
        motion = dict(shared, actual_observed_native_motion=True, actual_base_link_motion_row=list(motion_rows[-1]),
            world_velocity_carla=state["world_velocity_carla"], world_acceleration_carla=state["world_acceleration_carla"],
            world_angular_velocity_carla_deg_s=state["world_angular_velocity_carla_deg_s"])
        frozen._require(nav.encoded(pose)[0] == nav.encoded(receipt["actual_pose_provenance"])[0]
            and nav.encoded(motion)[0] == nav.encoded(receipt["actual_motion_provenance"])[0]
            and nav.encoded(motion)[1] == receipt["actual_motion_provenance_sha256"],
            "terminal pose/motion provenance is not the independently retained actual current source")
        provided, goal, anchor = nav._canonical_route_in_base(points, position_m=position,
            orientation_xyzw=quaternion, contract=nav.CANONICAL_NAVIGATION, context="independent actual full209 terminal replay")
        provided_arc = sum(math.dist(a,b) for a,b in zip(provided,provided[1:]))
        mission = receipt["mission_document"]
        expected_mission = dict(schema=proof.MISSION_SCHEMA,coordinates=dict(proof.COORDINATES),
            mission_id=episode_id+":original_town03_p07_COMPLETE",map_id="Town03",route_id="town03_straight_s0000_p07",
            source_route_artifact_sha256=ROUTE_SHA, source_available_timestamp_ns=stamp if mission_identity is None else mission_identity["source_available_timestamp_ns"],
            complete_mission_route=True, polyline_map_xy_m=points,goal_map_xy_m=points[-1],stop_reference_offset_m=0.)
        frozen._require(nav.encoded(mission)[0] == nav.encoded(expected_mission)[0],
                        "terminal COMPLETE mission identity, geometry or availability was altered")
        mission_identity = expected_mission
        observation = dict(schema=proof.OBSERVATION_SCHEMA,coordinates=dict(proof.COORDINATES),
            mission_id=mission["mission_id"],map_id="Town03",route_id=mission["route_id"],source_route_artifact_sha256=ROUTE_SHA,
            sequence_index=0 if previous_witness is None else previous_witness.sequence_index+1,
            anchor_timestamp_ns=stamp,ego_timestamp_ns=stamp,pose_timestamp_ns=stamp,navigation_timestamp_ns=stamp,
            camera_timestamp_ns=[stamp]*6,camera_source_sha256=current_camera["camera_sha256"],
            ego_source_sha256=ego_sha,pose_source_sha256=nav.encoded(pose)[1],position_map_m=position,
            orientation_xyzw=quaternion,provided_route_xy_base_m=provided,provided_route_support_m=provided_arc,
            route_anchor_arc_m=anchor,remaining_mission_arc_m=total-anchor,goal_base_m=goal)
        payload, observation_sha = nav.encoded(observation)
        mission_payload, mission_sha = nav.encoded(expected_mission)
        frozen._require(payload == nav.encoded(receipt["observation_document"])[0]
            and observation_sha == receipt["observation_sha256"] and mission_sha == receipt["mission_sha256"],
            "terminal observation source/timestamp/true arc or sealed document SHA differs")
        witness = proof.evaluate_terminal_witness(mission_payload=mission_payload,expected_mission_sha256=mission_sha,
            expected_route_artifact_sha256=ROUTE_SHA,observation_payload=payload,expected_observation_sha256=observation_sha,
            now_ns=stamp,requested_mode=proof.VERIFIED_TERMINAL,previous=previous_witness)
        frozen._require(witness.status == "VERIFIED" and nav.encoded(asdict(witness))[0] == nav.encoded(receipt["witness_decision"])[0],
                        "independent actual terminal witness rejects or differs from recorded proof")
        distance = min(provided_arc,total-anchor,64.)
        frozen._require(receipt["navigation_owned_D_m"] == distance
            and receipt["actual_provided_arc_m"] == provided_arc and receipt["actual_remaining_mission_arc_m"] == total-anchor,
            "terminal D was rounded, extrapolated or substituted for actual remaining/provided arc")
        raw = decision["raw_output"]
        replay = decoder(torch.tensor([raw["curvature_knots"]],dtype=torch.float32),
            torch.tensor([raw["preferred_speed_knots"]],dtype=torch.float32),speed,torch.tensor([GO]),torch.zeros(1),
            torch.tensor([distance],dtype=torch.float64),terminal_witness=witness,
            terminal_anchor_timestamp_ns=stamp,terminal_now_ns=stamp,terminal_observation_sha256=observation_sha,
            terminal_mission_sha256=mission_sha,terminal_route_artifact_sha256=ROUTE_SHA)
        decoded = decision["decoded_output"]
        for name,target in (("xy","candidate_xy"),("speed","candidate_speed"),
            ("instantaneous_speed","instantaneous_speed"),("arc_distance","arc_distance")):
            frozen._require(replay[name][0].tolist() == decoded[target], "actual terminal rawNN/decoder replay differs: "+name)
        requests = {name:value.tolist() for name,value in replay.items() if name.startswith("navigation_")}
        frozen._require(requests == receipt["physical_navigation_requests"]
            and replay["terminal_arrival_time_s"].tolist() == receipt["terminal_arrival_time_s"]
            and replay["terminal_stop_complete_within_horizon"].tolist() == receipt["terminal_stop_complete_within_horizon"],
            "terminal physical navigation request or forecast differs from frozen replay")
        previous_witness = witness.witness
        terminal_rows.append(dict(inference_frame=frame,timestamp_ns=stamp,actual_D_m=distance,
            observed_speed_mps=float(speed[0]),source_pose_motion_and_ego_SHA_passed=True,
            COMPLETE_witness_and_exact_rawNN_decoder_replay_passed=True))
    frozen._require(terminal_rows, "full209 has no accepted actual terminal fresh NN inference")
    return dict(passed=True,actual_terminal_inference_count=len(terminal_rows),ordinary_exact_delegate_count=ordinary_count,
        accepted_fresh_NN_denominator=len(fresh),all_terminal_source_witness_and_raw_decoder_outputs_recomputed=True,
        failed_or_rejected_raw_decisions_removed=False,model_forward_executed=False,camera_payload_read=False,
        actual_sensor_latency_proven_by_replay=False,learned_STOP_claim=False,rows=terminal_rows)


def audit_quota_walks(value, cameras, timings):
    """HH_261004 - Verify every actual owned-tree walk and its original delivery chronology, not a causal speedup claim."""
    frozen._require(value.get("schema") == SCHEMA + ".quota_walk_receipts"
        and value.get("clock") == "time.perf_counter_ns_monotonic"
        and value.get("maximum_walk_count") == MAX_CAMERAS
        and value.get("maximum_count_exceeded") is False
        and value.get("every_attempt_including_failures_retained") is True
        and value.get("complete_walk_every_call") is True and value.get("incremental_cache_used") is False
        and value.get("original_quota_free_floor_and_reserves_unchanged") is True
        and value.get("actual_sensor_latency_gain_proven_by_walk_timing") is False,
        "complete original-threshold quota-walk receipt contract required")
    rows = value["rows"]
    frozen._require(len(rows) == value.get("actual_walk_count") == len(cameras) <= MAX_CAMERAS
        and [row["sequence"] for row in rows] == list(range(1,len(rows)+1)),
        "all actual camera-corresponding quota walks including initial and final rows required")
    durations = []
    for row,camera in zip(rows,cameras):
        stamp = int(round(camera["timestamp"]*1.e9))
        frozen._require(row.get("status") == "PASS" and row.get("native_frame") == camera["frame"]
            and row.get("timestamp_simulation_ns") == stamp and row.get("capture_phase") == camera["capture_phase"]
            and row.get("complete_walk") is True and row.get("source_payload_read") is False
            and row.get("cached_or_incremental_byte_count_used") is False
            and type(row.get("entries")) is int and row["entries"] >= 0
            and type(row.get("regular_files")) is int and 0 <= row["regular_files"] <= row["entries"]
            and type(row.get("regular_file_bytes")) is int and row["regular_file_bytes"] >= 0
            and row["regular_file_bytes"] == row.get("returned_regular_file_bytes")
            and type(row.get("walk_started_monotonic_ns")) is int
            and type(row.get("walk_finished_monotonic_ns")) is int
            and 0 < row["walk_started_monotonic_ns"] <= row["walk_finished_monotonic_ns"],
            "actual quota walk omitted an entry, changed counts/source, failed or reversed clock")
        matching = [receipt for receipt in timings if receipt.get("command_frame") == camera["frame"]
            and (receipt.get("observed_bundle") or {}).get("native_frame") == camera["frame"]]
        frozen._require(matching and all(row["walk_finished_monotonic_ns"]
            <= receipt["observed_bundle"]["exact_bundle_delivery_monotonic_ns"] for receipt in matching),
            "owned complete walk did not precede the same actual camera bundle delivery")
        durations.append((row["walk_finished_monotonic_ns"]-row["walk_started_monotonic_ns"])/1.e9)
    return dict(passed=True,actual_walk_count=len(rows),all_actual_camera_walks_retained=True,
        metrics=callback._stats(durations),complete_walk_every_call=True,
        original_quota_free_floor_and_reserves_unchanged=True,
        disk_check_dispatch_and_scheduling_interval_separately_unresolved=True,
        actual_sensor_latency_gain_proven_by_walk_timing=False)
